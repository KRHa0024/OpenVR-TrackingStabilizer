"""Pseudo-occlude a designated waist or hand and compare hold / HMD translation / model.

Requires explicit serials: DeviceInfo controller role 3 does NOT identify a waist.
Uses only optically healthy input as the reference, with causal HMD samples <=100 ms old.
Outputs replay files and a CSV. No SteamVR connection or installed-driver changes.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import struct
import subprocess
from pathlib import Path

from otsrec import FILE_HEADER, RECORD_HEADER, POSE_PAYLOAD_HEADER, DRIVER_POSE, quat_rotate


def records(path):
    with Path(path).open("rb") as f:
        magic, version, size, freq, start, _, _ = FILE_HEADER.unpack(f.read(FILE_HEADER.size))
        if magic != b"OTSREC01" or version != 1 or freq <= 0:
            raise ValueError("unsupported recording")
        f.seek(size)
        while h := f.read(RECORD_HEADER.size):
            kind, _, length, qpc = RECORD_HEADER.unpack(h)
            data = f.read(length)
            if len(data) != length:
                raise ValueError("truncated recording")
            yield (qpc-start)/freq, qpc, kind, data


def poses(data):
    index, size, slot = POSE_PAYLOAD_HEADER.unpack_from(data)
    if size < DRIVER_POSE.size or len(data) < POSE_PAYLOAD_HEADER.size+size:
        raise ValueError("invalid DriverPose payload")
    p = DRIVER_POSE.unpack_from(data, POSE_PAYLOAD_HEADER.size)
    world = tuple(a+b for a, b in zip(quat_rotate(p[1:5], p[15:18]), p[5:8]))
    good = p[34] == 200 and p[35] and p[38] and any(p[15:18]) and all(map(math.isfinite, world))
    return index, slot, world, good


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording", type=Path)
    targets = ap.add_mutually_exclusive_group(required=True)
    targets.add_argument("--waist", help="explicit waist serial; never inferred")
    targets.add_argument("--hand", help="controller serial with a left/right role in DeviceInfo")
    ap.add_argument("--hmd", required=True, help="explicit HMD serial")
    ap.add_argument("--replay", type=Path, default=Path("build/tools/ots-replay/Release/ots-replay.exe"))
    ap.add_argument("--out", type=Path, default=Path("build/body-eval"))
    ap.add_argument("--duration", type=float, default=1.0)
    args = ap.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        ap.error("duration must be positive and finite")
    devices = {}
    for _, _, kind, data in records(args.recording):
        if kind == 4:
            d = json.loads(data)
            devices[d["serial"].upper()] = d
    target_serial = args.waist or args.hand
    waist = devices[target_serial.upper()]
    head = devices[args.hmd.upper()]
    if waist["class"] != (2 if args.hand else 3) or head["class"] != 1:
        ap.error("roles require the requested tracker/controller class and an HMD")
    if args.hand and waist.get("role") not in (1, 2):
        ap.error("hand target needs a SteamVR left/right controller role")
    target = []
    hpos, ht, hgood = None, -1, False
    for t, qpc, kind, data in records(args.recording):
        if kind != 1:
            continue
        index, slot, pos, good = poses(data)
        if slot == 100:
            continue
        if index == head["index"]:
            hpos, ht, hgood = pos, t, good
        elif index == waist["index"]:
            target.append((t, qpc, pos, good and hgood and 0 <= t-ht <= .1, hpos))
    # Disjoint masks, with at least one second of healthy warmup and recovery between them.
    spans, begin, prev = [], None, None
    for t, _, _, good, _ in target:
        if not good or (prev is not None and t-prev > .05):
            if begin is not None:
                spans.append((begin, prev))
            begin = None
        if good and begin is None:
            begin = t
        prev = t
    if begin is not None:
        spans.append((begin, prev))
    masks = []
    for begin, end in spans:
        t = begin+1
        while t+args.duration < end:
            masks.append((t, t+args.duration))
            t += args.duration+2
    if not masks:
        raise ValueError("no sufficiently long healthy interval for pseudo-occlusion")
    args.out.mkdir(parents=True, exist_ok=True)
    starts = [a for a, _ in masks]
    selected = {}
    origins = {}
    for t, qpc, pos, good, hpos in target:
        i = bisect.bisect_right(starts, t)-1
        if good and i >= 0 and t < masks[i][1]:
            origins.setdefault(i, (pos, hpos))
            base, hbase = origins[i]
            relative = tuple(p+h-h0 for p, h, h0 in zip(base, hpos, hbase))
            selected[qpc] = [i, t, pos, distance(pos, relative), None, None]
    del target
    coverage = {}
    for name, enabled in (("hold", 0), ("body", 1)):
        output = args.out / f"{args.recording.stem}.{name}.otsrec"
        command = [str(args.replay.resolve()), str(args.recording.resolve()), "-o", str(output.resolve())]
        command += (["--mask-device", str(waist["index"]), "--reference-hmd", str(head["index"]), "--set", f"handEnabled={enabled}"]
                    if args.hand else ["--body-waist", str(waist["index"]), "--body-hmd", str(head["index"]), "--set", f"bodyEnabled={enabled}"])
        for begin, end in masks:
            command += ["--mask", f"{begin:.9f}:{end:.9f}"]
        subprocess.run(command, check=True)
        active, used = False, 0
        for _, qpc, kind, data in records(output):
            if kind == 6:
                try:
                    event = json.loads(data)
                except (ValueError, UnicodeDecodeError):
                    continue
                if event.get("event") == ("hand_model" if args.hand else "body_model") and event.get("device") == waist["index"]:
                    active = event.get("status") == "estimating"
            elif kind == 1 and qpc in selected:
                index, slot, pos, _ = poses(data)
                if index == waist["index"] and slot != 100:
                    selected[qpc][4 if name == "hold" else 5] = distance(pos, selected[qpc][2])
                    used += active
        coverage[name] = used
    rows = list(selected.values())
    if not rows or any(r[4] is None or r[5] is None for r in rows):
        raise ValueError("replay output did not match the input timestamps")
    with (args.out / "errors.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["mask", "t", "truth_x", "truth_y", "truth_z", "relative_m", "hold_m", "body_m"])
        writer.writerows((i, t, *p, relative, hold, body) for i, t, p, relative, hold, body in rows)
    summary = [f"{len(masks)} masks, {len(rows)} healthy samples; explicit target {target_serial}",
               "method,mean_cm,p95_cm,max_cm"]
    for name, col in (("world hold", 4), ("HMD translation", 3), ("shoulder relative + fallback" if args.hand else "torso model + fallback", 5)):
        values = sorted(r[col]*100 for r in rows)
        summary.append(f"{name},{sum(values)/len(values):.3f},{values[int(.95*(len(values)-1))]:.3f},{values[-1]:.3f}")
    summary += [f"Model active coverage (approximate event timing): {coverage['body']/len(rows):.1%}",
                "Reference: healthy Lighthouse poses, not external ground truth. Mounting role is supplied by the caller.",
                "Masks preserve pose rotation; this does not validate IMU availability during real occlusion or stream stops."]
    report = "\n".join(summary)+"\n"
    (args.out / "summary.txt").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
