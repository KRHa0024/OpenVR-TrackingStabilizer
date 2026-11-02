"""Pseudo-occlude a controller and compare how far each hand estimate is from where it really was.

    python compare_hand_models.py <recording> --hand <serial> --hmd <serial> [--duration 1 3]
                                  [--variant name=key=value,key=value ...]

Healthy stretches of the controller are masked (ots-replay --mask: the result code is changed, the
pose and its rotation are kept, as lighthouse's out-of-range poses and the IMU would provide) and
replayed once per variant. The error is the distance between the replayed output and the recorded
healthy position. Default variants: world hold, shoulder follow, and the arm model at three strengths.
"""
from __future__ import annotations

import argparse
import bisect
import json
import subprocess
from pathlib import Path

from eval_body_model import distance, poses, records

DEFAULT_VARIANTS = [
    "hold=handEnabled=0",
    "shoulder=handEnabled=1,handArmFollow=0",
    "arm50=handEnabled=1,handArmFollow=0.5",
    "arm75=handEnabled=1,handArmFollow=0.75",
    "arm100=handEnabled=1,handArmFollow=1",
]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording", type=Path)
    ap.add_argument("--hand", required=True)
    ap.add_argument("--hmd", required=True)
    ap.add_argument("--duration", type=float, nargs="+", default=[1.0, 3.0])
    ap.add_argument("--variant", nargs="+", default=DEFAULT_VARIANTS)
    ap.add_argument("--replay", type=Path, default=Path("build/tools/ots-replay/Release/ots-replay.exe"))
    ap.add_argument("--out", type=Path, default=Path("build/hand-compare"))
    args = ap.parse_args()

    devices = {}
    for _, _, kind, data in records(args.recording):
        if kind == 4:
            d = json.loads(data)
            devices[d["serial"].upper()] = d
    hand, head = devices[args.hand.upper()], devices[args.hmd.upper()]
    if hand["class"] != 2 or hand.get("role") not in (1, 2) or head["class"] != 1:
        ap.error("needs a left/right controller and an HMD")

    target, hgood, ht = [], False, -1.0
    for t, qpc, kind, data in records(args.recording):
        if kind != 1:
            continue
        index, slot, pos, good = poses(data)
        if slot == 100:
            continue
        if index == head["index"]:
            hgood, ht = good, t
        elif index == hand["index"]:
            target.append((t, qpc, pos, good and hgood and 0 <= t - ht <= .1))
    spans, begin, prev = [], None, None
    for t, _, _, good in target:
        if not good or (prev is not None and t - prev > .05):
            if begin is not None:
                spans.append((begin, prev))
            begin = None
        if good and begin is None:
            begin = t
        prev = t
    if begin is not None:
        spans.append((begin, prev))

    args.out.mkdir(parents=True, exist_ok=True)
    print(f"{args.recording.name}: {args.hand} ({'left' if hand['role'] == 1 else 'right'} hand)")
    for duration in args.duration:
        masks = []
        for b, e in spans:
            t = b + 1.5
            while t + duration < e:
                masks.append((t, t + duration))
                t += duration + 2.5
        if not masks:
            print(f"  {duration:.0f} s masks: no healthy stretch long enough")
            continue
        starts = [a for a, _ in masks]
        truth = {}
        for t, qpc, pos, good in target:
            i = bisect.bisect_right(starts, t) - 1
            if good and i >= 0 and t < masks[i][1]:
                truth[qpc] = (i, t - masks[i][0], pos)
        print(f"  {duration:.0f} s masks: {len(masks)}, {len(truth)} samples")
        print(f"    {'variant':10} {'mean cm':>8} {'p95 cm':>8} {'max cm':>8} {'end mean cm':>12}")
        for v in args.variant:
            name, settings = v.split("=", 1)
            output = args.out / f"{args.recording.stem}.{name}.{duration:g}s.otsrec"
            command = [str(args.replay.resolve()), str(args.recording.resolve()), "-o", str(output.resolve()),
                       "--mask-device", str(hand["index"]), "--reference-hmd", str(head["index"])]
            for kv in settings.split(","):
                command += ["--set", kv]
            for b, e in masks:
                command += ["--mask", f"{b:.9f}:{e:.9f}"]
            subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
            errors, ends = [], {}
            for _, qpc, kind, data in records(output):
                if kind == 1 and qpc in truth:
                    index, slot, pos, _ = poses(data)
                    if index == hand["index"] and slot != 100:
                        i, since, real = truth[qpc]
                        err = distance(pos, real) * 100
                        errors.append(err)
                        if since > duration - .1:
                            ends.setdefault(i, err)
            errors.sort()
            end = sum(ends.values()) / len(ends) if ends else float("nan")
            print(f"    {name:10} {sum(errors) / len(errors):8.2f} {errors[int(.95 * (len(errors) - 1))]:8.2f}"
                  f" {errors[-1]:8.2f} {end:12.2f}")


if __name__ == "__main__":
    main()
