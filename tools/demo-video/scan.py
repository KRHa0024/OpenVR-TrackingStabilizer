"""Find stretches of a recording that show the hand or waist model well, for the demo video.

    python scan.py <recording> --device N --hmd N --kind hand|waist [--duration 3]

Masks every healthy stretch of the device in turn (ots-replay --mask, as compare_hand_models.py does),
replays once holding and once with the model, and prints per stretch: how far the device really moved,
and the mean distance of each output from the real position.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from eval_body_model import distance, poses, records  # noqa: E402

REPLAY = Path("build/tools/ots-replay/Release/ots-replay.exe")


def track(path, device, emitted):
    out = []
    for t, _, kind, data in records(path):
        if kind == 1:
            index, slot, pos, good = poses(data)
            if index == device and (slot == 100) == emitted:
                out.append((t, pos, good))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording", type=Path)
    ap.add_argument("--device", type=int, required=True)
    ap.add_argument("--hmd", type=int, required=True)
    ap.add_argument("--kind", choices=["hand", "waist"], required=True)
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--out", type=Path, default=Path("build/demo-video/scan"))
    args = ap.parse_args()

    truth = track(args.recording, args.device, False)
    spans, begin, prev = [], None, None
    for t, _, good in truth:
        if not good or (prev is not None and t - prev > .05):
            if begin is not None:
                spans.append((begin, prev))
            begin = None
        if good and begin is None:
            begin = t
        prev = t
    if begin is not None:
        spans.append((begin, prev))
    masks = []
    for b, e in spans:
        t = b + 2.0
        while t + args.duration + 1.0 < e:
            masks.append((t, t + args.duration))
            t += args.duration + 3.0
    args.out.mkdir(parents=True, exist_ok=True)
    model = "handEnabled=1" if args.kind == "hand" else "bodyEnabled=1"
    roles = ["--reference-hmd", str(args.hmd)] if args.kind == "hand" else ["--body-waist", str(args.device), "--body-hmd", str(args.hmd)]
    outputs = {}
    for name, setting in (("hold", "handEnabled=0" if args.kind == "hand" else "bodyEnabled=0"), ("model", model)):
        out = args.out / f"{args.recording.stem}.{args.device}.{name}.otsrec"
        cmd = [str(REPLAY), str(args.recording), "-o", str(out), *roles, "--mask-device", str(args.device), "--set", setting]
        for m in masks:
            cmd += ["--mask", f"{m[0]}:{m[1]}"]
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
        outputs[name] = track(out, args.device, False) + track(out, args.device, True)
        outputs[name].sort()

    def at(series, t):
        lo, hi = 0, len(series) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if series[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        return series[lo][1]

    print(f"{'begin':>7} {'moved':>6} {'hold':>6} {'model':>6}  (cm)")
    for b, e in masks:
        ts = [b + i * .02 for i in range(int((e - b) / .02))]
        start = at(truth, b)
        moved = max(distance(at(truth, t), start) for t in ts)
        err = {k: sum(distance(at(v, t), at(truth, t)) for t in ts) / len(ts) for k, v in outputs.items()}
        print(f"{b:7.1f} {moved * 100:6.1f} {err['hold'] * 100:6.1f} {err['model'] * 100:6.1f}")


if __name__ == "__main__":
    main()
