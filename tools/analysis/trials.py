"""Per-trial timeline for protocol recordings (occlude / timed).

    python trials.py <session.otsrec> --device SERIAL_OR_INDEX [--name occlude] [--gap-ms 20]

For each trial (markers <name>_cover/_start/_uncover/_end) prints, relative to <name>_cover:
tracking-state transitions, pose-stream gaps, lighthouse log lines for the device, and how far
the reported position strayed from where the device was resting before it was covered.
"""
from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path

from otsrec import TRACKING_RESULT, norm, read_session

PHASES = ("cover", "start", "uncover", "end")


@dataclass
class Trial:
    tag: str
    times: dict  # phase -> t


def find_trials(session, name: str) -> list[Trial]:
    trials: dict[str, Trial] = {}
    order = []
    for e in session.events:
        if e.kind != "marker":
            continue
        m = re.match(rf"{re.escape(name)}_(cover|start|uncover|end)\s*(.*)", e.text)
        if not m:
            continue
        phase, tag = m.group(1), m.group(2)
        if tag not in trials:
            trials[tag] = Trial(tag, {})
            order.append(tag)
        trials[tag].times[phase] = e.t
    return [trials[t] for t in order if all(p in trials[t].times for p in PHASES)]


def resolve_device(session, key: str) -> int:
    if key.isdigit():
        return int(key)
    for idx, info in session.devices.items():
        if info.get("serial", "").upper() == key.upper():
            return idx
    raise SystemExit(f"device {key} not found; known: {[i.get('serial') for i in session.devices.values()]}")


def dist(a, b) -> float:
    return norm(tuple(x - y for x, y in zip(a, b)))


def analyze_trial(session, poses, serial, trial: Trial, gap_ms: float, pre: float, post: float):
    t0 = trial.times["cover"]
    lo, hi = t0 - pre, trial.times["end"] + post
    window = [p for p in poses if lo <= p.t <= hi]
    rest = [p for p in window if p.t < t0 and p.pose_is_valid and p.result == 200]
    if not rest:
        print("    (no valid pose before cover)")
        return None
    anchor = rest[-1].world_position()

    rows = []
    prev = None
    for a, b in zip(window, window[1:]):
        if (b.t - a.t) * 1000 > gap_ms:
            rows.append((a.t, f"gap {(b.t - a.t) * 1000:.0f}ms (until {b.t - t0:+.3f})"))
    for p in window:
        key = (p.result, p.pose_is_valid, p.device_is_connected)
        if key != prev:
            rows.append((p.t, f"state {TRACKING_RESULT.get(p.result, p.result)} valid={int(p.pose_is_valid)} conn={int(p.device_is_connected)} "
                              f"off={dist(p.world_position(), anchor) * 100:.1f}cm |v|={norm(p.velocity):.2f}"))
            prev = key
    for e in session.events:
        if e.kind == "log" and e.serial == serial and lo <= e.t <= hi:
            rows.append((e.t, "log   " + e.text.split(": ", 1)[-1][:110]))
        if e.kind == "marker" and lo <= e.t <= hi and e.text.split(" ")[0].endswith(PHASES):
            rows.append((e.t, "mark  " + e.text.split(" ")[0]))
    for t, text in sorted(rows):
        print(f"    {t - t0:+8.3f}  {text}")

    covered = [p for p in window if trial.times["start"] <= p.t <= trial.times["uncover"]]
    whole = [p for p in window if t0 <= p.t <= trial.times["end"]]
    def worst(ps):
        return max((dist(p.world_position(), anchor) for p in ps if p.pose_is_valid), default=0.0)
    summary = {
        "max_off_covered_cm": worst(covered) * 100,
        "max_off_trial_cm": worst(whole) * 100,
        "max_speed": max((norm(p.velocity) for p in whole), default=0.0),
        "poses_covered": len(covered),
    }
    print(f"    => max offset while covered {summary['max_off_covered_cm']:.1f}cm, whole trial {summary['max_off_trial_cm']:.1f}cm, "
          f"max |v| {summary['max_speed']:.2f}m/s, poses while covered {summary['poses_covered']}")
    return summary


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--device", required=True)
    ap.add_argument("--name", default="occlude")
    ap.add_argument("--gap-ms", type=float, default=20)
    ap.add_argument("--pre", type=float, default=0.5, help="seconds before _cover to include")
    ap.add_argument("--post", type=float, default=1.0, help="seconds after _end to include")
    args = ap.parse_args()

    s = read_session(args.session)
    device = resolve_device(s, args.device)
    serial = s.device_name(device)
    poses = s.poses_by_device().get(device, [])
    trials = find_trials(s, args.name)
    print(f"{s.path.name}: device [{device}] {serial}, {len(trials)} complete trials\n")
    for i, tr in enumerate(trials, 1):
        print(f"--- trial {i}: {tr.tag}  (cover at t={tr.times['cover']:.2f}, start {tr.times['start'] - tr.times['cover']:+.2f}, "
              f"uncover {tr.times['uncover'] - tr.times['cover']:+.2f}, end {tr.times['end'] - tr.times['cover']:+.2f})")
        analyze_trial(s, poses, serial, tr, args.gap_ms, args.pre, args.post)
        print()


if __name__ == "__main__":
    main()
