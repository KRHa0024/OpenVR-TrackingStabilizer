"""Static-device comparison per trial: baseline (before _cover) vs condition (_start.._uncover) vs after (_end..).

    python static_conditions.py <session.otsrec> --name bs_block

Meant for experiments where every device rests still and only the environment changes (E3).
For each trial and device prints the shift of the mean position, the noise, any non-OK
tracking states, and which base stations the device stopped seeing ("SOB: drop").
"""
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

from otsrec import norm, read_session
from trials import find_trials

DROP_RE = re.compile(r"SOB: drop (S-\d+)(?: \(([0-9A-F]{8})\))?")


def stats(points):
    if len(points) < 10:
        return None
    mean = tuple(sum(c) / len(points) for c in zip(*points))
    std = math.sqrt(sum(sum((a - b) ** 2 for a, b in zip(p, mean)) for p in points) / len(points))
    return mean, std


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--name", default="bs_block")
    args = ap.parse_args()

    s = read_session(args.session)
    by_dev = s.poses_by_device()
    devices = [d for d, ps in by_dev.items() if len(ps) > 1000 and s.devices.get(d, {}).get("class") != 4]

    for i, tr in enumerate(find_trials(s, args.name), 1):
        c, st, un, en = (tr.times[k] for k in ("cover", "start", "uncover", "end"))
        print(f"--- trial {i}: {tr.tag}")
        drops = {}
        for e in s.events:
            if e.kind == "log" and c <= e.t <= un + 1.0:
                m = DROP_RE.search(e.text)
                if m:
                    drops.setdefault(m.group(2) or m.group(1), set()).add(e.serial)
        if drops:
            print("    SOB drops: " + ", ".join(f"{bs}: {len(sers)} device(s)" for bs, sers in drops.items()))
        for d in devices:
            ps = by_dev[d]
            base = stats([p.world_position() for p in ps if c - 2.0 <= p.t < c and p.pose_is_valid])
            cond = stats([p.world_position() for p in ps if st + 1.0 <= p.t <= un and p.pose_is_valid])
            after = stats([p.world_position() for p in ps if en + 1.0 <= p.t <= en + 4.0 and p.pose_is_valid])
            window = [p for p in ps if c <= p.t <= en + 1.0]
            bad = sum(1 for p in window if p.result != 200 or not p.pose_is_valid)
            gaps = max(((b.t - a.t) * 1000 for a, b in zip(window, window[1:])), default=0)
            peak = max((norm(tuple(x - y for x, y in zip(p.world_position(), base[0]))) for p in window if p.pose_is_valid),
                       default=0) if base else 0
            if not (base and cond):
                print(f"    [{d}] {s.device_name(d)}: insufficient data")
                continue
            shift = norm(tuple(a - b for a, b in zip(cond[0], base[0])))
            back = norm(tuple(a - b for a, b in zip(after[0], base[0]))) if after else float("nan")
            print(f"    [{d}] {s.device_name(d):13s} shift {shift * 1000:6.2f}mm  peak {peak * 1000:6.1f}mm  "
                  f"noise {base[1] * 1000:5.2f} -> {cond[1] * 1000:5.2f}mm  after {back * 1000:6.2f}mm  "
                  f"non-OK {bad:4d}  max gap {gaps:5.1f}ms")
        print()


if __name__ == "__main__":
    main()
