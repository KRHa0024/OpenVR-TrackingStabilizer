"""Per-trial, per-device tracking health for worn / moving experiments (E4, E5).

    python motion_trials.py <session.otsrec> --name body [--devices LHR-...,LHR-...]

For each trial (<name>_cover .. <name>_end + 1 s) and device prints:
  lost      time spent not Running_OK or not poseIsValid, and whether lighthouse reset tracking
  gap       longest pause in the pose stream
  jump      largest step between consecutive valid poses not explained by the reported velocity (cm)
  reacq     largest position change across a pause in the stream (> 50 ms), i.e. last pose before vs first after (cm)
  vjump     largest change of reported velocity between consecutive poses (|dv|, m/s)
  hold      how far the device wandered while the user held still (<name>_start+0.5 .. _uncover)
  offscale  "IMU went off scale" log lines for that device
"""
from __future__ import annotations

import argparse
from pathlib import Path

from otsrec import norm, read_session
from trials import find_trials


GAP_S = 0.05


def dist(a, b):
    return norm(tuple(x - y for x, y in zip(a, b)))


def analyze(s, trials, devices, post=1.0):
    rows = []
    by_dev = s.poses_by_device()
    for i, tr in enumerate(trials, 1):
        lo, hi = tr.times["cover"], tr.times["end"] + post
        hold_lo, hold_hi = tr.times["start"] + 0.5, tr.times["uncover"]
        for d in devices:
            serial = s.device_name(d)
            ps = [p for p in by_dev.get(d, []) if lo <= p.t <= hi]
            if len(ps) < 10:
                rows.append((i, tr.tag, serial, None))
                continue
            lost = 0.0
            for a, b in zip(ps, ps[1:]):
                if a.result != 200 or not a.pose_is_valid:
                    lost += b.t - a.t
            gap = max((b.t - a.t for a, b in zip(ps, ps[1:])), default=0)
            valid = [p for p in ps if p.pose_is_valid]
            # Step not explained by the reported velocity: an optical correction or a teleport.
            # Pairs across a stream pause are reported separately as `reacq` (where the pose was
            # when the stream stopped vs where it resumed).
            pairs = list(zip(valid, valid[1:]))
            jump = max((dist(b.world_position(),
                             tuple(x + v * (b.t - a.t) for x, v in zip(a.world_position(), a.world_velocity())))
                        for a, b in pairs if b.t - a.t <= GAP_S), default=0) * 100
            reacq = max((dist(b.world_position(), a.world_position()) for a, b in pairs if b.t - a.t > GAP_S),
                        default=0) * 100
            vjump = max((dist(b.velocity, a.velocity) for a, b in zip(valid, valid[1:])), default=0)
            held = [p for p in valid if hold_lo <= p.t <= hold_hi]
            hold = max((dist(p.world_position(), held[0].world_position()) for p in held), default=0) if held else 0
            logs = [e for e in s.events if e.kind == "log" and e.serial == serial and lo <= e.t <= hi]
            rows.append((i, tr.tag, serial, {
                "lost_ms": lost * 1000,
                "reset": any("Resetting tracking" in e.text for e in logs),
                "gap_ms": gap * 1000,
                "jump": jump,
                "reacq": reacq,
                "vjump": vjump,
                "hold_cm": hold * 100,
                "offscale": sum("off scale" in e.text for e in logs),
            }))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--name", required=True)
    ap.add_argument("--devices", default="", help="comma-separated serials (default: all non-base-station devices)")
    args = ap.parse_args()

    s = read_session(args.session)
    if args.devices:
        wanted = {x.strip().upper() for x in args.devices.split(",")}
        devices = [d for d, info in s.devices.items() if info.get("serial", "").upper() in wanted]
    else:
        devices = [d for d, ps in s.poses_by_device().items() if len(ps) > 1000 and s.devices.get(d, {}).get("class") != 4]
    trials = find_trials(s, args.name)
    print(f"{s.path.name}: {len(trials)} trials, devices {[s.device_name(d) for d in devices]}\n")
    print(f"{'#':>2} {'condition':28s} {'device':13s} {'lost ms':>8} {'reset':>5} {'gap ms':>7} {'jump cm':>8} "
          f"{'reacq cm':>8} {'|dv| m/s':>9} {'hold cm':>8} {'offscale':>8}")
    last = None
    for i, tag, serial, r in analyze(s, trials, devices):
        if last is not None and i != last:
            print()
        last = i
        if r is None:
            print(f"{i:>2} {tag[:28]:28s} {serial:13s}   (no data)")
            continue
        print(f"{i:>2} {tag[:28]:28s} {serial:13s} {r['lost_ms']:8.0f} {('yes' if r['reset'] else ''):>5} {r['gap_ms']:7.1f} "
              f"{r['jump']:8.1f} {r['reacq']:8.1f} {r['vjump']:9.2f} {r['hold_cm']:8.1f} {r['offscale']:8d}")


if __name__ == "__main__":
    main()
