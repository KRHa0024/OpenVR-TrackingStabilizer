"""Prints one device's raw and filtered world positions around its invalid / not-OK poses.

    python tools/analysis/trace_around.py <session.otsrec> <serial> [--before 3] [--after 2] [--step 0.1]
"""
import argparse
import math

from otsrec import TRACKING_RESULT, read_session


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("serial")
    ap.add_argument("--before", type=float, default=3)
    ap.add_argument("--after", type=float, default=2)
    ap.add_argument("--step", type=float, default=0.1)
    a = ap.parse_args()
    s = read_session(a.session)
    dev = next(d for d, i in s.devices.items() if i.get("serial") == a.serial)
    raw = sorted((p for p in s.poses if p.device == dev), key=lambda p: p.t)
    out = sorted((p for p in s.filtered_poses if p.device == dev), key=lambda p: p.t)
    marks = [p.t for p in raw if not (p.pose_is_valid and p.result == 200)]
    events = [e for e in s.events if e.serial == a.serial or a.serial[4:] in e.text]
    done = -1e9
    for m in marks:
        if m < done:
            continue
        done = m + a.after
        print(f"==== {a.serial} not OK at t={m:.3f}")
        for e in events:
            if m - a.before <= e.t <= m + a.after:
                print(f"   event t={e.t:9.3f} [{e.kind}] {e.text[:140]}")
        t = m - a.before
        ri = oi = 0
        while t <= m + a.after:
            while ri + 1 < len(raw) and raw[ri + 1].t <= t:
                ri += 1
            while oi + 1 < len(out) and out[oi + 1].t <= t:
                oi += 1
            r, o = raw[ri], out[oi] if out else None
            rw = r.world_position()
            line = (f"t={t:9.3f} raw ({rw[0]:+.3f},{rw[1]:+.3f},{rw[2]:+.3f}) {TRACKING_RESULT.get(r.result, r.result)[:12]:12s} "
                    f"valid={int(r.pose_is_valid)} age={1000 * (t - r.t):5.0f}ms")
            if o:
                ow = o.world_position()
                line += f"  out ({ow[0]:+.3f},{ow[1]:+.3f},{ow[2]:+.3f}) d={math.dist(rw, ow):.3f}"
            print(line)
            t += a.step


if __name__ == "__main__":
    main()
