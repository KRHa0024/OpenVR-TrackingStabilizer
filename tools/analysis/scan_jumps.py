"""Looks for sudden jumps in a recording's tracker poses, raw and (if recorded) filtered.

A "jump" is a change of world position larger than --jump metres between two consecutive poses that both
claim to be valid and Running_OK. Prints each one with the poses around it, which is how a pose that
snaps to (or near) the origin while tracking is reported as fine shows up.

    python tools/analysis/scan_jumps.py <session.otsrec> [--jump 0.15] [--classes 2,3]
"""
import argparse
import math

from otsrec import TRACKING_RESULT, read_session


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def scan(poses, name, jump, label):
    found = 0
    prev = None
    for p in poses:
        if prev is not None and p.pose_is_valid and p.result == 200 and prev.pose_is_valid and prev.result == 200:
            d = dist(p.world_position(), prev.world_position())
            if d > jump and p.t - prev.t < 0.1:
                found += 1
                if found <= 12:
                    wp = p.world_position()
                    print(f"  [{label}] {name} t={p.t:9.3f} jump {d:.3f} m in {1000 * (p.t - prev.t):.1f} ms -> world "
                          f"({wp[0]:+.3f},{wp[1]:+.3f},{wp[2]:+.3f}) driver ({p.position[0]:+.3f},{p.position[1]:+.3f},{p.position[2]:+.3f}) "
                          f"|wfd|={math.sqrt(sum(v * v for v in p.t_world_from_driver)):.3f}")
        prev = p
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--jump", type=float, default=0.15)
    ap.add_argument("--classes", default="2,3")
    a = ap.parse_args()
    classes = {int(c) for c in a.classes.split(",")}
    s = read_session(a.session)
    print(f"{a.session}: {len(s.poses)} poses, {len(s.filtered_poses)} filtered")
    for label, poses in (("raw", s.poses), ("filtered", s.filtered_poses)):
        by = {}
        for p in poses:
            by.setdefault(p.device, []).append(p)
        for dev, ps in sorted(by.items()):
            info = s.devices.get(dev, {})
            if info.get("class") not in classes:
                continue
            ps.sort(key=lambda p: p.t)
            n = scan(ps, f"{info.get('serial', dev)} ({info.get('model', '')})", a.jump, label)
            states = {}
            for p in ps:
                states[TRACKING_RESULT.get(p.result, p.result), p.pose_is_valid] = states.get((TRACKING_RESULT.get(p.result, p.result), p.pose_is_valid), 0) + 1
            print(f"{label} {info.get('serial', dev)} {info.get('model', '')}: {len(ps)} poses, {n} jumps; results {states}")


if __name__ == "__main__":
    main()
