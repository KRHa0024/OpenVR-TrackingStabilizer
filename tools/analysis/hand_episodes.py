"""Why did (or didn't) an occluded hand move? One line of facts per controller loss episode.

    python hand_episodes.py <session.otsrec>

For each loss of a left/right controller in a live recording: the hand model's status changes, how far
the head and the other hand moved, how steeply the head looked up or down, how far the output moved
and turned, how long lighthouse was silent, and how many raw IMU samples arrived.
"""
import json
import math
import sys
from collections import defaultdict

from otsrec import quat_rotate, read_session


def qmul(a, b):
    return (a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3], a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1], a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0])


def angle(a, b):
    return math.degrees(2 * math.acos(min(1.0, abs(sum(x*y for x, y in zip(a, b))))))


def wrot(p):
    return qmul(p.q_world_from_driver, p.rotation)


def main():
    s = read_session(sys.argv[1])
    hmd = next((i for i, d in s.devices.items() if d.get("class") == 1), None)
    hands = {i: d for i, d in s.devices.items() if d.get("class") == 2 and d.get("role") in (1, 2)}
    print(f"{s.path.name}: HMD {s.device_name(hmd) if hmd is not None else '-'}, "
          f"hands {', '.join(f'{s.device_name(i)} ({'L' if d['role'] == 1 else 'R'})' for i, d in hands.items())}, "
          f"{len(s.imu)} IMU samples")
    for e in s.events:
        if e.kind in ("note", "log") and ("IMU" in e.text or "hand_model" in e.text[:30] or "settings" in e.text[:40]):
            if "hand_model" not in e.text:
                print(f"  {e.t:8.2f} {e.kind}: {e.text[:160]}")

    raw, out = defaultdict(list), defaultdict(list)
    for p in s.poses:
        if p.hook_slot < 100:
            raw[p.device].append(p)
    for p in s.filtered_poses:
        out[p.device].append(p)
    imu = defaultdict(list)
    for smp in s.imu:
        imu[smp.device].append(smp.t)

    status = defaultdict(list)  # device -> [(t, status)]
    episodes = defaultdict(list)
    start = {}
    for e in s.events:
        try:
            j = json.loads(e.text)
        except ValueError:
            continue
        if j.get("event") == "hand_model":
            status[j["device"]].append((e.t, j["status"]))
        elif e.kind == "filter" and "from" in j:
            d = j["device"]
            if j["from"] in ("TRACKING", "BLEND") and j["to"] in ("HOLD", "SUSPECT") and d not in start:
                start[d] = (e.t, j["reason"])
            elif j["to"] == "TRACKING" and d in start:
                t0, why = start.pop(d)
                episodes[d].append((t0, e.t, why))

    def near(ps, t):
        best = None
        for p in ps:
            if p.t > t:
                break
            if p.pose_is_valid and any(p.position):
                best = p
        return best

    for d in hands:
        other = next((i for i in hands if i != d), None)
        print(f"\n{s.device_name(d)} ({'left' if hands[d]['role'] == 1 else 'right'}): {len(episodes[d])} loss episodes")
        for t0, t1, why in episodes[d]:
            if t1 - t0 < 0.3:
                continue
            seq = [f"{t - t0:+.2f}s {st}" for t, st in status[d] if t0 - 0.05 <= t <= t1]
            head0 = near(raw[hmd], t0) if hmd is not None else None
            heads = [p for p in raw[hmd] if t0 <= p.t <= t1 and p.pose_is_valid and p.result == 200] if hmd is not None else []
            head_move = max((math.dist(p.world_position(), head0.world_position()) for p in heads), default=float("nan")) if head0 else float("nan")
            pitch = min((math.hypot(*(lambda f: (f[0], f[2]))(quat_rotate(wrot(p), (0, 0, -1)))) for p in heads), default=float("nan"))
            o0 = near(raw[other], t0) if other is not None else None
            others = [p for p in raw[other] if t0 <= p.t <= t1 and p.pose_is_valid and p.result == 200] if other is not None else []
            other_move = max((math.dist(p.world_position(), o0.world_position()) for p in others), default=float("nan")) if o0 else float("nan")
            outs = [p for p in out[d] if t0 <= p.t <= t1]
            moved = max((math.dist(p.world_position(), outs[0].world_position()) for p in outs), default=0) if outs else 0
            turned = max((angle(wrot(p), wrot(outs[0])) for p in outs), default=0) if outs else 0
            seg = [p for p in raw[d] if t0 <= p.t <= t1]
            silent = max((b.t - a.t for a, b in zip(seg, seg[1:])), default=t1 - t0)
            n_imu = sum(1 for t in imu[d] if t0 <= t <= t1)
            print(f"  {t0:8.2f} +{t1 - t0:5.2f}s ({why}); lighthouse silent {silent:4.1f}s, IMU samples {n_imu}")
            print(f"           head moved {head_move * 100:5.1f} cm (forward's horizontal part min {pitch:.2f}; <0.30 = too steep),"
                  f" other hand moved {other_move * 100:5.1f} cm ({len(others)} visible poses)")
            print(f"           output moved {moved * 100:5.1f} cm, turned {turned:5.1f} deg; hand model: {', '.join(seq) or 'no status change'}")


if __name__ == "__main__":
    main()
