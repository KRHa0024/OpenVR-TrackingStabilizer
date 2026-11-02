"""Compare a raw session with its ots-replay output (docs/phase1.md acceptance metrics).

    python eval_filter.py <raw.otsrec> <filtered.otsrec> [--trials NAME] [--static-device SERIAL]
    python eval_filter.py <live.otsrec> [--trials NAME] [--static-device SERIAL]

With a single file, compares the poses lighthouse sent with what the live filter forwarded
(both are recorded by the driver since Phase 1).

Per filtered device:
  changed   share of input poses whose output differs from the input (should be tiny on healthy data)
  states    time spent outside TRACKING and the transitions that happened
  added     part of that time after lighthouse was Running_OK again (probation + blend = lag the filter adds)
  steps     largest one-update step not explained by the reported velocity, raw vs filtered,
            and how many exceed 5 cm
With --trials and --static-device: per trial, the worst distance of the device from where it rested
before the trial (for experiments where that device never really moved, e.g. E2).
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

from otsrec import EMITTED_HOOK_SLOT, norm, read_session
from trials import find_trials

GAP_S = 0.05
STEP_LIMIT = 0.05


def dist(a, b):
    return norm(tuple(x - y for x, y in zip(a, b)))


def step_stats(poses):
    valid = [p for p in poses if p.pose_is_valid]
    steps = []
    for a, b in zip(valid, valid[1:]):
        dt = b.t - a.t
        if 0 < dt <= GAP_S:
            pred = tuple(x + v * dt for x, v in zip(a.world_position(), a.world_velocity()))
            steps.append(dist(b.world_position(), pred))
    return max(steps, default=0.0), sum(s > STEP_LIMIT for s in steps)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw", type=Path)
    ap.add_argument("filtered", type=Path, nargs="?")
    ap.add_argument("--trials")
    ap.add_argument("--static-device")
    args = ap.parse_args()

    raw = read_session(args.raw)
    flt = read_session(args.filtered) if args.filtered else raw.filtered_view()
    raw_by, flt_by = raw.poses_by_device(), flt.poses_by_device()
    events = [(e.t, json.loads(e.text)) for e in flt.events if e.kind == "filter"]
    filtered_devices = sorted({ev["device"] for _, ev in events} |
                              {d for d, ps in flt_by.items() if any(p.hook_slot == EMITTED_HOOK_SLOT for p in ps)})
    # Devices the filter saw but never transitioned still count as filtered when the replay ran on them.
    filtered_devices = sorted(set(filtered_devices) | {d for d in flt_by if flt.devices.get(d, {}).get("class") in (2, 3)})

    print(f"{raw.path.name}  ->  {flt.path.name}\n")
    print(f"{'device':13s} {'changed':>8} {'not TRACKING':>13} {'added':>7} {'suspect':>8} {'false':>6} {'hold':>5} "
          f"{'max step raw':>13} {'>5cm raw':>9} {'max step flt':>13} {'>5cm flt':>9}")
    for d in filtered_devices:
        rin = raw_by.get(d, [])
        fout = [p for p in flt_by.get(d, []) if p.hook_slot != EMITTED_HOOK_SLOT]
        if rin and fout and abs(len(rin) - len(fout)) <= 2:
            # Live recordings can start/stop between recording a pose and its filtered output.
            if fout[0].t < rin[0].t:
                fout = fout[1:]
            n = min(len(rin), len(fout))
            rin, fout = rin[:n], fout[:n]
        if not rin or len(rin) != len(fout):
            print(f"{raw.device_name(d):13s} (pose count mismatch {len(rin)} vs {len(fout)})")
            continue
        changed = sum(a.position != b.position or a.velocity != b.velocity or a.result != b.result
                      or a.pose_is_valid != b.pose_is_valid for a, b in zip(rin, fout))
        dev_events = [(t, ev) for t, ev in events if ev["device"] == d]
        trans = Counter((ev["from"], ev["to"]) for _, ev in dev_events)
        # time outside TRACKING, and the part of it the filter added after lighthouse was OK again
        # (from the last HOLD->PROBATION of an episode until TRACKING)
        outside, since, added, resumed = 0.0, None, 0.0, None
        for t, ev in dev_events:
            if ev["from"] == "TRACKING":
                since, resumed = t, None
            if ev["to"] == "PROBATION":
                resumed = t
            if ev["to"] == "TRACKING" and since is not None:
                outside += t - since
                if resumed is not None:
                    added += t - resumed
                since = None
        if since is not None:
            outside += rin[-1].t - since
        r_max, r_n = step_stats(rin)
        f_max, f_n = step_stats(flt_by.get(d, []))
        print(f"{raw.device_name(d):13s} {changed / len(rin) * 100:7.2f}% {outside:12.1f}s {added:6.1f}s "
              f"{trans[('TRACKING', 'SUSPECT')]:8d} {trans[('SUSPECT', 'BLEND')]:6d} "
              f"{trans[('TRACKING', 'HOLD')] + trans[('SUSPECT', 'HOLD')]:5d} "
              f"{r_max * 100:12.1f}cm {r_n:9d} {f_max * 100:12.1f}cm {f_n:9d}")

    if args.trials and args.static_device:
        dev = next(i for i, info in raw.devices.items() if info.get("serial", "").upper() == args.static_device.upper())
        print(f"\nworst distance from the resting position per trial ({args.static_device}) [cm]")
        rows = []
        for tr in find_trials(raw, args.trials):
            t0, t1 = tr.times["cover"], tr.times["end"] + 3.0
            rest = [p for p in raw_by[dev] if t0 - 0.5 <= p.t < t0 and p.pose_is_valid and p.result == 200]
            if not rest:
                rows.append((tr.tag, None, None))
                continue
            anchor = rest[-1].world_position()
            worst = lambda ps: max((dist(p.world_position(), anchor) for p in ps
                                    if t0 <= p.t <= t1 and p.pose_is_valid), default=0.0)  # noqa: E731
            rows.append((tr.tag, worst(raw_by[dev]), worst(flt_by[dev])))
        for tag, r, f in rows:
            print(f"   {tag[:40]:40s} raw {('-' if r is None else f'{r * 100:6.1f}'):>6}   filtered "
                  f"{('-' if f is None else f'{f * 100:6.1f}'):>6}")


if __name__ == "__main__":
    main()
