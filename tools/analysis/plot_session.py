"""Per-device time-series plots of a session, with markers and notable lighthouse log lines.

    python plot_session.py <session.otsrec> [--out DIR] [--device INDEX ...] [--window T0 T1]

Requires matplotlib (pip install -r requirements.txt).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from otsrec import TRACKING_RESULT, norm, read_session  # noqa: E402

# Log lines worth drawing on the timeline (substring -> color).
LOG_HIGHLIGHTS = {
    "Resetting tracking": "red",
    "IMU went off scale": "magenta",
    "IMU Only": "orange",
    "-> Lighthouse": "green",
    "SOB: drop": "lightgray",
    "SOB: add": "lightblue",
    "BOOTSTRAPPED": "blue",
}


GAP_S = 0.05


def plot_device(session, device, poses, out_dir: Path, window):
    serial = session.device_name(device)
    if window:
        poses = [p for p in poses if window[0] <= p.t <= window[1]]
    if not poses:
        return None
    # Break lines across stream gaps and hide invalid poses, so nothing is drawn where the
    # driver reported nothing usable.
    nan = float("nan")
    t, world, speed, ang = [], [], [], []
    for prev, p in zip([None] + poses[:-1], poses):
        if prev is not None and p.t - prev.t > GAP_S:
            t.append((prev.t + p.t) / 2)
            world.append((nan, nan, nan))
            speed.append(nan)
            ang.append(nan)
        t.append(p.t)
        world.append(p.world_position() if p.pose_is_valid else (nan, nan, nan))
        speed.append(norm(p.velocity) if p.pose_is_valid else nan)
        ang.append(norm(p.angular_velocity) if p.pose_is_valid else nan)

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    for i, axis in enumerate("xyz"):
        axes[0].plot(t, [w[i] for w in world], label=axis, linewidth=0.8)
    axes[0].set_ylabel("world pos [m] (valid only)")
    axes[0].legend(loc="upper right")

    axes[1].plot(t, speed, linewidth=0.8, label="|v| (m/s)")
    axes[1].plot(t, ang, linewidth=0.6, alpha=0.6, label="|ω| (rad/s)")
    axes[1].set_ylabel("speed")
    axes[1].legend(loc="upper right")

    # Lighthouse leaves vecAcceleration at zero, so show the update interval instead.
    axes[2].plot([b.t for b in poses[1:]], [(b.t - a.t) * 1000 for a, b in zip(poses, poses[1:])], linewidth=0.6)
    axes[2].set_yscale("log")
    axes[2].set_ylabel("update interval [ms]")

    codes = sorted({p.result for p in poses} | {200})
    pose_t = [p.t for p in poses]
    axes[3].step(pose_t, [codes.index(p.result) for p in poses], where="post", label="result")
    axes[3].step(pose_t, [0.5 if p.pose_is_valid else 0.0 for p in poses], where="post", label="poseIsValid", alpha=0.7)
    axes[3].set_yticks(range(len(codes)), [TRACKING_RESULT.get(c, str(c)) for c in codes])
    axes[3].set_xlabel("t [s]")
    axes[3].legend(loc="upper right")

    t0, t1 = pose_t[0], pose_t[-1]
    for e in session.events:
        if not (t0 <= e.t <= t1):
            continue
        color = None
        if e.kind == "marker":
            phase = e.text.split(" ")[0].rsplit("_", 1)[-1]
            color = {"cover": "olive", "start": "green", "uncover": "red", "end": "gray"}.get(phase, "black")
        elif e.kind == "log" and (not e.serial or e.serial == serial):
            color = next((c for key, c in LOG_HIGHLIGHTS.items() if key in e.text), None)
        if color:
            for ax in axes:
                ax.axvline(e.t, color=color, linewidth=0.7, alpha=0.8)
            if e.kind == "marker":
                axes[0].annotate(e.text.split(" ")[0], (e.t, 1.0), xycoords=("data", "axes fraction"),
                                 rotation=90, fontsize=7, va="top")

    fig.suptitle(f"{session.path.name}  [{device}] {serial}")
    fig.tight_layout()
    out = out_dir / f"{session.path.stem}_{device:02d}_{serial}.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--out", type=Path, default=Path("analysis_out"))
    ap.add_argument("--device", type=int, nargs="*")
    ap.add_argument("--window", type=float, nargs=2, metavar=("T0", "T1"))
    args = ap.parse_args()

    session = read_session(args.session)
    args.out.mkdir(parents=True, exist_ok=True)
    for device, poses in session.poses_by_device().items():
        if args.device and device not in args.device:
            continue
        out = plot_device(session, device, poses, args.out, args.window)
        if out:
            print(out)


if __name__ == "__main__":
    main()
