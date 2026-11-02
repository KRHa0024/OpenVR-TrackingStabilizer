"""Turn recordings into the per-frame data the demo video renderer draws (build/demo-video/data.js).

    python tools/demo-video/prepare.py [--only scene ...]

Each scene names a recording, a time window and the replays to run (ots-replay with the real filter).
For every frame (60 fps) the data holds where each device was according to the recording and to each
replay, so the page can draw the real body, what SteamVR got without the filter, and the filter's output.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "analysis"))
from otsrec import quat_rotate, read_session  # noqa: E402

REPLAY = ROOT / "build" / "tools" / "ots-replay" / "Release" / "ots-replay.exe"
SESSIONS = Path.home() / "AppData" / "Local" / "OpenVR-TrackingStabilizer" / "sessions"
OUT = ROOT / "build" / "demo-video"
FPS = 60

BODY = SESSIONS / "20261024-170855_body.otsrec"
# Device indices in BODY: 0 HMD, 1 right hand, 2 left hand, 5 waist, 6/7 feet.
JOINTS = {"head": 0, "rhand": 1, "lhand": 2, "waist": 5, "lfoot": 6, "rfoot": 7}

SCENES = {
    # The waist is covered with a cloth while standing: what lighthouse sends, and the filter's hold.
    "cover": dict(recording=BODY, window=(80.0, 89.0), device=5, replays={
        "hold": []}),
    # A hand pseudo-occluded while standing up from a chair: hold vs the hand model.
    "hand": dict(recording=BODY, window=(254.0, 261.0), device=2, mask=(256.0, 259.0), replays={
        "hold": ["--reference-hmd", "0", "--set", "handEnabled=0"],
        "model": ["--reference-hmd", "0", "--set", "handEnabled=1"]}),
    # The waist pseudo-occluded while moving: VRChat's freeze vs the body model.
    "waist": dict(recording=BODY, window=(275.0, 284.0), device=5, mask=(277.1, 280.1), replays={
        "hold": ["--body-waist", "5", "--body-hmd", "0", "--set", "bodyEnabled=0"],
        "model": ["--body-waist", "5", "--body-hmd", "0", "--set", "bodyEnabled=1"]}),
}


def qmul(a, b) -> tuple:
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def world_rotation(p) -> tuple:
    """Rotation of the point apps see (the head for an HMD: driverFromHead included, as SteamVR does)."""
    return qmul(qmul(p.q_world_from_driver, p.rotation), p.q_driver_from_head)


class Track:
    """Latest pose of one device at a given time (sample and hold)."""

    def __init__(self, poses):
        self.poses = sorted(poses, key=lambda p: p.t)
        self.times = [p.t for p in self.poses]

    def at(self, t, max_age=.25):
        i = bisect.bisect_right(self.times, t) - 1
        if i < 0 or t - self.times[i] > max_age:
            return None
        return self.poses[i]


def good(p) -> bool:
    return p is not None and p.result == 200 and p.pose_is_valid and p.device_is_connected and any(p.position)


def r3(v):
    return [round(c, 3) for c in v]


def prepare(name: str, scene: dict) -> dict:
    t0, t1 = scene["window"]
    session = read_session(scene["recording"])
    raw = {d: Track([p for p in ps if p.hook_slot != 100]) for d, ps in session.poses_by_device().items()}
    outputs = {}
    for replay, args in scene["replays"].items():
        out = OUT / f"{name}.{replay}.otsrec"
        cmd = [str(REPLAY), str(scene["recording"]), "-o", str(out), *args]
        if "mask" in scene:
            cmd += ["--mask-device", str(scene["device"]), "--mask", f"{scene['mask'][0]}:{scene['mask'][1]}"]
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
        replayed = read_session(out)
        out.unlink()  # a full copy of the session; only this window is kept
        outputs[replay] = Track([p for p in replayed.poses if p.device == scene["device"]])

    frames = [t0 + i / FPS for i in range(int((t1 - t0) * FPS))]
    floor = min(p.world_head_position()[1] for d in (6, 7) for p in raw[d].poses if t0 <= p.t <= t1 and good(p))
    heads = [raw[0].at(t) for t in frames]
    centre = [sum(p.world_head_position()[i] for p in heads if p) / sum(1 for p in heads if p) for i in (0, 2)]
    # Face the camera: the HMD's mean forward (-z) direction in the window, flattened.
    fx = fz = 0.0
    for p in heads:
        if p:
            f = quat_rotate(world_rotation(p), (0, 0, -1))
            fx, fz = fx + f[0], fz + f[2]
    facing = math.atan2(fx, fz)

    def place(v):
        return r3((v[0] - centre[0], v[1] - floor, v[2] - centre[1]))

    data = {"name": name, "fps": FPS, "t0": t0, "facing": round(facing, 4), "frames": len(frames),
            "mask": [round(m - t0, 3) for m in scene["mask"]] if "mask" in scene else None, "joints": {}, "outputs": {}}
    for joint, d in JOINTS.items():
        pos, rot, ok = [], [], []
        for t in frames:
            p = raw[d].at(t)
            if p and not p.pose_is_valid:
                p = None  # SteamVR ignores invalid poses; their position is meaningless
            pos.append(place(p.world_head_position()) if p else None)
            rot.append([round(c, 4) for c in world_rotation(p)] if p else None)
            ok.append(1 if good(p) else 0)
        data["joints"][joint] = {"pos": pos, "rot": rot, "ok": ok}
    for replay, track in outputs.items():
        data["outputs"][replay] = [place(p.world_head_position()) if (p := track.at(t)) else None for t in frames]
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", nargs="*")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "data.json"
    scenes = json.loads(path.read_text()) if path.exists() else {}
    for name, scene in SCENES.items():
        if args.only and name not in args.only:
            continue
        scenes[name] = prepare(name, scene)
        print(name, scenes[name]["frames"], "frames")
    path.write_text(json.dumps(scenes, separators=(",", ":")))
    (OUT / "data.js").write_text("window.SCENES = " + json.dumps(scenes, separators=(",", ":")) + ";\n")


if __name__ == "__main__":
    main()
