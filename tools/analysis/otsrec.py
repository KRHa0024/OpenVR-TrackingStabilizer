"""Reader / CSV exporter for *.otsrec sessions written by the trackingstabilizer driver.

Format is defined in common/RecordFormat.h; keep the two in sync.

    python otsrec.py <session.otsrec> [--out DIR]

writes DIR/<session>/{summary.txt, devices.json, events.csv, raw_poses.csv, poses_<idx>_<serial>.csv}
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import struct
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

MAGIC = b"OTSREC01"
FILE_HEADER = struct.Struct("<8sIIqqq64s")
RECORD_HEADER = struct.Struct("<HHIq")
POSE_PAYLOAD_HEADER = struct.Struct("<III")
LOG_PAYLOAD_HEADER = struct.Struct("<Q")
RAW_POSES_HEADER = struct.Struct("<II")
IMU_HEADER = struct.Struct("<III")
# vr::ImuSample_t: sample time, accel xyz, gyro xyz, off-scale flags (+4 bytes padding = 64)
IMU_SAMPLE = struct.Struct("<d3d3dI")

# vr::DriverPose_t (280 bytes)
DRIVER_POSE = struct.Struct("<d4d3d4d3d3d3d3d4d3d3di4?")
# vr::TrackedDevicePose_t (80 bytes)
TRACKED_DEVICE_POSE = struct.Struct("<12f3f3fi??2x")

REC_POSE, REC_LOG, REC_MARKER, REC_DEVICE_INFO, REC_RAW_POSES, REC_NOTE, REC_FILTER, REC_FILTERED_POSE = 1, 2, 3, 4, 5, 6, 7, 8
REC_IMU = 9
EMITTED_HOOK_SLOT = 100  # poses synthesized by the filter's emitter

TRACKING_RESULT = {
    1: "Uninitialized",
    100: "Calibrating_InProgress",
    101: "Calibrating_OutOfRange",
    200: "Running_OK",
    201: "Running_OutOfRange",
    300: "Fallback_RotationOnly",
}
DEVICE_CLASS = {0: "Invalid", 1: "HMD", 2: "Controller", 3: "GenericTracker", 4: "TrackingReference", 5: "DisplayRedirect"}

SERIAL_RE = re.compile(r"\b(LH[RB]-[0-9A-F]{8})\b")


@dataclass
class Pose:
    t: float
    device: int
    hook_slot: int
    pose_time_offset: float
    q_world_from_driver: tuple
    t_world_from_driver: tuple
    q_driver_from_head: tuple
    t_driver_from_head: tuple
    position: tuple
    velocity: tuple
    acceleration: tuple
    rotation: tuple  # w, x, y, z
    angular_velocity: tuple
    angular_acceleration: tuple
    result: int
    pose_is_valid: bool
    will_drift_in_yaw: bool
    should_apply_head_model: bool
    device_is_connected: bool

    def world_position(self) -> tuple:
        """Tracked reference point (vecPosition) in the driver's world (raw universe) space."""
        return tuple(a + b for a, b in zip(quat_rotate(self.q_world_from_driver, self.position), self.t_world_from_driver))

    def world_velocity(self) -> tuple:
        """vecVelocity is in driver space; rotate it like the position."""
        return quat_rotate(self.q_world_from_driver, self.velocity)

    def world_head_position(self) -> tuple:
        """Point apps see (vecPosition offset by driverFromHead), comparable to TrackedDevicePose_t."""
        local = tuple(a + b for a, b in zip(self.position, quat_rotate(self.rotation, self.t_driver_from_head)))
        return tuple(a + b for a, b in zip(quat_rotate(self.q_world_from_driver, local), self.t_world_from_driver))


@dataclass
class Event:
    t: float
    kind: str  # log | marker | note | filter
    text: str
    log_object: int = 0

    @property
    def serial(self) -> str:
        m = SERIAL_RE.search(self.text)
        return m.group(1) if m else ""


@dataclass
class RawPose:
    t: float
    device: int
    matrix: tuple  # 3x4 row-major
    velocity: tuple
    angular_velocity: tuple
    result: int
    valid: bool
    connected: bool


@dataclass
class ImuSample:
    t: float            # arrival in the driver (recording clock)
    device: int
    sample_time: float  # the device's own clock
    accel: tuple
    gyro: tuple
    off_scale: int


@dataclass
class Session:
    path: Path
    label: str
    unix_ms_start: int
    poses: list[Pose] = field(default_factory=list)
    filtered_poses: list[Pose] = field(default_factory=list)  # what the live filter forwarded (Phase 1)
    events: list[Event] = field(default_factory=list)
    raw_poses: list[RawPose] = field(default_factory=list)
    imu: list[ImuSample] = field(default_factory=list)  # raw IMU samples (driver with the IMU option on)
    devices: dict[int, dict] = field(default_factory=dict)
    truncated: bool = False

    def poses_by_device(self) -> dict[int, list[Pose]]:
        out: dict[int, list[Pose]] = defaultdict(list)
        for p in self.poses:
            out[p.device].append(p)
        return dict(sorted(out.items()))

    def device_name(self, index: int) -> str:
        info = self.devices.get(index, {})
        return info.get("serial") or f"device{index}"

    def filtered_view(self) -> "Session":
        """Same session, but `poses` are the live filter's output (only filtered devices)."""
        return Session(path=self.path, label=self.label, unix_ms_start=self.unix_ms_start, poses=self.filtered_poses,
                       events=self.events, raw_poses=self.raw_poses, devices=self.devices, truncated=self.truncated)


def quat_rotate(q: tuple, v: tuple) -> tuple:
    w, x, y, z = q
    vx, vy, vz = v
    # v + 2w(u x v) + 2u x (u x v), u = (x, y, z)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz))


def read_session(path: str | Path) -> Session:
    path = Path(path)
    data = path.read_bytes()
    magic, version, header_size, qpc_freq, qpc_start, unix_ms, label = FILE_HEADER.unpack_from(data, 0)
    if magic != MAGIC:
        raise ValueError(f"{path}: not an otsrec file")
    if version != 1:
        raise ValueError(f"{path}: unsupported version {version}")

    session = Session(path=path, label=label.split(b"\0", 1)[0].decode("utf-8", "replace"), unix_ms_start=unix_ms)
    to_seconds = lambda qpc: (qpc - qpc_start) / qpc_freq  # noqa: E731

    off = header_size
    while off + RECORD_HEADER.size <= len(data):
        rtype, _flags, size, qpc = RECORD_HEADER.unpack_from(data, off)
        off += RECORD_HEADER.size
        if off + size > len(data):
            session.truncated = True  # driver crashed or file copied mid-write
            break
        payload = memoryview(data)[off:off + size]
        off += size
        t = to_seconds(qpc)

        if rtype in (REC_POSE, REC_FILTERED_POSE):
            device, struct_size, slot = POSE_PAYLOAD_HEADER.unpack_from(payload, 0)
            if struct_size < DRIVER_POSE.size:
                continue
            f = DRIVER_POSE.unpack_from(payload, POSE_PAYLOAD_HEADER.size)
            (session.poses if rtype == REC_POSE else session.filtered_poses).append(Pose(
                t=t, device=device, hook_slot=slot, pose_time_offset=f[0],
                q_world_from_driver=f[1:5], t_world_from_driver=f[5:8],
                q_driver_from_head=f[8:12], t_driver_from_head=f[12:15],
                position=f[15:18], velocity=f[18:21], acceleration=f[21:24],
                rotation=f[24:28], angular_velocity=f[28:31], angular_acceleration=f[31:34],
                result=f[34], pose_is_valid=f[35], will_drift_in_yaw=f[36],
                should_apply_head_model=f[37], device_is_connected=f[38]))
        elif rtype == REC_LOG:
            (obj,) = LOG_PAYLOAD_HEADER.unpack_from(payload, 0)
            text = bytes(payload[LOG_PAYLOAD_HEADER.size:]).decode("utf-8", "replace")
            session.events.append(Event(t, "log", text.rstrip("\n"), obj))
        elif rtype == REC_MARKER:
            session.events.append(Event(t, "marker", bytes(payload).decode("utf-8", "replace")))
        elif rtype == REC_NOTE:
            session.events.append(Event(t, "note", bytes(payload).decode("utf-8", "replace")))
        elif rtype == REC_FILTER:
            session.events.append(Event(t, "filter", bytes(payload).decode("utf-8", "replace")))
        elif rtype == REC_DEVICE_INFO:
            info = json.loads(bytes(payload).decode("utf-8", "replace"))
            session.devices[int(info["index"])] = info
        elif rtype == REC_RAW_POSES:
            count, struct_size = RAW_POSES_HEADER.unpack_from(payload, 0)
            if struct_size < TRACKED_DEVICE_POSE.size:
                continue
            for i in range(count):
                f = TRACKED_DEVICE_POSE.unpack_from(payload, RAW_POSES_HEADER.size + i * struct_size)
                # 0..11 matrix, 12..14 velocity, 15..17 angular velocity, 18 result, 19 valid, 20 connected
                if not f[20]:
                    continue
                session.raw_poses.append(RawPose(t, i, f[0:12], f[12:15], f[15:18], f[18], f[19], f[20]))
        elif rtype == REC_IMU:
            device, count, struct_size = IMU_HEADER.unpack_from(payload, 0)
            if struct_size < IMU_SAMPLE.size:
                continue
            for i in range(count):
                f = IMU_SAMPLE.unpack_from(payload, IMU_HEADER.size + i * struct_size)
                session.imu.append(ImuSample(t, device, f[0], f[1:4], f[4:7], f[7]))
    return session


def norm(v: tuple) -> float:
    return math.sqrt(sum(c * c for c in v))


def summarize(session: Session) -> str:
    lines = [f"session: {session.path.name}", f"label: {session.label}", f"truncated: {session.truncated}", ""]
    for device, poses in session.poses_by_device().items():
        info = session.devices.get(device, {})
        dur = poses[-1].t - poses[0].t if len(poses) > 1 else 0.0
        dts = [b.t - a.t for a, b in zip(poses, poses[1:])]
        results = Counter(TRACKING_RESULT.get(p.result, str(p.result)) for p in poses)
        invalid = sum(1 for p in poses if not p.pose_is_valid)
        max_speed = max(norm(p.velocity) for p in poses)
        lines.append(f"[{device}] {session.device_name(device)} class={DEVICE_CLASS.get(info.get('class'), info.get('class'))} model={info.get('model', '')}")
        lines.append(f"    poses={len(poses)} duration={dur:.2f}s rate={len(poses) / dur if dur else 0:.1f}Hz "
                     f"max_gap={max(dts) * 1000 if dts else 0:.1f}ms")
        lines.append(f"    result={dict(results)} invalid={invalid} max_speed={max_speed:.2f}m/s")
    lines.append("")
    kinds = Counter(e.kind for e in session.events)
    lines.append(f"events: {dict(kinds)}")
    return "\n".join(lines)


def export(session: Session, out_dir: Path) -> Path:
    target = out_dir / session.path.stem
    target.mkdir(parents=True, exist_ok=True)

    (target / "summary.txt").write_text(summarize(session), encoding="utf-8")
    (target / "devices.json").write_text(json.dumps(session.devices, indent=2, ensure_ascii=False), encoding="utf-8")

    with open(target / "events.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "kind", "serial", "log_object", "text"])
        for e in session.events:
            w.writerow([f"{e.t:.6f}", e.kind, e.serial, hex(e.log_object) if e.log_object else "", e.text])

    with open(target / "raw_poses.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "device", "serial", "valid", "result", "x", "y", "z"])
        for r in session.raw_poses:
            m = r.matrix
            w.writerow([f"{r.t:.6f}", r.device, session.device_name(r.device), int(r.valid), r.result, m[3], m[7], m[11]])

    header = ["t", "hook_slot", "result", "valid", "connected", "pose_time_offset",
              "wx", "wy", "wz", "px", "py", "pz", "vx", "vy", "vz", "speed", "ax", "ay", "az", "acc",
              "qw", "qx", "qy", "qz", "avx", "avy", "avz", "ang_speed"]
    for device, poses in session.poses_by_device().items():
        name = re.sub(r"[^A-Za-z0-9_-]", "_", session.device_name(device))
        with open(target / f"poses_{device:02d}_{name}.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(header)
            for p in poses:
                wp = p.world_position()
                w.writerow([f"{p.t:.6f}", p.hook_slot, p.result, int(p.pose_is_valid), int(p.device_is_connected),
                            f"{p.pose_time_offset:.6f}", *(f"{c:.6f}" for c in wp), *(f"{c:.6f}" for c in p.position),
                            *(f"{c:.5f}" for c in p.velocity), f"{norm(p.velocity):.5f}",
                            *(f"{c:.4f}" for c in p.acceleration), f"{norm(p.acceleration):.4f}",
                            *(f"{c:.6f}" for c in p.rotation), *(f"{c:.5f}" for c in p.angular_velocity),
                            f"{norm(p.angular_velocity):.5f}"])
    return target


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--out", type=Path, default=Path("analysis_out"))
    args = ap.parse_args()

    session = read_session(args.session)
    target = export(session, args.out)
    print(summarize(session))
    print(f"\nexported to {target}")


if __name__ == "__main__":
    main()
