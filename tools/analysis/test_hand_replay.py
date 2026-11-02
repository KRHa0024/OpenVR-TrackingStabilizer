"""Three-point binary replay integration: python test_hand_replay.py <ots-replay.exe>."""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from eval_body_model import records, poses, distance
from otsrec import FILE_HEADER, RECORD_HEADER, POSE_PAYLOAD_HEADER, DRIVER_POSE
from test_body_replay import pose


def make_source(path, roles=True):
    with path.open("wb") as f:
        f.write(FILE_HEADER.pack(b"OTSREC01", 1, FILE_HEADER.size, 1000000, 0, 0, b"three point hand"))

        def write(kind, t, data):
            f.write(RECORD_HEADER.pack(kind, 0, len(data), round(t*1000000)))
            f.write(data)

        for index, cls, role in ((0, 1, 0), (1, 2, 1), (2, 2, 2)):
            write(4, 0, json.dumps({"index": index, "class": cls, "serial": f"TEST-{index}",
                                    "role": role if roles else 0}, separators=(",", ":")).encode())
        for i in range(601):
            t = i*.004
            x = 1+max(0, t-1)*.2
            for index, position in ((0, (x, 1.62, -.08)), (1, (x-.35, 1.1, -.3)), (2, (x+.35, 1.1, -.3))):
                write(1, t, POSE_PAYLOAD_HEADER.pack(index, DRIVER_POSE.size, 0)+pose(position))


def main():
    replay = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix="ots-hand-") as directory:
        root = Path(directory)
        source = root / "source.otsrec"
        make_source(source)
        originals = {(qpc, poses(data)[0]): data for _, qpc, kind, data in records(source) if kind == 1}
        for targets in ((1,), (2,), (1, 2)):
            errors = {}
            for enabled in (0, 1):
                out = root / "out.otsrec"
                # Exact shoulder follow of a walking body: no dead zone (the 10 cm default would absorb part of it).
                cmd = [replay, str(source), "-o", str(out), "--reference-hmd", "0", "--set", f"handEnabled={enabled}",
                       "--set", "handFollowDeadzone=0", "--mask", "1:1.8"]
                for target in targets:
                    cmd += ["--mask-device", str(target)]
                subprocess.run(cmd, capture_output=True, check=True)
                samples, events = [], []
                for t, qpc, kind, data in records(out):
                    if kind == 1:
                        index, slot, pos, _ = poses(data)
                        if slot == 100:
                            continue
                        if index == 0 or index not in targets or t < 1:
                            assert data == originals[qpc, index], "healthy pose was modified"
                        elif 1 <= t < 1.8:
                            samples.append(distance(pos, poses(originals[qpc, index])[2]))
                    elif kind == 6:
                        events.append(json.loads(data))
                errors[enabled] = sum(samples)/len(samples)
                if enabled:
                    for target in targets:
                        assert any(e.get("event") == "hand_model" and e.get("device") == target and e.get("status") == "estimating" for e in events)
            assert errors[1] < .001 and errors[0] > .07, (targets, errors)
        # No role guessing when the recording does not identify left and right.
        unknown = root / "unknown.otsrec"
        make_source(unknown, roles=False)
        assert subprocess.run([replay, str(unknown), "--reference-hmd", "0", "--set", "handEnabled=1"], capture_output=True).returncode == 2
        print("hand replay integration passed: left loss, right loss, both loss, healthy passthrough, role validation")


if __name__ == "__main__":
    main()
