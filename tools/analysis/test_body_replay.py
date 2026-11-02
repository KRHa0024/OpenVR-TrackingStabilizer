"""Binary replay integration test: python test_body_replay.py <ots-replay.exe>."""
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

from eval_body_model import records, poses, distance
from otsrec import FILE_HEADER, RECORD_HEADER, POSE_PAYLOAD_HEADER, DRIVER_POSE


def pose(position, rotation=(1, 0, 0, 0)):
    return DRIVER_POSE.pack(0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            *position, 0, 0, 0, 0, 0, 0, *rotation, 0, 0, 0, 0, 0, 0,
                            200, True, False, False, True)


def main():
    replay = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix="ots-body-") as directory:
        root = Path(directory)
        source = root / "source.otsrec"
        with source.open("wb") as f:
            f.write(FILE_HEADER.pack(b"OTSREC01", 1, FILE_HEADER.size, 1000000, 0, 0, b"synthetic body"))

            def write(kind, t, payload):
                f.write(RECORD_HEADER.pack(kind, 0, len(payload), round(t*1000000)))
                f.write(payload)

            for index, cls in ((0, 1), (1, 3)):
                write(4, 0, json.dumps({"index": index, "class": cls, "serial": f"TEST-{index}"}, separators=(",", ":")).encode())
            for i in range(501):
                t = i*.004
                movement = max(0, t-1)
                x = 1+movement*.15
                angle = movement*.3
                write(1, t, POSE_PAYLOAD_HEADER.pack(0, DRIVER_POSE.size, 0)+pose((x, 1.62, -.08)))
                position = (x+.5*math.sin(angle), 1.5-.5*math.cos(angle), 0)
                write(1, t, POSE_PAYLOAD_HEADER.pack(1, DRIVER_POSE.size, 0)+pose(position, (math.cos(angle/2), 0, 0, math.sin(angle/2))))
        originals = {(qpc, poses(data)[0]): poses(data)[2] for _, qpc, kind, data in records(source) if kind == 1}
        means = {}
        for enabled in (0, 1):
            out = root / f"out{enabled}.otsrec"
            cmd = [replay, str(source), "-o", str(out), "--body-waist", "1", "--body-hmd", "0",
                   "--mask", "1:1.5", "--set", f"bodyEnabled={enabled}"]
            subprocess.run(cmd, check=True, capture_output=True)
            errors, body_notes = [], []
            for t, qpc, kind, data in records(out):
                if kind == 1:
                    index, slot, position, _ = poses(data)
                    if slot == 100:
                        continue
                    if index == 0:
                        assert position == originals[qpc, index], "reference HMD was modified"
                    elif 1 <= t < 1.5:
                        errors.append(distance(position, originals[qpc, index]))
                elif kind == 6:
                    body_notes.append(json.loads(data))
            means[enabled] = sum(errors)/len(errors)
            assert any(n["event"] == "pseudo_occlusion" for n in body_notes)
            if enabled:
                assert any(n.get("status") == "estimating" for n in body_notes), "no recorded model activation"
        assert means[1] < .001 and means[0] > .05, means
        # Missing roles and same-file aliases are rejected before writing output.
        before = source.read_bytes()
        for args in (("--set", "bodyEnabled=1"), ("-o", str(source.parent / "." / source.name)), ("--emit-hz", "0")):
            assert subprocess.run([replay, str(source), *args], capture_output=True).returncode == 2
            assert source.read_bytes() == before
        print("body replay integration passed; hold/model mean errors (m):", means)


if __name__ == "__main__":
    main()
