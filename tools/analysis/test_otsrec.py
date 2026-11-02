"""Checks otsrec.py against a file produced by tests/recorder_selftest.

    python test_otsrec.py <selftest.otsrec>
"""
import sys
from pathlib import Path

from otsrec import read_session


def main() -> int:
    s = read_session(Path(sys.argv[1]))
    by_dev = s.poses_by_device()
    assert not s.truncated
    assert s.label == "selftest", s.label
    assert sorted(by_dev) == [0, 1, 2], sorted(by_dev)
    for d, poses in by_dev.items():
        assert len(poses) == 2000, (d, len(poses))
        # Per-device order is preserved even with concurrent producers.
        ys = [p.position[1] for p in poses]
        assert ys == sorted(ys), d
        assert all(p.position[0] == d for p in poses)
        assert all(p.velocity == (0.0, 0.0, -1.5) for p in poses)
        assert sum(p.result == 201 for p in poses) == 4 * 50
        assert all(p.pose_is_valid and p.device_is_connected for p in poses)
        assert s.devices[d]["serial"] == f"TEST-{d}"
    markers = [e.text for e in s.events if e.kind == "marker"]
    assert markers == ["begin", "end"], markers
    logs = [e for e in s.events if e.kind == "log"]
    assert len(logs) == 1 and logs[0].serial == "LHR-7E570000" and logs[0].log_object == 0x1234
    assert len(s.raw_poses) == 0  # selftest raw poses are not "connected"
    print("otsrec selftest OK:", sum(len(p) for p in by_dev.values()), "poses")
    return 0


if __name__ == "__main__":
    sys.exit(main())
