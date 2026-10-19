"""Compile controller tests against the actual sketch function; optionally replay CSV.

Run on a host with g++: python3 scripts/diagnostics/check_fall_prevention.py
    --replay /path/to/telemetry.csv
No hardware access or motor commands. Temporary compiler output is removed.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


def check(replay=None):
    root = Path(__file__).resolve().parents[2]
    sketch = (root / "firmware/BaseLink/BaseLink.ino").read_text()
    controller = sketch[sketch.index("void runController(float dt)"):sketch.index(
        "// ============================================================\n//  SETUP")]
    with tempfile.TemporaryDirectory(prefix="2pybot-fall-tests-") as temp:
        directory = Path(temp)
        (directory / "controller_under_test.inc").write_text(controller)
        binary = directory / "test-fall-prevention"
        subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I", temp,
                        str(root / "tests/test_fall_prevention.cpp"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)] + ([str(replay)] if replay else []), check=True)
        automatic=directory/"test-auto-climb"
        subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror",
                        str(root/"tests/test_auto_climb.cpp"),"-o",str(automatic)],check=True)
        subprocess.run([str(automatic)],check=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path)
    args = parser.parse_args()
    check(args.replay)
