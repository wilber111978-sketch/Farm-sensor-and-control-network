#!/usr/bin/env python3
"""Merge the go2rtc camera-page update INTO an existing FarmPi app.py.

The deployed app.py on FarmPi can be newer than the copy in Git (tank history,
fault states, ...), so this never replaces the file. It applies only the camera
changes, each matched exactly. If any piece does not match, nothing is written.

Usage: python3 apply_camera_update.py /opt/farm-dashboard/app.py
Exit codes: 0 = applied or already applied, 2 = did not match (file untouched).
"""
import json
import py_compile
import sys
import tempfile
from pathlib import Path

MARKER = "GO2RTC_PLAYER = "


def main():
    target = Path(sys.argv[1] if len(sys.argv) > 1 else "/opt/farm-dashboard/app.py")
    hunks = json.loads((Path(__file__).parent / "camera_update_hunks.json").read_text(encoding="utf-8"))
    text = target.read_text(encoding="utf-8")

    if MARKER in text:
        print(f"Camera update already present in {target} - nothing to do.")
        return 0

    problems = []
    for number, hunk in enumerate(hunks, 1):
        count = text.count(hunk["old"])
        if count != 1:
            first = next((l.strip() for l in hunk["old"].splitlines() if l.strip()), "")
            problems.append(f"  piece {number}: found {count} times (starts: {first[:70]!r})")
    if problems:
        print(f"Camera update does NOT match {target}; file left unchanged:")
        print("\n".join(problems))
        return 2

    for hunk in hunks:
        text = text.replace(hunk["old"], hunk["new"], 1)

    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False, encoding="utf-8") as tmp:
        tmp.write(text)
    try:
        py_compile.compile(tmp.name, doraise=True)
    except py_compile.PyCompileError as error:
        print(f"Merged file failed to compile; {target} left unchanged:\n{error}")
        return 2
    finally:
        Path(tmp.name).unlink(missing_ok=True)

    target.write_text(text, encoding="utf-8")
    print(f"Camera update merged into {target} ({len(hunks)} pieces).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
