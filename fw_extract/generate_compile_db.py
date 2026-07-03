#!/usr/bin/env python3
"""Generate compile_commands.json for clangd from an arduino-cli verbose build.

ESP32 core 2.x does not emit a compilation database; this script runs a verbose
build, parses the g++ -c lines, and rewrites sketch paths from the Arduino
cache back to the real sketch directory.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SKETCH = os.path.join(
    ROOT,
    "arduino-programming-projects/4.1 Arduino IDE安装及偏差调试/remote",
)
DEFAULT_FQBN = "esp32:esp32:esp32:PartitionScheme=huge_app"
COMPILE_RE = re.compile(
    r"^(?P<compiler>\S+/xtensa-esp32-elf-g\+\+)\s+"
    r"(?P<args>.*)\s+"
    r"(?P<file>\S+\.cpp)\s+-o\s+(?P<out>\S+\.o)\s*$"
)


def sketch_cache_glob(sketch: str) -> list[str]:
    base = os.path.expanduser("~/.cache/arduino/sketches")
    if not os.path.isdir(base):
        return []
    name = os.path.basename(sketch.rstrip("/"))
    return sorted(
        os.path.join(base, d)
        for d in os.listdir(base)
        if os.path.isdir(os.path.join(base, d, "sketch", f"{name}.ino.cpp"))
    )


def clean_sketch_cache(sketch: str) -> None:
    for path in sketch_cache_glob(sketch):
        shutil.rmtree(path, ignore_errors=True)


def run_verbose_build(sketch: str, fqbn: str) -> str:
    cmd = [
        "arduino-cli", "compile", "-v", "--clean",
        "--fqbn", fqbn,
        sketch,
    ]
    proc = subprocess.run(
        cmd,
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    log = proc.stdout + proc.stderr
    if proc.returncode != 0:
        print(log[-4000:], file=sys.stderr)
        raise SystemExit(f"arduino-cli compile failed (exit {proc.returncode})")
    return log


def map_source_path(cached: str, sketch_abs: str) -> str:
    cached = os.path.abspath(cached)
    sketch_abs = os.path.abspath(sketch_abs)
    marker = os.sep + "sketch" + os.sep
    if marker in cached:
        return os.path.join(sketch_abs, cached.split(marker, 1)[1])
    return cached


def parse_compile_commands(log: str, sketch: str) -> list[dict]:
    sketch_abs = os.path.abspath(sketch)
    entries: list[dict] = []
    seen: set[str] = set()

    for raw in log.splitlines():
        line = raw.strip()
        if "xtensa-esp32-elf-g++" not in line or not line.endswith(".cpp.o"):
            continue
        m = COMPILE_RE.match(line)
        if not m:
            continue
        src = map_source_path(m.group("file"), sketch_abs)
        if src in seen:
            continue
        seen.add(src)
        args = [m.group("compiler")] + m.group("args").split() + ["-c", src]
        entries.append({
            "directory": ROOT,
            "file": src,
            "arguments": args,
        })

    return entries


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sketch", default=DEFAULT_SKETCH)
    ap.add_argument("--fqbn", default=DEFAULT_FQBN)
    ap.add_argument(
        "--output",
        default=os.path.join(ROOT, "compile_commands.json"),
        help="where to write compile_commands.json (default: repo root)",
    )
    ap.add_argument(
        "--no-clean",
        action="store_true",
        help="do not delete cached sketch build dir before compiling",
    )
    args = ap.parse_args()

    sketch = os.path.abspath(args.sketch)
    if not args.no_clean:
        clean_sketch_cache(sketch)

    print(f"Building {sketch} with {args.fqbn} ...", file=sys.stderr)
    log = run_verbose_build(sketch, args.fqbn)
    entries = parse_compile_commands(log, sketch)
    if not entries:
        raise SystemExit("No compile commands parsed from verbose build log")

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(entries, f, indent=2)
        f.write("\n")

    print(f"Wrote {len(entries)} entries to {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
