#!/usr/bin/env python3
"""Translate Chinese comments/docstrings -> English across the MiniHexa repo.

A per-line splitter isolates ONLY the comment (or docstring) text and translates
just that span, leaving all code bytes byte-for-byte identical. The exact same
splitter is used to (a) extract the distinct translatable units and (b) apply
the translation map, so keys always match.

Chinese that lives inside *functional* string literals (e.g. WonderLLM LLM-prompt
strings) has no comment span and is therefore left untouched by design.

Usage:
  python cjk_tool.py extract           # print distinct Han-bearing units + counts
  python cjk_tool.py residual          # print Han-bearing units missing from map
  python cjk_tool.py apply             # rewrite files using translations.json
"""
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.dirname(os.path.abspath(__file__))
TARGET_DIRS = [
    "arduino-programming-projects", "python-programming-projects",
    "scratch-programming-projects", "app-remote-control",
    "host-control-and-action-editor", "serial-communication-tutorial", "sources",
]
C_EXTS = {".cpp", ".h", ".ino", ".hpp", ".c"}
PY_EXTS = {".py"}
SKIP_PARTS = {"build", ".git", ".fw_venv", "node_modules", ".vscode"}
HAN = re.compile(r"[\u4e00-\u9fff]")


def iter_files():
    for d in TARGET_DIRS:
        base = os.path.join(ROOT, d)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [x for x in dirnames if x not in SKIP_PARTS]
            for fn in filenames:
                ext = os.path.splitext(fn)[1]
                if ext in C_EXTS:
                    yield os.path.join(dirpath, fn), "c"
                elif ext in PY_EXTS:
                    yield os.path.join(dirpath, fn), "py"


def find_comment_span_c(line, in_block):
    """Return (cstart, cend, new_in_block) for the translatable comment span, or
    (None, None, new_in_block) if the line has no comment text to translate."""
    if in_block:
        end = line.find("*/")
        if end == -1:
            return (0, len(line), True)          # whole line is inside the block
        return (0, end, False)                   # up to the closing */
    i, n = 0, len(line)
    in_str = None
    while i < n:
        ch = line[i]
        if in_str:
            if ch == "\\":
                i += 2
                continue
            if ch == in_str:
                in_str = None
            i += 1
            continue
        if ch in "\"'":
            in_str = ch
            i += 1
            continue
        if ch == "/" and i + 1 < n and line[i + 1] == "/":
            return (i + 2, len(line), False)
        if ch == "/" and i + 1 < n and line[i + 1] == "*":
            end = line.find("*/", i + 2)
            if end == -1:
                return (i + 2, len(line), True)
            return (i + 2, end, False)
        i += 1
    return (None, None, False)


def find_comment_span_py(line, in_doc):
    """Return (cstart, cend, new_in_doc). in_doc is None or the active triple delim."""
    if in_doc:
        end = line.find(in_doc)
        if end == -1:
            return (0, len(line), in_doc)
        return (0, end, None)
    i, n = 0, len(line)
    in_str = None
    while i < n:
        ch = line[i]
        if in_str:
            if ch == "\\":
                i += 2
                continue
            if ch == in_str:
                in_str = None
            i += 1
            continue
        # triple-quoted docstring open
        if line.startswith("'''", i) or line.startswith('"""', i):
            delim = line[i:i + 3]
            close = line.find(delim, i + 3)
            if close == -1:
                return (i + 3, len(line), delim)     # opens a block
            return (i + 3, close, None)              # single-line docstring
        if ch in "\"'":
            in_str = ch
            i += 1
            continue
        if ch == "#":
            return (i + 1, len(line), None)
        i += 1
    return (None, None, None)


def process_line(line, lang, state, tmap):
    """Return (new_line, new_state, unit_or_None). If tmap is None, only detect
    the unit; otherwise perform replacement when the unit is in tmap."""
    if lang == "c":
        cstart, cend, new_state = find_comment_span_c(line, state)
    else:
        cstart, cend, new_state = find_comment_span_py(line, state)
    unit = None
    new_line = line
    if cstart is not None:
        raw = line[cstart:cend]
        key = raw.strip()
        if key and HAN.search(key):
            unit = key
            if tmap is not None and key in tmap:
                new_line = line[:cstart] + raw.replace(key, tmap[key], 1) + line[cend:]
    return new_line, new_state, unit


def scan(tmap, rewrite):
    from collections import Counter
    counts = Counter()
    files_changed = lines_changed = 0
    for path, lang in iter_files():
        try:
            with open(path, encoding="utf-8") as f:
                text = f.read()
        except (UnicodeDecodeError, OSError):
            continue
        lines = text.splitlines(keepends=True)
        state = None if lang == "py" else False
        out = []
        changed = 0
        for line in lines:
            body = line.rstrip("\n")
            nl = line[len(body):]
            new_body, state, unit = process_line(body, lang, state, tmap)
            if unit is not None:
                counts[unit] += 1
            if new_body != body:
                changed += 1
            out.append(new_body + nl)
        if rewrite and changed:
            with open(path, "w", encoding="utf-8") as f:
                f.write("".join(out))
            files_changed += 1
            lines_changed += changed
    return counts, files_changed, lines_changed


def load_map():
    p = os.path.join(HERE, "translations.json")
    if not os.path.exists(p):
        return {}
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "extract"
    if mode == "extract":
        counts, _, _ = scan(None, False)
        for unit, c in counts.most_common():
            print(f"{c}\t{unit}")
        print(f"\n# distinct units: {len(counts)}", file=sys.stderr)
    elif mode == "residual":
        tmap = load_map()
        counts, _, _ = scan(None, False)
        missing = [(c, u) for u, c in counts.items() if u not in tmap]
        for c, u in sorted(missing, reverse=True):
            print(f"{c}\t{u}")
        print(f"\n# residual distinct units: {len(missing)}", file=sys.stderr)
    elif mode == "apply":
        tmap = load_map()
        _, fc, lc = scan(tmap, True)
        print(f"Rewrote {lc} comment spans across {fc} files.")
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
