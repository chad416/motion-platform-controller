#!/usr/bin/env python3
"""Render captured console output (a test run log) as a terminal-style PNG.

The text is the real output of the command, filtered with --grep if given;
nothing is edited. Uses rich (SVG) and rsvg-convert (PNG).

Usage: log_to_png.py run.log out.png --title "renode-test" [--grep REGEX] [--tail N]
"""
from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import tempfile

from rich.console import Console
from rich.text import Text

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("log", type=pathlib.Path)
    p.add_argument("png", type=pathlib.Path)
    p.add_argument("--title", default="")
    p.add_argument("--grep", default=None, help="keep only lines matching this regex")
    p.add_argument("--tail", type=int, default=60)
    p.add_argument("--width", type=int, default=110)
    a = p.parse_args()

    lines = [ANSI.sub("", ln).rstrip() for ln in a.log.read_text(errors="replace").splitlines()]
    lines = ["".join(ch for ch in ln if ch.isprintable()) for ln in lines]
    if a.grep:
        rx = re.compile(a.grep)
        lines = [ln for ln in lines if rx.search(ln)]
    lines = lines[-a.tail:]

    console = Console(record=True, width=a.width, file=open("/dev/null", "w"))
    # Red only for an actual failure ("failed: 0, error: 0" is a success line).
    bad = re.compile(r"status failed|\bFAIL(ED)?\b|failed:\s*[1-9]|error:\s*[1-9]|[1-9]\d* (failed|errored)")
    good = re.compile(r"\b(OK|passed|PASS)\b|successfully")
    for ln in lines:
        style = "bold red" if bad.search(ln) else ("green" if good.search(ln) else "")
        console.print(Text(ln, style=style), soft_wrap=True)

    with tempfile.TemporaryDirectory() as tmp:
        svg = pathlib.Path(tmp) / "out.svg"
        console.save_svg(str(svg), title=a.title)
        a.png.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["rsvg-convert", "-w", "1400", "-b", "#292929", "-o", str(a.png), str(svg)],
                       check=True)
    print(f"wrote {a.png} ({len(lines)} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
