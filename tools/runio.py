#!/usr/bin/env python3
"""
runio.py — tolerant reader for flight_log.csv.

The logger is killed when a run ends, so the final row can be a partial write
containing NUL bytes. Python's csv module raises "line contains NUL" and the
whole analysis dies over one truncated line at the end of a good run. Strip the
damage and carry on, reporting what was dropped rather than hiding it.
"""

import csv
import io


def read_rows(path, verbose=False):
    with open(path, "rb") as f:
        raw = f.read()

    dropped = 0
    if b"\x00" in raw:
        keep = []
        for line in raw.split(b"\n"):
            if b"\x00" in line:
                dropped += 1
                continue
            keep.append(line)
        raw = b"\n".join(keep)

    text = raw.decode("utf-8", errors="replace")
    rows = list(csv.DictReader(io.StringIO(text)))

    # A partial final line can also parse but be short on fields.
    if rows and any(v is None for v in rows[-1].values()):
        rows.pop()
        dropped += 1

    if dropped and verbose:
        print(f"  (dropped {dropped} truncated row(s) at end of log)")
    return rows


def f(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default
