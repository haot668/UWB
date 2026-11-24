#!/usr/bin/env python3
"""Check that every <FilePath> in the three Keil projects exists on disk.

A Keil project that references a deleted source file fails to build with a
confusing error, and the reference is easy to miss by eye.  Run this after
touching any .uvprojx:

    python scripts/check_project_paths.py
"""
import os
import xml.etree.ElementTree as ET

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECTS = ("uwb_anchor", "uwb_tag", "uwb_gateway")

rc = 0
for proj in PROJECTS:
    files = [f for f in os.listdir(os.path.join(BASE, proj)) if f.endswith(".uvprojx")]
    if not files:
        print(f"{proj}: NO .uvprojx FOUND")
        rc = 1
        continue
    path = os.path.join(BASE, proj, files[0])
    root = ET.parse(path).getroot()
    missing = []
    n = 0
    for fp in root.iter("FilePath"):
        n += 1
        rel = fp.text.replace("\\", "/")
        if not os.path.exists(os.path.join(BASE, proj, rel)):
            missing.append(rel)
    if missing:
        rc = 1
        print(f"{proj}: {n} entries, MISSING {sorted(set(missing))}")
    else:
        print(f"{proj}: {n} entries, all present")

raise SystemExit(rc)
