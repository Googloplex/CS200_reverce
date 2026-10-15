#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rm_pages.py — print Reference Manual pages (Chinese text, extracted with PyMuPDF).
  py -3.11 tools/rm_pages.py 104-106 195 217-220
Text: docs/datasheets/LCM32F037_RM_V7.11.txt (regenerate: py -3.11 tools/rm_pages.py --extract)
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PDF = os.path.join(ROOT, "docs", "datasheets", "LCM32F037_RM_V7.11.pdf")
TXT = os.path.join(ROOT, "docs", "datasheets", "LCM32F037_RM_V7.11.txt")


def extract():
    import pymupdf
    doc = pymupdf.open(PDF)
    with open(TXT, "w", encoding="utf-8") as f:
        for i, page in enumerate(doc):
            f.write(f"\n=== PAGE {i + 1} ===\n")
            f.write(page.get_text())
    print("pages:", doc.page_count)


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if "--extract" in sys.argv:
        extract()
        return
    txt = open(TXT, encoding="utf-8").read()
    parts = re.split(r"\n=== PAGE (\d+) ===\n", txt)
    pages = {int(parts[i]): parts[i + 1] for i in range(1, len(parts), 2)}
    for arg in sys.argv[1:]:
        a, _, b = arg.partition("-")
        for p in range(int(a), int(b or a) + 1):
            print(f"----- page {p} -----")
            print("\n".join(l for l in pages[p].splitlines() if l.strip() and l.strip() != "杭州领芯微电子有限公司"))


if __name__ == "__main__":
    main()
