# Dumps

`servo_01/` — the first servo opened. Binary dumps of the stock firmware and the chip info area are not included in the public repository
(third-party code); kept are the checksums (`sha256.txt`), the decoded settings journal (`journal.json`), notes and bench logs
(`tests/*.csv`, format — `tools/pos_test.py`). To take your own dump: `py -3.11 tools/dump_one.py` (read-only).
