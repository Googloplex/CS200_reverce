#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
regs_swd.py — CLONE (fw/) register table via SWD without halting the core (attach), using ELF symbols.
Counterpart of tools/spin_native.py for the stock firmware. No bus/UART adapter needed.

  py -3.11 tools/regs_swd.py show                 # whole table 0x00..0x7F, decoded
  py -3.11 tools/regs_swd.py watch                # Present Position/Load/Voltage every 100 ms
  py -3.11 tools/regs_swd.py goal 1500 [TL]       # Goal Position (µs) + Torque Limit, enables torque
  py -3.11 tools/regs_swd.py torque 0             # disable torque
  py -3.11 tools/regs_swd.py set 0x1A 6           # write a byte
"""
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_one import parse_record  # noqa: E402

from pyocd.core.helpers import ConnectHelper
from pyocd.debug.elf.elf import ELFBinaryFile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, "fw", "build", "hb200t.elf")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    cmd = sys.argv[1]
    with ConnectHelper.session_with_chosen_probe(target_override="cortex_m",
                                                 options={"connect_mode": "attach"}) as s:
        t = s.target
        elf = ELFBinaryFile(ELF, t.memory_map)
        sym = lambda n: elf.symbol_decoder.get_symbol_for_name(n).address
        regs = sym("regs")

        def rd():
            return bytes(t.read_memory_block8(regs, 128))

        if cmd == "show":
            r = parse_record(rd())
            for k, v in r.items():
                print(f"  {k:20s} {v}")
        elif cmd == "watch":
            try:
                while True:
                    d = rd()
                    pos, load = struct.unpack_from("<hH", d, 0x24)[0], struct.unpack_from("<H", d, 0x28)[0]
                    print(f"\rpos={pos:5d} load={load:5d} V={d[0x2A] / 10:4.1f} TE={d[0x18]} "
                          f"goal={struct.unpack_from('<H', d, 0x1E)[0]:4d} TL={struct.unpack_from('<H', d, 0x22)[0]:4d}   ",
                          end="")
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print()
        elif cmd == "goal":
            us = int(sys.argv[2], 0)
            tl = int(sys.argv[3], 0) if len(sys.argv) > 3 else 300
            t.write16(regs + 0x22, tl)
            t.write16(regs + 0x1E, us)
            t.write8(regs + 0x18, 1)
            print("goal", us, "TL", tl, "torque on")
        elif cmd == "torque":
            t.write8(regs + 0x18, int(sys.argv[2], 0))
            print("torque enable =", int(sys.argv[2], 0))
        elif cmd == "set":
            a, v = int(sys.argv[2], 0), int(sys.argv[3], 0)
            t.write8(regs + a, v)
            print(f"regs[0x{a:02X}] = {v}")
        else:
            sys.exit(__doc__)


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
