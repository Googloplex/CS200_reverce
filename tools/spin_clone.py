#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spin_clone.py — short motor run on the CLONE (fw/) via SWD (attach, ELF symbols), counterpart of spin_native.py.
Enables torque with the given Torque Limit and Goal, prints Present Load/Voltage; when time is up
(or on any link error) disables torque; if the link is lost, reconnects until torque is off.

  py -3.11 tools/spin_clone.py [seconds=2] [torque_limit=150] [goal_us=2100]

servo_01 has no potentiometer: the loop never settles, the motor spins for the whole run. PSU limit ≥ 2 A
(inrush > 2× steady-state current — see docs/motor.md), no load on the output.
"""
import os
import struct
import sys
import time

from pyocd.core.helpers import ConnectHelper
from pyocd.debug.elf.elf import ELFBinaryFile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, "fw", "build", "hb200t.elf")
REG_TORQUE_ENABLE, REG_GOAL, REG_TORQUE_LIMIT = 0x18, 0x1E, 0x22


def session():
    return ConnectHelper.session_with_chosen_probe(target_override="cortex_m", options={"connect_mode": "attach"})


def regs_addr(t):
    return ELFBinaryFile(ELF, t.memory_map).symbol_decoder.get_symbol_for_name("regs").address


def status(t, regs):
    d = bytes(t.read_memory_block8(regs + 0x18, 0x14))
    pos, = struct.unpack_from("<h", d, 0x24 - 0x18)
    load, = struct.unpack_from("<H", d, 0x28 - 0x18)
    return (f"pos={pos:5d} load={load:5d} V={d[0x2A - 0x18] / 10:4.1f} TE={d[0]} "
            f"goal={struct.unpack_from('<H', d, 0x1E - 0x18)[0]} TL={struct.unpack_from('<H', d, 0x22 - 0x18)[0]}")


def torque_off(t, regs):
    t.write16(regs + REG_TORQUE_LIMIT, 0)
    t.write8(regs + REG_TORQUE_ENABLE, 0)


def emergency_off(timeout=15.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            with session() as s:
                r = regs_addr(s.target)
                torque_off(s.target, r)
                time.sleep(0.2)
                print("after (reconnected):", status(s.target, r))
                return
        except Exception as e:
            print("  reconnect failed:", e)
            time.sleep(0.5)
    print("!!! TORQUE NOT DISABLED — CUT POWER !!!")


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 150
    goal = int(sys.argv[3]) if len(sys.argv) > 3 else 2100
    try:
        with session() as s:
            t = s.target
            r = regs_addr(t)
            print("before:", status(t, r))
            try:
                # while torque is off, firmware zeroes Goal every tick — so Torque Enable goes first
                t.write16(r + REG_TORQUE_LIMIT, limit)
                t.write8(r + REG_TORQUE_ENABLE, 1)
                t.write16(r + REG_GOAL, goal)
                end = time.time() + seconds
                while time.time() < end:
                    print("  run:", status(t, r))
                    time.sleep(0.25)
            finally:
                torque_off(t, r)
                time.sleep(0.2)
                print("after:", status(t, r))
    except Exception as e:
        print("!!! link error:", e)
        emergency_off()


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
