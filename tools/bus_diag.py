#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bus_diag.py — bus diagnostics on the clone: what the MCU sees (via SWD) and what the adapter sees (pyserial).

  py -3.11 tools/bus_diag.py -p COM6 [-b 117647]

Steps: UART1/DMA/PA2 state → adapter echo test → PING over the bus → state again, RX ring contents,
state machine state, TX flag and reply buffer.
"""
import argparse
import sys
import time

import serial
from pyocd.core.helpers import ConnectHelper
from pyocd.debug.elf.elf import ELFBinaryFile

import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, "fw", "build", "hb200t.elf")
PING = bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("-b", "--baud", type=int, default=117647)
    a = ap.parse_args()

    with ConnectHelper.session_with_chosen_probe(target_override="cortex_m", options={"connect_mode": "attach"}) as s:
        t = s.target
        dec = ELFBinaryFile(ELF, t.memory_map).symbol_decoder

        def sym(n):
            x = dec.get_symbol_for_name(n)
            return x.address if x else None

        def state(tag):
            U = 0x40015000
            print(f"[{tag}] UART1 FR=0x{t.read32(U + 0x18):X} RSR=0x{t.read32(U + 0x04):X} CR=0x{t.read32(U + 0x30):X} IBRD={t.read32(U + 0x24)} FBRD={t.read32(U + 0x28)} | "
                  f"DMA ch0 CTL_H={t.read32(0x48004000 + 0x1C) & 0xFFF} CHEN=0x{t.read32(0x480043A0):X} | "
                  f"PA2 AF={(t.read32(0x48000024) >> 8) & 0xF} level={(t.read32(0x48000014) >> 2) & 1}")
            for n in ("rx_state", "rx_idx", "rx_cur", "rx_start", "rx_len", "g_tx_busy", "tx_n", "g_err"):
                ad = sym(n)
                if ad is not None:
                    print(f"      {n}={t.read8(ad) if n.startswith('g_') or n == 'tx_n' else t.read16(ad)}", end="")
            print()

        state("before")
        ser = serial.Serial(a.port, a.baud, timeout=0.05)
        ser.reset_input_buffer()
        ser.write(b"\x55\xAA\x55"); ser.flush()
        echo = ser.read(16)
        print("adapter echo of 55 AA 55:", echo.hex(" ") or "none (TX and RX not joined, or RX not on S)")
        ser.reset_input_buffer()
        ser.write(PING); ser.flush()
        time.sleep(0.05)
        rx = ser.read(64)
        print("received by adapter after PING:", rx.hex(" ") or "nothing")
        state("after PING")
        ring = sym("rx_ring")
        if ring:
            data = bytes(t.read_memory_block8(ring, 255))
            nz = data.rstrip(b"\x00")
            print("MCU RX ring (first bytes):", nz[:48].hex(" ") or "empty")
        txb = sym("txb")
        if txb:
            print("MCU reply buffer:", bytes(t.read_memory_block8(txb, 16)).hex(" "))
        ser.close()


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
