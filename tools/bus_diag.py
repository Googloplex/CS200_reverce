#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bus_diag.py — диагностика шины на клоне: что видит MCU (по SWD) и что видит адаптер (pyserial).

  py -3.11 tools/bus_diag.py -p COM6 [-b 117647]

Шаги: состояние UART1/DMA/PA2 → эхо-тест адаптера → PING по шине → снова состояние, содержимое кольца приёма,
состояние автомата, флаг передачи и буфер ответа.
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
                  f"PA2 AF={(t.read32(0x48000024) >> 8) & 0xF} уровень={(t.read32(0x48000014) >> 2) & 1}")
            for n in ("rx_state", "rx_idx", "rx_cur", "rx_start", "rx_len", "g_tx_busy", "tx_n", "g_err"):
                ad = sym(n)
                if ad is not None:
                    print(f"      {n}={t.read8(ad) if n.startswith('g_') or n == 'tx_n' else t.read16(ad)}", end="")
            print()

        state("до")
        ser = serial.Serial(a.port, a.baud, timeout=0.05)
        ser.reset_input_buffer()
        ser.write(b"\x55\xAA\x55"); ser.flush()
        echo = ser.read(16)
        print("эхо адаптера на 55 AA 55:", echo.hex(" ") or "нет (TX и RX не соединены или RX не на S)")
        ser.reset_input_buffer()
        ser.write(PING); ser.flush()
        time.sleep(0.05)
        rx = ser.read(64)
        print("после PING принято адаптером:", rx.hex(" ") or "ничего")
        state("после PING")
        ring = sym("rx_ring")
        if ring:
            data = bytes(t.read_memory_block8(ring, 255))
            nz = data.rstrip(b"\x00")
            print("кольцо приёма MCU (первые байты):", nz[:48].hex(" ") or "пусто")
        txb = sym("txb")
        if txb:
            print("буфер ответа MCU:", bytes(t.read_memory_block8(txb, 16)).hex(" "))
        ser.close()


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
