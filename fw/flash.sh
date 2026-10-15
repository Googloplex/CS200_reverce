#!/usr/bin/env bash
# Flashes the APPLICATION (0x08001000..) via pyOCD with the Keil pack algorithm. Bootloader 0x08000000–0x08000FFF
# and journal 0x08007C00–0x08007FFF are untouched (only sectors under the image are erased).
# Requires: py -3.11 with pyocd, ST-Link, servo powered from bench PSU. Dump before first flashing (tools/dump_one.py)!
set -e
cd "$(dirname "$0")"
PACK="../docs/datasheets/LCM32F0xx_DFP_0.4.72/zip/Keil.LCM32F0xx_DFP.0.4.72.pack"
BIN="${1:-build/hb200t.bin}"
py -3.11 -m pyocd flash --pack "$PACK" -t lcm32f037 -O connect_mode=under-reset \
    --format bin --base-address 0x08001000 --erase sector "$BIN"
