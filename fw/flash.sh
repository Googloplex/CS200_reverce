#!/usr/bin/env bash
# Прошивка ПРИЛОЖЕНИЯ (0x08001000..) через pyOCD с алгоритмом из Keil-пака. Загрузчик 0x08000000–0x08000FFF
# и журнал 0x08007C00–0x08007FFF не трогаются (стираются только сектора под образ).
# Требования: py -3.11 с pyocd, ST-Link, серва запитана от ЛБП. Перед первой прошивкой — дамп (tools/dump_one.py)!
set -e
cd "$(dirname "$0")"
PACK="../docs/datasheets/LCM32F0xx_DFP_0.4.72/zip/Keil.LCM32F0xx_DFP.0.4.72.pack"
BIN="${1:-build/hb200t.bin}"
py -3.11 -m pyocd flash --pack "$PACK" -t lcm32f037 -O connect_mode=under-reset \
    --format bin --base-address 0x08001000 --erase sector "$BIN"
