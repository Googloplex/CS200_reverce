#!/usr/bin/env bash
# Rollback to stock Kpower application: region 0x08001000..0x08007BFF (application + free space) is taken from the dump
# and flashed back. Bootloader (0x08000000..0x08000FFF) and journal (0x08007C00..) are untouched.
# Usage: ./restore_native.sh [dumps/servo_01/flash.bin]
set -e
cd "$(dirname "$0")"
DUMP="${1:-../dumps/servo_01/flash.bin}"
PACK="../docs/datasheets/LCM32F0xx_DFP_0.4.72/zip/Keil.LCM32F0xx_DFP.0.4.72.pack"
mkdir -p build
py -3.11 -I -c "import sys; d=open(sys.argv[1],'rb').read(); open(sys.argv[2],'wb').write(d[0x1000:0x7C00]); print('native app image:', len(d[0x1000:0x7C00]), 'bytes')" "$DUMP" build/native_app.bin
py -3.11 -m pyocd flash --pack "$PACK" -t lcm32f037 -O connect_mode=under-reset \
    --format bin --base-address 0x08001000 --erase sector build/native_app.bin
