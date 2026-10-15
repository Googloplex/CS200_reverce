#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_clone.py — проверка после прошивки клона (fw/build/hb200t.bin), без остановки ядра (attach):
  1) область приложения во Flash == образ; загрузчик и журнал == дамп dumps/servo_01/flash.bin;
  2) ядро живо: g_ms растёт, SysTick/NVIC включены;
  3) периферия настроена: TIM1 (ШИМ, MOE, CCER), TIM14/15/16, UART1 (IBRD/FBRD), DMAC, ADC;
  4) таблица регистров и АЦП.

  py -3.11 tools/verify_clone.py [dumps/servo_01/flash.bin]
"""
import hashlib
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
BIN = os.path.join(ROOT, "fw", "build", "hb200t.bin")
APP = 0x08001000
JOURNAL = 0x08007C00


def r32(t, a):
    return t.read32(a)


def main():
    dump_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "dumps", "servo_01", "flash.bin")
    dump = open(dump_path, "rb").read()
    image = open(BIN, "rb").read()
    ok = True
    with ConnectHelper.session_with_chosen_probe(target_override="cortex_m",
                                                 options={"connect_mode": "attach"}) as s:
        t = s.target
        elf = ELFBinaryFile(ELF, t.memory_map)
        sym = lambda n: elf.symbol_decoder.get_symbol_for_name(n).address

        # --- 1. Flash ---
        app = bytes(t.read_memory_block8(APP, len(image)))
        bl = bytes(t.read_memory_block8(0x08000000, 0x1000))
        jr = bytes(t.read_memory_block8(JOURNAL, 0x400))
        print("приложение во Flash == образ:", app == image)
        print("загрузчик == дамп:", bl == dump[:0x1000])
        print("журнал == дамп:", jr == dump[0x7C00:0x8000], "(sha", hashlib.sha256(jr).hexdigest()[:8] + ")")
        ok &= app == image and bl == dump[:0x1000]

        # --- 2. ядро ---
        ms1 = r32(t, sym("g_ms")); time.sleep(0.2); ms2 = r32(t, sym("g_ms"))
        print(f"g_ms: {ms1} → {ms2} за 200 мс ({'ЖИВО' if 150 <= ms2 - ms1 <= 260 else 'НЕ ТИКАЕТ'})")
        ok &= 150 <= ms2 - ms1 <= 260
        print(f"SysTick CTRL=0x{r32(t, 0xE000E010):X} LOAD={r32(t, 0xE000E014)}  NVIC ISER=0x{r32(t, 0xE000E100):08X} (ожидается 0x001A0480)")
        print(f"CHIPCTRL CLK_CFG=0x{r32(t, 0x40017000):08X} PLL_CFG=0x{r32(t, 0x4001700C):X} STS=0x{r32(t, 0x40017018):08X}  FLASH_ACR=0x{r32(t, 0x4001D000):X}")

        # --- 3. периферия ---
        T1 = 0x40010000
        print(f"TIM1: CR1=0x{r32(t, T1):X} CCER=0x{r32(t, T1 + 0x20):X} BDTR=0x{r32(t, T1 + 0x44):X} PSC={r32(t, T1 + 0x28)} ARR={r32(t, T1 + 0x2C)} "
              f"CCR=({r32(t, T1 + 0x34)},{r32(t, T1 + 0x38)},{r32(t, T1 + 0x3C)}) CNT={r32(t, T1 + 0x24)}")
        print(f"TIM14: CR1=0x{r32(t, 0x4000A000):X} ARR={r32(t, 0x4000A02C)} CNT={r32(t, 0x4000A024)}   "
              f"TIM15: CR1=0x{r32(t, 0x4001B000):X} CCMR1=0x{r32(t, 0x4001B018):X} CCER=0x{r32(t, 0x4001B020):X} SMCR=0x{r32(t, 0x4001B008):X}   "
              f"TIM16: CR1=0x{r32(t, 0x4001C000):X} CCER=0x{r32(t, 0x4001C020):X} CCR1={r32(t, 0x4001C034)} BDTR=0x{r32(t, 0x4001C044):X}")
        U = 0x40015000
        print(f"UART1: CR=0x{r32(t, U + 0x30):X} LCR_H=0x{r32(t, U + 0x2C):X} IBRD={r32(t, U + 0x24)} FBRD={r32(t, U + 0x28)} DMACR=0x{r32(t, U + 0x48):X} FR=0x{r32(t, U + 0x18):X}")
        print(f"GPIOA: MODER=0x{r32(t, 0x48000000):08X} AFRL=0x{r32(t, 0x48000024):08X} AFRH=0x{r32(t, 0x48000028):08X} IDR=0x{r32(t, 0x48000014):04X} (Холлы PA4..6 = {(r32(t, 0x48000014) >> 4) & 7:03b})")
        print(f"GPIOB: MODER=0x{r32(t, 0x48000200):08X} AFRL=0x{r32(t, 0x48000224):08X}   GPIOF: MODER=0x{r32(t, 0x48000A00):08X} AFRL=0x{r32(t, 0x48000A24):08X}")
        D = 0x48004000
        print(f"DMAC: CHEN=0x{r32(t, D + 0x3A0):X} CFG=0x{r32(t, D + 0x398):X} ch0 CTL_H={r32(t, D + 0x1C) & 0xFFF} ch2 CTL_L=0x{r32(t, D + 0xB0 + 0x18):X} CTL_H={r32(t, D + 0xB0 + 0x1C) & 0xFFF}   "
              f"EDU_CFG0=0x{r32(t, 0x48007020):08X}")
        A = 0x4001A000
        print(f"ADC: ISR=0x{r32(t, A):X} CR=0x{r32(t, A + 8):X} CFGR1=0x{r32(t, A + 0xC):X} CFGR2=0x{r32(t, A + 0x10):X} CHSELR=0x{r32(t, A + 0x1C):X} SMPR=0x{r32(t, A + 0x14):X}")

        # --- 4. данные ---
        adc = struct.unpack("<4H", bytes(t.read_memory_block8(sym("adc_buf"), 8)))
        print(f"adc_buf: PA1={adc[0]} Vbus={adc[1]} ток={adc[2]} пот={adc[3]}   present_pos={struct.unpack('<h', bytes(t.read_memory_block8(sym('g_present_pos'), 2)))[0]}")
        rec = parse_record(bytes(t.read_memory_block8(sym("regs"), 128)))
        keys = ["seq", "model", "fw_ver", "id", "baud", "torque_enable", "goal_position", "present_position", "present_load",
                "present_voltage", "torque_limit", "pwm_min_us", "pwm_center_us", "pwm_max_us", "travel_deg", "reg_46", "reg_4c", "reg_4e", "reg_5a"]
        print("regs:", ", ".join(f"{k}={rec[k]}" for k in keys))
    print("ИТОГ:", "OK" if ok else "ЕСТЬ ПРОБЛЕМЫ")


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
