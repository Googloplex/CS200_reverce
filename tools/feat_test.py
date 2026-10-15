#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
feat_test.py — проверка фич v1.5 через SWD на servo_01: кубическая интерполяция цели по метке времени (mode 2),
склейка интерполяций, мягкие упоры, детект упора, политика отказа (удержание / свободный ход).

  py -3.11 tools/feat_test.py
"""
import os
import re
import struct
import sys
import time

from pyocd.core.exceptions import ProbeError, TransferError
from pyocd.core.helpers import ConnectHelper
from pyocd.debug.elf.elf import ELFBinaryFile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, "fw", "build", "hb200t.elf")
A, B = 2048, 1231

R2 = dict(SOFT_MIN=0x24, SOFT_MAX=0x26, SOFT_KP=0x28, FAULT_MODE=0x29, STALL_DUTY=0x2A, STALL_MS=0x2C, STALL_ACT=0x2D,
          STATUS2=0x2E, HOLD_TMAX=0x30)   # смещения от 0x80


def tim1_ccer_addr():
    h = open(os.path.join(ROOT, "fw", "src", "lcm32.h"), encoding="utf-8").read()
    base = int(re.search(r"#define TIM1_B\s+(0x[0-9A-Fa-f]+)", h).group(1), 16)
    off = int(re.search(r"#define TIM_CCER\(\w\)\s+REG32\(\w,\s*(0x[0-9A-Fa-f]+)\)", h).group(1), 16)
    return base + off


def main():
    with ConnectHelper.session_with_chosen_probe(target_override="cortex_m", options={"connect_mode": "attach"}) as s:
        t = s.target
        dec = ELFBinaryFile(ELF, t.memory_map).symbol_decoder
        sym = lambda n: dec.get_symbol_for_name(n).address
        regs, regs2 = sym("regs"), sym("regs2")
        g_ms, last_rx, toff, g_status = sym("g_ms"), sym("g_last_rx_ms"), sym("g_time_offset"), sym("g_status")
        ccer = tim1_ccer_addr()

        def swd(fn, *a):
            for i in range(6):
                try:
                    return fn(*a)
                except (TransferError, ProbeError) as e:
                    print(f"   SWD: {e} — переподключение {i + 1}")
                    time.sleep(0.3)
                    try:
                        t.dp.disconnect(); t.dp.connect()
                    except Exception as e2:
                        print("   ", e2)
            raise RuntimeError("SWD не восстановился")

        def hb():
            swd(lambda: t.write32(last_rx, t.read32(g_ms)))

        def rd():
            d = bytes(swd(t.read_memory_block8, regs, 0x80)); d2 = bytes(swd(t.read_memory_block8, regs2, 0x80))
            return dict(pos=struct.unpack_from("<h", d, 0x24)[0], te=d[0x18], load=struct.unpack_from("<H", d, 0x28)[0],
                        duty=struct.unpack_from("<H", d, 0x74)[0], vel=struct.unpack_from("<h", d2, 0x12)[0],
                        ibus=struct.unpack_from("<H", d2, 0x14)[0], status=d2[0x17], st2=d2[R2["STATUS2"]],
                        vbus=struct.unpack_from("<H", d2, 0x18)[0], tlm_t=struct.unpack_from("<I", d2, 0x1C)[0])

        def w8(off, v):
            try:
                t.write8(regs2 + off, v)
            except Exception:
                pass

        def w16(off, v):
            swd(t.write16, regs2 + off, v)

        seq = [0]

        def cmd(pos, vel=0, kp=310, kd=60, ff=0, tmax=600, mode=1, at=0):
            seq[0] = (seq[0] + 1) & 0xFF
            swd(t.write_memory_block8, regs2, struct.pack("<hhHHhHBHB", pos, vel, kp, kd, ff, tmax, mode, at, seq[0]))

        def now16():
            return (swd(t.read32, g_ms) + swd(t.read32, toff)) & 0xFFFF

        def watch(sec, label, every=0.02, beat=True):
            t0 = time.time(); rows = []; hot = 0
            while time.time() - t0 < sec:
                if beat: hb()
                r = rd(); rows.append((time.time() - t0, r))
                hot = hot + 1 if r["load"] > 6000 else 0
                if r["pos"] < 150 or r["pos"] > 3950 or hot >= 2:
                    cmd(r["pos"], mode=0); t.write8(regs + 0x18, 0)
                    raise RuntimeError(f"{label}: стоп, pos={r['pos']} load={r['load']}")
                time.sleep(every)
            p = [r["pos"] for _, r in rows]
            print(f"{label:44s} pos {p[0]:5d} → {p[-1]:5d}  vel пик {max(abs(r['vel']) for _, r in rows):5d}  "
                  f"I пик {max(r['ibus'] for _, r in rows):5d} мА  st2 {' '.join('%02X' % r['st2'] for _, r in rows[::max(1, len(rows) // 8)])}")
            return rows

        def at_t(rows, tt):
            return min(rows, key=lambda x: abs(x[0] - tt))[1]["pos"]

        tmo_saved = t.read16(regs + 0x54)
        hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000); t.write16(regs + 0x50, 8000)
        ok = []
        try:
            r0 = rd(); print("старт:", r0)
            cmd(A); watch(0.6, "0. → A")

            # 1. кубическая интерполяция за 600 мс
            cmd(B, mode=2, at=(now16() + 600) & 0xFFFF)
            rows = watch(0.9, "1. mode 2: A → B за 600 мс")
            p1, p3, p5, p6 = at_t(rows, 0.1), at_t(rows, 0.3), at_t(rows, 0.5), at_t(rows, 0.65)
            mid = (A + B) // 2
            print(f"   pos@0.1 {p1}  @0.3 {p3} (середина {mid})  @0.5 {p5}  @0.65 {p6}")
            good = abs(p3 - mid) < 80 and abs(p6 - B) < 30 and p1 > A - 200
            ok.append(("интерполяция: S-кривая, приход к сроку", good))

            # 2. склейка: через 250 мс новая цель A за 500 мс — без рывка
            cmd(B, mode=2, at=(now16() + 600) & 0xFFFF)
            cmd(A, mode=2, at=(now16() + 600) & 0xFFFF)   # перезапись в начале — интерполяция от текущей цели
            cmd(B, mode=2, at=(now16() + 600) & 0xFFFF)
            time.sleep(0.25)
            cmd(A, mode=2, at=(now16() + 500) & 0xFFFF)
            rows = watch(0.8, "2. склейка B→A на ходу")
            vmax = max(abs(r["vel"]) for _, r in rows)
            ok.append(("склейка без рывка (vel пик < 7000, пришла в A)", vmax < 7000 and abs(rows[-1][1]["pos"] - A) < 30))

            # 3. мягкие упоры: цель режется, пружина за упором
            w16(R2["SOFT_MIN"], 1600); w16(R2["SOFT_MAX"], 2500)
            cmd(B); rows = watch(0.6, "3a. упор 1600: цель B=1231")
            end = rows[-1][1]["pos"]
            ok.append(("цель обрезана упором (pos ≈ 1600)", abs(end - 1600) < 40))
            w16(R2["SOFT_MIN"], end + 150)                  # теперь мы за упором → пружина выталкивает
            rows = watch(0.5, "3b. упор поднят на +150 (пружина)")
            seen_soft = any(r["st2"] & 0x02 for _, r in rows)
            ok.append(("пружина за упором: флаг и выход к упору", seen_soft and rows[-1][1]["pos"] >= end + 150 - 40))
            w16(R2["SOFT_MIN"], 0); w16(R2["SOFT_MAX"], 0)
            cmd(A); watch(0.5, "3c. упоры сняты → A")

            # 4. детект упора: потолок тока 0.4 А → ротор не едет, контур требует полный duty
            t.write16(regs + 0x50, 400); w16(R2["STALL_DUTY"], 300); w8(R2["STALL_MS"], 50); w8(R2["STALL_ACT"], 0)
            cmd(B); rows = watch(0.4, "4a. упор (ток ≤ 0.4 А), действие 0")
            ok.append(("флаг упора при стоящем роторе", bool(rows[-1][1]["st2"] & 0x01) and abs(rows[-1][1]["pos"] - A) < 60))
            w8(R2["STALL_ACT"], 2); w8(R2["FAULT_MODE"], 0)
            cmd(A); cmd(B); rows = watch(0.4, "4b. действие 2 (отказ → момент выкл)")
            ok.append(("упор → момент выключен", rows[-1][1]["te"] == 0))
            w16(R2["STALL_DUTY"], 0); w8(R2["STALL_ACT"], 0); t.write16(regs + 0x50, 8000)
            cmd(A); watch(0.5, "4c. снова → A")

            # 5. политика отказа 2: таймаут шины → удержание в точке
            w8(R2["FAULT_MODE"], 2); t.write16(regs + 0x54, 200)
            cmd(B); watch(0.4, "5a. → B")
            rows = watch(0.5, "5b. без сердцебиения 500 мс", beat=False)
            r = rows[-1][1]
            ok.append(("таймаут шины → удержание (TE=1, флаг hold, стоит)", r["te"] == 1 and bool(r["st2"] & 0x04) and abs(r["pos"] - B) < 40))
            t.write8(g_status, 0); hb()
            cmd(A); rows = watch(0.5, "5c. команда снимает удержание → A")
            ok.append(("после удержания едет", abs(rows[-1][1]["pos"] - A) < 30))
            t.write16(regs + 0x54, 500)

            # 6. политика 1: свободный ход при выключенном моменте
            w8(R2["FAULT_MODE"], 1); t.write8(regs + 0x18, 0); time.sleep(0.05)
            r = rd(); c = t.read32(ccer)
            print(f"6a. момент выкл, свободный ход: st2 0x{r['st2']:02X} CCER 0x{c:04X}")
            ok.append(("свободный ход: флаг и все выходы TIM1 сняты", bool(r["st2"] & 0x10) and (c & 0x555) == 0))
            cmd(B); rows = watch(0.5, "6b. из свободного хода → B")
            c = t.read32(ccer)
            ok.append(("коммутация восстановлена, пришла в B", abs(rows[-1][1]["pos"] - B) < 30 and (c & 0x444) == 0x444))
            w8(R2["FAULT_MODE"], 0)
            cmd(A); watch(0.5, "6c. → A")
        finally:
            cmd(rd()["pos"], mode=0); t.write8(regs + 0x18, 0); t.write16(regs + 0x54, tmo_saved)
            w8(R2["FAULT_MODE"], 0); w16(R2["SOFT_MIN"], 0); w16(R2["SOFT_MAX"], 0); w16(R2["STALL_DUTY"], 0)
            print("момент выключен:", rd())
            print("\nИТОГ:")
            for name, g in ok:
                print(f"  [{'OK' if g else '!!'}] {name}")


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
