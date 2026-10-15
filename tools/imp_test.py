#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imp_test.py — impedance command check (regs2 0x80..0x8F) via SWD: hold, steps with different Kd and tmax,
first-order hold (target moves at vel for 100 ms), deferred application by host time, direct torque ff.

  py -3.11 tools/imp_test.py [--tmax-final 1000]
"""
import os
import struct
import sys
import time

from pyocd.core.exceptions import ProbeError, TransferError
from pyocd.core.helpers import ConnectHelper
from pyocd.debug.elf.elf import ELFBinaryFile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, "fw", "build", "hb200t.elf")
A, B = 2048, 1231   # two travel points


def main():
    tmax_final = 1000
    if "--tmax-final" in sys.argv:
        tmax_final = int(sys.argv[sys.argv.index("--tmax-final") + 1])
    with ConnectHelper.session_with_chosen_probe(target_override="cortex_m", options={"connect_mode": "attach"}) as s:
        t = s.target
        dec = ELFBinaryFile(ELF, t.memory_map).symbol_decoder
        sym = lambda n: dec.get_symbol_for_name(n).address
        regs, regs2 = sym("regs"), sym("regs2")
        g_ms, last_rx, toff = sym("g_ms"), sym("g_last_rx_ms"), sym("g_time_offset")

        def swd(fn, *a):   # SWD link drops under full torque — reconnect without failing the test
            for i in range(6):
                try:
                    return fn(*a)
                except (TransferError, ProbeError) as e:
                    print(f"   SWD: {e} — reconnect {i + 1}")
                    time.sleep(0.3)
                    try:
                        t.dp.disconnect(); t.dp.connect()
                    except Exception as e2:
                        print("   ", e2)
            raise RuntimeError("SWD did not recover")

        def hb():
            swd(lambda: t.write32(last_rx, t.read32(g_ms)))

        def rd():
            d = bytes(swd(t.read_memory_block8, regs, 0x80)); d2 = bytes(swd(t.read_memory_block8, regs2, 0x40))
            return dict(pos=struct.unpack_from("<h", d, 0x24)[0], te=d[0x18], load=struct.unpack_from("<H", d, 0x28)[0],
                        duty=struct.unpack_from("<H", d, 0x74)[0], vel=struct.unpack_from("<h", d2, 0x12)[0],
                        ibus=struct.unpack_from("<H", d2, 0x14)[0], status=d2[0x17], vbus=struct.unpack_from("<H", d2, 0x18)[0],
                        tlm_t=struct.unpack_from("<I", d2, 0x1C)[0])

        seq = [0]

        def cmd(pos, vel=0, kp=310, kd=60, ff=0, tmax=600, mode=1, at=0):
            seq[0] = (seq[0] + 1) & 0xFF
            swd(t.write_memory_block8, regs2, struct.pack("<hhHHhHBHB", pos, vel, kp, kd, ff, tmax, mode, at, seq[0]))

        def watch(sec, label, target=None, every=0.02):
            t0 = time.time(); rows = []; hot = 0
            while time.time() - t0 < sec:
                hb(); r = rd(); rows.append((time.time() - t0, r))
                hot = hot + 1 if r["load"] > 6000 else 0
                if r["pos"] < 150 or r["pos"] > 3950 or hot >= 2:
                    cmd(r["pos"], mode=0); t.write8(regs + 0x18, 0)
                    for dt, x in rows[-8:]:
                        print(f"   {dt:5.2f} pos {x['pos']:5d} vel {x['vel']:6d} duty {x['duty']:4d} I {x['ibus']:5d} mA load {x['load']:5d} Vbus {x['vbus']} mV")
                    raise RuntimeError(f"{label}: stop, pos={r['pos']} load={r['load']}")
                time.sleep(every)
            p = [r["pos"] for _, r in rows]
            line = (f"{label:40s} pos {p[0]:5d} → {p[-1]:5d}  vel peak {max(abs(r['vel']) for _, r in rows):5d} units/s  "
                    f"I_bus peak {max(r['ibus'] for _, r in rows):5d} mA  duty peak {max(r['duty'] for _, r in rows):4d}  "
                    f"Vbus min {min(r['vbus'] for _, r in rows)} mV  st 0x{rows[-1][1]['status']:02X}")
            m = {}
            if target is not None:
                sgn = 1 if target >= p[0] else -1
                m["over"] = max(0, max(sgn * (x - target) for x in p))
                m["settle"] = next((dt for dt, r in rows if all(abs(x["pos"] - target) <= 15 for d2, x in rows if d2 >= dt)), None)
                line += f"  overshoot {m['over']:3d} units  settle {m['settle']:.2f} s" if m["settle"] is not None else f"  overshoot {m['over']:3d} units  not settled"
            print(line)
            return rows, m

        tmo_saved = t.read16(regs + 0x54); lim_saved = (t.read16(regs + 0x50), t.read8(regs + 0x32))
        hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000)   # stock Torque Limit must not clip impedance

        def lim(sw_ma, hw_100ma):   # current ceiling: model/winding estimate (0x50, mA) and hardware on the bus (0x32, ×100 mA)
            swd(t.write16, regs + 0x50, sw_ma)
            try:
                t.write8(regs + 0x32, hw_100ma)
            except Exception:
                pass
        try:
            lim(8000, 0)   # whole test under an 8 A model ceiling: bench PSU 10 A
            r0 = rd(); print("start:", r0)
            cmd(r0["pos"]); watch(0.5, "1. hold in place")
            res = {}
            for i, (kd, tgt) in enumerate(((20, B), (60, A), (150, B), (300, A)), 2):
                cmd(tgt, kd=kd, tmax=600); _, m = watch(0.8, f"{i}. tmax 600, kd={kd} → {tgt}", target=tgt); res[kd] = m
            best = min(res, key=lambda k: (res[k]["settle"] is None, res[k]["settle"] or 9, res[k]["over"]))
            print(f"   best kd={best}")
            # full duty on a 10 A bench PSU: stall current 24 V / 0.5 Ω = 48 A, hence the current ceiling
            lim(8000, 0); cmd(B, kd=best, tmax=tmax_final); watch(0.8, f"6a. tmax {tmax_final}, I ≤ 8 A by model → {B}", target=B)
            lim(5000, 0); cmd(A, kd=best, tmax=tmax_final); watch(0.8, f"6b. tmax {tmax_final}, I ≤ 5 A → {A}", target=A)
            lim(8000, 0); cmd(B, kd=best, tmax=tmax_final); watch(0.8, f"6c. tmax {tmax_final}, I ≤ 8 A → {B}", target=B)
            # from here on — 8 A ceiling, bench PSU 10 A
            cmd(B, vel=3000, kd=best); watch(0.6, "7. vel=+3000 units/s 100 ms (expect ≈%d)" % (B + 300))
            cmd(A, kd=best); watch(0.8, f"8. → {A}", target=A)
            now16 = (t.read32(g_ms) + t.read32(toff)) & 0xFFFF
            cmd(B, kd=best, at=(now16 + 500) & 0xFFFF)
            rows, _ = watch(1.2, f"9. at = +500 ms → {B}")
            moved = next((dt for dt, r in rows if abs(r["pos"] - A) > 50), None)
            print(f"   started after {moved:.2f} s (expect ~0.5)" if moved is not None else "   did not start")
            cmd(A, kd=best); watch(0.6, f"10. → {A}")
            a, _ = watch(0.3, "    pause")
            cmd(A, kp=100, kd=20, ff=600); a, _ = watch(0.6, "11. kp=1.0, ff=+600 mA")
            cmd(A, kp=100, kd=20, ff=-600); b, _ = watch(0.6, "12. kp=1.0, ff=−600 mA")
            print(f"   offset from ff: {a[-1][1]['pos'] - A:+d} / {b[-1][1]['pos'] - A:+d} units (expect symmetric)")
            cmd(A, kp=150, kd=best); watch(0.6, f"13. kp=1.5 → {A} (soft)", target=A)
        finally:
            cmd(rd()["pos"], mode=0); t.write8(regs + 0x18, 0); t.write16(regs + 0x54, tmo_saved); lim(*lim_saved)
            print("torque disabled:", rd())


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
