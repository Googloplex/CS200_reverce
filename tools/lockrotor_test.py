#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lockrotor_test.py — winding R from a locked rotor: short duty pulses in wheel mode, R = U/I_wind.
  py -3.11 tools/lockrotor_test.py [--duty 80,120]"""
import os, struct, sys, time
from pyocd.core.helpers import ConnectHelper
from pyocd.debug.elf.elf import ELFBinaryFile
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, "fw", "build", "hb200t.elf")

def main():
    duties = [80, 120]
    if "--duty" in sys.argv:
        duties = [int(x) for x in sys.argv[sys.argv.index("--duty") + 1].split(",")]
    with ConnectHelper.session_with_chosen_probe(target_override="cortex_m", options={"connect_mode": "attach"}) as s:
        t = s.target; dec = ELFBinaryFile(ELF, t.memory_map).symbol_decoder
        sym = lambda n: dec.get_symbol_for_name(n).address
        regs, g_ms, last_rx = sym("regs"), sym("g_ms"), sym("g_last_rx_ms")
        def hb(): t.write32(last_rx, t.read32(g_ms))
        def rd():
            d = bytes(t.read_memory_block8(regs, 0x80)); u = lambda a: struct.unpack_from("<H", d, a)[0]
            return dict(te=d[0x18], load=u(0x28), duty=u(0x74), hall=struct.unpack_from("<h", d, 0x60)[0], ibus=u(0x66), iwind=u(0x6A), vbus=u(0x68), rest=u(0x76), code=d[0x6D], status=d[0x6C])
        def stop():
            try: t.write8(regs + 0x18, 0)
            except Exception: pass
        saved = dict(tmo=t.read16(regs + 0x54), tl=t.read16(regs + 0x22), cw=t.read16(regs + 6), ccw=t.read16(regs + 8), lim=t.read16(regs + 0x50), speed=t.read16(regs + 0x20))
        hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000); t.write16(regs + 0x50, 8000); t.write16(regs + 6, 0); t.write16(regs + 8, 0)
        results = []
        try:
            print("start:", rd())
            for dty in duties:
                for sgn in (1, -1):
                    t.write16(regs + 0x20, (dty & 0x3FF) | (0x400 if sgn < 0 else 0))
                    try: t.write8(regs + 0x18, 1)
                    except Exception: pass
                    t0 = time.time(); rows = []
                    while time.time() - t0 < 0.4:
                        hb(); r = rd(); rows.append((time.time() - t0, r))
                        if abs(r["hall"]) > 300 or r["load"] > 9000:
                            stop(); print(f"   duty {sgn*dty:+d}: rotor turned ({r['hall']} step/s) or current {r['load']} — stop"); break
                        time.sleep(0.02)
                    stop(); time.sleep(0.4)
                    tail = [r for dt, r in rows if dt > 0.2]
                    if not tail: continue
                    ib = sum(r["ibus"] for r in tail) / len(tail); iw = sum(r["iwind"] for r in tail) / len(tail)
                    vb = sum(r["vbus"] for r in tail) / len(tail); du = max(r["duty"] for r in tail)
                    u = vb * du / 1024
                    r_mohm = u / iw * 1000 if iw else float("nan")
                    hall = max(abs(r["hall"]) for r in tail)
                    print(f"duty {sgn*dty:+4d}: U {u/1000:5.2f} V  I_bus {ib:5.0f} mA  I_wind {iw:5.0f} mA  ω {hall:4d}  → R ≈ {r_mohm:4.0f} mΩ   (online R_est {tail[-1]['rest']}, Hall code {tail[-1]['code']})")
                    if hall < 50 and iw > 500: results.append(r_mohm)
            if results:
                print(f"\nR phase-to-phase over {len(results)} points: {sum(results)/len(results):.0f} mΩ (spread {min(results):.0f}..{max(results):.0f}); R_REF now {t.read16(regs + 0x16)}")
        finally:
            stop()
            t.write16(regs + 0x54, saved["tmo"]); t.write16(regs + 0x22, saved["tl"]); t.write16(regs + 6, saved["cw"]); t.write16(regs + 8, saved["ccw"]); t.write16(regs + 0x50, saved["lim"]); t.write16(regs + 0x20, saved["speed"])
            print("torque disabled:", rd())

if __name__ == "__main__":
    try: sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception: pass
    main()
