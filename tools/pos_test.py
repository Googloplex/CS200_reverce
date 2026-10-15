#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pos_test.py — step tests of the position loop via SWD (attach). Clone/v1 via ELF symbols, stock (--native)
via the fixed table address 0x20000326. Log to dumps/<servo>/tests/pos_<fw>_tl<TL>_<time>.csv.

  py -3.11 tools/pos_test.py [--tl 150] [--dwell 1.5] [--servo servo_01] [--native] [goal_us ...]
  default steps 1500 1000 2000 1500 µs

On v1 also logs telemetry 0x60..0x77 (Hall speed and counter, currents, Vbus) and prints an estimate of
Hall steps per ADC unit and peak speed.
Protection: torque is not enabled if the potentiometer is at the edge or jumping; during the test torque is cut if
the position goes to the ADC edge, the error grows for more than 300 ms, or the current is high.
"""
import argparse
import os
import struct
import sys
import time

from pyocd.core.helpers import ConnectHelper
from pyocd.debug.elf.elf import ELFBinaryFile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.path.join(ROOT, "fw", "build", "hb200t.elf")
R_TE, R_GOAL, R_TL, R_POS, R_LOAD, R_V, R_GOAL_ADC = 0x18, 0x1E, 0x22, 0x24, 0x28, 0x2A, 0x5A
NATIVE_REGS = 0x20000326


def session():
    return ConnectHelper.session_with_chosen_probe(target_override="cortex_m", options={"connect_mode": "attach"})


def regs_addr(t, native):
    if native:
        return NATIVE_REGS
    return ELFBinaryFile(ELF, t.memory_map).symbol_decoder.get_symbol_for_name("regs").address


def torque_off(t, regs):
    t.write16(regs + R_TL, 0)
    t.write8(regs + R_TE, 0)


def emergency_off(native):
    end = time.time() + 15
    while time.time() < end:
        try:
            with session() as s:
                torque_off(s.target, regs_addr(s.target, native))
                print("torque disabled after reconnect")
                return
        except Exception as e:
            print("  reconnect failed:", e)
            time.sleep(0.5)
    print("!!! TORQUE NOT DISABLED — CUT POWER !!!")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tl", type=int, default=150)
    ap.add_argument("--dwell", type=float, default=1.5)
    ap.add_argument("--servo", default="servo_01")
    ap.add_argument("--native", action="store_true")
    ap.add_argument("goals", nargs="*", type=int, default=[1500, 1000, 2000, 1500])
    a = ap.parse_args()

    outdir = os.path.join(ROOT, "dumps", a.servo, "tests")
    os.makedirs(outdir, exist_ok=True)
    fw = "native" if a.native else "clone"
    csv_path = os.path.join(outdir, time.strftime(f"pos_{fw}_tl{a.tl}_%Y%m%d_%H%M%S.csv"))
    csv = open(csv_path, "w", encoding="utf-8")
    csv.write("t_ms,goal_us,goal_adc,pos,load,volt,speed,hall,i_bus,vbus_mv,i_wind,status,duty\n")

    try:
        with session() as s:
            t = s.target
            regs = regs_addr(t, a.native)
            hb = None
            if not a.native:   # heartbeat: if SWD is lost, the firmware drops torque itself after 0.5 s
                dec = ELFBinaryFile(ELF, t.memory_map).symbol_decoder
                hb = (dec.get_symbol_for_name("g_last_rx_ms").address, dec.get_symbol_for_name("g_ms").address)
                tmo_saved = t.read16(regs + 0x54)
                t.write32(hb[0], t.read32(hb[1])); t.write16(regs + 0x54, 500)

            def rd():
                d = bytes(t.read_memory_block8(regs, 0x80))
                r = {"pos": struct.unpack_from("<h", d, R_POS)[0], "load": struct.unpack_from("<H", d, R_LOAD)[0],
                     "volt": d[R_V], "goal_adc": struct.unpack_from("<H", d, R_GOAL_ADC)[0],
                     "goal": struct.unpack_from("<H", d, R_GOAL)[0], "te": d[R_TE]}
                if a.native:
                    r.update(speed=0, hall=0, i_bus=0, vbus_mv=0, i_wind=0, status=0, duty=0)
                else:
                    r.update(speed=struct.unpack_from("<h", d, 0x60)[0], hall=struct.unpack_from("<i", d, 0x62)[0],
                             i_bus=struct.unpack_from("<H", d, 0x66)[0], vbus_mv=struct.unpack_from("<H", d, 0x68)[0],
                             i_wind=struct.unpack_from("<H", d, 0x6A)[0], status=d[0x6C], duty=struct.unpack_from("<H", d, 0x74)[0])
                return r

            r0 = rd(); time.sleep(0.1); r1 = rd()
            print(f"potentiometer: pos={r0['pos']} / {r1['pos']}, load={r1['load']}, V={r1['volt'] / 10:.1f}"
                  + ("" if a.native else f", Vbus={r1['vbus_mv']} mV, hall={r1['hall']}, status=0x{r1['status']:02X}"))
            if not (150 < r0["pos"] < 3950) or abs(r0["pos"] - r1["pos"]) > 30:
                sys.exit("potentiometer at the edge of the scale or jumping — not enabling torque")

            t0 = time.time()
            t.write16(regs + R_TL, a.tl)
            t.write8(regs + R_TE, 1)
            try:
                for g in a.goals:
                    t.write16(regs + R_GOAL, g)
                    t_step = time.time()
                    first = rd()
                    start_pos, start_hall = first["pos"], first["hall"]
                    best, best_t = None, t_step
                    rows = []
                    while time.time() - t_step < a.dwell:
                        if hb: t.write32(hb[0], t.read32(hb[1]))
                        r = rd()
                        now = time.time()
                        err = abs(r["pos"] - r["goal_adc"])
                        rows.append((now - t_step, r))
                        csv.write(f"{int((now - t0) * 1000)},{g},{r['goal_adc']},{r['pos']},{r['load']},{r['volt']},"
                                  f"{r['speed']},{r['hall']},{r['i_bus']},{r['vbus_mv']},{r['i_wind']},{r['status']},{r['duty']}\n")
                        if best is None or err < best:
                            best, best_t = err, now
                        elif err > best + 150 and now - best_t > 0.3:
                            raise RuntimeError(f"error growing (pos={r['pos']}, target={r['goal_adc']}) — direction? stop")
                        if r["pos"] < 120 or r["pos"] > 3975:
                            raise RuntimeError(f"position at ADC edge ({r['pos']}) — stop")
                        if r["load"] > 3000:
                            raise RuntimeError(f"current {r['load']} — stop")
                        if r["te"] != 1:
                            raise RuntimeError(f"torque turned off by itself (status=0x{r['status']:02X})")
                        time.sleep(0.02)
                    goal_adc = rows[-1][1]["goal_adc"]
                    tail = [r["pos"] for dt, r in rows if dt > a.dwell - 0.3]
                    final = sum(tail) / len(tail)
                    direction = 1 if goal_adc >= start_pos else -1
                    overshoot = max(0, max((r["pos"] - goal_adc) * direction for _, r in rows))
                    settle = next((dt for dt, r in rows if abs(r["pos"] - goal_adc) <= 30 and
                                   all(abs(x["pos"] - goal_adc) <= 30 for d2, x in rows if d2 >= dt)), None)
                    peak_load = max(r["load"] for _, r in rows)
                    line = (f"goal {g:4d} µs → target {goal_adc:4d}: {start_pos:4d} → {final:6.0f} (error {round(final) - goal_adc:+4d}), "
                            f"overshoot {overshoot:3d}, settle {'%.2f s' % settle if settle is not None else 'none'}, peak Load {peak_load}")
                    if not a.native:
                        last = rows[-1][1]
                        dpos = last["pos"] - start_pos
                        dhall = last["hall"] - start_hall
                        peak_speed = max(abs(r["speed"]) for _, r in rows)
                        peak_iw = max(r["i_wind"] for _, r in rows)
                        status_or = 0
                        for _, r in rows: status_or |= r["status"]
                        moving = [r for dt, r in rows if r["duty"] > 0 and abs(r["speed"]) > 0]
                        mean_iw = sum(r["i_wind"] for r in moving) / len(moving) if moving else 0
                        line += (f"\n           Halls: Δ={dhall:+d} steps over Δpos={dpos:+d} units → {abs(dhall) / abs(dpos) if dpos else 0:.3f} step/unit; "
                                 f"peak speed {peak_speed} step/s; I_wind peak {peak_iw} mA, mean while moving {mean_iw:.0f} mA; "
                                 f"Vbus {last['vbus_mv']} mV; status per step 0x{status_or:02X}")
                    print(line)
            finally:
                torque_off(t, regs)
                if hb: t.write16(regs + 0x54, tmo_saved)
                time.sleep(0.2)
                r = rd()
                print(f"torque disabled: pos={r['pos']} load={r['load']} V={r['volt'] / 10:.1f}")
    except Exception as e:
        print("!!!", e)
        emergency_off(a.native)
    finally:
        csv.close()
        print("log:", csv_path)


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
