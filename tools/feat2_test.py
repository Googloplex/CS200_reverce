#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
feat2_test.py — v1.5 feature check (packages 2–4) via SWD on servo_01: identity record (UID, date, die temperature),
counters (energy, odometer, packets), flight recorder, config save with reboot, contact detection by braking,
self-test/identification, sinusoidal vs six-step commutation.

  py -3.11 tools/feat2_test.py [--no-sine] [--no-selftest]
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
A, B = 2048, 1231
# regs2 offsets from 0x80
O = dict(STATUS2=0x2E, TEMP=0x32, UID=0x34, ENERGY=0x40, ODO=0x44, PKT_OK=0x48, PKT_ERR=0x4A, FRIC_IC=0x4C, FRIC_KV=0x4E, STATE=0x33, TAU_AVAIL=0x2F,
         BB_CTRL=0x50, BB_PAGE=0x51, BB_HEAD=0x52, BB_TRIG=0x54, CFG2_CMD=0x55, CONTACT=0x56, TORQUE=0x58, N_GEAR=0x5A, KT=0x5C,
         ST_RES=0x5E, ST_FRIC=0x5F, ST_R=0x60, ST_KV=0x62, ST_SPC=0x64, SINE_ADV=0x66, ST_DIR=0x67, BB_WIN=0x68,
         SOFT_MIN=0x24, FAULT_MODE=0x29)


def main():
    do_sine = "--no-sine" not in sys.argv
    do_st = "--no-selftest" not in sys.argv
    sess = ConnectHelper.session_with_chosen_probe(target_override="cortex_m", options={"connect_mode": "attach"})
    sess.open()
    t = sess.target
    dec = ELFBinaryFile(ELF, t.memory_map).symbol_decoder
    sym = lambda n: dec.get_symbol_for_name(n).address
    regs, regs2 = sym("regs"), sym("regs2")
    g_ms, last_rx, sine_on, temp_raw = sym("g_ms"), sym("g_last_rx_ms"), sym("sine_on"), sym("g_mcu_temp_raw")

    def swd(fn, *a):
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

    def r2():
        return bytes(swd(t.read_memory_block8, regs2, 0x80))

    def r1():
        return bytes(swd(t.read_memory_block8, regs, 0x80))

    def rd():
        d, d2 = r1(), r2()
        return dict(pos=struct.unpack_from("<h", d, 0x24)[0], te=d[0x18], load=struct.unpack_from("<H", d, 0x28)[0],
                    duty=struct.unpack_from("<H", d, 0x74)[0], vel=struct.unpack_from("<h", d2, 0x12)[0],
                    ibus=struct.unpack_from("<H", d2, 0x14)[0], status=d2[0x17], st2=d2[O["STATUS2"]],
                    vbus=struct.unpack_from("<H", d2, 0x18)[0], torque=struct.unpack_from("<h", d2, O["TORQUE"])[0])

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

    def watch(sec, label, every=0.02):
        t0 = time.time(); rows = []; hot = 0
        while time.time() - t0 < sec:
            hb(); r = rd(); rows.append((time.time() - t0, r))
            hot = hot + 1 if r["load"] > 6000 else 0
            if r["pos"] < 150 or r["pos"] > 3950 or hot >= 2:
                cmd(r["pos"], mode=0); t.write8(regs + 0x18, 0)
                raise RuntimeError(f"{label}: stop, pos={r['pos']} load={r['load']}")
            time.sleep(every)
        p = [r["pos"] for _, r in rows]
        settle = next((dt for dt, r in rows if all(abs(x["pos"] - p[-1]) <= 15 for d2, x in rows if d2 >= dt)), None)
        print(f"{label:40s} pos {p[0]:5d} → {p[-1]:5d}  vel peak {max(abs(r['vel']) for _, r in rows):5d}  "
              f"I peak {max(r['ibus'] for _, r in rows):5d} mA  duty peak {max(r['duty'] for _, r in rows):4d}  "
              f"τ peak {max(abs(r['torque']) for _, r in rows):5d} mN·m  st2 {rows[-1][1]['st2']:02X}"
              + (f"  settle {settle:.2f} s" if settle is not None else ""))
        return rows

    ok = []
    tmo_saved = t.read16(regs + 0x54)
    hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000); t.write16(regs + 0x50, 8000)
    cfg_saved = t.read8(regs + 0x5E)
    try:
        d, d2 = r1(), r2()
        info = bytes(t.read_memory_block8(0x1FFFF854, 4))
        print(f"fw 0x{d[2]:02X}  UID {d2[O['UID']:O['UID'] + 12].hex()}  chip date {info[0] | (info[1] << 8)}-{info[2]:02d}-{info[3]:02d}  "
              f"T_mcu {struct.unpack_from('<b', d2, O['TEMP'])[0]} °C (raw {t.read16(temp_raw)})  state {d2[O['STATE']]}  τ_avail {d2[O['TAU_AVAIL']]} %  "
              f"energy {struct.unpack_from('<I', d2, O['ENERGY'])[0]} mJ  odometer {struct.unpack_from('<I', d2, O['ODO'])[0]}  "
              f"packets ok/err {struct.unpack_from('<H', d2, O['PKT_OK'])[0]}/{struct.unpack_from('<H', d2, O['PKT_ERR'])[0]}")
        tm = struct.unpack_from("<b", d2, O["TEMP"])[0]
        ok.append(("die temperature 10..70 °C", 10 <= tm <= 70))
        ok.append(("UID not blank", d2[O["UID"]:O["UID"] + 12] not in (b"\xff" * 12, b"\x00" * 12)))

        # 1. energy/odometer grow while moving; telemetry torque is nonzero while moving
        e0, o0 = struct.unpack_from("<I", d2, O["ENERGY"])[0], struct.unpack_from("<I", d2, O["ODO"])[0]
        cmd(A); watch(0.5, "1a. → A")
        cmd(B); rows = watch(0.5, "1b. → B")
        d2 = r2()
        e1, o1 = struct.unpack_from("<I", d2, O["ENERGY"])[0], struct.unpack_from("<I", d2, O["ODO"])[0]
        print(f"   energy +{e1 - e0} mJ, odometer +{o1 - o0} steps (expect ~{abs(A - B) * 0.222:.0f})")
        ok.append(("energy and odometer count", e1 > e0 and 0.8 * abs(A - B) * 0.206 < (o1 - o0) < 1.8 * abs(A - B) * 0.206))   # + backlash on reversal and jitter
        ok.append(("telemetry torque nonzero while moving", max(abs(r["torque"]) for _, r in rows) > 100))

        # 2. flight recorder: unfreeze (may have frozen on an earlier event), freeze manually, read the last records
        w8(O["BB_CTRL"], 0); time.sleep(0.05)
        w8(O["BB_CTRL"], 1); time.sleep(0.05)
        d2 = r2()
        head, trig, ctrl = struct.unpack_from("<H", d2, O["BB_HEAD"])[0], d2[O["BB_TRIG"]], d2[O["BB_CTRL"]]
        print(f"2. flight recorder: ctrl {ctrl} trig 0x{trig:02X} head {head}")
        ents = []
        for page in range(26):
            w8(O["BB_PAGE"], page); time.sleep(0.005)
            w = r2()[O["BB_WIN"]:O["BB_WIN"] + 16]
            for k in range(2):
                e = w[k * 8:(k + 1) * 8]
                ents.append(struct.unpack("<hhHBB", e))
        last = ents[-6:]
        for e in last:
            print(f"   pos {e[0]:5d} vel {e[1]:6d} I {e[2]:5d} duty {e[3] * 4:4d} st 0x{e[4]:02X}")
        ok.append(("flight recorder frozen and holds current position", ctrl == 1 and trig == 0xFF and abs(last[-1][0] - rd()["pos"]) < 50))
        w8(O["BB_CTRL"], 0); time.sleep(0.02)
        ok.append(("flight recorder unfreezes", r2()[O["BB_CTRL"]] == 0))

        # 3. contact by braking: no false triggers on plain steps
        w16(O["CONTACT"], 1500)
        cmd(A); r_a = watch(0.5, "3a. contact detector, → A")
        cmd(B); r_b = watch(0.5, "3b. → B")
        seen = any(r["st2"] & 0x20 for _, r in r_a + r_b)
        ok.append(("no false contact on free steps", not seen))
        w16(O["CONTACT"], 0)

        # 4. self-test / identification
        if do_st:
            t.write8(regs + 0x18, 0); time.sleep(0.05)
            t.write16(regs + 0x50, 3000)
            cmd(rd()["pos"], mode=0)
            w8(O["CFG2_CMD"], 2)
            t0 = time.time(); res = 0
            while time.time() - t0 < 4.0:
                hb(); d2 = r2(); res = d2[O["ST_RES"]]
                r = rd()
                if r["pos"] < 150 or r["pos"] > 3950 or r["load"] > 6000:
                    t.write8(regs + 0x18, 0); raise RuntimeError(f"self-test: stop pos={r['pos']} load={r['load']}")
                if res & 0x80: break
                time.sleep(0.02)
            d2 = r2()
            print(f"4. self-test: result 0x{res:02X} (1 Halls, 2 direction, 4 R, 8 Kv, 16 step/unit, 40 aborted, 80 done)  "
                  f"dir {struct.unpack_from('<b', d2, O['ST_DIR'])[0]:+d}  step/unit {struct.unpack_from('<H', d2, O['ST_SPC'])[0]} (expect ~222)  "
                  f"R {struct.unpack_from('<H', d2, O['ST_R'])[0]} mΩ (expect ~500)  Kv {struct.unpack_from('<H', d2, O['ST_KV'])[0]} (expect ~1470)  "
                  f"friction duty {d2[O['ST_FRIC']]}  pos {rd()['pos']}")
            ok.append(("self-test completed without abort", (res & 0xC0) == 0x80))
            ok.append(("self-test: Halls, direction, Kv, step/unit plausible (R without load — from journal)", (res & 0x1B) == 0x1B))
            print(f"   friction: IC {struct.unpack_from('<H', d2, O['FRIC_IC'])[0]} mA, KV {struct.unpack_from('<H', d2, O['FRIC_KV'])[0]} µA/(step/s); backlash on reversal {struct.unpack_from('<H', d2, 0x78)[0]} Hall steps")
            t.write16(regs + 0x50, 8000)

        # 4b. state machine and fault latch: VBUS_MAX below current → FAULT → LATCHED → clear with command 3
        st_ready = r2()[O['STATE']]
        cmd(rd()['pos']); time.sleep(0.03); st_en = r2()[O['STATE']]
        t.write8(regs + 0x18, 0); time.sleep(0.03)
        t.write8(regs + 0x5E, cfg_saved | 0x20); t.write16(regs + 0x56, 5000); time.sleep(0.05)
        st_f = r2()[O['STATE']]
        cmd(rd()['pos']); time.sleep(0.05); te_l = rd()['te']
        t.write16(regs + 0x56, 0); time.sleep(0.05); st_l2 = r2()[O['STATE']]
        w8(O['CFG2_CMD'], 3); time.sleep(0.05); st_c = r2()[O['STATE']]
        t.write8(regs + 0x5E, cfg_saved)
        print(f"4b. states: READY {st_ready} → ENABLED {st_en} → Vbus fault {st_f} (TE on enable attempt: {te_l}) → fault gone {st_l2} → after clear {st_c}")
        ok.append(("state machine: 1 → 2 → 6 (latch, torque does not enable) → 1", (st_ready, st_en, st_l2, te_l, st_c) == (1, 2, 6, 0, 1)))
        # 4c. TEST_UNLIMITED: flag is set
        w8(O['CFG2_CMD'], 4); time.sleep(0.03); unl = r2()[O['STATUS2']] & 0x80
        ok.append(("TEST_UNLIMITED sets flag 0x80", bool(unl)))

        # 5. config save with reboot
        w16(O["SOFT_MIN"], 1234); w8(O["FAULT_MODE"], 2); w16(O["CONTACT"], 777)
        w8(O["CFG2_CMD"], 1); time.sleep(0.1)
        print("5. config saved, rebooting…")
        t.reset(); time.sleep(1.5)
        try:
            t.dp.disconnect(); t.dp.connect()
        except Exception:
            pass
        d2 = r2()
        got = (struct.unpack_from("<H", d2, O["SOFT_MIN"])[0], d2[O["FAULT_MODE"]], struct.unpack_from("<H", d2, O["CONTACT"])[0])
        print(f"   after reset: SOFT_MIN {got[0]} FAULT_MODE {got[1]} CONTACT {got[2]}  fw 0x{r1()[2]:02X}")
        ok.append(("config survived reboot", got == (1234, 2, 777)))
        w16(O["SOFT_MIN"], 0); w8(O["FAULT_MODE"], 0); w16(O["CONTACT"], 0)
        w8(O["CFG2_CMD"], 1); time.sleep(0.1)
        hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000); t.write16(regs + 0x50, 8000)

        # 6. sinusoidal vs six-step: identical steps at tmax 300, 3 A ceiling
        if do_sine:
            t.write16(regs + 0x50, 3000)
            cmd(A); watch(0.5, "6a. six-step → A")
            r6 = watch(0.0, "")  if False else None
            rows6 = []
            cmd(B, tmax=300); rows6 += watch(0.6, "6b. six-step → B, tmax 300")
            cmd(A, tmax=300); rows6 += watch(0.6, "6c. six-step → A, tmax 300")
            t.write8(regs + 0x18, 0); time.sleep(0.03)
            t.write8(regs + 0x5E, cfg_saved | 0x10); time.sleep(0.03)
            son = t.read8(sine_on)
            print(f"   sine mode: sine_on = {son}")
            ok.append(("sine enabled at TE = 0", son == 1))
            res_adv = {}
            for adv in (0, -20, 20):
                w8(O["SINE_ADV"], adv & 0xFF)
                rows = []
                cmd(B, tmax=300); rows += watch(0.6, f"6d. sine adv {adv:+d} → B, tmax 300")
                cmd(A, tmax=300); rows += watch(0.6, f"6e. sine adv {adv:+d} → A, tmax 300")
                res_adv[adv] = (max(abs(r["vel"]) for _, r in rows), max(r["ibus"] for _, r in rows))
            w8(O["SINE_ADV"], 0)
            v6, i6 = max(abs(r["vel"]) for _, r in rows6), max(r["ibus"] for _, r in rows6)
            print(f"   six-step: vel peak {v6}, I peak {i6};  sine: " + ", ".join(f"adv {a:+d}: vel {v}, I {i}" for a, (v, i) in res_adv.items()))
            ok.append(("sine moves (speed ≥ 60 % of six-step at the same duty)", res_adv[0][0] >= v6 * 0.6))
            t.write8(regs + 0x18, 0); time.sleep(0.03)
            t.write8(regs + 0x5E, cfg_saved); time.sleep(0.03)
            ok.append(("six-step restored", t.read8(sine_on) == 0))
            t.write16(regs + 0x50, 8000)
            cmd(A); watch(0.5, "6f. six-step → A")
    finally:
        try:
            cmd(rd()["pos"], mode=0); t.write8(regs + 0x18, 0); t.write16(regs + 0x54, tmo_saved); t.write8(regs + 0x5E, cfg_saved)
            print("torque disabled:", rd())
        except Exception as e:
            print("final:", e)
        print("\nRESULT:")
        for name, g in ok:
            print(f"  [{'OK' if g else '!!'}] {name}")
        sess.close()


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
