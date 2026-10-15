#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
feat2_test.py — проверка фич v1.5 (пакеты 2–4) через SWD на servo_01: паспорт (UID, дата, температура кристалла),
счётчики (энергия, одометр, пакеты), чёрный ящик, сохранение конфигурации с перезагрузкой, контакт по торможению,
самотест/идентификация, синусоидальная коммутация против шестишаговой.

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
# смещения regs2 от 0x80
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
                print(f"   SWD: {e} — переподключение {i + 1}")
                time.sleep(0.3)
                try:
                    t.dp.disconnect(); t.dp.connect()
                except Exception as e2:
                    print("   ", e2)
        raise RuntimeError("SWD не восстановился")

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
                raise RuntimeError(f"{label}: стоп, pos={r['pos']} load={r['load']}")
            time.sleep(every)
        p = [r["pos"] for _, r in rows]
        settle = next((dt for dt, r in rows if all(abs(x["pos"] - p[-1]) <= 15 for d2, x in rows if d2 >= dt)), None)
        print(f"{label:40s} pos {p[0]:5d} → {p[-1]:5d}  vel пик {max(abs(r['vel']) for _, r in rows):5d}  "
              f"I пик {max(r['ibus'] for _, r in rows):5d} мА  duty пик {max(r['duty'] for _, r in rows):4d}  "
              f"τ пик {max(abs(r['torque']) for _, r in rows):5d} мН·м  st2 {rows[-1][1]['st2']:02X}"
              + (f"  устан {settle:.2f} с" if settle is not None else ""))
        return rows

    ok = []
    tmo_saved = t.read16(regs + 0x54)
    hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000); t.write16(regs + 0x50, 8000)
    cfg_saved = t.read8(regs + 0x5E)
    try:
        d, d2 = r1(), r2()
        info = bytes(t.read_memory_block8(0x1FFFF854, 4))
        print(f"fw 0x{d[2]:02X}  UID {d2[O['UID']:O['UID'] + 12].hex()}  дата чипа {info[0] | (info[1] << 8)}-{info[2]:02d}-{info[3]:02d}  "
              f"T_mcu {struct.unpack_from('<b', d2, O['TEMP'])[0]} °C (raw {t.read16(temp_raw)})  state {d2[O['STATE']]}  τ_avail {d2[O['TAU_AVAIL']]} %  "
              f"энергия {struct.unpack_from('<I', d2, O['ENERGY'])[0]} мДж  одометр {struct.unpack_from('<I', d2, O['ODO'])[0]}  "
              f"пакеты ok/err {struct.unpack_from('<H', d2, O['PKT_OK'])[0]}/{struct.unpack_from('<H', d2, O['PKT_ERR'])[0]}")
        tm = struct.unpack_from("<b", d2, O["TEMP"])[0]
        ok.append(("температура кристалла 10..70 °C", 10 <= tm <= 70))
        ok.append(("UID не пустой", d2[O["UID"]:O["UID"] + 12] not in (b"\xff" * 12, b"\x00" * 12)))

        # 1. энергия/одометр растут при движении; момент телеметрии ненулевой при движении
        e0, o0 = struct.unpack_from("<I", d2, O["ENERGY"])[0], struct.unpack_from("<I", d2, O["ODO"])[0]
        cmd(A); watch(0.5, "1a. → A")
        cmd(B); rows = watch(0.5, "1b. → B")
        d2 = r2()
        e1, o1 = struct.unpack_from("<I", d2, O["ENERGY"])[0], struct.unpack_from("<I", d2, O["ODO"])[0]
        print(f"   энергия +{e1 - e0} мДж, одометр +{o1 - o0} шагов (ждём ~{abs(A - B) * 0.222:.0f})")
        ok.append(("энергия и одометр считают", e1 > e0 and 0.8 * abs(A - B) * 0.206 < (o1 - o0) < 1.8 * abs(A - B) * 0.206))   # + мёртвый ход реверса и дрожание
        ok.append(("момент телеметрии ненулевой в движении", max(abs(r["torque"]) for _, r in rows) > 100))

        # 2. чёрный ящик: разморозить (мог замёрзнуть по событию раньше), заморозить вручную, прочитать последние записи
        w8(O["BB_CTRL"], 0); time.sleep(0.05)
        w8(O["BB_CTRL"], 1); time.sleep(0.05)
        d2 = r2()
        head, trig, ctrl = struct.unpack_from("<H", d2, O["BB_HEAD"])[0], d2[O["BB_TRIG"]], d2[O["BB_CTRL"]]
        print(f"2. чёрный ящик: ctrl {ctrl} trig 0x{trig:02X} head {head}")
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
        ok.append(("чёрный ящик заморожен и содержит текущее положение", ctrl == 1 and trig == 0xFF and abs(last[-1][0] - rd()["pos"]) < 50))
        w8(O["BB_CTRL"], 0); time.sleep(0.02)
        ok.append(("чёрный ящик размораживается", r2()[O["BB_CTRL"]] == 0))

        # 3. контакт по торможению: на обычных ступеньках ложных срабатываний нет
        w16(O["CONTACT"], 1500)
        cmd(A); r_a = watch(0.5, "3a. контакт-детектор, → A")
        cmd(B); r_b = watch(0.5, "3b. → B")
        seen = any(r["st2"] & 0x20 for _, r in r_a + r_b)
        ok.append(("нет ложного контакта на свободных ступеньках", not seen))
        w16(O["CONTACT"], 0)

        # 4. самотест / идентификация
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
                    t.write8(regs + 0x18, 0); raise RuntimeError(f"самотест: стоп pos={r['pos']} load={r['load']}")
                if res & 0x80: break
                time.sleep(0.02)
            d2 = r2()
            print(f"4. самотест: результат 0x{res:02X} (1 Холлы, 2 направление, 4 R, 8 Kv, 16 шаг/ед, 40 прервано, 80 готово)  "
                  f"dir {struct.unpack_from('<b', d2, O['ST_DIR'])[0]:+d}  шаг/ед {struct.unpack_from('<H', d2, O['ST_SPC'])[0]} (ждём ~222)  "
                  f"R {struct.unpack_from('<H', d2, O['ST_R'])[0]} мОм (ждём ~500)  Kv {struct.unpack_from('<H', d2, O['ST_KV'])[0]} (ждём ~1470)  "
                  f"трение duty {d2[O['ST_FRIC']]}  pos {rd()['pos']}")
            ok.append(("самотест завершён без прерывания", (res & 0xC0) == 0x80))
            ok.append(("самотест: Холлы, направление, Kv, шаг/ед правдоподобны (R без нагрузки — из журнала)", (res & 0x1B) == 0x1B))
            print(f"   трение: IC {struct.unpack_from('<H', d2, O['FRIC_IC'])[0]} мА, KV {struct.unpack_from('<H', d2, O['FRIC_KV'])[0]} мкА/(шаг/с); мёртвый ход при реверсе {struct.unpack_from('<H', d2, 0x78)[0]} шагов Холлов")
            t.write16(regs + 0x50, 8000)

        # 4b. автомат состояний и защёлка отказа: VBUS_MAX ниже текущего → FAULT → LATCHED → снятие командой 3
        st_ready = r2()[O['STATE']]
        cmd(rd()['pos']); time.sleep(0.03); st_en = r2()[O['STATE']]
        t.write8(regs + 0x18, 0); time.sleep(0.03)
        t.write8(regs + 0x5E, cfg_saved | 0x20); t.write16(regs + 0x56, 5000); time.sleep(0.05)
        st_f = r2()[O['STATE']]
        cmd(rd()['pos']); time.sleep(0.05); te_l = rd()['te']
        t.write16(regs + 0x56, 0); time.sleep(0.05); st_l2 = r2()[O['STATE']]
        w8(O['CFG2_CMD'], 3); time.sleep(0.05); st_c = r2()[O['STATE']]
        t.write8(regs + 0x5E, cfg_saved)
        print(f"4b. состояния: READY {st_ready} → ENABLED {st_en} → отказ Vbus {st_f} (TE при попытке включить: {te_l}) → без отказа {st_l2} → после снятия {st_c}")
        ok.append(("автомат: 1 → 2 → 6 (защёлка, момент не включается) → 1", (st_ready, st_en, st_l2, te_l, st_c) == (1, 2, 6, 0, 1)))
        # 4c. TEST_UNLIMITED: флаг ставится
        w8(O['CFG2_CMD'], 4); time.sleep(0.03); unl = r2()[O['STATUS2']] & 0x80
        ok.append(("TEST_UNLIMITED ставит флаг 0x80", bool(unl)))

        # 5. сохранение конфигурации с перезагрузкой
        w16(O["SOFT_MIN"], 1234); w8(O["FAULT_MODE"], 2); w16(O["CONTACT"], 777)
        w8(O["CFG2_CMD"], 1); time.sleep(0.1)
        print("5. конфигурация сохранена, перезагрузка…")
        t.reset(); time.sleep(1.5)
        try:
            t.dp.disconnect(); t.dp.connect()
        except Exception:
            pass
        d2 = r2()
        got = (struct.unpack_from("<H", d2, O["SOFT_MIN"])[0], d2[O["FAULT_MODE"]], struct.unpack_from("<H", d2, O["CONTACT"])[0])
        print(f"   после сброса: SOFT_MIN {got[0]} FAULT_MODE {got[1]} CONTACT {got[2]}  fw 0x{r1()[2]:02X}")
        ok.append(("конфигурация пережила перезагрузку", got == (1234, 2, 777)))
        w16(O["SOFT_MIN"], 0); w8(O["FAULT_MODE"], 0); w16(O["CONTACT"], 0)
        w8(O["CFG2_CMD"], 1); time.sleep(0.1)
        hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000); t.write16(regs + 0x50, 8000)

        # 6. синус против шестишаговой: одинаковые ступеньки при tmax 300, потолок 3 А
        if do_sine:
            t.write16(regs + 0x50, 3000)
            cmd(A); watch(0.5, "6a. шестишаговая → A")
            r6 = watch(0.0, "")  if False else None
            rows6 = []
            cmd(B, tmax=300); rows6 += watch(0.6, "6b. шестишаговая → B, tmax 300")
            cmd(A, tmax=300); rows6 += watch(0.6, "6c. шестишаговая → A, tmax 300")
            t.write8(regs + 0x18, 0); time.sleep(0.03)
            t.write8(regs + 0x5E, cfg_saved | 0x10); time.sleep(0.03)
            son = t.read8(sine_on)
            print(f"   режим синуса: sine_on = {son}")
            ok.append(("синус включился при TE = 0", son == 1))
            res_adv = {}
            for adv in (0, -20, 20):
                w8(O["SINE_ADV"], adv & 0xFF)
                rows = []
                cmd(B, tmax=300); rows += watch(0.6, f"6d. синус adv {adv:+d} → B, tmax 300")
                cmd(A, tmax=300); rows += watch(0.6, f"6e. синус adv {adv:+d} → A, tmax 300")
                res_adv[adv] = (max(abs(r["vel"]) for _, r in rows), max(r["ibus"] for _, r in rows))
            w8(O["SINE_ADV"], 0)
            v6, i6 = max(abs(r["vel"]) for _, r in rows6), max(r["ibus"] for _, r in rows6)
            print(f"   шестишаговая: vel пик {v6}, I пик {i6};  синус: " + ", ".join(f"adv {a:+d}: vel {v}, I {i}" for a, (v, i) in res_adv.items()))
            ok.append(("синус едет (скорость ≥ 60 % шестишаговой при том же duty)", res_adv[0][0] >= v6 * 0.6))
            t.write8(regs + 0x18, 0); time.sleep(0.03)
            t.write8(regs + 0x5E, cfg_saved); time.sleep(0.03)
            ok.append(("шестишаговая вернулась", t.read8(sine_on) == 0))
            t.write16(regs + 0x50, 8000)
            cmd(A); watch(0.5, "6f. шестишаговая → A")
    finally:
        try:
            cmd(rd()["pos"], mode=0); t.write8(regs + 0x18, 0); t.write16(regs + 0x54, tmo_saved); t.write8(regs + 0x5E, cfg_saved)
            print("момент выключен:", rd())
        except Exception as e:
            print("финал:", e)
        print("\nИТОГ:")
        for name, g in ok:
            print(f"  [{'OK' if g else '!!'}] {name}")
        sess.close()


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
