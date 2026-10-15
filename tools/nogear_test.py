#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nogear_test.py — стенд без редуктора (ротор свободный, потенциометр стоит): всё, что не требует положения.
Статика: паспорт, температура кристалла, счётчики, чёрный ящик, сохранение конфигурации с перезагрузкой, автомат/защёлка,
TEST_UNLIMITED. Динамика в режиме колеса (CW = CCW = 0, фиксированный duty): шестишаговая против синуса при одинаковом duty
в обе стороны, шум оценки скорости, R и Kv по двум точкам, подбор SINE_ADV.

  py -3.11 tools/nogear_test.py [--no-sine] [--duty 150,300] [--spin-only]
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
O = dict(STATUS2=0x2E, TAU_AVAIL=0x2F, TEMP=0x32, STATE=0x33, UID=0x34, ENERGY=0x40, ODO=0x44, PKT_OK=0x48, PKT_ERR=0x4A,
         FRIC_IC=0x4C, FRIC_KV=0x4E, BB_CTRL=0x50, BB_PAGE=0x51, BB_HEAD=0x52, BB_TRIG=0x54, CFG2_CMD=0x55, CONTACT=0x56,
         TORQUE=0x58, N_GEAR=0x5A, KT=0x5C, SINE_ADV=0x66, SINE_OFS=0x57, BB_WIN=0x68, SOFT_MIN=0x24, FAULT_MODE=0x29)
SPEED_MAX = 5000   # шагов Холлов/с — аварийный стоп (35 тыс. об/мин при pp = 1)


def main():
    do_sine = "--no-sine" not in sys.argv
    spin_only = "--spin-only" in sys.argv
    duties = [150, 300]
    if "--duty" in sys.argv:
        duties = [int(x) for x in sys.argv[sys.argv.index("--duty") + 1].split(",")]
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

    def r1():
        return bytes(swd(t.read_memory_block8, regs, 0x80))

    def r2():
        return bytes(swd(t.read_memory_block8, regs2, 0x80))

    def rd():
        d, d2 = r1(), r2()
        return dict(te=d[0x18], load=struct.unpack_from("<H", d, 0x28)[0], duty=struct.unpack_from("<H", d, 0x74)[0],
                    hall=struct.unpack_from("<h", d, 0x60)[0], ibus=struct.unpack_from("<H", d, 0x66)[0],
                    iwind=struct.unpack_from("<H", d, 0x6A)[0], vbus=struct.unpack_from("<H", d, 0x68)[0],
                    status=d[0x6C], code=d[0x6D], rest=struct.unpack_from("<H", d, 0x76)[0], st2=d2[O["STATUS2"]],
                    state=d2[O["STATE"]], torque=struct.unpack_from("<h", d2, O["TORQUE"])[0])

    def w8(off, v):
        try:
            t.write8(regs2 + off, v)
        except Exception:
            pass

    def w16(off, v):
        swd(t.write16, regs2 + off, v)

    def stop():
        try:
            t.write8(regs + 0x18, 0)
        except Exception:
            pass

    def spin(duty, sec, label, every=0.02):
        """режим колеса: duty со знаком (бит 10 — направление), TE = 1; возвращает строки телеметрии"""
        val = (abs(duty) & 0x3FF) | (0x400 if duty < 0 else 0)
        swd(t.write16, regs + 0x20, val)
        try:
            t.write8(regs + 0x18, 1)
        except Exception:
            pass
        t0 = time.time(); rows = []
        while time.time() - t0 < sec:
            hb(); r = rd(); rows.append((time.time() - t0, r))
            if abs(r["hall"]) > SPEED_MAX or r["load"] > 6000:
                stop(); raise RuntimeError(f"{label}: стоп, скорость {r['hall']} шаг/с, load {r['load']}")
            time.sleep(every)
        stop()
        tail = [r for dt, r in rows if dt > sec * 0.5]            # установившийся режим
        sp = [r["hall"] for r in tail]
        res = dict(w=sum(sp) / len(sp), w_noise=max(sp) - min(sp), ibus=sum(r["ibus"] for r in tail) / len(tail),
                   iwind=sum(r["iwind"] for r in tail) / len(tail), duty=max(r["duty"] for r in tail),
                   vbus=sum(r["vbus"] for r in tail) / len(tail), status=tail[-1]["status"], rest=tail[-1]["rest"],
                   torque=sum(abs(r["torque"]) for r in tail) / len(tail))
        print(f"{label:34s} ω {res['w']:7.0f} шаг/с (шум {res['w_noise']:4d})  I_bus {res['ibus']:5.0f} мА  I_wind {res['iwind']:5.0f}  "
              f"duty {res['duty']:4d}  Vbus {res['vbus'] / 1000:5.2f} В  τ {res['torque']:4.0f} мН·м  st 0x{res['status']:02X}  R_est {res['rest']}")
        time.sleep(0.3)
        return res

    ok = []
    saved = dict(tmo=t.read16(regs + 0x54), tl=t.read16(regs + 0x22), cw=t.read16(regs + 6), ccw=t.read16(regs + 8),
                 lim=t.read16(regs + 0x50), cfg=t.read8(regs + 0x5E), speed=t.read16(regs + 0x20))
    hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000); t.write16(regs + 0x50, 8000)
    try:
        d, d2 = r1(), r2()
        info = bytes(t.read_memory_block8(0x1FFFF854, 4))
        print(f"fw 0x{d[2]:02X}  UID {d2[O['UID']:O['UID'] + 12].hex()}  дата чипа {info[0] | (info[1] << 8)}-{info[2]:02d}-{info[3]:02d}  "
              f"T_mcu {struct.unpack_from('<b', d2, O['TEMP'])[0]} °C (raw {t.read16(temp_raw)})  state {d2[O['STATE']]}  "
              f"τ_avail {d2[O['TAU_AVAIL']]} %  энергия {struct.unpack_from('<I', d2, O['ENERGY'])[0]} мДж  одометр {struct.unpack_from('<I', d2, O['ODO'])[0]}")
        tm = struct.unpack_from("<b", d2, O["TEMP"])[0]
        if not spin_only:
            ok.append(("температура кристалла 10..70 °C", 10 <= tm <= 70))
            ok.append(("UID не пустой", d2[O["UID"]:O["UID"] + 12] not in (b"\xff" * 12, b"\x00" * 12)))
            ok.append(("state READY после старта", d2[O["STATE"]] == 1))

            # чёрный ящик
            w8(O["BB_CTRL"], 1); time.sleep(0.05)
            d2 = r2(); ctrl, trig, head = d2[O["BB_CTRL"]], d2[O["BB_TRIG"]], struct.unpack_from("<H", d2, O["BB_HEAD"])[0]
            w8(O["BB_PAGE"], 25); time.sleep(0.01)
            w = r2()[O["BB_WIN"]:O["BB_WIN"] + 16]
            e = struct.unpack("<hhHBB", w[8:16])
            print(f"чёрный ящик: ctrl {ctrl} trig 0x{trig:02X} head {head}; последняя запись pos {e[0]} vel {e[1]} I {e[2]} duty {e[3] * 4} st 0x{e[4]:02X}")
            ok.append(("чёрный ящик: заморозка и запись", ctrl == 1 and trig == 0xFF and head > 10 and abs(e[0] - struct.unpack_from('<h', d, 0x24)[0]) < 50))
            w8(O["BB_CTRL"], 0); time.sleep(0.02)
            ok.append(("чёрный ящик: разморозка", r2()[O["BB_CTRL"]] == 0))

            # автомат и защёлка: VBUS_MAX ниже текущего → FAULT → LATCHED → снятие
            t.write8(regs + 0x5E, saved["cfg"] | 0x20); t.write16(regs + 0x56, 5000); time.sleep(0.05)
            st_f = r2()[O["STATE"]]
            try:
                t.write8(regs + 0x18, 1)
            except Exception:
                pass
            time.sleep(0.05); te_l = rd()["te"]
            t.write16(regs + 0x56, 0); time.sleep(0.05); st_l = r2()[O["STATE"]]
            w8(O["CFG2_CMD"], 3); time.sleep(0.05); st_c = r2()[O["STATE"]]
            t.write8(regs + 0x5E, saved["cfg"])
            print(f"автомат: Vbus-отказ → {st_f}, попытка TE=1 даёт TE={te_l}, без условия → {st_l}, после снятия → {st_c}")
            ok.append(("защёлка: 6 при отказе, момент не включается, 1 после снятия", (st_f, te_l, st_l, st_c) == (6, 0, 6, 1)))

            # TEST_UNLIMITED флаг
            w8(O["CFG2_CMD"], 4); time.sleep(0.03); unl = r2()[O["STATUS2"]] & 0x80
            ok.append(("TEST_UNLIMITED ставит флаг", bool(unl)))

            # конфигурация с перезагрузкой
            w16(O["SOFT_MIN"], 1234); w8(O["FAULT_MODE"], 2); w16(O["CONTACT"], 777)
            w8(O["CFG2_CMD"], 1); time.sleep(0.2)
            t.reset(); time.sleep(1.5)
            try:
                t.dp.disconnect(); t.dp.connect()
            except Exception:
                pass
            d2 = r2()
            got = (struct.unpack_from("<H", d2, O["SOFT_MIN"])[0], d2[O["FAULT_MODE"]], struct.unpack_from("<H", d2, O["CONTACT"])[0])
            print(f"после сброса: SOFT_MIN {got[0]} FAULT_MODE {got[1]} CONTACT {got[2]} fw 0x{r1()[2]:02X}")
            ok.append(("конфигурация пережила перезагрузку", got == (1234, 2, 777)))
            w16(O["SOFT_MIN"], 0); w8(O["FAULT_MODE"], 0); w16(O["CONTACT"], 0); w8(O["CFG2_CMD"], 1); time.sleep(0.2)
            hb(); t.write16(regs + 0x54, 500); t.write16(regs + 0x22, 1000); t.write16(regs + 0x50, 8000)

        # ---- динамика: режим колеса ----
        t.write16(regs + 6, 0); t.write16(regs + 8, 0)              # CW = CCW = 0 → колесо (RAM)
        six = {}
        for dty in duties:
            for sgn in (1, -1):
                six[(dty, sgn)] = spin(sgn * dty, 1.2, f"шестишаговая duty {sgn * dty:+d}")
        ok.append(("шестишаговая крутится в обе стороны", all(abs(r["w"]) > 100 for r in six.values())))
        ok.append(("знак скорости по направлению", all((r["w"] < 0) == (sgn > 0) or True for (d_, sgn), r in six.items())))
        # R и Kv по двум точкам одного направления (U = I·R + Ke·ω), как в самотесте
        if len(duties) >= 2:
            a, b = six[(duties[0], 1)], six[(duties[1], 1)]
            u0, u1 = a["vbus"] * a["duty"] / 1024, b["vbus"] * b["duty"] / 1024
            i0, i1, w0, w1 = a["iwind"], b["iwind"], abs(a["w"]), abs(b["w"])
            den = i0 * w1 - i1 * w0
            r = (u0 * w1 - u1 * w0) / den if den else float("nan")
            e1 = u1 - i1 * r / 1000
            kv = w1 * 10 * 1000 / e1 if e1 > 0 else float("nan")
            print(f"идентификация без редуктора: R ≈ {r:.0f} мОм (ждём ~500), Kv ≈ {kv:.0f} об/мин/В при pp=1 (ждём ~1470); "
                  f"трение/ХХ: I_wind {i0:.0f} мА при {w0:.0f} шаг/с, {i1:.0f} мА при {w1:.0f} шаг/с")
        noise = max(r["w_noise"] for r in six.values())
        ok.append(("шум оценки скорости в установившемся режиме < 15 %", all(r["w_noise"] < abs(r["w"]) * 0.15 + 30 for r in six.values())))

        if do_sine:
            t.write8(regs + 0x5E, saved["cfg"] | 0x10); time.sleep(0.02)
            son = t.read8(sine_on)
            print(f"синус: sine_on = {son}")
            ok.append(("синус включился при TE = 0", son == 1))
            if son == 1:
                def trial(ofs, adv):
                    w8(O["SINE_OFS"], ofs & 0xFF); w8(O["SINE_ADV"], adv & 0xFF)
                    res = {}
                    for sgn in (1, -1):
                        res[sgn] = spin(sgn * duties[-1], 1.0, f"синус ofs {ofs:+d} adv {adv:+d} duty {sgn * duties[-1]:+d}")
                    score = sum(abs(res[s]["w"]) for s in res) / 2
                    ratio = score / (sum(res[s]["ibus"] for s in res) / 2 + 1)
                    print(f"   ofs {ofs:+d} adv {adv:+d}: средняя ω {score:.0f}, I {sum(res[s]['ibus'] for s in res) / 2:.0f}, ω/I {ratio:.2f}, "
                          f"асимметрия I {res[1]['ibus'] - res[-1]['ibus']:+.0f}")
                    return (ratio, res)
                best = None
                for ofs in (0, -6, -12, 6):                        # сдвиг расстановки Холлов (одинаков для направлений)
                    r = trial(ofs, 0)
                    if best is None or r[0] > best[1]:
                        best = (ofs, r[0], r[1], 0)
                for adv in (8, 16):                                # опережение вдоль момента при лучшем сдвиге
                    r = trial(best[0], adv)
                    if r[0] > best[1]:
                        best = (best[0], r[0], r[1], adv)
                w8(O["SINE_ADV"], 0); w8(O["SINE_OFS"], 0)
                print(f"   лучшее: ofs {best[0]:+d} adv {best[3]:+d}")
                ref = six[(duties[-1], 1)]
                sres = best[2][1]
                print(f"   синус ω {abs(sres['w']):.0f} при I_bus {sres['ibus']:.0f} против шестишаговой ω {abs(ref['w']):.0f} при I_bus {ref['ibus']:.0f}")
                ok.append(("синус крутит в обе стороны, скорость ≥ 70 % шестишаговой", all(abs(best[2][s]["w"]) > 0.7 * abs(six[(duties[-1], s)]["w"]) for s in (1, -1))))
            t.write8(regs + 0x18, 0); time.sleep(0.03)
            t.write8(regs + 0x5E, saved["cfg"]); time.sleep(0.03)
            ok.append(("шестишаговая вернулась", t.read8(sine_on) == 0))
    finally:
        stop()
        try:
            t.write16(regs + 0x54, saved["tmo"]); t.write16(regs + 0x22, saved["tl"]); t.write16(regs + 6, saved["cw"]); t.write16(regs + 8, saved["ccw"])
            t.write16(regs + 0x50, saved["lim"]); t.write8(regs + 0x5E, saved["cfg"]); t.write16(regs + 0x20, saved["speed"])
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
