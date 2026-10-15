#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dump_one.py — read-only дамп одной сервы Kpower HB200T (LCM32F037) по SWD + разбор журнала.

  py -3.11 tools/dump_one.py servo_02                 # дамп + разбор → dumps/servo_02/
  py -3.11 tools/dump_one.py --parse-only servo_01    # только разобрать уже снятый дамп
  py -3.11 tools/dump_one.py --table                  # сводная таблица по dumps/*/ → dumps/summary.md

В целевой чип ничего не пишет (только чтение памяти под halt). Существующую папку дампа
не перезаписывает — для повторного снятия указать --out. Формат журнала и таблица регистров —
docs/native_fw_config.md; карта памяти — docs/memory_map.md.

Нужен Python с pyOCD (py -3.11), пробник ST-Link/CMSIS-DAP, серва запитана от ЛБП (12–24 В, ~100 мА).
"""
import argparse
import datetime
import hashlib
import json
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DUMPS = ROOT / "dumps"

# (файл, адрес, размер) — только заведомо читаемые области (docs/memory_map.md)
REGIONS = [
    ("flash.bin", 0x08000000, 0x8000),
    ("option_bytes_1FFFF800.bin", 0x1FFFF800, 0x200),
    ("sysmem_bootloader_1FFFF000.bin", 0x1FFFF000, 0x800),
]
CPUID_ADDR = 0xE000ED00
FLASH_BASE = 0x08000000
APP_OFF = 0x1000            # загрузчик Kpower 0x0000–0x0FFF, приложение с 0x1000
JOURNAL_OFF, SLOT, NSLOTS = 0x7C00, 128, 8

# Таблица регистров (Dynamixel AX-12 + свои Kpower): (адрес, имя, размер)
REGS = [
    (0x00, "model", 2), (0x02, "fw_ver", 1), (0x03, "id", 1), (0x04, "baud", 1),
    (0x05, "return_delay", 1), (0x06, "cw_limit", 2), (0x08, "ccw_limit", 2),
    (0x0B, "temp_limit", 1), (0x0C, "min_voltage", 1), (0x0D, "max_voltage", 1),
    (0x0E, "max_torque", 2), (0x10, "status_return", 1), (0x11, "alarm_led", 1),
    (0x12, "alarm_shutdown", 1), (0x18, "torque_enable", 1),
    (0x1A, "compl_margin_cw", 1), (0x1B, "compl_margin_ccw", 1),
    (0x1C, "compl_slope_cw", 1), (0x1D, "compl_slope_ccw", 1),
    (0x1E, "goal_position", 2), (0x22, "torque_limit", 2), (0x24, "present_position", 2),
    (0x28, "present_load", 2), (0x2A, "present_voltage", 1), (0x2B, "present_temp", 1),
    (0x2F, "lock", 1), (0x33, "reg_33", 1),
    (0x40, "pwm_min_us", 2), (0x42, "pwm_center_us", 2), (0x44, "pwm_max_us", 2),
    (0x46, "reg_46", 2), (0x48, "reg_48", 2), (0x4A, "travel_deg", 2),
    (0x4C, "reg_4c", 2), (0x4E, "reg_4e", 2), (0x5A, "reg_5a", 2),
]


def sha256(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


# ---------- разбор ----------

def parse_record(r: bytes):
    if len(r) != SLOT or r == b"\xff" * SLOT:
        return None
    rec = {
        "seq": struct.unpack_from("<i", r, 0x78)[0],
        "flag_z": r[0x7D] == 0x5A,
        "chk_ok": r[0x7E] == ((~sum(r[:0x7E])) & 0xFF),
        "marker_ok": r[0x7F] == 0x4B,
    }
    for off, name, size in REGS:
        rec[name] = r[off] if size == 1 else struct.unpack_from("<H", r, off)[0]
    return rec


def parse_journal(flash: bytes):
    recs = []
    for i in range(NSLOTS):
        rec = parse_record(flash[JOURNAL_OFF + i * SLOT: JOURNAL_OFF + (i + 1) * SLOT])
        if rec:
            rec["slot"] = i
            recs.append(rec)
    recs.sort(key=lambda x: x["seq"])
    return recs


def parse_image(flash: bytes):
    body = flash[:JOURNAL_OFF].rstrip(b"\xff")
    code_end = len(body)
    sp_bl, rst_bl = struct.unpack_from("<II", flash, 0)
    sp_app, rst_app = struct.unpack_from("<II", flash, APP_OFF)
    return {
        "code_end": FLASH_BASE + code_end,
        "bl_sp": sp_bl, "bl_reset": rst_bl,
        "app_sp": sp_app, "app_reset": rst_app,
        "bl_sha256": sha256(flash[:APP_OFF]),
        "app_sha256": sha256(flash[APP_OFF:code_end]) if code_end > APP_OFF else None,
    }


def parse_info(ob: bytes | None):
    if not ob:
        return {}
    info = {"rdp_byte": ob[0], "bytes_48_4f": ob[0x48:0x50].hex(" ")}
    y, m, d = struct.unpack_from("<HBB", ob, 0x54)
    if 2000 <= y <= 2100 and 1 <= m <= 12 and 1 <= d <= 31:
        info["date_guess"] = f"{y:04d}-{m:02d}-{d:02d}"   # [ГИП] дата калибровки/выпуска чипа
    return info


def analyze(dirpath: Path):
    flash = (dirpath / "flash.bin").read_bytes()
    obp = dirpath / "option_bytes_1FFFF800.bin"
    ob = obp.read_bytes() if obp.exists() else None
    return {
        "label": dirpath.name,
        "flash_sha256": sha256(flash),
        "image": parse_image(flash),
        "info": parse_info(ob),
        "journal": parse_journal(flash),
    }


def print_report(a: dict):
    im, inf, recs = a["image"], a["info"], a["journal"]
    print(f"[{a['label']}] flash sha256 {a['flash_sha256'][:16]}…  code end 0x{im['code_end']:08X}")
    print(f"  загрузчик SP=0x{im['bl_sp']:08X} Reset=0x{im['bl_reset']:08X} sha {im['bl_sha256'][:8]}; "
          f"приложение SP=0x{im['app_sp']:08X} Reset=0x{im['app_reset']:08X} sha {(im['app_sha256'] or '-')[:8]}")
    if inf:
        print(f"  инфо-зона: RDP-байт 0x{inf['rdp_byte']:02X}, дата? {inf.get('date_guess', '-')}, "
              f"байты 0x48: {inf['bytes_48_4f']}")
    print(f"  журнал: {len(recs)} записей" + (f", seq {recs[0]['seq']}…{recs[-1]['seq']}" if recs else ""))
    for r in recs:
        bad = "" if (r["chk_ok"] and r["marker_ok"]) else "  !! chk/marker"
        print(f"   seq {r['seq']:3d} slot {r['slot']}: id={r['id']} baud={r['baud']} "
              f"goal={r['goal_position']} pres={r['present_position']} load={r['present_load']} "
              f"V={r['present_voltage'] / 10:.1f} T={r['present_temp']} "
              f"pwm={r['pwm_min_us']}/{r['pwm_center_us']}/{r['pwm_max_us']} "
              f"ход={r['travel_deg']} r46/48={r['reg_46']}/{r['reg_48']} r4c={r['reg_4c']} "
              f"r4e=0x{r['reg_4e']:04X} r5a={r['reg_5a']}{'  Z' if r['flag_z'] else ''}{bad}")


# ---------- дамп по SWD ----------

def dump_target(outdir: Path, probe_uid, connect_mode: str):
    import pyocd
    from pyocd.core.helpers import ConnectHelper

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=probe_uid, return_first=True, blocking=False,
        target_override="cortex_m", connect_mode=connect_mode, resume_on_disconnect=True)
    if session is None:
        sys.exit("Пробник не найден (pyocd list)")
    data = {}
    with session:
        t = session.board.target
        t.halt()
        cpuid = t.read32(CPUID_ADDR)
        for name, addr, size in REGIONS:
            data[name] = bytes(t.read_memory_block8(addr, size))
        probe = session.probe.unique_id

    outdir.mkdir(parents=True, exist_ok=False)
    lines = []
    for name, _, _ in REGIONS:
        (outdir / name).write_bytes(data[name])
        lines.append(f"{sha256(data[name])} *{name}")
    (outdir / "sha256.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (outdir / "dump_info.txt").write_text(
        f"date: {datetime.datetime.now().isoformat(timespec='seconds')}\n"
        f"cpuid: 0x{cpuid:08X}\nprobe: {probe}\npyocd: {pyocd.__version__}\n"
        f"connect_mode: {connect_mode}\ntarget_override: cortex_m\n", encoding="utf-8")
    print(f"CPUID 0x{cpuid:08X}, пробник {probe}, дамп в {outdir}")


# ---------- сводная таблица ----------

def table(dirs):
    rows = ["| метка | дата чипа? | записей (seq) | ID/baud | goal→present | V | pwm min/ctr/max | ход | 0x4E | bl / app sha |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for d in dirs:
        a = analyze(d)
        im, inf, recs = a["image"], a["info"], a["journal"]
        last = recs[-1] if recs else None
        rows.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} / {} |".format(
            a["label"], inf.get("date_guess", "-"),
            f"{len(recs)} ({recs[0]['seq']}…{recs[-1]['seq']})" if recs else "0",
            f"{last['id']}/{last['baud']}" if last else "-",
            f"{last['goal_position']}→{last['present_position']}" if last else "-",
            f"{last['present_voltage'] / 10:.1f}" if last else "-",
            f"{last['pwm_min_us']}/{last['pwm_center_us']}/{last['pwm_max_us']}" if last else "-",
            last["travel_deg"] if last else "-",
            f"0x{last['reg_4e']:04X}" if last else "-",
            im["bl_sha256"][:8], (im["app_sha256"] or "-")[:8]))
    return "\n".join(rows) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("label", nargs="?", help="метка сервы, например servo_02")
    ap.add_argument("--out", type=Path, help="папка дампа (по умолчанию dumps/<label>)")
    ap.add_argument("--parse-only", action="store_true", help="не подключаться, разобрать существующий дамп")
    ap.add_argument("--table", action="store_true", help="сводная таблица по всем dumps/*/")
    ap.add_argument("--probe", help="unique ID пробника (если подключено несколько)")
    ap.add_argument("--connect", default="under-reset", choices=["under-reset", "halt", "attach"],
                    help="pyOCD connect_mode (по умолчанию under-reset)")
    args = ap.parse_args()

    if args.table:
        dirs = sorted(p for p in DUMPS.iterdir() if (p / "flash.bin").exists())
        md = table(dirs)
        (DUMPS / "summary.md").write_text(f"# Сводка по сервам ({datetime.date.today()})\n\n{md}", encoding="utf-8")
        print(md, end="")
        return
    if not args.label:
        ap.error("нужна метка сервы или --table")

    outdir = args.out or (DUMPS / args.label)
    if not args.parse_only:
        if outdir.exists():
            sys.exit(f"{outdir} уже существует — не перезаписываю (укажи --out или --parse-only)")
        dump_target(outdir, args.probe, args.connect)

    a = analyze(outdir)
    (outdir / "journal.json").write_text(json.dumps(a, ensure_ascii=False, indent=1), encoding="utf-8")
    print_report(a)


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    main()
