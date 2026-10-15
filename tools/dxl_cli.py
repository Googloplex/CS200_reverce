#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dxl_cli.py — Dynamixel Protocol 1.0 с ПК для серв Kpower HB200T (родная прошивка и клон fw/).

  py -3.11 tools/dxl_cli.py -p COM5 scan                 # поиск ID 0..253
  py -3.11 tools/dxl_cli.py -p COM5 ping 1
  py -3.11 tools/dxl_cli.py -p COM5 read 1 0 128         # вся таблица регистров
  py -3.11 tools/dxl_cli.py -p COM5 write 1 0x1E 1500    # Goal Position (u16, мкс 900..2100)
  py -3.11 tools/dxl_cli.py -p COM5 writeb 1 0x18 1      # Torque Enable (байт)
  py -3.11 tools/dxl_cli.py -p COM5 watch 1              # Present Position/Load/Voltage раз в 100 мс
  py -3.11 tools/dxl_cli.py -p COM5 boot 1               # команда 0x64 — в загрузчик

Скорость по умолчанию 117647 (регистр Baud=16 → 2 000 000/17). Провод S — полудуплекс: нужен адаптер
USB-UART с объединёнными TX/RX (резистор/диод); собственное эхо отфильтровывается.
"""
import argparse, struct, sys, time
try:
    import serial
except ImportError:
    sys.exit("нужен pyserial: py -3.11 -m pip install pyserial")

BROADCAST = 0xFE
INST = {"ping": 1, "read": 2, "write": 3, "reg_write": 4, "action": 5, "reset": 6, "boot": 0x64, "sync_write": 0x83}
ERR_BITS = {0x01: "voltage", 0x02: "angle", 0x04: "overheat", 0x08: "range", 0x10: "checksum", 0x20: "overload", 0x40: "instruction"}


def chk(b):
    return (~sum(b)) & 0xFF


def packet(dev_id, inst, params=b""):
    body = bytes([dev_id, len(params) + 2, inst]) + bytes(params)
    return b"\xff\xff" + body + bytes([chk(body)])


class Bus:
    def __init__(self, port, baud, timeout=0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def xfer(self, dev_id, inst, params=b"", expect_reply=True):
        tx = packet(dev_id, inst, params)
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        self.ser.flush()
        if not expect_reply:
            return None
        deadline = time.time() + 0.1
        buf = b""
        while time.time() < deadline:
            buf += self.ser.read(64)
            if buf.startswith(tx):          # эхо собственной передачи
                buf = buf[len(tx):]
            i = buf.find(b"\xff\xff")
            if i >= 0 and len(buf) >= i + 4:
                ln = buf[i + 3]
                if len(buf) >= i + 4 + ln:
                    frame = buf[i + 2:i + 4 + ln]
                    if chk(frame[:-1]) != frame[-1]:
                        return {"error": "bad checksum", "raw": frame.hex(" ")}
                    err = frame[2]
                    return {"id": frame[0], "err": err, "errs": [n for b, n in ERR_BITS.items() if err & b],
                            "params": frame[3:-1]}
        return None


def fmt_regs(addr, data):
    out = []
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        out.append(f"{addr + i:02X}: " + " ".join(f"{b:02X}" for b in chunk))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("-b", "--baud", type=int, default=117647)
    ap.add_argument("cmd", choices=["scan", "ping", "read", "write", "writeb", "watch", "boot", "reset"])
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()
    bus = Bus(a.port, a.baud)
    ival = lambda s: int(s, 0)

    if a.cmd == "scan":
        for i in range(0, 254):
            r = bus.xfer(i, INST["ping"])
            if r:
                print(f"ID {i}: ответ, ошибки={r['errs']}")
        return
    dev = ival(a.args[0])
    if a.cmd == "ping":
        print(bus.xfer(dev, INST["ping"]))
    elif a.cmd == "read":
        addr, n = ival(a.args[1]), ival(a.args[2])
        r = bus.xfer(dev, INST["read"], bytes([addr, n]))
        if r and "params" in r:
            print(fmt_regs(addr, r["params"]), "\nошибки:", r["errs"])
        else:
            print(r)
    elif a.cmd == "write":
        addr, v = ival(a.args[1]), ival(a.args[2])
        print(bus.xfer(dev, INST["write"], bytes([addr]) + struct.pack("<H", v), dev != BROADCAST))
    elif a.cmd == "writeb":
        addr = ival(a.args[1])
        print(bus.xfer(dev, INST["write"], bytes([addr] + [ival(x) & 0xFF for x in a.args[2:]]), dev != BROADCAST))
    elif a.cmd == "watch":
        try:
            while True:
                r = bus.xfer(dev, INST["read"], bytes([0x24, 8]))
                if r and "params" in r and len(r["params"]) == 8:
                    p = r["params"]
                    pos, spd, load = struct.unpack_from("<hHH", p, 0)
                    print(f"\rpos={pos:5d} load={load:5d} V={p[6] / 10:4.1f} T={p[7]:3d} err={r['errs']}   ", end="")
                else:
                    print("\rнет ответа", end="")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print()
    elif a.cmd == "boot":
        print(bus.xfer(dev, INST["boot"]))
    elif a.cmd == "reset":
        print(bus.xfer(dev, INST["reset"]))


if __name__ == "__main__":
    main()
