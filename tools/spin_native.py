"""Spin the motor briefly using the NATIVE Kpower firmware over SWD (no flashing).

Pokes the Dynamixel-like control table in RAM while the core runs (attach mode),
mimicking what the WRITE handler does for Goal Position. With the pot removed the
position loop never converges, so the motor just runs until torque is disabled.

usage: py -3.11 tools/spin_native.py [seconds] [torque_limit 0..1023] [goal_us 900..2100]
"""
import sys
import time

from pyocd.core.helpers import ConnectHelper

TABLE = 0x20000326            # control table base in RAM (native fw)
TORQUE_ENABLE = TABLE + 0x18
GOAL = TABLE + 0x1E
TORQUE_LIMIT = TABLE + 0x22
PRESENT_POS = TABLE + 0x24
PRESENT_LOAD = TABLE + 0x28
PRESENT_VOLT = TABLE + 0x2A
ERROR = 0x20000048
GOAL_FLAGS = (0x2000006C, 0x2000002C)   # set to 1 by the WRITE handler on goal change
GOAL_CLEAR = 0x2000001C                 # set to 0 by the WRITE handler on goal change


def status(t):
    return (f"pos={t.read16(PRESENT_POS):4d} load={t.read16(PRESENT_LOAD):5d} "
            f"V={t.read8(PRESENT_VOLT) / 10:4.1f} err=0x{t.read8(ERROR):02x} "
            f"TE={t.read8(TORQUE_ENABLE)} TL={t.read16(TORQUE_LIMIT)}")


def session():
    return ConnectHelper.session_with_chosen_probe(
        target_override='cortex_m', options={'connect_mode': 'attach'})


def torque_off(t):
    t.write16(TORQUE_LIMIT, 0)
    t.write8(TORQUE_ENABLE, 0)


def emergency_off(timeout=15.0):
    """Link lost mid-run: keep reconnecting until torque is confirmed off."""
    end = time.time() + timeout
    while time.time() < end:
        try:
            with session() as s:
                torque_off(s.target)
                time.sleep(0.2)
                print("after (reconnected):", status(s.target))
                return
        except Exception as e:
            print("  reconnect failed:", e)
            time.sleep(0.5)
    print("!!! TORQUE NOT CONFIRMED OFF - CUT POWER !!!")


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    goal = int(sys.argv[3]) if len(sys.argv) > 3 else 2100
    try:
        with session() as s:
            t = s.target
            print("before:", status(t))
            try:
                t.write16(TORQUE_LIMIT, limit)
                t.write16(GOAL, goal)
                for a in GOAL_FLAGS:
                    t.write8(a, 1)
                t.write8(GOAL_CLEAR, 0)
                t.write8(TORQUE_ENABLE, 1)
                end = time.time() + seconds
                while time.time() < end:
                    print("  run:", status(t))
                    time.sleep(0.25)
            finally:
                torque_off(t)
                time.sleep(0.2)
                print("after:", status(t))
    except Exception as e:
        print("!!! link error:", e)
        emergency_off()


if __name__ == '__main__':
    main()
