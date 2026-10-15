# servo_01

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

- Not golden — OK to modify (2026-10-15).
- **Potentiometer removed** [MEAS] → ADC15 (PA10) is floating, Present Position (reg. 0x24) = garbage (read 442).
  Stock position loop without feedback: with Torque Enable the motor will spin continuously. Do not enable torque.
- 2026-10-15, over SWD (attach, core not halted): Torque Enable = 0, Goal = 0, Present Voltage = 207 (20.7 V) with Max Voltage 14.0 V.
- 2026-10-15 ~06:55: **flashed with the clone** `fw/` (commit 0370e40) into the 0x08001000 region; bootloader and journal (seq 14) untouched,
  Flash before flashing matched the morning dump. Rollback — `fw/restore_native.sh`. The clone starts and runs (`tools/verify_clone.py`).
- 2026-10-15 ~07:05: motor run on the clone (`tools/spin_clone.py 3 150 2100`): Load 142 → 184–219, V 20.6 → 20.5, normal shutdown.
