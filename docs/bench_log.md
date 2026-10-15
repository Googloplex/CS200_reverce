# Bench test log (servo_01, 24 V, bench PSU; chronological)

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

All numbers are [MEAS] on servo_01 unless stated otherwise. Register reference — `docs/registers_v1.md`, motor — `docs/motor.md`.

## 2026-10-15, clone (v0) and v1.1
- Stock clone flashed, A/B against stock firmware on the same servo: steps at TL 150/300/600 matched within 20 ms and ±3 units
  (`docs/clone_ab_test.md`, logs `dumps/servo_01/tests/`).
- v1.1: stock controller with no regression; v1 with trapezoidal profile 3000 units/s, 50000 units/s² — same accuracy ±6 units, peak current 0.6 A instead of 2.5 A.
  Bus timeout, Vbus max/min, IWDG trip. Online R_EST 503 mΩ.
- Rotor speed ∝ voltage: ~1025 steps/s at 7 V, ~2090 at 14 V (TL 300/600). Hall steps per ADC unit ≈ 0.22 (including backlash).

## v1.2 — ADC synchronous with PWM, hardware limit
- Trigger from TIM1_OC4 requires CC4E. ADC watchdog → OCREF_CLR: at TL 600 peak Load 1100 → 800–870, average current 1.1 → 0.6 A;
  cut takes several periods (shunt is filtered). Wrong window/level combination — runaway (aborted at 6.4 A).
- The erased journal byte 0x17 enabled a 0.2 A limit — later found: LIMIT_HW overlapped R_REF (moved to 0x32).

## v1.3–v1.4 — impedance and torque in amps (`tools/imp_test.py`)
- The first runs effectively ran at duty 300: stock Torque Limit = 300 from the journal clipped the impedance output.
- The speed estimate held its old value for up to 100 ms after stopping → at Kd ≥ 0.15 oscillation at full duty, 16 A on the bus
  ("short circuit" on a 4 A bench PSU). Fixed by bounding with 1/(time since edge).
- Full duty from standstill without a ceiling: 14.8 A, Vbus down to 7.9 V, 10 A bench PSU trips its protection, servo loses power, SWD drops.
- Model-based current ceiling (R, Ke·ω, Vbus): step of 817 units at tmax 1000 under 8 A — 14 600 units/s, 0.12 s, overshoot 2–5 units, bus current ≤ 3.5 A,
  Vbus ≥ 21.7 V; under 5 A — 10 600 units/s, 0.14 s. Kd: 0.02 — overshoot 84, does not settle; 0.06 — 3–6 units, 0.12 s; 0.15 — 0 units, 0.27 s.
- Measured-current limiter: R_EST drifted 0.5 → 0.18 Ω during fast transients (filtered current lags duty) — model uses R_REF,
  estimate only at steady duty; frozen ceiling below duty 80 — deadlock, fixed.
- Execution at host time (+500 ms) — started after 0.54 s. ff ±600 mA at rest not visible (14 duty versus friction).

## v1.5 without gearbox (`tools/nogear_test.py`, `tools/lockrotor_test.py`), 14/14
- Identity record: UID 37323237343439372e0d98d1, chip date 2024-11-10, T_mcu 24 °C (raw 2000–2004). Flight recorder, latch (1 → 6 → 1),
  TEST_UNLIMITED, config after reset — work.
- Wheel, six-step: duty 150 → 440 steps/s at 200 mA; 300 → 915 steps/s at 450 mA; Kv ≈ 1475 at pp = 1.
- Speed from 6 edges: noise 60–170 → 1–10 steps/s.
- Sinusoidal: 890–895 steps/s at 487/511 mA versus 915 at 450 for six-step; tuned SINE_OFS +6, SINE_ADV +8 (ofs −12 — current +40 %).
- Locked shaft: duty ±100, 2.32 V, 4.6 A → R = 504–505 mΩ (both directions); duty 120 cannot be held by hand.

## v1.5 with gearbox (after review: `tools/imp_test.py`, `feat_test.py`, `feat2_test.py`, `pos_test.py`)
- Stock in integers versus old A/B logs (TL 300, t90/settling ±15 units): 1200→2900 379/425 → 382/432 ms, 2900→2000 216/237 → 214/242,
  2000→1200 189/211 → 217/244 (mechanics after rebuild).
- imp_test = v1.4 reference (kd 60: 0 overshoot, 0.15 s; 8 A: 12 100 units/s, 3.1 A, 0.16 s). feat_test 10/10 (interpolation over 600 ms with S-curve,
  joining without a jerk, limits and spring, stall → flag/fault, timeout → hold, coast). feat2_test: counters, flight recorder,
  no false contacts, latch, config after reset, sinusoidal on steps 97 % speed.
- Self-test: direction −1, steps/unit 195–206, Kv 1466–1481, R from journal, friction 0.85–0.89 A + 0.32–0.36 mA/(step/s), breakaway duty 54–57,
  backlash on reversal 41–43 steps, return to the start point.
- The gearbox came apart twice; when slipping the rotor ran at 950 steps/s with the output stationary. After reassembly: backlash on reversal
  38–48 Hall steps in both directions at points 1500/2050/2600, without reversal 3–5.

## Bench lessons
- Bench PSU: any test with motion — only with LIMIT_CURRENT ≤ 8 A; stock Torque Limit 300 in the servo_01 journal clips all loops — set 1000 in RAM.
- ST-Link: a hung pyOCD flash (probe busy with another session), once the process is killed, drops the ST-Link off USB until replugged;
  run flash.sh and tests via `timeout`.
- Register writes over SWD: the firmware may overwrite a byte before the ST-Link verify — catch the exception and check by reading back.
- The impedance command stays active (mode ≠ 0) until a mode = 0 command — otherwise wheel/stock mode gets no control.
