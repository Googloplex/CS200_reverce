# Clone (fw/) vs. stock firmware on the same servo — servo_01, 2026-10-15

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

Conditions: servo_01 with potentiometer, output unloaded, bench PSU 24 V (servo sensor reads 20.6–20.7 V).
Control and logging over SWD (`tools/pos_test.py`, attach, core not halted): Torque Enable → Goal,
steps 1500 → 1000 → 2000 → 1500 µs, 1.5 s each, position and Present Load sampled every 20 ms.
Stock firmware installed with `fw/restore_native.sh`, clone restored with `fw/flash.sh`; journal stayed at seq 14 throughout.
Logs: `dumps/servo_01/tests/pos_{clone,native}_tl{150,300,600}_*.csv`.

Goal-to-ADC-units conversion matched exactly in both firmwares: 1000 µs → 1231, 1500 → 2048, 2000 → 2915
(registers 0x4E/0x4F = 110/117, 120° travel).

## Steps: time to 50 % / 90 % of travel, final error, hold jitter [MEAS]

| TL | Step | clone t50/t90, ms | stock t50/t90, ms | clone error | stock error | hold (clone / stock) |
|---|---|---|---|---|---|---|
| 150 | 2048→1231 | 186 / 354 | 180 / 346 | +2 | +3 | ±2 / ±2 |
| 150 | 1231→2915 | 461 / 784 | 454 / 745 | −5 | −6 | ±0 / ±1 |
| 150 | 2915→2048 | 263 / 431 | 265 / 402 | +3 | +4 | ±2 / ±1 |
| 300 | 2048→1231 | 116 / 189 | 104 / 184 | +2 | +4 | ±2 / ±3 |
| 300 | 1231→2915 | 240 / 379 | 249 / 384 | −3 | −4 | ±1 / ±0 |
| 300 | 2915→2048 | 145 / 216 | 137 / 217 | +1 | +2 | ±2 / ±2 |
| 600 | 2048→1231 | 69 / 117 | 73 / 100 | 0 | +3 | ±2 / ±3 |
| 600 | 1231→2915 | 143 / 213 | 141 / 226 | −2 | −2 | ±1 / ±1 |
| 600 | 2915→2048 | 99 / 125 | 75 / 131 | −2 | −4 | ±2 / ±1 |

Overshoot 0–6 units for both. Mean Present Load over the first 400 ms of a step: clone 134–371, stock 137–375;
the spread between firmwares is of the same order as between consecutive runs (20 ms sampling, 2 mA/unit).

**Conclusion:** trajectories, end points and currents match within one sampling interval (20 ms) and potentiometer
noise (±3 units). Position loop, profile, commutation and goal conversion are reproduced 1:1.

## Not compared yet
- Bus communication (no UART adapter): protocol verified by code reading only.
- PWM input on PA3.
- Wheel mode, Punch, compliance margin/slope (tests used defaults 6/32).
- Overload behaviour (threshold 9500 units > 1 s) and ID/baud change.

## Observations
- Startup current at TL=600 from 20 ms samples up to ~1080 units (~1.9 A): keep bench PSU limit ≥ 2 A.
- Static error ±5 units at all TL: P controller with a 5-unit deadband (margin) and Punch = 0.
- Stock firmware journal (seq 8–14) showed Present 1190–1214 at goal 1209–1231: records were apparently taken
  before settling or with a different 0x14 offset; on the bench both firmwares reach the goal.
