# Firmware v1.5 (fw/, version 0x25): register and behavior reference

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

Compatible with stock (Dynamixel 1.0, AX-12 table + Kpower fields, see `docs/native_fw_config.md`): addresses 0x00–0x4F and the Flash
journal are the same. Bench test chronology and measurements — `docs/bench_log.md`, motor — `docs/motor.md`.

**Units.** Position — potentiometer ADC units 0..4095 (≈16.8 units/°, [HYP]); speed — units/s or Hall steps/s
(6·pp steps per rotor revolution, pp = 1; 0.206 steps per ADC unit); duty — 0..1023 of Vbus; current — mA; Vbus — mV.

**Where things are stored.**
- 0x00–0x7F — stock table + v1 settings: a bus write to the EEPROM part (0x00–0x17) and to 0x50–0x5F saves the journal
  (0x08007C00, stock format). SWD writes do not touch the journal (symbol `g_save_flag` = 1 — save manually).
- 0x80–0xFF (`regs2`) — RAM. Its settings are saved by CFG2_CMD = 1 to their own sector 0x08007A00 (16 slots × 32 B,
  the record stores the map length — an old record does not overwrite new fields) and loaded at startup.
- Flash writes (journal, config) are performed only with torque off, otherwise deferred: the core stalls during
  programming, and the ISRs are in Flash.

## 1. Settings in the stock table (0x00–0x5F), beyond stock

| Addr | Type | Name | Units | What it does |
|---|---|---|---|---|
| 0x13 | u8 | STEPS_PER_CNT | ×0.001 | Hall steps per ADC unit; servo_01: 206 (measured by self-test on steady-state travel) |
| 0x16 | u16 | R_REF | mΩ | phase-to-phase winding resistance for the model (current ceiling, ff, I²t, T_EST); servo_01: 505 (locked shaft) |
| 0x32 | u8 | LIMIT_HW | ×100 mA | hardware limit of average bus current: ADC watchdog (channel 11, HT) → OCREF_CLR TIM1; 0 = off |
| 0x34 | u16 | KV | rpm/V | 1470–1480 at pp = 1 |
| 0x36 | u8 | PP | | pole pairs: 1 [MEAS, two bench tests] |
| 0x37 | u8 | CTRL_MODE | | loop on Goal: 0 stock, 1 v1 (trapezoidal profile + PD on Halls; with CW = CCW = 0 — closed-loop speed) |
| 0x38 | u16 | PROFILE_VMAX | units/s | v1 profile, 0 = unlimited |
| 0x3A | u16 | PROFILE_ACC | ×100 units/s² | v1 profile, 0 = unlimited |
| 0x3C | u8 | KP | ×0.1 duty/unit | v1 loop (31 ≈ stock) |
| 0x3D | u8 | KD | duty per 100 steps/s | v1 loop |
| 0x3E | u16 | VBUS_MIN | mV | below for 5 ms in a row → torque off, STATUS 0x40 (not earlier than 500 ms after startup) |
| 0x50 | u16 | LIMIT_CURRENT | mA | **torque in amps**: duty ceiling from the model (I·R_REF + Ke·ω)/Vbus every ms; measured current is a backstop (1.5·limit, ≥ 5 ms) |
| 0x52 | u16 | LIMIT_SPEED | steps/s | rotor speed limit, soft rollback |
| 0x54 | u16 | BUS_TIMEOUT | ms | no packets addressed to us for longer → fault per FAULT_MODE, STATUS 0x04 (cleared by the next packet) |
| 0x56 | u16 | VBUS_MAX | mV | above for 5 ms in a row → torque off, STATUS 0x10 |
| 0x58 | u16 | I_CONT | mA | continuous current for I²t: when the budget is exhausted the ceiling is cut to I_CONT (the model one too) |
| 0x5C | u16 | LOAD_OFFSET | Load units | current sensor zero offset, auto-calibrated with torque off |
| 0x5E | u8 | CFG_FLAGS | bits | see §4 |
| 0x5F | u8 | BUS_SLOT | ×10 µs | response slot in SYNC_READ / UID_SCAN (30) |

Stock semantics are preserved: a bus write of Goal Position enables torque and sets Torque Limit = Max Torque;
Present Voltage (0x2A) is computed from the bench calibration and saturates at 25.5 V — for 48 V use VBUS_MV.

## 2. Telemetry in the stock table (0x60–0x77, read-only)

| Addr | Type | Name | Units | Note |
|---|---|---|---|---|
| 0x60 | s16 | HALL_SPEED | steps/s | rotor speed, sign = direction; averaged over 6 edges (one revolution) + bounded by 1/(time since edge) |
| 0x62 | s32 | HALL_COUNT | steps | rotor counter |
| 0x66 | u16 | I_BUS | mA | bus current: (sample − offset) × 6.47; ADC synchronous with PWM (TIM1_OC4); the signal is filtered on the board → average current |
| 0x68 | u16 | VBUS_MV | mV | 13.12 mV/unit (1829 units ↔ 24.0 V) |
| 0x6A | u16 | I_WIND | mA | winding current estimate = I_BUS/duty (below duty 80 = I_BUS); overestimated in transients |
| 0x6C | u8 | STATUS | bits | see §4 |
| 0x6D | u8 | HALL_CODE | | Hall code at the last edge |
| 0x6E | u16 | UPTIME | s | |
| 0x70 | u8 | THERMAL | % | I²t thermal budget: 1600 J, τ 60 s [HYP] |
| 0x71 | s8 | T_EST | °C | 25 + (R_EST/R_REF − 1)/0.00393 |
| 0x72 | u16 | I_WIND_MAX | mA | peak since startup |
| 0x74 | u16 | DUTY | | current duty |
| 0x76 | u16 | R_EST | mΩ | online (Vbus·D − Ke·ω)/I with steady duty ≥ 20 ms and current > 0.8 A |

## 3. regs2 block (0x80–0xFF, RAM; "cfg" = saved by CFG2_CMD = 1)

| Addr | Type | Name | Units | What it does |
|---|---|---|---|---|
| 0x80 | s16 | CMD_POS | units | **impedance command** (write block 0x80..0x8F in a single write): target p |
| 0x82 | s16 | CMD_VEL | units/s | target velocity v: the target moves for 100 ms after the command (first-order hold), damping is relative to v |
| 0x84 | u16 | CMD_KP | ×0.01 duty/unit | 310 ≈ stock |
| 0x86 | u16 | CMD_KD | ×0.001 duty/(units/s) | 60 by default (bench: 20 — overshoot 80 units; 60 — 0–4 units, 0.12–0.15 s; 150 — 0.27 s) |
| 0x88 | s16 | CMD_FF | mA | feedforward torque via the model u = I·R_REF + Ke·ω; with CFG_ILOOP in torque mode it is closed on current |
| 0x8A | u16 | CMD_TMAX | duty | limit on |duty| (0 = 1023) |
| 0x8C | u8 | CMD_MODE | | 0 — old loop on Goal; 1 — impedance (step at time CMD_AT); 2 — impedance with cubic interpolation from the current target and its velocity to (p, v) by time CMD_AT, a new point mid-move is joined without a jerk |
| 0x8D | u16 | CMD_AT | host ms | when to apply (low 16 bits of host clock), 0 = immediately; a late one — immediately |
| 0x8F | u8 | CMD_SEQ | | last byte of the block: change = command fully written |
| 0x90 | s16 | TLM_POS | units | **fast telemetry** — one SYNC_READ 0x90 len 16 for all |
| 0x92 | s16 | TLM_VEL | units/s | from Halls |
| 0x94 | u16 | TLM_IBUS | mA | |
| 0x96 | s8 | TLM_TEMP | °C | winding estimate |
| 0x97 | u8 | TLM_STATUS | | = STATUS |
| 0x98 | u16 | TLM_VBUS | mV | |
| 0x9A | u16 | TLM_DUTY | | |
| 0x9C | u32 | TLM_TIME | ms | host clock after TIME_SYNC, otherwise local |
| 0xA0 | u32 | TIME_OFFSET | ms | host − local |
| 0xA4 | u16 | SOFT_MIN | units | cfg; lower soft limit (0 = none): target is clamped, a spring beyond the limit |
| 0xA6 | u16 | SOFT_MAX | units | cfg; upper soft limit |
| 0xA8 | u8 | SOFT_KP | ×0.1 duty/unit | cfg; spring stiffness beyond the limit (50) |
| 0xA9 | u8 | FAULT_MODE | | cfg; with torque off and on a soft fault: 0 brake (low-side switches), 1 coast (all TIM1 outputs released), 2 hold at the fault point (impedance, tmax = HOLD_TMAX) |
| 0xAA | u16 | STALL_DUTY | duty | cfg; stall detection: |loop output| ≥ this at speed < 100 units/s for longer than STALL_MS (0 = off) |
| 0xAC | u8 | STALL_MS | ms | cfg (50) |
| 0xAD | u8 | STALL_ACT | | cfg; 0 flag, 1 output ×¼ while stalled, 2 fault per FAULT_MODE. The flag is cleared by a new command or by motion |
| 0xAE | u8 | STATUS2 | bits | see §4 |
| 0xAF | u8 | TAU_AVAIL | % | SOA: torque available now — LIMIT_CURRENT decreases linearly toward I_CONT as the thermal budget is consumed |
| 0xB0 | u16 | HOLD_TMAX | duty | cfg; hold limit on fault (300) |
| 0xB2 | s8 | TEMP_MCU | °C | die, via ADC_IN16 once per second, factory calibration 0x1FFFF580 (servo_01: 26.6 °C ↔ 2018, 105 °C ↔ 2544) |
| 0xB3 | u8 | STATE | | state machine: 0 BOOT, 1 READY, 2 ENABLED, 3 HOLD, 4 SELFTEST, 5 FAULT (Vbus/Halls), 6 LATCHED |
| 0xB4 | 12 B | UID | | 96-bit chip UID — copy of 0x1FFFF5B0 ("72274497" + 4 bytes) [HYP: address inferred from the data, verify on a second servo] |
| 0xC0 | u32 | ENERGY_MJ | mJ | drawn from the bus since startup (the unipolar shunt does not see regeneration) |
| 0xC4 | u32 | ODO_STEPS | steps | rotor odometer, absolute (including backlash and dithering in hold) |
| 0xC8 | u16 | PKT_OK | | packets addressed to us with a valid checksum |
| 0xCA | u16 | PKT_ERR | | with an error |
| 0xCC | u16 | FRIC_IC | mA | Coulomb friction without load (self-test); subtracted from TLM_TORQUE |
| 0xCE | u16 | FRIC_KV | µA per step/s | viscous friction (self-test) |
| 0xD0 | u8 | BB_CTRL | | **flight recorder**: write 0 — start, 1 — freeze; read: 1 = frozen. Ring of 52 × 8 B at 5 ms intervals (260 ms); event (Halls, Vbus, timeout, I²t, stall, end of TEST_UNLIMITED) → 10 more records and freeze |
| 0xD1 | u8 | BB_PAGE | | window page 0..25 (2 records each, oldest to newest) |
| 0xD2 | u16 | BB_HEAD | | number of records since startup |
| 0xD4 | u8 | BB_TRIG | | what froze it: STATUS bits, 0x80 stall, 0xFF manual |
| 0xD5 | u8 | CFG2_CMD | | 1 save cfg; 2 self-test (only from rest); 3 clear latch; 4 TEST_UNLIMITED — 10 s without current/speed/I²t ceilings, flag STATUS2 0x80, on expiry the flight recorder freezes |
| 0xD6 | u16 | CONTACT_DECEL | units/s per 1 ms | cfg; contact/impact without an IMU: rotor decelerated along the motion by ≥ this for 2 ms in a row while the loop is pushing → STATUS2 0x20 for 50 ms (0 = off) |
| 0xD7 | s8 | SINE_OFS | °el. | cfg; Hall sensor placement offset relative to the phases for sinusoidal; servo_01: +6 |
| 0xD8 | s16 | TLM_TORQUE | mN·m | output torque: (I_wind − friction)·Kt·N without gearbox efficiency; sign — output direction |
| 0xDA | u16 | N_GEAR | ×0.1 | cfg; gear ratio (2230) [HYP] |
| 0xDC | u16 | KT | µN·m/A | cfg; rotor; 0 = 9549296/KV |
| 0xDE | u8 | ST_RESULT | bits | self-test: 1 Halls, 2 direction as in CFG, 4 R measured (only under load; otherwise R_REF), 8 Kv, 16 steps/unit ±30 %, 0x40 aborted, 0x80 done |
| 0xDF | u8 | ST_FRICTION | duty | rotor breakaway |
| 0xE0 | u16 | ST_R | mΩ | |
| 0xE2 | u16 | ST_KV | rpm/V | |
| 0xE4 | u16 | ST_SPC | ×0.001 | steps/unit on steady-state travel |
| 0xE6 | s8 | SINE_ADV | °el. | cfg; vector phase advance along the torque; servo_01: +8 |
| 0xE7 | s8 | ST_DIR | | −1: Hall counter decreases as position increases (assumed in firmware), +1 the opposite |
| 0xE8 | 16 B | BB_WIN | | flight recorder window: 2 records {pos s16, vel s16, I_bus u16, duty/4 u8, status u8} |
| 0xF8 | u16 | ST_BACKLASH | steps | backlash on reversal: Hall steps until the output moves by 5 units; 0xFFFF — output did not move |

## 4. Bits and codes

- **CFG_FLAGS (0x5E)**: 0x01 IWDG 1 s (with reboot-loop protection); 0x02 Hall failure (code 0/7 three times) → torque off; 0x04 wrap-around
  error 0..4095 (360° sensor, shortest path, in interpolation too); 0x08 invert speed sign; 0x10 **sinusoidal** (see §6, switchable
  only with TE = 0); 0x20 latch hard faults (Vbus, Halls) until CFG2_CMD = 3; 0x40 current loop on top of the model in torque mode (Kp = Kd = 0).
- **STATUS (0x6C / 0x97)**: 0x01 current limit clipping, 0x02 speed limit, 0x04 bus timeout, 0x08 Hall code 0/7, 0x10 Vbus above limit,
  0x20 I²t, 0x40 Vbus below limit, 0x80 IWDG disabled by loop protection.
- **STATUS2 (0xAE)**: 0x01 stall, 0x02 beyond soft limit, 0x04 hold on fault, 0x08 interpolation in progress, 0x10 coast, 0x20 contact,
  0x40 self-test, 0x80 TEST_UNLIMITED.

## 5. Bus instructions beyond stock

| Code | Format | What it does |
|---|---|---|
| 0x84 SYNC_READ | `FF FF FE LEN 0x84 addr len ID1 … IDn CHK` | each servo in the list replies with a status packet of `len` bytes from `addr` after index × BUS_SLOT × 10 µs + Return Delay; 20 servos × 8 B at 1 Mbit ≈ 6 ms |
| 0x86 TIME_SYNC | `FF FF FE 06 0x86 t0 t1 t2 t3 CHK` | broadcast, no reply: host clock (ms) → TIME_OFFSET; then CMD_AT = a single execution time for all actuators on the bus |
| 0x87 SET_ID_BY_UID | `FF FF FE 0F 0x87 uid[12] new_id CHK` | set ID by chip serial number without physical access to the servo; only the matching one replies and saves the journal |
| 0x88 UID_SCAN | `FF FF FE 02 0x88 CHK` | all servos reply with their UID (12 B) in a slot derived from the UID bytes × BUS_SLOT |

Bus speed — stock Baud register: 2 000 000/(Baud+1); Baud = 0 → 2 Mbit (PL011 dividers are computed generically). Bidirectional bus with a PC
not yet tested (the adapter needs a 470 Ω resistor in TX).

## 6. Behavior

- **Loops.** CTRL_MODE 0/1 — on stock Goal (µs 900..2100 → units). The impedance command (CMD_MODE ≠ 0) takes priority:
  `duty = (p_t − pos)·Kp + (v_t − vel)·Kd + ff(mA) + limit spring`, |duty| ≤ TMAX, then stock Torque Limit and the ceilings.
  CMD_MODE 2 moves the target along a Hermite cubic from the current (p, v) to (p, v, at), 50–100 Hz from the host → smooth at 1 kHz.
- **Torque in amps.** LIMIT_CURRENT → duty ceiling from the model every ms (when rotating against the command the ceiling drops to 0 = brake);
  measured current is only a backstop. Full duty from standstill without a ceiling is Vbus/R ≈ 48 A at 24 V: the bench PSU trips its protection.
  I²t: THERMAL % and ceiling reduction toward I_CONT; TAU_AVAIL shows what is left. TEST_UNLIMITED removes everything for 10 s with pre-trigger history recorded.
- **Faults.** Soft (bus timeout, stall with STALL_ACT = 2) → FAULT_MODE: brake / coast / hold. Hard (Vbus with 5 ms
  hysteresis, Halls) → torque off; with CFG 0x20 they latch (STATE 6) until CFG2_CMD = 3. The impedance command does not enable torque bypassing the latch.
  All checks run in one context (TIM14, 1 kHz). Unhandled interrupt / HardFault → reset. IWDG 1 s by flag.
- **Self-test (CFG2_CMD = 2, at rest, position 1000..3000, excursion up to +750/−200 units, no current ceilings in phases 1–5):** +120 duty 150 ms →
  backlash; +150 200 ms → direction and steps/unit from the second half; −300 200 ms and +300 150 ms → R from two points (only if the currents
  differ by ≥ 0.4 A, otherwise R_REF), Kv from back-EMF, friction Coulomb + viscous; then duty 1/ms until breakaway; return by impedance under the ceiling.
- **Sinusoidal (CFG 0x10).** Three-phase PWM, angle = Hall sector center ± step per PWM period since the last edge (like hall-FOC), 3rd harmonic 1/6
  (line voltage = Vbus·duty, as in six-step), updated in the TIM1 interrupt at 23.4 kHz without divisions (~150 cycles out of 3077).
  SINE_OFS — Hall sensor placement offset (absolute), SINE_ADV — phase advance along the torque. Bench: 97 % of six-step speed at the same duty, +10 % current.
- **Sensors.** Position: potentiometer IIR as in stock. Die temperature once per second by swapping the ADC sequence (DMA is restarted
  after each ADC stop — otherwise the channel ring shifts). Hall speed: average over 6 edges, bounces closer than 60 µs and repeated codes
  are ignored.

## 7. What is known about the hardware from the RM and bench
- ADC trigger: ADC_EXTCFG group 3 = TIM1_OC4 (OC4 in PWM2, CCR4 = sampling point); event only with CC4E = 1 [MEAS].
- ADC watchdog (AWDEN, AWDSGL, channel 11, HT) → OCREF_CLR TIM1 via SysCtrl_EDU_CFG4 bit 5; watchdog output is an "inside window" level,
  active low (TIM1_SP[5] = 1); the source is enabled only together with the limit [MEAS]. The cut is not cycle-by-cycle — the shunt is filtered.
- ACMP2 on CPP21 = PF6 does not toggle at any DAC0 threshold (on CPP20/23 it toggles) — the comparator is not used.
- Temperature sensor ADC_IN16 only in single-channel mode; calibration 0x1FFFF580 (T1, T2, samples at 3.3 V) [MEAS].
- UID: 12 bytes at 0x1FFFF5B0 [HYP], chip date at 0x1FFFF854 (servo_01: 2024-11-10).
- Current input PF6 = pin 17 of QFN40 (AD11/OP0N3/CPP21); the RC filter and amplifier (6-pin SOT?) on the board are not identified — FOC needs an oscilloscope.

## 8. Verification status (servo_01, 2026-10-15) and open items
Bench-verified: everything in §1–§6 except: bidirectional bus (SYNC_READ/WRITE, TIME_SYNC, 0x87/0x88 over the wire), CFG_ILOOP, ff under load,
hardware limit at 48 V, coast as MOE=0+OSSI (currently by clearing CCxE, works). Details — `docs/bench_log.md`.
Open hardware items: gearbox backlash on reversal 38–48 Hall steps (~12° at the output) — see `docs/motor.md`; what pad F outputs; cycle-by-cycle
current cut (ACMP2); shunt RC.
