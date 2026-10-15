Русская версия: [README.ru.md](README.ru.md)

# Reverse engineering the CS200 (Kpower HB200T) servo from downed Shahed and Geran drones

Custom firmware in the stock microcontroller on the stock board: from a Flash dump over SWD to a 1:1 clone and beyond —
torque in amperes, impedance in a single command, sinusoidal commutation, self-test, flight recorder. Not a single resistor
added to the board.

*Reverse engineering of the Kpower HB200T brushless servo salvaged from downed Shahed/Geran drones: SWD dump of the
stock firmware, a bit-exact GCC clone of it running on the original Linsynix LCM32F037 (Cortex-M0, 32 KB flash, 4 KB RAM),
then a new control stack on top — model-based current limiting, MIT-style impedance commands with time-stamped cubic
interpolation, sinusoidal commutation with Hall interpolation, self-identification, flight recorder.*

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

## What the servo is
- 200 kg·cm stall torque, 12–24 V, 15 A, brushless inrunner Ø29×42 mm with Hall sensors and a 4-stage steel
  gearbox ≈ 1:220, aluminium housing. On the UAV it served as a control-surface actuator driven by 50–333 Hz PWM.
- Board: MCU **Linsynix LCP037BT32EU8** (LCM32F037 core, Cortex-M0 72 MHz, 32 KB Flash, 4 KB SRAM, **built-in
  three-phase predriver**), 12 × NCEP6090AGU (60 V / 90 A), two per leg, a single 2.5 mΩ shunt in the bus, XL7005A buck.
- Stock firmware: Dynamixel Protocol 1.0 (AX-12 table plus Kpower fields) over half-duplex UART on the signal wire,
  bootloader with updates over the same wire, settings journal in Flash.

## What it looked like on day one
The on-image labels are in Russian and are first-day hypotheses made before continuity checks and reverse engineering
(hence the question marks); the verified pinout is in [docs/pinmap.md](docs/pinmap.md).

| Board, MCU side | Back side: logic power |
|---|---|
| ![Board: SWD header, potentiometer, G/S/F pads, power pads (labels in Russian)](docs/photos/servo_pinout.png) | ![Back side: XL7005A, SS34, electrolytics, ceramics, TVS, harness (labels in Russian)](docs/photos/servo_pinout_back.png) |

![LCP037BT MCU close-up: column of 5 pads (VDD, GND, SWDIO, SWCLK, nRST), 12 NCEP6090AGU MOSFETs, 2R5m shunt, gate resistors (labels in Russian)](docs/photos/servo_mcu_pinout.png)

## What has been done
| Stage | Result |
|---|---|
| **Opening up** | SWD header found and probed, Flash not protected (RDP 0) — dump of the firmware, bootloader and chip info area ([docs/memory_map.md](docs/memory_map.md)) |
| **Reverse engineering** | Ghidra with the SVD from the Keil pack: clocking, TIM1 → predriver routing, Halls, ADC for potentiometer/current/bus, protocol, journal, position loop — [docs/native_fw_config.md](docs/native_fw_config.md), [docs/pinmap.md](docs/pinmap.md) |
| **Clone** | `fw/` — functional clone of the stock application under GCC/CMake, runs under the stock bootloader. A/B on the same servo: step responses match within the 20 ms sampling and potentiometer noise ([docs/clone_ab_test.md](docs/clone_ab_test.md)) |
| **Custom firmware v1.5** | everything below; reference — [docs/registers_v1.md](docs/registers_v1.md), bench tests — [docs/bench_log.md](docs/bench_log.md) |
| **Motor** | pp = 1, Kv ≈ 1475 rpm/V, R = 505 mΩ, friction, gearbox backlash ~12° on reversal — [docs/motor.md](docs/motor.md) |

### What firmware v1.5 does (Flash 21.8 of 26.5 KB, RAM 2.6 KB, no soft-float)
- **Torque in amperes.** Duty ceiling from the model (I·R + Ke·ω)/Vbus every ms — no shunt filter lag. A 50° step at
  full duty: 0.12 s, overshoot 2–5 units of 4096, bus current ≤ 3.5 A instead of 15 A and the bench PSU tripping.
- **Impedance in one write** (like MIT Mini Cheetah / moteus): p, v, Kp, Kd, τ_ff, tmax, execution time. Clock sync
  over the bus and cubic interpolation of the target by timestamps: the host sends at 50–100 Hz, the servo tracks the target smoothly at 1 kHz, new
  points are spliced in on the fly without a jerk.
- **Sinusoidal commutation** with Hall-interpolated angle and 3rd harmonic, updated in the TIM1 interrupt at 23.4 kHz
  with no divisions (~150 of 3077 cycles). Hall placement offset and phase advance tuned by current asymmetry.
- **Halls as a sensor**: speed over 6 edges (noise 150 → 10 steps/s), counter, odometer.
- **Protections as features, not an emergency stop**: soft limits with a spring, stall and contact detection (rotor stalled while the loop
  was pushing — 2–3 ms, no IMU), failsafe policy (brake / coast / hold position on loss of comms), state
  machine and latch, I²t with "percent of available torque", TEST_UNLIMITED for 10 s with pre-trigger history recording.
- **Self-test in one command**: Hall order, direction, steps per sensor unit, Kv, friction (Coulomb + viscous),
  breakaway duty, backlash on reversal; returns to the starting point.
- **Identity and telemetry**: chip UID and ID assignment by UID without physical access to the servo, die temperature from the factory
  calibration, energy drawn from the bus, packet counters, output torque in mN·m with friction subtracted, 260 ms flight recorder at 5 ms
  with freeze on event, fast telemetry in a single SYNC_READ, persistent config in its own Flash sector.
- Stock compatibility kept: Dynamixel 1.0, the same journal, stock bootloader in place, rollback in one command.

## How it was done

**1. Opening up.** At hand — a box of servos from downed drones; the only documentation was the spec sheet on the Kpower website. Inside, a board
with an unfamiliar Chinese chip, the Linsynix LCP037BT32EU8: Cortex-M0 and a three-phase predriver in one QFN40 package, 12 MOSFETs, two
per leg, a single 2.5 mΩ shunt in the negative bus, a potentiometer on a side gear. Probing: a column of five pads left of the MCU —
VDD, GND, SWDIO, SWCLK, nRST. Servo powered from a bench PSU limited to 100 mA, ST-Link + pyOCD, `cortex_m` blind: CPUID
0x410CC200, Flash protection at level 0. Dump of Flash, bootloader and info area, sha256, one servo set aside untouched.

**2. Documents.** Chip datasheets on LCSC, a Keil pack with SVD and the Flash algorithm, then the Reference Manual from the Linsynix site — in Chinese;
text is extracted with PyMuPDF (`tools/rm_pages.py`), then grep by Chinese characters. Without the RM, a lot had to be found by experiment
(see item 7).

**3. Reverse engineering.** Ghidra, Cortex-M0 Thumb, peripheral map from the SVD via a script (`re/ghidra_scripts`). The registers revealed: 16 MHz crystal →
72 MHz PLL; TIM1 with complementary channels to the predriver's internal inputs; Halls on PA4–PA6 via EXTI; potentiometer, current via an
external amplifier and the bus divider on the ADC via DMA; UART1 on the signal wire with DMA and direction switching. The protocol turned out to be
Dynamixel 1.0 with the AX-12 table and Kpower fields, target in microseconds 900..2100, settings journal as 128-byte records in the last
two sectors, Kpower bootloader in the first 4 KB forwarding interrupts to the application ([docs/native_fw_config.md](docs/native_fw_config.md),
[docs/pinmap.md](docs/pinmap.md)).

**4. Clone.** GCC + CMake, own vector table, linked at 0x08001000 — the stock bootloader stays and can still update over the wire.
Functions were ported address by address 1:1, down to the double call of the clock init and uninitialised struct fields.
First flash — only a peripheral check by a script that reads registers by ELF symbols over SWD; then the motor under a current
limit; then A/B against the stock firmware on the same servo: trajectories matched within sampling ([docs/clone_ab_test.md](docs/clone_ab_test.md)).

**5. Bench over SWD, not over the bus.** The USB-UART adapter for half-duplex never got finished (power jumper, soldering, resistor in TX),
so the whole bench moved to SWD: scripts write registers straight into RAM by ELF symbols, read telemetry 50 times a second, and, so that
the bus timeout does not drop torque, send a "heartbeat". Along the way it turned out that ST-Link complains about write verification when the firmware
overwrites a byte faster than it does, and that a hung pyOCD drops the probe off USB until it is replugged.

**6. Custom firmware on top of the clone.** Halls as a sensor (counter, period, speed), telemetry, protections, controller with trapezoidal profile and PD,
online winding resistance and temperature estimate, IWDG with reboot-loop protection, SYNC_READ with slots.

**7. What the Reference Manual gave.** ADC synchronous with PWM from TIM1_OC4 — worked only with CC4E set, even though the channel is not
routed to a pin. ADC watchdog → OCREF_CLR as a hardware current limit: of the four combinations of window and polarity tried, one gives runaway, another a
dead stop, the working one is window 0..HT with active-low level. The comparator on the current input does not toggle at any threshold — it
remained an open question. The main finding: the shunt signal is filtered on the board, synchronous sampling yields the average bus current, not the current
within the switch window — this defined the whole torque architecture that followed.

**8. Impedance and torque in amperes.** Command p, v, Kp, Kd, τ_ff in one write. The first tests "passed" suspiciously calmly: the stock
Torque Limit = 300 from the journal was clipping the output. Removed it — and a full-duty step from standstill tripped the bench PSU: 24 V / 0.5 Ω ≈ 48 A by
Ohm's law, and even more on reversal while moving. The measured-current limiter cannot react in time (filter), nor can the ADC watchdog. Solution — a duty ceiling from the
model (I·R + Ke·ω)/Vbus every millisecond: current physically cannot exceed the limit, measurement is only a safety net. The same step
under 8 A became faster and more accurate than without the ceiling. Along the way: the Hall speed estimate held its old value for up to 100 ms after
stopping and made the drive oscillate at 16 A; two registers overlapped by a byte; the measured limiter froze the ceiling forever.

**9. Room for features.** The soft-float library took 9.5 KB — the stock firmware computed Present Load in double. Everything moved to integers
(Q8 profiles, microjoules, exact stock fractions) — Flash 25 → 16 KB. The freed space went to: timestamp interpolation, soft limits,
stall and contact detection, failsafe policy, state machine, identity by UID, die temperature (calibration in the chip info area),
counters, flight recorder, persistent config, self-test, sinusoidal commutation.

**10. Review before the bench.** Five packages were written blind (the probe was unavailable), so before flashing — two independent review
passes: 20-odd findings, among them an ADC ring shift when stopping mid channel sequence, latch bypass by a command,
torn command write over the bus, register overlap, heat accumulator overflow. All fixed before the first run.

**11. Bench without the gearbox.** 14 of 14 checks. Sinusoidal worked first time: 97 % of six-step speed at the same duty; from the
current asymmetry in both directions the Hall placement offset (+6° el.) and phase advance (+8°) were found. Speed over 6 edges removed
the noise from uneven Hall placement (150 → 10 steps/s). Resistance — with a locked shaft: 505 mΩ (at duty 120 the shaft can no longer
be held by hand).

**12. Bench with the gearbox.** Stock regression against the old A/B logs, impedance, all features, self-test produced the motor profile. The gearbox twice
came apart — the rotor spun while the output stood still, visible in the Halls before anything else. After reassembly backlash on reversal
was measured: 38–48 Hall steps, about 12° at the output, the same over the whole travel ([docs/motor.md](docs/motor.md)) — the main mechanical
conclusion and the reason why the next step is an encoder on the output.

## What is where
- `docs/` — reverse engineering results and references: [registers_v1.md](docs/registers_v1.md) (registers, bits, instructions, behaviour),
  [bench_log.md](docs/bench_log.md) (all bench tests in order), [motor.md](docs/motor.md), [pinmap.md](docs/pinmap.md),
  [native_fw_config.md](docs/native_fw_config.md), [memory_map.md](docs/memory_map.md), [clone_ab_test.md](docs/clone_ab_test.md), [roadmap.md](docs/roadmap.md).
- `fw/` — firmware: `src/` (board, system, regs, dxl, sense, motor, startup), `ld/app.ld`, `cmake/`, `build.sh`, `flash.sh`, `restore_native.sh`, [README](fw/README.md).
- `tools/` — bench tests and utilities on pyOCD, ESP32-S3 probe.
- `dumps/servo_01/` — dump checksums, decoded journal, step response logs.
- `re/` — Ghidra scripts and the function list of the stock firmware.

## Tools
- `fw/build.sh`, `fw/flash.sh`, `fw/restore_native.sh` — GCC 12 + CMake/Ninja, pyOCD with the Flash algorithm from the Keil pack.
- `tools/` — bench tests over SWD by ELF symbols: `imp_test.py` (impedance, current ceiling), `feat_test.py` / `feat2_test.py`
  (all v1.5 features), `nogear_test.py` (motor without gearbox, sinusoidal vs six-step), `lockrotor_test.py` (winding R),
  `pos_test.py` (step responses, A/B against stock), `dump_one.py` (read-only dump), `dxl_cli.py` / `bus_diag.py` (bus),
  `rm_pages.py` (Reference Manual text with Chinese characters for grep).
- `tools/esp32_dap/` — custom CMSIS-DAP probe on ESP32-S3 (USB), for when no ST-Link is at hand.
- `re/` — Ghidra scripts (SVD → peripheral map, markup, decompilation) and the function list of the stock firmware.

## What's next
Magnetic encoder on the output (MT6701, analog → I2C on free pads) and continuous rotation, backlash compensation
with two sensors, two-way bus with a PC, 48 V, single-shunt FOC after scoping the shunt path — [docs/roadmap.md](docs/roadmap.md),
checklist — [TODO.md](TODO.md).

## Honesty
All numbers are tagged: [MEAS] — measured, [DS] — from a datasheet, [SPEC] — from the Kpower spec sheet, [HYP] — hypothesis. Binary dumps of the stock
firmware and third-party PDFs are not included in the repository (links in `docs/datasheets/README.md`). The project is educational.
