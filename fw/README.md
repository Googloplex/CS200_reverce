# fw/ — GCC clone of the Kpower HB200T stock firmware

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

Application for LCM32F037, linked at **0x08001000**: the stock Kpower bootloader (0x08000000–0x08000FFF)
stays in place, forwards interrupts through our vector table and can upload the application over wire S.
The settings journal (0x08007C00–0x08007FFF) is used in the same format as in the stock firmware.

Status (2026-10-15): **v1.5 (version 0x25) flashed into servo_01 and bench-tested** with and without the gearbox
(`docs/bench_log.md`). The stock clone is verified 1:1 against stock (`docs/clone_ab_test.md`); on top of it — custom loops and features,
register and behaviour reference — `docs/registers_v1.md`. No soft-float in the code (Flash 21.8 of 26.5 KB, RAM 2.6 KB).
Not verified: bidirectional bus with a PC (adapter), CFG_ILOOP, direct torque under load, 48 V.

## Build and flash
```
./build.sh                 # Git Bash: CMake + Ninja from ESP-IDF tools, GCC 12.2 from Arm GNU Toolchain
./flash.sh                 # pyOCD + Flash algorithm from the Keil pack, application area only (512 B sectors)
./restore_native.sh        # restore the stock application from dumps/servo_01/flash.bin
```
The servo is powered from a bench PSU at 12–24 V with a ~100 mA limit, ST-Link on the SWD header.

## Files
| File | What | Where in stock (`re/decomp_named.c`) |
|---|---|---|
| `ld/app.ld`, `src/startup.c` | vector table at 0x08001000, start after the bootloader (SysTick/NVIC reset, .data copy) | — |
| `src/system.c` | clocking: 16 MHz crystal → PLL 72 MHz, LVR, delay, PF0/PF1 | FUN_08002a4c, 080027cc, 080023e4 |
| `src/lcm32.h` | registers per SVD (GPIO AFRL at 0x24!, PL011 UART, DW DMAC, ADC, TIM, FLASH) | — |
| `src/board.c` | peripheral clocks, board GPIO, Hall EXTI, NVIC/SysTick | FUN_08003f78, 0800346c, 080038a8 |
| `src/regs.c` | register table, factory defaults, calibration, Flash journal; v1.5 configuration (sector 0x08007A00) | FUN_080045dc, 08003a28, 08003908, 08004030, 080048f8 |
| `src/dxl.c` | UART1+DMA, receive state machine, Dynamixel 1.0 (PING/READ/WRITE/REG_WRITE/ACTION/RESET/0x64/SYNC_WRITE) + custom 0x84 SYNC_READ, 0x86 TIME_SYNC, 0x87 SET_ID_BY_UID, 0x88 UID_SCAN; reply delayed by TIM6 | FUN_080044cc, 0800333c, 080042fc, 0800338c, 08003e50, 080049c8 |
| `src/sense.c` | ADC synchronous with PWM (4 channels via DMA), position/current/Vbus, hardware limit (ADC watchdog), telemetry, energy, torque, flight recorder, die temperature, feedback PWM on PA0 (TIM16) | FUN_08003220, 080041b0, 080025a0, 08004758 |
| `src/motor.c` | TIM1 (23.4 kHz, dead-time 0x2F), six-step and sinusoidal commutation, Halls as a sensor, loops (stock, v1, impedance with interpolation), current/rpm/I²t ceilings, faults and state machine, self-test, PWM input (TIM15) | FUN_080047c8, 08003754, 080036dc, 08003b10, 08003eec, IRQ20 |
| `src/main.c` | init order and main loop | FUN_080035e8 |

## What is known about the stock logic (brief)
- **Bus:** 2 000 000 / (BAUD+1) baud (BAUD=16 → 117 647), ID from register 3, reply per Status Return Level,
  reply delay in 2 µs ticks. `FD FD …` packets (bootloader protocol) are skipped by the application.
- **Goal Position (0x1E)** — in pulse microseconds 900..2100, not 0..1023. Conversion to ADC units:
  `2048 ± (|goal−1500|/600) · (K/460) · 4095`, K = registers 0x4E/0x4F (0 → travel 0x4A, i.e. 120).
- **Present Position (0x24)** = `0xFFF − IIR(potentiometer ADC)/8 + offset(0x14)`.
- **Present Load (0x28)** = current ADC × 3.2328 (≈ mA); > 9500 for longer than 1 s → Torque Limit = 0.
- **Present Voltage (0x2A)** = calibrated voltage in 0.1 V, saturates at 25.5 V due to the u8 format;
  the full value for a 48 V bus is available in `VBUS_MV` (0x68, mV). Temperature is not measured in the stock field.
- **Loop:** profile with a linearly ramping "velocity" `vel += speed / speed_k`, then P controller
  `out = err · 100 / slope` (+ I·0.01 after a new goal), margin 1/5, Punch — minimum output,
  Torque Limit clamp, direction → Hall code inversion.
- **Commutation:** six-step, one phase PWM (high) + complementary low, second phase to ground, third floating.
  At rest all CCR = 0 → all low-side switches on (dynamic brake).
- **PWM input (PA3):** 900..2100 µs → Goal Position, enables Torque Enable; a jump > 50 µs resets the integrator.

## Differences from stock (deliberate)
1. READ: length bounded by the table size (stock has no bounds).
2. SYNC_WRITE: loop bound from LEN (in stock the bound is computed wrong: `pkt[3] − pkt[6] + 1`).
3. Receive: packet buffer overflow protection.
4. Clocking is configured once (stock calls SystemInit twice).
5. TIM16_CH1 (feedback output on PA0) is enabled explicitly — in stock the OutputState field is uninitialized [HYP].
6. Uninitialized struct fields of the stock firmware (TIM1 BDTR OSSI bit, TIM16 BDTR) reproduced as 0x042F / 0x8000.
7. PA2 (bus) with internal pull-up (stock has none): the line does not sit at zero without a master and works with an adapter through a diode.

## "1:1" verification
1. Flash servo_01, connect the bus: PING ID 1 → reply; READ 0x00..0x80 → compare with journal/stock.
2. Stock servo and clone on one bus with different IDs: identical WRITE Goal → compare Present Position logs.
3. PWM input 1000/1500/2000 µs → position as in stock (1190 / ~2060 / 2936 per journal).
4. Read Present Load/Voltage at idle — same order of magnitude as in the journal (42..87 / 10.6..13.9 V).

## Tests (all over SWD, `py -3.11`)
| Script | What it checks |
|---|---|
| `tools/verify_clone.py` | peripherals after flashing (clocks, timers, UART/DMA, ADC, journal) |
| `tools/pos_test.py` | loop steps by Goal (stock/v1, `--native` for stock), CSV log |
| `tools/imp_test.py` | impedance: hold, Kd sweep, full duty under the current ceiling, 1st-order hold, host time, ff |
| `tools/feat_test.py` | interpolation and splicing, soft limits, stall detection, hold on timeout, coast |
| `tools/feat2_test.py` | identity record, counters, flight recorder, contact, self-test, latch, config with reboot, sinusoidal |
| `tools/nogear_test.py` | bench without gearbox: statics + wheel, six-step vs. sinusoidal, SINE_OFS/ADV tuning |
| `tools/lockrotor_test.py` | winding R with locked shaft |

## Next
- Position sensor layer: potentiometer / analog encoder / MT6701 and AS5600 over I2C (PA10 + PA0), filter with 4095→0 wrap,
  turn counter, backlash compensation from two sensors (Halls + encoder).
- 360°: remove the 900..2100 µs limit in the stock loop (impedance already has none), circular error is available (CFG 0x04).
- 48 V: Vbus divider, rpm limit, board replacements — after the hardware.
