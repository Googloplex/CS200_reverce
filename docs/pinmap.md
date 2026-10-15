# Kpower HB200T board pinout (MCU LCP037BT32EU8)

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

Source: reverse engineering of the stock firmware (`re/decomp_named.c`) + AF/analog function table from the LCP037BT32EU8 datasheet
(`docs/datasheets/LCP037BT32EU8_datasheet.txt`, pp. 7–10) + SVD from the Keil pack `LCM32F0xx_DFP 0.4.72`.
[FW] = configured this way in the stock firmware (a fact about the code); the board-level assignment is a hypothesis until probed.

## Power switches (TIM1 → integrated predriver) [FW + DS]
| Predriver input | GPIO | AF | TIM1 signal | Phase |
|---|---|---|---|---|
| HIN1 | PF4 | AF2 | TIM1_CH3 | 1 (U) |
| LIN1 | PB6 | AF4 | TIM1_CH3N | 1 (U) |
| HIN2 | PF3 | AF2 | TIM1_CH2 | 2 (V) |
| LIN2 | PB4 | AF2 | TIM1_CH2N | 2 (V) |
| HIN3 | PB7 | AF5 | TIM1_CH1 | 3 (W) |
| LIN3 | PB3 | AF1 | TIM1_CH1N | 3 (W) |

- TIM1: PSC=2 (÷3), ARR=1023, PWM mode 1, dead time DTG=0x2F (47 clocks) [FW].
  PWM frequency = Fclk/3/1024 → **23.4 kHz at 72 MHz** (20.8 kHz at 64 MHz) [HYP: clock].
- Stock control is **six-step Hall commutation** (EXTI → FUN_08003754), not FOC [HYP, strong].

## Sensors [FW]
| What | GPIO | Channel | Note |
|---|---|---|---|
| Hall A/B/C | PA4 / PA5 / PA6 | EXTI4/5/6 (IRQ7 EXTI4_15) | input with pull-up |
| **Potentiometer (wiper)** | PA10 | ADCIN15 | Present Position = 0xFFF − IIR(1/8) + offset (reg. 0x14–0x15) |
| **Shunt current** | PF6 | ADCIN11 | internal OPA0 is not enabled by the firmware → probably an external op-amp (6-pin SOT) on PF6 [HYP] |
| **Bus voltage** | PA7 | ADCIN7 | reg. 0x2A = ADC × 1.13 / 10; at 24.0 V on the PSU reads 20.7 → stock coefficient is low by ≈1.16 [MEAS] → actual divider ≈ 1:16 [HYP: Vref 3.3 V] |
| ? | PA1 | ADCIN1 | sampled but unused in code (thermistor not populated?) |
| Temperature | — | — | **not measured**: reg. 0x2B is never written |

- ADC: 4 channels (1, 7, 11, 15), DMA → RAM 0x200003A6 (u16 ×4), start — trigger `0x08000000` in the config (probably from TIM1) [FW].
- Overload: Present Load > 9500 for longer than 1 s → Torque Limit = 0 [FW].

## External pads [FW + MEAS]
| Pad | GPIO | Function |
|---|---|---|
| **S** | PA2 | UART1 half-duplex: AF7 = UART1_RX (receive), AF1 = UART1_TX (while transmitting) — Dynamixel 1.0 bus and bootloader |
| S or F? | PA3 | AF0 = TIM15_CH2 (capture) + pull-up — probably the 50–333 Hz PWM input (IRQ20 TIM15) [HYP] |
| F? | PA0 | AF3 = TIM16_CH1 — output (position feedback PWM?) [HYP] |
| SWD | PA13 / PA14 | SWDIO / SWCLK |

## Peripherals (SVD) [DS]
UART1 0x40015000 (IRQ28), TIM1 0x40010000, TIM15 0x4001B000, TIM16 0x4001C000, TIM17 0x4001E000, TIM14 0x4000A000,
TIM6 0x40009000, ADC 0x4001A000, ANACTRL (op-amps/comparators/DAC) 0x40017C00, EXTI 0x40011000, DMAC 0x48004000,
FLASH 0x4001D000, GPIOA 0x48000000 / GPIOB 0x48000200 / GPIOF 0x48000A00 (AFRL +0x24, AFRH +0x28).
IRQs used by the application: 7 EXTI4_15 (Halls), 10 DMAC_CH1_2 (UART), 17 TIM6, 19 TIM14, 20 TIM15.

## ⚠ For 48 V
Vbus divider ≈1:16 (checked against PSU at 24.0 V) → measurement limit ≈53 V at Vref 3.3 V. On 12S (up to 50.4 V) tight, but fits;
check the resistor values and ADC reference before deciding on replacement (Phase 6).

## To verify by probing/measurement
- PA2/PA3 ↔ pad S (one or both, through what), PA0 ↔ F.
- PF6 ↔ output of the 6-pin IC (external shunt op-amp?) and its gain.
- PA1 — where it goes.
- Divider values on PA7.
