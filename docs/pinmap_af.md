# LCP037BT32EU8: pin alternate functions and board mapping

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

Source: `docs/datasheets/LCP037BT32EU8_datasheet.txt` (tables 3–5, p. 10) [DS],
SVD `docs/datasheets/LCM32F0xx_DFP_0.4.72/pack/SVD/LCM32F037.svd` [DS].
This is groundwork for `docs/pinmap.md`: what follows from the documentation; confirmation from
the stock firmware code is marked separately [MEAS].

## 1. GPIO registers differ from STM32 [DS, SVD]
| Offs. | Register | Note |
|---|---|---|
| 0x00 | MODER | |
| 0x04 | OD / OD_BSRR | open-drain |
| 0x0C / 0x10 | PU / PD | pull-up / pull-down (separate set/reset) |
| 0x14 / 0x18 / 0x1C | IDR / ODR / BSRR | |
| 0x20 | LCKR | |
| **0x24** | **AFRL** (PA0–7) | on STM32, 0x24 is AFRH — don't mix up when reading decompiled code |
| **0x28** | **AFRH** (PA8–15) | |
| 0x2C | BTGLR | toggle |
| 0x30 / 0x34 / 0x38 | DR / CS / OS | drive strength / Schmitt / open-source |

Bases: GPIOA 0x48000000, GPIOB 0x48000200, GPIOF 0x48000A00 [DS, SVD].

## 2. Predriver ↔ TIM1 [DS → inference]
Predriver inputs sit on internal GPIOs (datasheet, p. 9). The only assignment in which
each HIN/LIN pair gets the complementary pair of a single TIM1 channel:

| Leg | HIN (high) | AF | LIN (low) | AF | TIM1 channel |
|---|---|---|---|---|---|
| 1 | PF4 | AF2 = TIM1_CH3 | PB6 | AF4 = TIM1_CH3N | **CH3** |
| 2 | PF3 | AF2 = TIM1_CH2 | PB4 | AF2 = TIM1_CH2N | **CH2** |
| 3 | PB7 | AF5 = TIM1_CH1 | PB3 | AF1 = TIM1_CH1N | **CH1** |

I.e. board "phase 1" = timer CH3, "phase 3" = CH1 [HYP, strong — verify against AFRL values of
GPIOF/GPIOB in the stock firmware]. Alternatives (PF4→CH1N, PF3→CH1N, PB3→CH2N) give no complementary pairs.

**Check LIN polarity before enabling power.** The datasheet says "低电平有效" (active low) for LIN
and right after "0 — off, 1 — on". Contradiction. Check what the stock firmware writes to
TIM1_CCER (CCxNP bits) and TIM1_BDTR (dead-time, OSSR/OSSI) [MEAS — to do].

## 3. Wire S = PA2, UART1 bus [DS + MEAS]
- UART1 = 0x40015000 (SVD; UART0 = 0x40014000). PL011-style registers: DR 0x00, FR 0x18,
  IBRD 0x24, FBRD 0x28, LCR_H 0x2C, CR 0x30, DMACR 0x48. Baud = UARTCLK / (16 · (IBRD + FBRD/64)).
- The bootloader toggles GPIOA+0x24 bits 8–11 between AF1 and AF7 (`docs/native_fw_config.md`).
  +0x24 = AFRL, bits 8–11 = **PA2**; PA2: AF1 = UART1_TX, AF7 = UART1_RX [DS].
  → single-wire half-duplex by AF switching, PA2 = pad **S** [HYP → almost confirmed].
- IBRD=52, FBRD=5 → 115 200 at UARTCLK 96 MHz; 76 800 at 64 MHz (Dclock from the pack); 57 600 at 48 MHz.
  Take the UARTCLK frequency from the clocking setup in the firmware.

## 4. AF tables (datasheet, tables 3–5) [DS]

### Port A
| Pin | AF0 | AF1 | AF2 | AF3 | AF4 | AF5 | AF6 | AF7 |
|---|---|---|---|---|---|---|---|---|
| PA0 | - | UART1_CTS | TIM1_ETR | TIM16_CH1 | UART1_RX | - | CP0_OUT | - |
| PA1 | EVENTOUT | UART1_RTS | TIM16_CH1N | - | UART1_TX | TIM15_CH1N | IR_OUT | - |
| PA2 | TIM15_CH1 | UART1_TX | - | - | - | - | CP1_OUT | UART1_RX |
| PA3 | TIM15_CH2 | UART1_RX | - | - | TIM1_CH1N | SSP0_TXD | CP0_OUT | SSP0_RXD |
| PA4 | SSP0_FSS | UART0_RTS | - | TIM17_CH1 | TIM14_CH1 | WT_BUZ | CP1_OUT | - |
| PA5 | SSP0_SCK | - | - | TIM15_CH1 | TIM1_CH2N | WT_nBUZ | CP2_OUT | TIM3_ETR |
| PA6 | SSP0_RXD | TIM3_CH1 | TIM1_BKIN | CP0_OUT | - | TIM16_CH1 | EVENTOUT | SSP0_TXD |
| PA7 | SSP0_TXD | TIM3_CH2 | TIM1_CH1N | CP1_OUT | TIM14_CH1 | TIM17_CH1 | EVENTOUT | SSP0_RXD |
| PA8 | MCO | UART0_CTS | TIM1_CH1 | EVENTOUT | SSP2_TXD | SSP2_RXD | CP0_OUT | - |
| PA9 | TIM15_CH2N | TIM15_BKIN | UART0_TX | TIM1_CH2 | I2C0_SCL | SSP2_RXD | SSP2_TXD | UART0_RX |
| PA10 | TIM15_CH2 | TIM17_BKIN | UART0_RX | TIM1_CH3 | I2C0_SDA | - | - | - |
| PA11 | EVENTOUT | UART0_CTS | TIM1_CH4 | TIM15_CH1N | - | - | CP0_OUT | I2C0_SCL |
| PA12 | EVENTOUT | UART0_RTS | TIM1_ETR | TIM16_CH1 | TIM15_CH1 | - | - | I2C0_SDA |
| PA13 | SWDIO | IR_OUT | TIM1_CH1N | TIM1_CH4 | UART1_TX | WT_BUZ | CP1_OUT | - |
| PA14 | SWCLK | UART1_TX | - | - | - | WT_nBUZ | CP2_OUT | UART1_RX |
| PA15 | SSP0_FSS | UART1_RX | TIM17_CH1 | EVENTOUT | TIM1_CH2 | TIM16_CH1N | TIM1_BKIN | TIM15_BKIN |

### Port B (PB3–PB7 — internal, predriver inputs)
| Pin | AF0 | AF1 | AF2 | AF3 | AF4 | AF5 | AF6 | AF7 |
|---|---|---|---|---|---|---|---|---|
| PB0 | EVENTOUT | TIM3_CH3 | TIM1_CH2N | - | UART0_TX | - | UART0_RX | - |
| PB1 | TIM14_CH1 | TIM3_CH4 | TIM1_CH3N | - | UART1_RTS | UART0_RX | - | UART0_TX |
| PB3 (LIN3) | SSP0_SCK | TIM1_CH1N | TIM1_CH2N | TIM16_CH1 | - | - | CP1_OUT | - |
| PB4 (LIN2) | SSP0_RXD | TIM3_CH1 | TIM1_CH2N | UART0_RX | - | TIM17_BKIN | - | SSP0_TXD |
| PB5 | SSP0_TXD | TIM3_CH2 | TIM16_BKIN | TIM1_CH3 | UART0_TX | UART1_CTS | CP2_OUT | SSP0_RXD |
| PB6 (LIN1) | UART0_TX | I2C0_SCL | TIM16_CH1N | TIM15_CH2 | TIM1_CH3N | - | - | UART0_RX |
| PB7 (HIN3) | UART0_RX | I2C0_SDA | TIM17_CH1N | TIM1_CH2N | UART0_TX | TIM1_CH1 | TIM17_CH1 | SSP2_FSS |

### Port F (PF2 = nRST, PF3/PF4 — internal, predriver inputs)
| Pin | AF0 | AF1 | AF2 | AF3 | AF4 | AF5 | AF6 | AF7 |
|---|---|---|---|---|---|---|---|---|
| PF0 | SSP2_TXD | I2C0_SDA | TIM1_CH3 | TIM15_CH1 | UART0_TX | - | - | SSP2_RXD |
| PF1 | SSP2_RXD | I2C0_SCL | TIM1_CH2 | TIM15_CH1N | UART0_RX | - | - | SSP2_TXD |
| PF2 | - | - | TIM1_CH1 | TIM3_CH3 | TIM15_CH1 | TIM16_CH1N | - | - |
| PF3 (HIN2) | SSP2_SCK | TIM1_CH1N | TIM1_CH2 | TIM3_CH2 | TIM15_CH2 | TIM16_CH1 | TIM17_CH1N | TIM3_ETR |
| PF4 (HIN1) | SSP2_FSS | TIM1_CH1N | TIM1_CH3 | TIM3_CH1 | - | - | - | - |
| PF6 | SSP2_FSS | TIM1_CH2N | TIM1_BKIN | - | UART1_TX | TIM15_BKIN | EVENTOUT | - |
| PF7 | SSP2_SCK | TIM1_CH1N | TIM3_CH4 | - | UART1_RX | - | - | - |

Empty cells in the datasheet text (pdftotext drops columns) are marked "-"; when in doubt, check the PDF.

## 5. What else to take from the datasheet for pinmap.md
- ADC channels (ADx in pin names, e.g. PA10 = AD15) → potentiometer, shunt (via op-amp OP0/OP1/OP2),
  Vbus divider, Halls (more likely digital inputs / EXTI or TIM3 XOR input).
- Comparators CP0–CP2 and their inputs CPPxx/CPNxx → hardware overcurrent protection.
