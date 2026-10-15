# LCP037BT32EU8: альтернативные функции выводов и привязка к плате

Источник: `docs/datasheets/LCP037BT32EU8_datasheet.txt` (таблицы 3–5, стр. 10) [ДШ],
SVD `docs/datasheets/LCM32F0xx_DFP_0.4.72/pack/SVD/LCM32F037.svd` [ДШ].
Это заготовка для `docs/pinmap.md`: здесь — что следует из документации; подтверждение по
коду родной прошивки отмечать отдельно [ИЗМ].

## 1. Регистры GPIO отличаются от STM32 [ДШ, SVD]
| Смещ. | Регистр | Примечание |
|---|---|---|
| 0x00 | MODER | |
| 0x04 | OD / OD_BSRR | open-drain |
| 0x0C / 0x10 | PU / PD | подтяжки вверх / вниз (раздельные set/reset) |
| 0x14 / 0x18 / 0x1C | IDR / ODR / BSRR | |
| 0x20 | LCKR | |
| **0x24** | **AFRL** (PA0–7) | у STM32 по 0x24 лежит AFRH — не путать при чтении декомпиляции |
| **0x28** | **AFRH** (PA8–15) | |
| 0x2C | BTGLR | toggle |
| 0x30 / 0x34 / 0x38 | DR / CS / OS | сила выхода / Шмитт / open-source |

Базы: GPIOA 0x48000000, GPIOB 0x48000200, GPIOF 0x48000A00 [ДШ, SVD].

## 2. Предрайвер ↔ TIM1 [ДШ → вывод]
Входы предрайвера сидят на внутренних GPIO (даташит, стр. 9). Единственная раскладка, при которой
каждая пара HIN/LIN получает комплементарную пару одного канала TIM1:

| Плечо | HIN (верх) | AF | LIN (низ) | AF | Канал TIM1 |
|---|---|---|---|---|---|
| 1 | PF4 | AF2 = TIM1_CH3 | PB6 | AF4 = TIM1_CH3N | **CH3** |
| 2 | PF3 | AF2 = TIM1_CH2 | PB4 | AF2 = TIM1_CH2N | **CH2** |
| 3 | PB7 | AF5 = TIM1_CH1 | PB3 | AF1 = TIM1_CH1N | **CH1** |

Т.е. «фаза 1» платы = CH3 таймера, «фаза 3» = CH1 [ГИП, сильная — проверить по значениям AFRL
GPIOF/GPIOB в родной прошивке]. Альтернативы (PF4→CH1N, PF3→CH1N, PB3→CH2N) комплементарных пар не дают.

**Полярность LIN — проверить до включения силы.** Даташит пишет для LIN «低电平有效» (активный низкий)
и тут же «0 — закрыть, 1 — открыть». Противоречие. Смотреть, что ставит родная прошивка в
TIM1_CCER (биты CCxNP) и TIM1_BDTR (dead-time, OSSR/OSSI) [ИЗМ — сделать].

## 3. Провод S = PA2, шина UART1 [ДШ + ИЗМ]
- UART1 = 0x40015000 (SVD; UART0 = 0x40014000). Регистры в стиле PL011: DR 0x00, FR 0x18,
  IBRD 0x24, FBRD 0x28, LCR_H 0x2C, CR 0x30, DMACR 0x48. Baud = UARTCLK / (16 · (IBRD + FBRD/64)).
- Загрузчик переключает GPIOA+0x24 биты 8–11 между AF1 и AF7 (`docs/native_fw_config.md`).
  +0x24 = AFRL, биты 8–11 = **PA2**; PA2: AF1 = UART1_TX, AF7 = UART1_RX [ДШ].
  → полудуплекс на одном проводе переключением AF, PA2 = пад **S** [ГИП → почти подтверждено].
- IBRD=52, FBRD=5 → 115 200 при UARTCLK 96 МГц; 76 800 при 64 МГц (Dclock из пака); 57 600 при 48 МГц.
  Частоту UARTCLK взять из настройки тактирования в прошивке.

## 4. Таблицы AF (даташит, табл. 3–5) [ДШ]

### Порт A
| Пин | AF0 | AF1 | AF2 | AF3 | AF4 | AF5 | AF6 | AF7 |
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

### Порт B (PB3–PB7 — внутренние, входы предрайвера)
| Пин | AF0 | AF1 | AF2 | AF3 | AF4 | AF5 | AF6 | AF7 |
|---|---|---|---|---|---|---|---|---|
| PB0 | EVENTOUT | TIM3_CH3 | TIM1_CH2N | - | UART0_TX | - | UART0_RX | - |
| PB1 | TIM14_CH1 | TIM3_CH4 | TIM1_CH3N | - | UART1_RTS | UART0_RX | - | UART0_TX |
| PB3 (LIN3) | SSP0_SCK | TIM1_CH1N | TIM1_CH2N | TIM16_CH1 | - | - | CP1_OUT | - |
| PB4 (LIN2) | SSP0_RXD | TIM3_CH1 | TIM1_CH2N | UART0_RX | - | TIM17_BKIN | - | SSP0_TXD |
| PB5 | SSP0_TXD | TIM3_CH2 | TIM16_BKIN | TIM1_CH3 | UART0_TX | UART1_CTS | CP2_OUT | SSP0_RXD |
| PB6 (LIN1) | UART0_TX | I2C0_SCL | TIM16_CH1N | TIM15_CH2 | TIM1_CH3N | - | - | UART0_RX |
| PB7 (HIN3) | UART0_RX | I2C0_SDA | TIM17_CH1N | TIM1_CH2N | UART0_TX | TIM1_CH1 | TIM17_CH1 | SSP2_FSS |

### Порт F (PF2 = nRST, PF3/PF4 — внутренние, входы предрайвера)
| Пин | AF0 | AF1 | AF2 | AF3 | AF4 | AF5 | AF6 | AF7 |
|---|---|---|---|---|---|---|---|---|
| PF0 | SSP2_TXD | I2C0_SDA | TIM1_CH3 | TIM15_CH1 | UART0_TX | - | - | SSP2_RXD |
| PF1 | SSP2_RXD | I2C0_SCL | TIM1_CH2 | TIM15_CH1N | UART0_RX | - | - | SSP2_TXD |
| PF2 | - | - | TIM1_CH1 | TIM3_CH3 | TIM15_CH1 | TIM16_CH1N | - | - |
| PF3 (HIN2) | SSP2_SCK | TIM1_CH1N | TIM1_CH2 | TIM3_CH2 | TIM15_CH2 | TIM16_CH1 | TIM17_CH1N | TIM3_ETR |
| PF4 (HIN1) | SSP2_FSS | TIM1_CH1N | TIM1_CH3 | TIM3_CH1 | - | - | - | - |
| PF6 | SSP2_FSS | TIM1_CH2N | TIM1_BKIN | - | UART1_TX | TIM15_BKIN | EVENTOUT | - |
| PF7 | SSP2_SCK | TIM1_CH1N | TIM3_CH4 | - | UART1_RX | - | - | - |

Пустые ячейки в тексте даташита (pdftotext теряет колонки) помечены «-»; при сомнении смотреть PDF.

## 5. Что ещё взять из даташита для pinmap.md
- Каналы ADC (ADx в именах выводов, напр. PA10 = AD15) → потенциометр, шунт (через ОУ OP0/OP1/OP2),
  делитель Vbus, Холлы (скорее цифровые входы / EXTI или TIM3 XOR-вход).
- Компараторы CP0–CP2 и их входы CPPxx/CPNxx → аппаратная защита по току.
