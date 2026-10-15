# LCP037BT32EU8 (LCM32F037) memory map

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

Obtained by SWD reads (ST-Link V2 + pyOCD 0.45.1, `-t cortex_m`, `connect_mode=under-reset`), 2026-10-15, servo `servo_01`.
Cross-checked against the Keil pack `LCM32F0xx_DFP 0.4.72` (`docs/datasheets/LCM32F0xx_DFP_0.4.72/`): IROM1 0x08000000/0x8000,
IRAM1 0x20000000/0x1000, algorithm `Flash/LCM32F037_32K.FLM`, SVD `SVD/LCM32F037.svd`, Dclock 64 MHz [DS].
FLM: page = sector = 512 B, 64 sectors [DS]. pyOCD takes the target from the pack: `pyocd <cmd> --pack docs/datasheets/LCM32F0xx_DFP_0.4.72/zip/Keil.LCM32F0xx_DFP.0.4.72.pack -t lcm32f037`
(for flashing in phase 2). Read-only dump + journal parsing — `tools/dump_one.py` (target `cortex_m`; tested on servo_01, sha256 matched).

## Core
- CPUID `0xE000ED00` = **0x410CC200** → Cortex-M0 r0p0 [MEAS]

## Protection
- Whole Flash readable via debug → **RDP level 0** (effectively) [MEAS]
- Byte at 0x1FFFF800 = `0x5A` — probably the RDP field; level encoding unknown [HYP]

## Regions
| Address | Size | What | Confidence |
|---|---|---|---|
| 0x00000000 | — | Flash mirror (vectors match 0x08000000) | [MEAS] |
| 0x08000000–0x08007FFF | 32 KB | main Flash; used up to 0x08007F7F | [MEAS] |
| 0x08008000–… | step 0x1000 | mirror of info block 0x1FFFF000 (repeats at 0x08009000, 0x0800A000) | [MEAS] |
| 0x1FFFEC00 | — | not readable (memory transfer failed) | [MEAS] |
| 0x1FFFF000–0x1FFFF7FF | 2 KB | system bootloader: SP=0x20000620, Reset=0x1FFFF0D5 | [MEAS]/[HYP] |
| 0x1FFFF800–0x1FFFF9FF | 512 B | option bytes / factory info area | [MEAS] readable / [HYP] purpose |
| 0x1FFFFA00–… | — | not readable | [MEAS] |
| 0x20000000 | 4 KB | SRAM; stock firmware initial SP 0x20000E00 | [DS]/[MEAS] |

## From reversing the stock firmware (details — `native_fw_config.md`)
- Flash page = 512 B; Flash controller = 0x4001D000 (STM32-like, keys 0x45670123/0xCDEF89AB) [MEAS]
- 0x08000000–0x08000FFF — Kpower bootloader, 0x08001000 — application, 0x08007C00–0x08007FFF — journal [MEAS]
- Bus UART = 0x40015000; GPIOA = 0x48000000 [MEAS]

## Stock firmware (servo_01)
- Initial SP = 0x20000E00, Reset_Handler = 0x0800019D, NMI = 0x08000661, HardFault = 0x08000625 [MEAS]

## Info area 0x1FFFF800 (first bytes)
```
00: 5A 00 FF FF FA 00 FF FF FA 00 FF FF FF 00 FF FF   — "byte + 00 + FFFF" format, looks like OB with complement
40: 55 AA FF FF 55 AA FF FF 65 D9 A2 01 0D FF FF FF
50: 3E 00 18 00 E8 07 0B 0A 01 00 FF FF 01 00 FF FF   — E8 07 0B 0A = 2024-11-10? (calibration/production date) [HYP]
60: 01 00 FF FF 80 00 FF FF
```
