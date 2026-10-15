# Kpower stock firmware (servo_01)

Evidence tags: [MEAS] measured, [DS] datasheet, [SPEC] vendor spec sheet, [HYP] hypothesis.

Source: `dumps/servo_01/flash.bin`, Ghidra 12.1.4 (`re/decomp.c`, `re/functions.txt`, scripts in `re/ghidra_scripts/`).
servo_01 is not the golden one, OK to modify.

## 1. Flash layout [MEAS — from code]
| Address | What |
|---|---|
| 0x08000000–0x08000FFF | **Kpower bootloader** (4 KB): application flashing over UART |
| 0x08001000–0x08004E53 | **Application** (vectors: SP=0x20000A18, Reset=0x0800119D); at the end — softfloat (`__aeabi_f*`) and scatter-load data |
| 0x08004E54–0x08007BFF | empty |
| 0x08007C00–0x08007DFF | journal, sector 0 (4 records × 128 B) |
| 0x08007E00–0x08007FFF | journal, sector 1 (4 records × 128 B) |

- **Flash page = 512 B** (erase in 0x200 steps) [MEAS].
- **Flash controller = 0x4001D000**, STM32-like: KEYR +0x04 (keys 0x45670123/0xCDEF89AB), CR +0x10 (PG=0x01, PER=0x02, STRT=0x40, LOCK=0x80), AR +0x14 [MEAS].
- Application SysTick LOAD=71999 → 1 kHz at 72 MHz [HYP: core 72 MHz].

## 2. Bootloader (0x08000000) [MEAS]
- After reset, listens on UART @0x40015000 for ~0.2 s (SysTick 14.4M cycles); if there are no valid packets — jumps to the application at 0x08001000.
- Packet: `FD FD <ID> <LEN> … [5]=0x97 [6]=subcommand [7..8]=u16 … data from [11] … CHK`,
  CHK = `~sum(bytes from ID)`. ID = from the journal (byte +3) or 0xFE (broadcast).
- Subcommands: 1 — start (size), 2 — erase ⌈size/512⌉ pages from 0x08001000
  (**with ID=0xFE also erases the journal = settings reset**), 3 — data block (512 B buffer → page write).
- Response: `FD FD <ID> 07 …` (11 bytes). During transmission it switches the pin AF (GPIOA, reg. +0x24, bits 8–11: AF1 ↔ AF7) — half-duplex on a single wire [HYP: this is PA2 = wire S].
- Bootloader UART divider: BRR int=52, frac=5 → 115200 at 96 MHz or 57600 at 48 MHz [HYP].
- **So servos can be reflashed over wire S, without SWD** (keeping the bootloader) [HYP, to verify].

## 3. Application bus — Dynamixel Protocol 1.0, AX-12 emulation [MEAS]
- Header: the receive state machine checks `FF` and `FD` (probably `FF FF` — Dynamixel 1.0, `FD FD` — bootloader) [HYP regarding FD].
- UART @0x40015000 via DMA. Default speed — register Baud=16 (in Dynamixel 2 000 000/17 ≈ 117 647 → probably 115200) [HYP].
- CHK = `~sum(ID..parameters)`. Errors — Dynamixel bits (0x10 checksum, 0x40 instruction).
- Instructions (dispatcher 0x0800338C):

| Code | What |
|---|---|
| 0x01 | PING |
| 0x02 | READ |
| 0x03 | WRITE |
| 0x04 | REG_WRITE |
| 0x05 | ACTION |
| 0x06 | RESET (factory defaults + save; only with LEN=2) |
| 0x64 | **custom:** save journal and reboot (SYSRESETREQ) → enter bootloader |
| 0x83 | SYNC_WRITE |

- WRITE is allowed to addresses 3…0x4F, except 10 and 0x24…0x2E (present values, read-only).
  A write to addresses < 0x18 (EEPROM) → journal save flag, unless Lock is set (0x2F=1).

## 4. Register table (= contents of a journal record)
Values: "Default" — from the factory defaults function 0x080045DC; "servo_01" — from the journal.

| Addr | Size | Dynamixel AX-12 | Default | servo_01 | Note |
|---|---|---|---|---|---|
| 0x00 | 2 | Model Number | 12 | 12 | **AX-12** |
| 0x02 | 1 | Firmware Version | 0x12 | 0x12 | set in main |
| 0x03 | 1 | ID | 1 | 1 | |
| 0x04 | 1 | Baud Rate | 16 | 16 | ≈115200? |
| 0x05 | 1 | Return Delay | 0 | 0 | |
| 0x06 | 2 | CW Angle Limit | 0 | 0 | |
| 0x08 | 2 | CCW Angle Limit | 1023 | 1023 | |
| 0x0B | 1 | Temp Limit | 70 °C | 70 | |
| 0x0C | 1 | Min Voltage | 60 (6.0 V) | 60 | |
| 0x0D | 1 | Max Voltage | 140 (14.0 V) | 140 | ⚠ at 24 V/48 V the protection will trip — change |
| 0x0E | 2 | Max Torque | 1000 | 1000 | |
| 0x10 | 1 | Status Return Level | 2 | 2 | |
| 0x11 | 1 | Alarm LED | 0x24 | 0x24 | |
| 0x12 | 1 | Alarm Shutdown | 0x24 | 0x24 | overheat + overload |
| 0x18 | 1 | Torque Enable | | 1 | |
| 0x1A/1B | 1+1 | Compliance Margin CW/CCW | | 6/6 | |
| 0x1C/1D | 1+1 | Compliance Slope | | 32/32 | |
| 0x1E | 2 | Goal Position | | 1000 / 2000 | |
| 0x22 | 2 | Torque Limit | | 1000 | |
| 0x24 | 2 | Present Position | | 1190…2936 | scale ≠ 0..1023 [HYP] |
| 0x28 | 2 | Present Load | | 42…87 | |
| 0x2A | 1 | Present Voltage | | 106 / 139 | **10.6 V / 13.9 V** |
| 0x2B | 1 | Present Temperature | | 0 | |
| 0x2F | 1 | Lock | | 0 | |
| 0x33 | 1 | ? | | 0x12 | |
| 0x40 | 2 | (custom) PWM min, µs | 900 | 900 | |
| 0x42 | 2 | (custom) PWM center, µs | 1500 | 1500 | |
| 0x44 | 2 | (custom) PWM max, µs | 2100 | 2100 | |
| 0x46/48 | 2+2 | (custom) calibration? | computed | 1068/1068 | |
| 0x4A | 2 | (custom) travel, ° | 120 | 120 | write → recompute (float) |
| 0x4C | 2 | (custom) ? | 230 | 230 | |
| 0x4E | 2 | (custom) ? | | 0x0071…0x756E | write → save; trim/offset? |
| 0x5A | 2 | (custom) ? position | | 1209…2930 | |

## 5. Journal (format corrected) [MEAS]
- 2 sectors of 512 B × 4 records of 128 B; record = image of register table 0x00–0x77 + housekeeping:
  - +0x78: int32 seq (maximum = current);
  - +0x7D: flag 'Z' (0x5A = bootloader entry was requested);
  - +0x7E: CHK8 = `~sum(bytes 0..0x7D)`;
  - +0x7F: marker `'K'` (0x4B) = record valid.
- Saving (0x080048F8): on a flag from the main loop (after WRITE to the EEPROM area/RESET),
  on command 0x64 (before reset), at startup if there is no valid record (factory values).
  **On power loss/error — does not write.** → the journal is a configuration history, not a flight recorder.
- servo_01: seq 8…14. Present Voltage in records: seq 8–12 → 10.6 V, seq 13–14 → 13.9 V
  (two separate configuration sessions with different supply). Goal 1000/2000 ↔ Present 1190…2936.
