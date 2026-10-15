// Register table (Dynamixel AX-12 + Kpower custom) and settings journal in Flash.
// Layout and field meaning — docs/native_fw_config.md.
#pragma once
#include <stdint.h>

#define REG_MODEL          0x00   // u16 = 12 (AX-12)
#define REG_FW_VER         0x02
#define REG_ID             0x03
#define REG_BAUD           0x04   // baud = 2 000 000 / (BAUD + 1)
#define REG_RETURN_DELAY   0x05   // × 2 µs
#define REG_CW_LIMIT       0x06   // u16
#define REG_CCW_LIMIT      0x08   // u16
#define REG_TEMP_LIMIT     0x0B
#define REG_MIN_VOLT       0x0C
#define REG_MAX_VOLT       0x0D
#define REG_MAX_TORQUE     0x0E   // u16
#define REG_STATUS_RETURN  0x10
#define REG_ALARM_LED      0x11
#define REG_ALARM_SHUTDOWN 0x12
#define REG_POS_OFFSET     0x14   // s16: Present Position offset [FW]
#define REG_TORQUE_ENABLE  0x18
#define REG_LED            0x19
#define REG_COMPL_MARGIN_CW  0x1A
#define REG_COMPL_MARGIN_CCW 0x1B
#define REG_COMPL_SLOPE_CW   0x1C
#define REG_COMPL_SLOPE_CCW  0x1D
#define REG_GOAL           0x1E   // u16, pulse µs 900..2100 [FW]
#define REG_SPEED          0x20   // u16, in wheel mode: bit 10 — sign
#define REG_TORQUE_LIMIT   0x22   // u16
#define REG_PRESENT_POS    0x24   // s16, ADC units (0..4095)
#define REG_PRESENT_LOAD   0x28   // u16, ≈ mA
#define REG_PRESENT_VOLT   0x2A   // 0.1 V, calibrated; saturates at 25.5 V (legacy u8)
#define REG_PRESENT_TEMP   0x2B   // not measured
#define REG_REGISTERED     0x2C
#define REG_LOCK           0x2F
#define REG_PUNCH          0x30   // u16, minimum output
#define REG_33             0x33   // byte from stock firmware code at 0x08002000 (= 0x12)
#define REG_PWM_MIN        0x40   // µs (900)
#define REG_PWM_CENTER     0x42   // µs (1500)
#define REG_PWM_MAX        0x44   // µs (2100)
#define REG_46             0x46   // u16, = travel·2048/REG_4C (1068)
#define REG_48             0x48
#define REG_TRAVEL         0x4A   // u16, degrees (120)
#define REG_4C             0x4C   // u16 (230)
#define REG_K_CW           0x4E   // byte: travel scale below center (0 → travel)
#define REG_K_CCW          0x4F   // byte: travel scale above center (0 → travel)
#define REG_GOAL_ADC       0x5A   // u16, goal in ADC units (read-only)

/* ---- custom settings v1 (0x50..0x5F, saved to journal; 0 = disabled) ---- */
#define REG_LIMIT_CURRENT  0x50   // u16 mA, limit on estimated winding current
#define REG_LIMIT_SPEED    0x52   // u16 Hall steps/s, speed limit
#define REG_BUS_TIMEOUT    0x54   // u16 ms without packets → torque off
#define REG_VBUS_MAX       0x56   // u16 mV, above → torque off
#define REG_I_CONT         0x58   // u16 mA, continuous current for I²t
#define REG_LOAD_OFFSET    0x5C   // u16, current sensor zero offset (auto-calibrated with torque off)
#define REG_CFG_FLAGS      0x5E   // u8: 1 IWDG, 2 Hall fault → torque off, 4 wrap-around error (0..4095), 8 invert speed sign
#define REG_BUS_SLOT       0x5F   // u8 ×10 µs, reply slot in SYNC_READ (default 30 → 300 µs)
/* block 0x13..0x17 (EEPROM area, Lock applies) and 0x34..0x3F */
#define REG_STEPS_PER_CNT  0x13   // u8 ×0.001 Hall steps per ADC unit (measured 222)
#define REG_R_REF          0x16   // u16 mΩ, winding resistance at 25 °C (measured ~550) — for temperature estimate
#define REG_LIMIT_HW       0x32   // u8 ×100 mA: hardware limit of average bus current (ADC watchdog → OCREF_CLR TIM1), 0 = off
#define REG_KV             0x34   // u16 rpm/V (measured 1470 at pp = 1)
#define REG_PP             0x36   // u8 pole pairs
#define REG_CTRL_MODE      0x37   // u8: 0 — stock controller, 1 — v1 (trapezoidal profile + PD on Hall speed)
#define REG_PROFILE_VMAX   0x38   // u16 ADC units/s, 0 = unlimited
#define REG_PROFILE_ACC    0x3A   // u16 ×100 ADC units/s², 0 = unlimited
#define REG_KP             0x3C   // u8 ×0.1 duty per unit of error (stock ≈ 31)
#define REG_KD             0x3D   // u8 duty per 100 steps/s of speed error
#define REG_VBUS_MIN       0x3E   // u16 mV, below → torque off, flag 0x40
#define CFG_WDT        0x01
#define CFG_HALL_OFF   0x02
#define CFG_WRAP       0x04
#define CFG_SPEED_INV  0x08
#define CFG_SINE      0x10   // sinusoidal commutation with Hall angle interpolation (switched while TE = 0)
#define CFG_LATCH     0x20   // hard faults (Vbus, Halls) latch: torque cannot be enabled until CFG2_CMD = 3
#define CFG_ILOOP     0x40   // in torque mode (Kp = Kd = 0) ff in mA is closed on measured current (slow PI on top of the model)

/* ---- telemetry v1 (0x60..0x77, read-only) ---- */
#define REG_HALL_SPEED     0x60   // s16, Hall steps/s, sign = direction
#define REG_HALL_COUNT     0x62   // s32, accumulated Hall steps
#define REG_I_BUS          0x66   // u16 mA, bus current from shunt
#define REG_VBUS_MV        0x68   // u16 mV (calibrated against bench PSU: 1829 counts = 24.0 V)
#define REG_I_WIND         0x6A   // u16 mA, winding current estimate = I_bus / duty
#define REG_STATUS         0x6C   // u8 flags: 1 current limit, 2 speed limit, 4 bus timeout, 8 Halls, 0x10 Vbus, 0x20 I²t
#define REG_HALL_CODE      0x6D   // u8
#define REG_UPTIME         0x6E   // u16 s
#define REG_THERMAL        0x70   // u8 % of I²t thermal budget
#define REG_I_WIND_MAX     0x72   // u16 mA, peak since start
#define REG_DUTY           0x74   // u16, duty 0..1023
#define REG_R_EST          0x76   // u16 mΩ, online winding resistance estimate (0 = no data)
#define REG_T_EST          0x71   // s8 °C, winding temperature estimate from R_EST/R_REF

#define ST_CUR_LIMIT  0x01
#define ST_SPD_LIMIT  0x02
#define ST_BUS_TMO    0x04
#define ST_HALL_BAD   0x08
#define ST_VBUS_HIGH  0x10
#define ST_I2T        0x20
#define ST_VBUS_LOW   0x40
#define ST_WDT_GUARD  0x80   // IWDG disabled by reboot-loop guard

#define REGS_SIZE          0x80

/* ---- extended table 0x80..0xBF (RAM, not written to journal) ---- */
#define REGS2_BASE         0x80
#define REGS2_SIZE         0x80
/* impedance command (written with one SYNC_WRITE: addr 0x80, 16 bytes) */
#define R2_CMD_POS         0x80   // s16 ADC units — goal
#define R2_CMD_VEL         0x82   // s16 ADC units/s — velocity; goal moves with it for 100 ms after the command (first-order hold)
#define R2_CMD_KP          0x84   // u16 ×0.01 duty/unit (310 ≈ stock)
#define R2_CMD_KD          0x86   // u16 ×0.001 duty/(unit/s)
#define R2_CMD_FF          0x88   // s16 mA — feedforward torque (via R_EST and Ke·ω → duty)
#define R2_CMD_TMAX        0x8A   // u16 duty limit 0..1023 (0 = 1023)
#define R2_CMD_MODE        0x8C   // u8: 0 — old loop on Goal, 1 — impedance (enables torque), 2 — impedance with cubic
                                  //     interpolation of goal from current (p, v) to (CMD_POS, CMD_VEL) by time CMD_AT
#define R2_CMD_AT          0x8D   // u16 host time (low 16 bits of ms) to apply at; 0 = immediately
#define R2_CMD_SEQ         0x8F   // u8: command number, LAST byte of block — value change = command fully written (SWD and bus)
/* fast telemetry (SYNC_READ 0x90, 16 bytes) */
#define R2_TLM_POS         0x90   // s16 ADC units
#define R2_TLM_VEL         0x92   // s16 ADC units/s (from Halls)
#define R2_TLM_IBUS        0x94   // u16 mA
#define R2_TLM_TEMP        0x96   // s8 °C winding (estimate)
#define R2_TLM_STATUS      0x97   // u8 flags
#define R2_TLM_VBUS        0x98   // u16 mV
#define R2_TLM_DUTY        0x9A   // u16
#define R2_TLM_TIME        0x9C   // u32 ms by host clock (after TIME_SYNC), otherwise local
#define R2_TIME_OFFSET     0xA0   // u32 offset host − local (read-only)
/* v1.5: soft limits, failsafe policy, stall/collision detection */
#define R2_SOFT_MIN        0xA4   // u16 ADC units — lower soft limit (0 and 0 = off): goal is clipped, beyond limit a SOFT_KP spring
#define R2_SOFT_MAX        0xA6   // u16
#define R2_SOFT_KP         0xA8   // u8 ×0.1 duty/unit spring stiffness beyond limit (50)
#define R2_FAULT_MODE      0xA9   // u8: action with torque off/on fault: 0 brake (low-side switches), 1 coast (all switches off),
                                  //     2 hold — on bus timeout/collision, impedance at the fault point (tmax = HOLD_TMAX)
#define R2_STALL_DUTY      0xAA   // u16: |loop output| ≥ this at speed < 100 units/s longer than STALL_MS → stall (0 = off)
#define R2_STALL_MS        0xAC   // u8 ms (50)
#define R2_STALL_ACT       0xAD   // u8: 0 flag only, 1 reduce output to 25 % while stalled, 2 fault per FAULT_MODE
#define R2_STATUS2         0xAE   // u8: 1 stall, 2 beyond soft limit, 4 fault hold, 8 interpolating, 16 coast
#define R2_TAU_AVAIL       0xAF   // u8 % of torque available now: LIMIT_CURRENT derated linearly to I_CONT as I²t is consumed (SOA)
#define R2_HOLD_TMAX       0xB0   // u16 duty limit for fault hold (300)

/* v1.5: identity record, counters, die temperature, flight recorder */
#define R2_TEMP_MCU        0xB2   // s8 °C die: ADC_IN16 with factory calibration (0x1FFFF580: T1, T2, counts at 3.3 V)
#define R2_STATE           0xB3   // u8 state machine: 0 BOOT, 1 READY, 2 ENABLED, 3 HOLD (fault hold), 4 SELFTEST, 5 FAULT, 6 LATCHED (waits for CFG2_CMD = 3)
#define R2_UID             0xB4   // 12 bytes — 96-bit chip UID, copy of 0x1FFFF5B0 [HYP: address inferred from data]
#define R2_ENERGY_MJ       0xC0   // u32 mJ drawn from bus since start (Vbus·I_bus; shunt does not see regen)
#define R2_ODO_STEPS       0xC4   // u32 absolute Hall steps since start (rotor odometer)
#define R2_PKT_OK          0xC8   // u16 packets to us with valid checksum
#define R2_PKT_ERR         0xCA   // u16 packets to us with checksum error
#define R2_FRIC_IC         0xCC   // u16 mA — Coulomb friction current (from self-test: points +150/+300 duty, no load) [HYP]
#define R2_FRIC_KV         0xCE   // u16 µA per step/s — viscous friction; I_fric = IC + KV·|ω|/1000; subtracted from telemetry torque
#define R2_BB_CTRL         0xD0   // u8 flight recorder: write 0 — run, 1 — freeze; read: 1 = frozen
#define R2_BB_PAGE         0xD1   // u8 window page 0..16 (3 records each, oldest to newest)
#define R2_BB_HEAD         0xD2   // u16 record count since start (low 16 bits)
#define R2_BB_TRIG         0xD4   // u8 freeze cause: STATUS bits, 0x80 = stall
#define R2_CFG2_CMD        0xD5   // u8 command: 1 — save v1.5 settings to Flash 0x08007A00, 2 — self-test/identification,
                                  //    3 — clear latched fault, 4 — TEST_UNLIMITED: 10 s without current/speed/I²t ceilings, flag 0x80 in STATUS2,
                                  //    on expiry the flight recorder freezes with pre-trigger history
#define R2_CONTACT_DECEL   0xD6   // u16 units/s per 1 ms: rotor deceleration along motion while loop is pushing → contact (0 = off)
#define R2_TLM_TORQUE      0xD8   // s16 mN·m at output: I_wind·Kt·N (excluding gearbox efficiency), sign = output direction
#define R2_N_GEAR          0xDA   // u16 ×0.1 gear ratio (2230) [HYP]
#define R2_KT              0xDC   // u16 µN·m/A at rotor; 0 = from KV: 9549296/KV (6496 at 1470)
#define R2_ST_RESULT       0xDE   // u8 self-test: 1 Halls, 2 direction matches CFG, 4 R measured (two points at different current — only under load; otherwise R_REF), 8 Kv, 16 steps/unit, 0x40 aborted, 0x80 done
#define R2_ST_FRICTION     0xDF   // u8 breakaway duty of the rotor
#define R2_ST_R            0xE0   // u16 mΩ from two operating points
#define R2_ST_KV           0xE2   // u16 rpm/V
#define R2_ST_SPC          0xE4   // u16 ×0.001 Hall steps per ADC unit
#define R2_SINE_ADV        0xE6   // s8 el. degrees: vector phase advance along torque in sinusoidal mode
#define R2_SINE_OFS        0xD7   // s8 el. degrees: constant angle offset (Hall sensor placement vs phases; bench test: ≈ −6)
#define R2_ST_DIR          0xE7   // s8: −1 Hall counter decreases as position increases (firmware convention), +1 the opposite
#define R2_BB_WIN          0xE8   // 16 B: 2 records × {pos s16, vel s16, I_bus u16, duty/4 u8, status u8}, 5 ms step, 260 ms ring
#define BB_N               52
#define BB_PER_PAGE        2
#define R2_ST_BACKLASH     0xF8   // u16 Hall steps of backlash on reversal (rotor moves, output stays) — from self-test
#define BB_PERIOD_MS       5
#define BB_POST            10     // records after event before freeze

#define INFO_T1_X10        (*(volatile uint32_t *)0x1FFFF580u)   // calibration temperature 1, ×0.1 °C (26.6)
#define INFO_T2_X10        (*(volatile uint32_t *)0x1FFFF584u)   // calibration temperature 2 (105.0)
#define INFO_VTS1          (*(volatile uint32_t *)0x1FFFF588u)   // count at T1 (low 16 bits — 3.3 V reference)
#define INFO_VTS2          (*(volatile uint32_t *)0x1FFFF58Cu)   // count at T2
#define INFO_UID           ((const volatile uint8_t *)0x1FFFF5B0u)
#define INFO_DATE          ((const volatile uint8_t *)0x1FFFF854u)   // year u16, month, day [HYP]

#define ST2_STALL     0x01
#define ST2_SOFT      0x02
#define ST2_HOLD      0x04
#define ST2_INTERP    0x08
#define ST2_COAST     0x10
#define ST2_CONTACT   0x20   // impact/contact: rotor decelerated sharply while loop was pushing
#define ST2_SELFTEST  0x40   // self-test running
#define ST2_UNLIMITED 0x80   // TEST_UNLIMITED active
#define REGS_DATA_END      0x78   // followed by journal service part: seq, flag 'Z', CHK, 'K'

extern uint8_t regs[REGS_SIZE];
extern uint8_t regs2[REGS2_SIZE];
static inline uint16_t r2_16(unsigned a) { return (uint16_t)(regs2[a - REGS2_BASE] | (regs2[a - REGS2_BASE + 1] << 8)); }
static inline void set_r2_16(unsigned a, uint16_t v) { regs2[a - REGS2_BASE] = (uint8_t)v; regs2[a - REGS2_BASE + 1] = (uint8_t)(v >> 8); }
static inline void set_r2_32(unsigned a, uint32_t v) { set_r2_16(a, (uint16_t)v); set_r2_16(a + 2, (uint16_t)(v >> 16)); }

static inline uint16_t reg16(unsigned a) { return (uint16_t)(regs[a] | (regs[a + 1] << 8)); }
static inline void set_reg16(unsigned a, uint16_t v) { regs[a] = (uint8_t)v; regs[a + 1] = (uint8_t)(v >> 8); }

/* calibration values derived from registers (RAM_20000072..8C in stock) */
typedef struct {
    int16_t  pwm_min, pwm_max;
    int16_t  r46, r48;
    uint16_t center;        // position (ADC) at center pulse
    uint16_t travel;        // REG_TRAVEL
    uint16_t r4c;           // REG_4C
    uint32_t speed_k_x256;  // stock profile speed divider ×256: 1023·87·r4c/(4095·60) = r4c·0.36224
} calib_t;
/* fixed point instead of soft-float (v1.5): M0 has no FPU, the float/double library cost 9.5 KB Flash */
extern calib_t calib;

uint8_t dxl_checksum(const uint8_t *p, unsigned n);   // ~sum

void regs_factory_defaults(void);   // factory values of EEPROM area (FUN_080045dc)
void regs_ram_reset(void);          // RAM area after load/reset (FUN_0800464c)
void calib_from_regs(void);         // recompute calib from registers (FUN_08003908)

void journal_load(void);            // at startup: find the current record or create a factory one
void journal_save(uint8_t boot_flag);   // write current registers to the next slot
void cfg2_load(void);               // v1.5: regs2 settings from sector 0x08007A00
void cfg2_save(void);
