#include <string.h>
#include "regs.h"
#include "lcm32.h"

uint8_t regs[REGS_SIZE];
uint8_t regs2[REGS2_SIZE];
calib_t calib;

/* ---------- журнал: 2 сектора по 512 Б × 4 записи по 128 Б ---------- */
static const uint32_t journal_sector[2] = { 0x08007C00u, 0x08007E00u };
static int32_t  j_seq;      // RAM_20000064
static uint16_t j_slot;     // смещение записи внутри сектора (0, 0x80, 0x100, 0x180)
static uint8_t  j_sector;   // 0/1
static uint8_t  j_found;

uint8_t dxl_checksum(const uint8_t *p, unsigned n)
{
    uint8_t s = 0;
    for (unsigned i = 0; i < n; i++) {
        s = (uint8_t)(s + p[i]);
    }
    return (uint8_t)~s;
}

/* ---------- заводские настройки ---------- */
static void calib_from_travel(void)   // FUN_08003a28
{
    calib.travel = reg16(REG_TRAVEL);
    calib.r4c    = reg16(REG_4C) ? reg16(REG_4C) : 1u;
    uint16_t r46 = (uint16_t)((uint32_t)calib.travel * 2048u / calib.r4c);   // scalbn(x, 11)
    calib.r46 = (int16_t)r46;
    calib.r48 = (int16_t)r46;
    calib.speed_k_x256 = (uint32_t)calib.r4c * 92734u / 1000u;               // 256·1023·87/(4095·60) = 92.734
    set_reg16(REG_46, r46);
    set_reg16(REG_48, r46);
}

void regs_factory_defaults(void)
{
    regs[0x00] = 0x0C; regs[0x01] = 0x00;           // Model Number = 12
    regs[REG_ID] = 1;
    regs[REG_BAUD] = 0x10;                          // ≈115 200
    regs[REG_RETURN_DELAY] = 0;
    regs[0x06] = 0; regs[0x07] = 0;                 // CW limit
    regs[0x08] = 0xFF; regs[0x09] = 0x03;           // CCW limit = 1023
    regs[REG_TEMP_LIMIT] = 70;
    regs[REG_MIN_VOLT] = 60;
    regs[REG_MAX_VOLT] = 140;
    regs[0x0E] = 0xE8; regs[0x0F] = 0x03;           // Max Torque = 1000
    regs[REG_STATUS_RETURN] = 2;
    regs[REG_ALARM_LED] = 0x24;
    regs[REG_ALARM_SHUTDOWN] = 0x24;
    set_reg16(REG_PWM_CENTER, 1500);
    set_reg16(REG_TRAVEL, 120);
    set_reg16(REG_4C, 230);
    calib_from_travel();
    set_reg16(REG_PWM_MIN, 900);
    set_reg16(REG_PWM_MAX, 2100);
    // v1: защиты по умолчанию выключены, смещение тока — автокалибровка
    set_reg16(REG_LIMIT_CURRENT, 0);
    set_reg16(REG_LIMIT_SPEED, 0);
    set_reg16(REG_BUS_TIMEOUT, 0);
    set_reg16(REG_VBUS_MAX, 0);
    set_reg16(REG_I_CONT, 0);
    set_reg16(REG_LOAD_OFFSET, 0);
    regs[REG_CFG_FLAGS] = 0;
    regs[REG_BUS_SLOT] = 30;
    regs[REG_STEPS_PER_CNT] = 222;
    set_reg16(REG_R_REF, 550);
    regs[REG_LIMIT_HW] = 0;
    set_reg16(REG_KV, 1470);
    regs[REG_PP] = 1;
    regs[REG_CTRL_MODE] = 0;
    set_reg16(REG_PROFILE_VMAX, 0);
    set_reg16(REG_PROFILE_ACC, 0);
    regs[REG_KP] = 31;
    regs[REG_KD] = 0;
    set_reg16(REG_VBUS_MIN, 0);
}

void regs_ram_reset(void)
{
    regs[REG_TORQUE_ENABLE] = 0;
    regs[REG_LED] = 0;
    regs[REG_COMPL_MARGIN_CW] = 6;
    regs[REG_COMPL_MARGIN_CCW] = 6;
    regs[REG_COMPL_SLOPE_CW] = 0x20;
    regs[REG_COMPL_SLOPE_CCW] = 0x20;
    regs[0x20] = 0; regs[0x21] = 0;                 // Moving Speed
    regs[0x22] = regs[0x0E];                        // Torque Limit = Max Torque
    regs[0x23] = regs[0x0F];
    regs[REG_REGISTERED] = 0;
    regs[0x2E] = 0;
    regs[REG_LOCK] = 0;
    regs[0x30] = 0; regs[0x31] = 0;                 // Punch
}

/* пересчёт калибровки по регистрам при старте (FUN_08003908) */
void calib_from_regs(void)
{
    calib.r46 = (int16_t)reg16(REG_46);
    calib.r48 = (int16_t)reg16(REG_48);
    calib.travel = reg16(REG_TRAVEL);
    calib.r4c    = reg16(REG_4C) ? reg16(REG_4C) : 1u;
    calib.speed_k_x256 = (uint32_t)calib.r4c * 92734u / 1000u;
    calib.pwm_min = (int16_t)reg16(REG_PWM_MIN);
    calib.pwm_max = (int16_t)reg16(REG_PWM_MAX);
    int32_t span = (int32_t)calib.pwm_max - (int32_t)calib.pwm_min;
    int32_t c = (int32_t)(uint16_t)reg16(REG_PWM_CENTER) - (int32_t)calib.pwm_min;
    calib.center = (uint16_t)(span > 0 ? c * 4095 / span : 2048);   // (center − min)·4095/span, усечение как у float
}

/* ---------- Flash ---------- */
static int flash_status(void)   // 1 занято, 2 ошибка программирования, 3 ошибка защиты, 4 готово
{
    uint32_t sr = FLASH_SR;
    if (sr & 1u) return 1;
    if (sr & 0x10u) return 2;
    if (sr & 0x4u) return 3;
    return 4;
}

static int flash_wait(void)
{
    uint32_t n = 0xB0000u;
    int st = flash_status();
    while (st == 1 && n) {
        st = flash_status();
        n--;
    }
    return n ? st : 5;
}

static void flash_unlock(void)
{
    if (FLASH_CR & 0x80u) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void) { FLASH_CR |= 0x80u; }

static void flash_erase_page(uint32_t addr)
{
    if (flash_wait() == 4) {
        FLASH_CR |= 2u;        // PER
        FLASH_AR = addr;
        FLASH_CR |= 0x40u;     // STRT
        flash_wait();
        FLASH_CR &= ~2u;
    }
}

static int flash_program_u16(uint32_t addr, uint16_t data)
{
    int st = flash_wait();
    if (st == 4) {
        FLASH_CR |= 1u;        // PG
        *(volatile uint16_t *)addr = data;
        st = flash_wait();
        FLASH_CR &= ~1u;
    }
    return st;
}

/* ---------- журнал ---------- */
void journal_load(void)
{
    for (unsigned s = 0; s < 2; s++) {
        for (unsigned i = 0; i < 4; i++) {
            const uint8_t *r = (const uint8_t *)(journal_sector[s] + i * 0x80u);
            if (r[0x7F] != 'K' || dxl_checksum(r, 0x7E) != r[0x7E]) {
                continue;
            }
            int32_t seq;
            memcpy(&seq, r + 0x78, 4);
            if (seq >= j_seq) {
                j_seq = seq;
                j_slot = (uint16_t)(i * 0x80u);
                j_sector = (uint8_t)s;
                j_found = 1;
            }
        }
    }
    if (!j_found) {
        regs_factory_defaults();
        journal_save(0);
    } else {
        memcpy(regs, (const void *)(journal_sector[j_sector] + j_slot), REGS_SIZE);
    }
    uint8_t saved_by = regs[REG_FW_VER];            // версия прошивки, сохранившей запись
    regs_ram_reset();
    regs[REG_33] = 0x12;
    set_r2_16(R2_CMD_KP, 310);
    set_r2_16(R2_CMD_KD, 60);          // стенд: 60 — без перерегулирования, установление 0.12 с
    set_r2_16(R2_CMD_TMAX, 1023);
    regs2[R2_SOFT_KP - REGS2_BASE] = 50;
    regs2[R2_STALL_MS - REGS2_BASE] = 50;
    set_r2_16(R2_HOLD_TMAX, 300);
    set_r2_16(R2_N_GEAR, 2230);
    regs2[R2_SINE_OFS - REGS2_BASE] = 6;   // стенд servo_01 без редуктора: минимум тока и симметрия направлений
    regs2[R2_SINE_ADV - REGS2_BASE] = 8;
    cfg2_load();
    for (unsigned i = 0; i < 12; i++) regs2[R2_UID - REGS2_BASE + i] = INFO_UID[i];
    if (saved_by < 0x24) regs[REG_LIMIT_HW] = 0;     // поле переехало на 0x32 в v1.4 (на 0x17 перекрывалось с R_REF)
    // v1: значения для записей, сделанных стоком (там нули)
    if (regs[REG_STEPS_PER_CNT] == 0) regs[REG_STEPS_PER_CNT] = 222;
    if (reg16(REG_R_REF) < 200 || reg16(REG_R_REF) > 3000) set_reg16(REG_R_REF, 550);   // в старых записях испорчено перекрытием с 0x17
    if (reg16(REG_KV) == 0) set_reg16(REG_KV, 1470);
    if (regs[REG_PP] == 0) regs[REG_PP] = 1;
    if (regs[REG_KP] == 0) regs[REG_KP] = 31;
    if (regs[REG_BUS_SLOT] == 0) regs[REG_BUS_SLOT] = 30;
}

void journal_save(uint8_t boot_flag)
{
    j_seq++;
    memcpy(regs + 0x78, &j_seq, 4);
    regs[0x7D] = boot_flag;
    regs[0x7F] = 'K';
    flash_unlock();
    while (1) {
        FLASH_SR = 0x34u;                       // сброс флагов
        j_slot = (uint16_t)(j_slot + 0x80u);
        if ((j_slot & 0x1FFu) == 0) {           // сектор заполнен — переходим в другой и стираем его
            j_slot = 0;
            j_sector = (uint8_t)(j_sector == 0);
            flash_erase_page(journal_sector[j_sector]);
        }
        uint32_t base = journal_sector[j_sector] + j_slot;
        unsigned i;
        for (i = 0; i < 0x7E; i += 2) {
            if (flash_program_u16(base + i, (uint16_t)(regs[i] | (regs[i + 1] << 8))) != 4) {
                break;
            }
        }
        if (i < 0x7E) {
            continue;                           // ошибка — пробуем следующий слот
        }
        uint8_t chk = dxl_checksum((const uint8_t *)base, 0x7E);
        regs[0x7E] = chk;
        if (flash_program_u16(base + 0x7E, (uint16_t)(chk | ((uint16_t)regs[0x7F] << 8))) == 4) {
            flash_lock();
            return;
        }
    }
}

/* ---------- конфигурация v1.5 (regs2) во Flash: сектор 0x08007A00, 16 слотов по 32 Б ---------- */
#define CFG2_SECTOR 0x08007A00u
#define CFG2_SLOT   32u
#define CFG2_PAYLOAD 24u
static const uint8_t cfg2_map[][2] = {   // {адрес regs2, длина}
    {R2_SOFT_MIN, 2}, {R2_SOFT_MAX, 2}, {R2_SOFT_KP, 1}, {R2_FAULT_MODE, 1}, {R2_STALL_DUTY, 2}, {R2_STALL_MS, 1},
    {R2_STALL_ACT, 1}, {R2_HOLD_TMAX, 2}, {R2_CONTACT_DECEL, 2}, {R2_N_GEAR, 2}, {R2_KT, 2}, {R2_SINE_ADV, 1}, {R2_SINE_OFS, 1},
};
static uint16_t cfg2_seq;
static int8_t   cfg2_slot = -1;

void cfg2_load(void)
{
    for (unsigned i = 0; i < 512u / CFG2_SLOT; i++) {
        const uint8_t *r = (const uint8_t *)(CFG2_SECTOR + i * CFG2_SLOT);
        if (r[0] != 'C' || r[1] != '2' || r[2] != 1 || r[31] != 'K' || dxl_checksum(r, 30) != r[30]) continue;
        uint16_t seq = (uint16_t)(r[28] | (r[29] << 8));
        if (cfg2_slot < 0 || (int16_t)(seq - cfg2_seq) > 0) { cfg2_seq = seq; cfg2_slot = (int8_t)i; }
    }
    if (cfg2_slot < 0) return;
    const uint8_t *r = (const uint8_t *)(CFG2_SECTOR + (unsigned)cfg2_slot * CFG2_SLOT);
    unsigned k = 4, n = r[3] > CFG2_PAYLOAD ? CFG2_PAYLOAD : r[3];   // запись старее карты — хвост остаётся по умолчанию
    for (unsigned m = 0; m < sizeof(cfg2_map) / 2; m++) {
        if (k + cfg2_map[m][1] > 4u + n) break;
        for (unsigned j = 0; j < cfg2_map[m][1]; j++) regs2[cfg2_map[m][0] - REGS2_BASE + j] = r[k++];
    }
}

void cfg2_save(void)
{
    uint8_t rec[CFG2_SLOT];
    for (unsigned i = 0; i < CFG2_SLOT; i++) rec[i] = 0xFF;
    unsigned used = 0;
    for (unsigned m = 0; m < sizeof(cfg2_map) / 2; m++) used += cfg2_map[m][1];
    rec[0] = 'C'; rec[1] = '2'; rec[2] = 1; rec[3] = (uint8_t)used;   // фактическая длина карты
    unsigned k = 4;
    for (unsigned m = 0; m < sizeof(cfg2_map) / 2; m++) {
        for (unsigned j = 0; j < cfg2_map[m][1]; j++) rec[k++] = regs2[cfg2_map[m][0] - REGS2_BASE + j];
    }
    cfg2_seq++;
    rec[28] = (uint8_t)cfg2_seq; rec[29] = (uint8_t)(cfg2_seq >> 8);
    rec[30] = dxl_checksum(rec, 30); rec[31] = 'K';
    flash_unlock();
    for (int attempt = 0; attempt < 2; attempt++) {
        FLASH_SR = 0x34u;
        int slot = cfg2_slot + 1;
        uint32_t base = CFG2_SECTOR + (uint32_t)slot * CFG2_SLOT;
        if (slot >= (int)(512u / CFG2_SLOT) || *(const volatile uint32_t *)base != 0xFFFFFFFFu) {   // конец или слот не пуст (чужие данные)
            flash_erase_page(CFG2_SECTOR);
            slot = 0; base = CFG2_SECTOR;
        }
        unsigned i;
        for (i = 0; i < CFG2_SLOT; i += 2) {
            if (flash_program_u16(base + i, (uint16_t)(rec[i] | (rec[i + 1] << 8))) != 4) break;
        }
        const volatile uint8_t *chk = (const volatile uint8_t *)base;
        unsigned ok = (i == CFG2_SLOT);
        for (unsigned j = 0; ok && j < CFG2_SLOT; j++) if (chk[j] != rec[j]) ok = 0;
        cfg2_slot = (int8_t)slot;
        if (ok) break;                                   // иначе второй заход: страница стирается, слот 0
        cfg2_slot = (int8_t)(512u / CFG2_SLOT);          // принудить стирание
    }
    flash_lock();
}
