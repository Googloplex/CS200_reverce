// Sensors. v1.2: ADC is triggered by TIM1_OC4 once per PWM period (synchronous with PWM). The shunt signal on the board
// is filtered [MEAS: synchronous sample = average bus current], so I_BUS is measured, and winding current = I_BUS/duty.
// Hardware limit: analog ADC watchdog (ADC_TR.HT, channel 11) → TIM1 OCREF_CLR via EDU_CFG4 — cuts PWM until the end
// of the period while average bus current is above threshold. ACMP2 (CPP21 = PF6 vs DAC0) removed from routing: it held
// OCREF_CLR permanently, cause not investigated (see docs/registers_v1.md).
#include "sense.h"
#include "regs.h"
#include "board.h"
#include "motor.h"
#include "dxl.h"

#define DMA_CH_ADC 2
#define ADC_CH_CURR 11

volatile uint16_t adc_buf[4];
int16_t g_present_pos;
volatile uint16_t g_i_wind_ma;     // phase (winding) current, mA — measured if g_i_valid
volatile uint8_t  g_i_valid;
volatile uint16_t g_awd_count;     // hardware limit trips (ADC watchdog)
volatile uint8_t  g_bb_freeze_req; // flight recorder freeze request from ISR

static uint16_t s_raw;
static int16_t  s_offset;
static uint16_t s_load;
static uint32_t s_overload_t;
static uint32_t s_iir;
static int32_t  s_max;
static uint16_t s_thr;
static uint16_t s_pos_u;
static int32_t  fb_a, fb_b;
#define K_LOAD_NUM 323277u     // Present Load = count × 3.23277 (stock uses double 0x4009DCC63F141206)
#define K_LOAD_DEN 100000u
static int32_t  cur_off_x16 = 38 * 16;    // current sensor zero offset in ADC units ×16 (≈ Load 122 / 3.23)
static uint16_t i_wind_max;
static uint8_t  hw_limit_applied;

#define VBUS_MV_NUM        1312u    // 13.12 mV/count: 1829 units ↔ 24.0 V on bench PSU [MEAS]
#define VBUS_MV_DEN        100u
#define MA_PER_ADC_COUNT   647u     // ×0.01: 6.47 mA per shunt ADC unit (2 mA/unit Load × 3.2328 units/count) [MEAS]

static uint16_t vbus_mv_from_adc(uint16_t adc)
{
    uint32_t mv = ((uint32_t)adc * VBUS_MV_NUM + VBUS_MV_DEN / 2u) / VBUS_MV_DEN;
    return (uint16_t)(mv > 65535u ? 65535u : mv);
}

/* ADC DMA from index 0. Mandatory after every ADSTP: if the stop landed mid-sequence of the 4 channels,
   DMA would continue from index k while ADC restarts from channel 1 — the adc_buf ring would shift forever (Vbus in "position") */
static void adc_dma_setup(void)
{
    dma_ch_deinit(DMA_CH_ADC);
    DMA_SAR(DMA_CH(DMA_CH_ADC)) = ADC_DR_ADDR;
    DMA_DAR(DMA_CH(DMA_CH_ADC)) = (uint32_t)adc_buf;
    DMA_CTL_L(DMA_CH(DMA_CH_ADC)) = 0x200612u;
    DMA_CTL_H(DMA_CH(DMA_CH_ADC)) = 4u;
    DMA_CFG_L(DMA_CH(DMA_CH_ADC)) = (1u << 30) | (1u << 31) | (1u << 5);
    DMA_CFG_H(DMA_CH(DMA_CH_ADC)) = 4u | (4u << 11) | (4u << 7);
}

static void adc_restart(void)   // after ADSTP: clear flags, restart DMA, ADSTART
{
    ADC_ISR = 0x1Fu;
    adc_dma_setup();
    dma_ch_enable(DMA_CH_ADC, 1);
    ADC_CR |= 4u;
}

/* ---------- ADC: 4 channels, triggered by TIM1_OC4 (group 3 ADC_EXTCFG), circular DMA into adc_buf ---------- */
void sense_adc_init(void)
{
    ADC_CFGR1 = 0x0001u;                         // DMAEN; CONT = 0 — one sequence per trigger
    ADC_EXTCFG = (1u << 30) | 1u;                // EXTSE: group 3 (EXTSELG3 = 00 → TIM1_OC4), EXTEN = 01 rising edge, delay 0
    ADC_TESTCFG = 0xDCu;
    ADC_CFGR2 = 0x8000000u;
    ADC_TRIM = (ADC_TRIM_FACTORY_WORD != 0xFFFFFFFFu) ? ADC_TRIM : 0x440055u;
    cc_pwr_cfg_and(0xFFF0FFFFu);
    ADC_CFGR2 = (ADC_CFGR2 & ~0xAu) | 2u;        // clock mode 5 (as in stock): PLL/2
    ADC_CHSELR |= (1u << 1) | (1u << 7) | (1u << ADC_CH_CURR) | (1u << 15);
    ADC_SMPR = (ADC_SMPR & ~0x1Fu) | 0x7F7u;
    ADC_TR = 0x0FFFu << 16;                      // watchdog always on: HT = max while LIMIT_HW = 0
    ADC_CFGR1 |= ((uint32_t)ADC_CH_CURR << 27) | (1u << 23) | (1u << 22);   // AWDCH = 11, AWDEN, AWDSGL
    ADC_CR |= 1u;
    ADC_TESTCFG &= ~0x7800u;
    while (!(ADC_ISR & 1u)) {
    }
    adc_dma_setup();
    sc_dma_req_map(4, 4);
    dma_ch_enable(DMA_CH_ADC, 1);
    ADC_CR |= 4u;                                // ADSTART: waits for hardware trigger

    // routing into TIM1: OCREF_CLR source (ADC watchdog, OCCSE bit 5) is enabled only together with the limit —
    // with HT = 0xFFF the watchdog output is low for some reason, and with active-low polarity PWM would always be cut [MEAS]
    sc_edu_cfg4(1u << 5);
}

/* recompute hardware limit thresholds (call when REG_LIMIT_HW or the offset changes) */
static void hw_limit_apply(void)
{
    uint8_t lim = regs[REG_LIMIT_HW];                       // ×100 mA
    uint32_t off = (uint32_t)(cur_off_x16 >> 4);
    uint32_t ht = lim ? off + (uint32_t)lim * 100u * 100u / MA_PER_ADC_COUNT : 0x0FFFu;   // ADC units (average bus current)
    if (ht > 4095u) ht = 4095u;
    ADC_CR |= 0x10u;
    while (ADC_CR & 4u) {
    }
    // Watchdog output = "count inside window" level [MEAS, tried 4 variants]: window 0..HT, OCREF_CLR source
    // is active LOW (TIM1_SP[5] = 1) → PWM is cut until the end of the period while current is above HT.
    ADC_TR = (ht << 16);
    sc_edu_cfg4(lim ? (((1u << 5) << 24) | (1u << 5)) : (1u << 5));
    ADC_ISR = 1u << 7;
    adc_restart();
    hw_limit_applied = (uint8_t)(lim != 0);
}

void sense_tim16_init(void)
{
    TIM_CR1(TIM16_B) = (TIM_CR1(TIM16_B) & 0xFF8Fu & 0xFCFFu);
    TIM_ARR(TIM16_B) = 0x7CFu;
    TIM_PSC(TIM16_B) = 0x47u;
    TIM_RCR(TIM16_B) = 0;
    TIM_EGR(TIM16_B) = 1u;
    TIM_CCMR1(TIM16_B) = (TIM_CCMR1(TIM16_B) & 0xFF8Cu) | 0x70u;
    TIM_CCR1(TIM16_B) = 0;
    TIM_CCER(TIM16_B) = (TIM_CCER(TIM16_B) & 0xFFF0u) | 1u;
    TIM_CR2(TIM16_B) &= 0xFCFFu;
    TIM_BDTR(TIM16_B) = 0x8000u;
    TIM_CR1(TIM16_B) |= 0x80u;
    TIM_CR1(TIM16_B) |= 1u;
}

static void feedback_pwm(int32_t pos)
{
    calib.travel = reg16(REG_TRAVEL);
    fb_a = regs[REG_K_CW]  ? regs[REG_K_CW]  : (int32_t)calib.travel;
    fb_b = regs[REG_K_CCW] ? regs[REG_K_CCW] : (int32_t)calib.travel;
    if (fb_a <= 0) fb_a = 1;
    if (fb_b <= 0) fb_b = 1;
    // stock: t = 540 / (K/460·4095·500/600) = 6624/(91·K) per ADC unit (exact fraction), truncation as with float
    int32_t out;
    if (pos < 0x801) {
        uint32_t den = 91u * (uint32_t)fb_a;
        out = 810 - (int32_t)(((uint32_t)(0x800 - pos) * 6624u + den - 1u) / den);
    } else {
        out = 810 + (int32_t)((uint32_t)(pos - 0x800) * 6624u / (91u * (uint32_t)fb_b));
    }
    if (out < 0) out = 0;
    TIM_CCR1(TIM16_B) = (uint32_t)out;
}

void sense_init_calib(void)
{
    uint16_t off = reg16(REG_LOAD_OFFSET);          // stored in Load units (v1.1 compatibility)
    if (off) cur_off_x16 = (int32_t)((uint32_t)off * 100u / 323u) * 16;
}

static void blackbox_update(int16_t pos, uint16_t i_bus, uint16_t duty);

void sense_update(void)
{
    // position (as in the clone)
    uint16_t raw = adc_buf[ADC_IDX_POT];
    s_raw = raw;
    s_offset = (int16_t)reg16(REG_POS_OFFSET);
    s_iir = (s_iir - (s_iir >> 3)) + raw;
    int16_t pos = (int16_t)((int16_t)(0xFFF - (int16_t)(s_iir >> 3)) + s_offset);
    g_present_pos = pos;
    s_pos_u = (uint16_t)pos;
    set_reg16(REG_PRESENT_POS, (uint16_t)pos);
    if ((uint32_t)s_thr < (uint32_t)((int32_t)pos - s_max)) {
        s_max = pos;
    }
    feedback_pwm(pos);

    // current: sampling synchronous with PWM → phase current; Present Load kept in stock units (now instantaneous, not average)
    uint16_t cur = adc_buf[ADC_IDX_CURR];
    s_load = (uint16_t)((uint32_t)cur * K_LOAD_NUM / K_LOAD_DEN);
    set_reg16(REG_PRESENT_LOAD, s_load);
    if (s_load > 9500u) {
        if ((int32_t)(g_ms - s_overload_t) >= 0) {
            set_reg16(REG_TORQUE_LIMIT, 0);
        }
    } else {
        s_overload_t = g_ms + 1000u;
    }
    uint16_t duty = motor_duty();
    if (regs[REG_TORQUE_ENABLE] == 0 || duty == 0) {           // zero auto-calibration: switches off, shunt current = 0
        cur_off_x16 += ((int32_t)cur * 16 - cur_off_x16) >> 6;
        set_reg16(REG_LOAD_OFFSET, (uint16_t)((uint32_t)(cur_off_x16 >> 4) * 323u / 100u));
    }
    uint32_t off = (uint32_t)(cur_off_x16 >> 4);
    uint32_t i_bus = (cur > off) ? ((uint32_t)(cur - off) * MA_PER_ADC_COUNT) / 100u : 0u;   // average bus current
    g_i_valid = (uint8_t)(duty >= 80u);                        // below 8 % division by duty is unreliable
    uint32_t i_ph = g_i_valid ? (i_bus * 1024u / duty) : i_bus;
    if (i_ph > i_bus * 12u) i_ph = i_bus * 12u;
    if (i_ph > 30000u) i_ph = 30000u;
    g_i_wind_ma = (uint16_t)i_ph;
    if (i_ph > i_wind_max) i_wind_max = (uint16_t)i_ph;
    set_reg16(REG_I_BUS, (uint16_t)(i_bus > 65535u ? 65535u : i_bus));
    set_reg16(REG_I_WIND, (uint16_t)i_ph);
    set_reg16(REG_I_WIND_MAX, i_wind_max);

    // hardware limit: apply on register change, count trips
    static uint8_t last_lim = 0xFF;
    if (regs[REG_LIMIT_HW] != last_lim) {
        last_lim = regs[REG_LIMIT_HW];
        hw_limit_apply();
    }
    if (ADC_ISR & (1u << 7)) {
        ADC_ISR = 1u << 7;
        g_awd_count++;
        __disable_irq(); g_status |= ST_CUR_LIMIT; __enable_irq();
    }

    // voltage: both registers from the same bench calibration; legacy 0x2A saturates at 25.5 V
    uint16_t vbus_mv = vbus_mv_from_adc(adc_buf[ADC_IDX_VBUS]);
    uint32_t v_01 = ((uint32_t)vbus_mv + 50u) / 100u;
    regs[REG_PRESENT_VOLT] = (uint8_t)(v_01 > 255u ? 255u : v_01);
    set_reg16(REG_VBUS_MV, vbus_mv);

    // v1 telemetry
    int32_t sp = g_speed;
    if (sp > 32767) sp = 32767;
    if (sp < -32768) sp = -32768;
    set_reg16(REG_HALL_SPEED, (uint16_t)(int16_t)sp);
    int32_t hc = g_hall_count;
    regs[REG_HALL_COUNT] = (uint8_t)hc;
    regs[REG_HALL_COUNT + 1] = (uint8_t)(hc >> 8);
    regs[REG_HALL_COUNT + 2] = (uint8_t)(hc >> 16);
    regs[REG_HALL_COUNT + 3] = (uint8_t)(hc >> 24);
    regs[REG_STATUS] = g_status;
    regs[REG_HALL_CODE] = g_hall_code;
    set_reg16(REG_UPTIME, (uint16_t)(g_ms / 1000u));
    regs[REG_THERMAL] = g_thermal_pct;
    set_reg16(REG_DUTY, duty);
    set_reg16(REG_R_EST, g_r_est_mohm);
    regs[REG_T_EST] = (uint8_t)g_t_est_c;

    // fast telemetry 0x90..0x9F (one SYNC_READ for all)
    set_r2_16(R2_TLM_POS, (uint16_t)pos);
    int32_t vc = g_vel_cnt; if (vc > 32767) vc = 32767; if (vc < -32768) vc = -32768;
    set_r2_16(R2_TLM_VEL, (uint16_t)(int16_t)vc);
    set_r2_16(R2_TLM_IBUS, (uint16_t)(i_bus > 65535u ? 65535u : i_bus));
    regs2[R2_TLM_TEMP - REGS2_BASE] = (uint8_t)g_t_est_c;
    regs2[R2_TLM_STATUS - REGS2_BASE] = g_status;
    set_r2_16(R2_TLM_VBUS, reg16(REG_VBUS_MV));
    set_r2_16(R2_TLM_DUTY, duty);
    set_r2_32(R2_TLM_TIME, g_ms + g_time_offset);

    // energy drawn from the bus: mV·mA = µW, over 1 ms = nJ; once per ms (main loop spins faster)
    static uint32_t e_last_ms, e_acc_nj, e_mj;
    if (g_ms != e_last_ms) {
        uint32_t dt = g_ms - e_last_ms; if (dt > 50u) dt = 50u;   // missed ms (Flash, temperature poll)
        e_last_ms = g_ms;
        e_acc_nj += (uint32_t)vbus_mv * (i_bus > 40000u ? 40000u : i_bus) * dt;
        if (e_acc_nj >= 1000000u) { e_mj += e_acc_nj / 1000000u; e_acc_nj %= 1000000u; }
        set_r2_32(R2_ENERGY_MJ, e_mj);
        set_r2_32(R2_ODO_STEPS, g_odo_steps);
    }
    blackbox_update(pos, (uint16_t)(i_bus > 65535u ? 65535u : i_bus), duty);

    // output torque, mN·m: I_wind·Kt·N (Kt from KV if not set); sign — output direction
    {
        uint32_t kt = r2_16(R2_KT);
        uint16_t kv = reg16(REG_KV);
        if (!kt && kv) kt = 9549296u / kv;                              // µN·m/A = 60/(2π·Kv)
        if (kt > 100000u) kt = 100000u;
        // more honest: minus gearbox friction (Coulomb + viscous model from self-test), it does not reach the output
        uint32_t w = (uint32_t)(g_speed < 0 ? -g_speed : g_speed);
        uint32_t i_fric = r2_16(R2_FRIC_IC) + (w > 20u ? r2_16(R2_FRIC_KV) * w / 1000u : 0u);
        uint32_t i_eff = (uint32_t)g_i_wind_ma > i_fric ? (uint32_t)g_i_wind_ma - i_fric : 0u;
        uint32_t t_rotor = i_eff * kt / 1000u;                           // µN·m
        uint32_t t_out = t_rotor / 100u * r2_16(R2_N_GEAR) / 100u;      // mN·m = µN·m·N/10/1000
        int32_t tq = (int32_t)(t_out > 32767u ? 32767u : t_out) * motor_out_sign();
        set_r2_16(R2_TLM_TORQUE, (uint16_t)(int16_t)tq);
        // SOA: how much torque is available now — current ceiling drops linearly to I_CONT as the thermal budget is spent
        uint32_t ilim = reg16(REG_LIMIT_CURRENT), ic = reg16(REG_I_CONT), pct = 100u;
        if (ilim && ic < ilim) {
            uint32_t th = g_thermal_pct > 100u ? 100u : g_thermal_pct;
            uint32_t i_av = ilim - (ilim - ic) * th / 100u;
            pct = i_av * 100u / ilim;
        }
        regs2[R2_TAU_AVAIL - REGS2_BASE] = (uint8_t)pct;
    }
}

/* ---------- flight recorder: ring 50 × 8 B, 5 ms step; event → BB_POST more records and freeze ---------- */
static uint8_t  bb_ring[BB_N * 8];
static uint8_t  bb_idx, bb_frozen, bb_post, bb_prev_f;
static uint16_t bb_count;
static uint32_t bb_last_ms;

static void blackbox_update(int16_t pos, uint16_t i_bus, uint16_t duty)
{
    uint8_t *ctl = &regs2[R2_BB_CTRL - REGS2_BASE];
    if (g_bb_freeze_req) { g_bb_freeze_req = 0; *ctl = 1; }
    if (bb_frozen && *ctl == 0) { bb_frozen = 0; bb_post = 0; regs2[R2_BB_TRIG - REGS2_BASE] = 0; }
    if (!bb_frozen && *ctl == 1 && !bb_post) { bb_post = 1; regs2[R2_BB_TRIG - REGS2_BASE] = 0xFF; }
    if (!bb_frozen && (uint32_t)(g_ms - bb_last_ms) >= BB_PERIOD_MS) {
        bb_last_ms = g_ms;
        uint8_t *e = &bb_ring[(unsigned)bb_idx * 8u];
        int32_t v = g_vel_cnt; if (v > 32767) v = 32767; if (v < -32768) v = -32768;
        e[0] = (uint8_t)pos; e[1] = (uint8_t)((uint16_t)pos >> 8);
        e[2] = (uint8_t)v;   e[3] = (uint8_t)((uint16_t)(int16_t)v >> 8);
        e[4] = (uint8_t)i_bus; e[5] = (uint8_t)(i_bus >> 8);
        e[6] = (uint8_t)(duty >> 2); e[7] = g_status;
        bb_idx = (uint8_t)((bb_idx + 1u) % BB_N);
        bb_count++;
        uint8_t f = (uint8_t)((g_status & (ST_HALL_BAD | ST_VBUS_HIGH | ST_VBUS_LOW | ST_BUS_TMO | ST_I2T)) |
                              ((regs2[R2_STATUS2 - REGS2_BASE] & ST2_STALL) ? 0x80u : 0u));
        if ((f & (uint8_t)~bb_prev_f) && !bb_post) { bb_post = BB_POST; regs2[R2_BB_TRIG - REGS2_BASE] = f; }
        bb_prev_f = f;
        if (bb_post && --bb_post == 0) bb_frozen = 1;
    }
    *ctl = bb_frozen;
    set_r2_16(R2_BB_HEAD, bb_count);
    // window: page of 3 records, from oldest (bb_idx) to newest; copy only on a new record or page change
    static uint16_t win_count = 0xFFFF; static uint8_t win_page = 0xFF;
    unsigned page = regs2[R2_BB_PAGE - REGS2_BASE] % (BB_N / BB_PER_PAGE);
    if (win_count == bb_count && win_page == (uint8_t)page) return;
    win_count = bb_count; win_page = (uint8_t)page;
    for (unsigned k = 0; k < BB_PER_PAGE; k++) {
        unsigned src = ((unsigned)bb_idx + page * BB_PER_PAGE + k) % BB_N;
        for (unsigned j = 0; j < 8; j++) regs2[R2_BB_WIN - REGS2_BASE + k * 8 + j] = bb_ring[src * 8 + j];
    }
}

volatile uint16_t g_mcu_temp_raw;   // raw temperature sensor count (global: read by symbol via SWD)

/* ---------- die temperature: ADC_IN16 only in single mode → once per second swap the sequence ---------- */
void sense_mcu_temp_poll(void)
{
    static uint32_t last_ms;
    if ((uint32_t)(g_ms - last_ms) < 1000u) return;
    last_ms = g_ms;
    ADC_CR |= 0x10u;                                   // ADSTP
    while (ADC_CR & 4u) {
    }
    uint32_t chsel = ADC_CHSELR, cfgr1 = ADC_CFGR1;
    ADC_CFGR1 = cfgr1 & ~1u;                           // no DMA, so as not to shift the adc_buf ring
    ADC_CHSELR = 1u << 17;                             // ST = 1, SV = 0 → temperature sensor
    uint16_t raw = 0;
    for (int i = 0; i < 2; i++) {                      // first sample after enabling the sensor — warm-up
        ADC_ISR = 0x1Fu;
        ADC_CR |= 4u;                                  // waits for TIM1_OC4 trigger (≤ 43 µs)
        uint32_t t0 = micros();
        while (!(ADC_ISR & 0x0Cu) && (micros() - t0) < 500u) {
        }
        raw = (uint16_t)(REG32(ADC_B, 0x20) & 0xFFFu);
        ADC_CR |= 0x10u;
        while (ADC_CR & 4u) {
        }
    }
    ADC_CHSELR = chsel;
    ADC_CFGR1 = cfgr1;
    adc_restart();
    g_mcu_temp_raw = raw;
    // factory calibration (servo_01: 26.6 °C ↔ 2018, 105.0 °C ↔ 2544 → +6.7 units/°C), otherwise 5 mV/°C from the datasheet
    int32_t t1 = (int32_t)INFO_T1_X10, t2 = (int32_t)INFO_T2_X10;
    int32_t a1 = (int32_t)(INFO_VTS1 & 0xFFFu), a2 = (int32_t)(INFO_VTS2 & 0xFFFu);
    int32_t t_x10;
    if (INFO_T1_X10 != 0xFFFFFFFFu && INFO_T2_X10 != 0xFFFFFFFFu && t2 > t1 + 100 && a2 > a1 + 50 && t2 < 2000) {
        t_x10 = t1 + ((int32_t)raw - a1) * (t2 - t1) / (a2 - a1);
    } else {
        t_x10 = 250 + ((int32_t)raw - 2018) * 10 * 806 / 5000;
    }
    int32_t t_c = t_x10 / 10;
    if (t_c > 127) t_c = 127;
    if (t_c < -128) t_c = -128;
    regs2[R2_TEMP_MCU - REGS2_BASE] = (uint8_t)(int8_t)t_c;
}
