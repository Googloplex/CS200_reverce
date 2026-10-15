// Motor: TIM1 → predriver, six-step Hall commutation, 1 kHz control loops, PWM input.
// CTRL_MODE = 0: stock controller (port of the stock firmware, see tag v0-clone).
// CTRL_MODE = 1: trapezoidal profile + PD with Hall speed feedback, closed-loop speed mode
// (CW = CCW = 0), wrap-around error for the 360° sensor.
// Common: Halls as a sensor (counter, period, speed), protections (current, speed, I²t), winding R and temperature estimate.
#include "motor.h"
#include "regs.h"
#include "board.h"
#include "sense.h"
#include "dxl.h"

typedef struct {
    uint8_t  running;
    uint8_t  prev_hall;
    uint8_t  hall;
    int16_t  cmd;
    int16_t  err;
    int16_t  out;
    int16_t  duty;
    int16_t  start;
    int16_t  target;
    int16_t  dist;
    int16_t  cw_limit;
    int16_t  ccw_limit;
    int16_t  speed;
    int16_t  goal;
    int32_t  integ;
    int32_t  kA, kB;
    uint32_t t_settle;
    int32_t  i_en;
    int32_t  goal_copy;
    int32_t  duty_copy;
    int32_t  vel_q8;            // stock profile speed, ADC units ×256
} ctl_t;

static ctl_t c;
static volatile uint8_t  g_recommutate;
static uint8_t           g_dir;
static volatile uint32_t g_last_hall_ms;
static volatile uint8_t  g_new_goal;
static int16_t pwm_in[3];

/* ---- Halls as a sensor ---- */
static const int8_t hall_sector[8] = { -1, 3, 1, 2, 5, 4, 0, -1 };
volatile int32_t  g_hall_count;
volatile uint32_t g_hall_period_us;
volatile uint32_t g_hall_last_us;
volatile int8_t   g_hall_dir = 1;
volatile uint8_t  g_hall_code;
volatile int32_t  g_speed;
volatile uint8_t  g_status;
volatile uint8_t  g_thermal_pct;
volatile uint16_t g_r_est_mohm;
volatile int8_t   g_t_est_c;
static int8_t     hall_prev_sector = -1;
static uint8_t    hall_bad_count;
static uint32_t   thermal_uj;       // heat in the winding, µJ
static uint8_t    i2t_limiting;
static int32_t    cur_cap = 1023, spd_cap = 1023;
static int32_t    r_est_acc;        // R estimate, mΩ ×1024
static uint16_t   r_est_n;

/* ---- impedance command (v1.3) ---- */
typedef struct { int16_t pos, vel; uint16_t kp, kd; int16_t ff; uint16_t tmax; uint8_t mode; uint16_t at; } imp_cmd_t;
static imp_cmd_t imp_pend, imp_act;
static uint8_t   imp_pend_valid;
static uint8_t   imp_last_seq;
// cubic target interpolation (mode 2): from (p0, v0) at t0 to (imp_act.pos, imp_act.vel) over T ms
static uint8_t   imp_interp;
static int32_t   imp_p0, imp_v0T, imp_v1T;   // units; v·T/1000 — distance over T at speed v
static uint32_t  imp_t0, imp_T;
static int32_t   imp_pt, imp_vt;              // current target and its speed (units, units/s)
static uint8_t   stall_ms_cnt, g_coast;
static int32_t   v_prev_cnt;                  // contact by deceleration
static uint8_t   contact_cnt, contact_hold;
/* sinusoidal commutation (CFG_SINE): angle 0..4095 = 360° el., vector = sector center + time interpolation */
static const int16_t g_q15[256] = {0,1206,2409,3607,4797,5976,7143,8294,9427,10539,11629,12695,13733,14743,15723,16671,17585,18464,19307,20113,20881,21610,22300,22950,23561,24131,24661,25152,25603,26016,26391,26729,27031,27298,27532,27733,27904,28045,28159,28246,28310,28352,28373,28376,28363,28335,28294,28243,28183,28116,28044,27969,27892,27814,27739,27666,27597,27533,27476,27425,27383,27350,27325,27311,27306,27311,27325,27350,27383,27425,27476,27533,27597,27666,27739,27814,27892,27969,28044,28116,28183,28243,28294,28335,28363,28376,28373,28352,28310,28246,28159,28045,27904,27733,27532,27298,27031,26729,26391,26016,25603,25152,24661,24131,23561,22950,22300,21610,20881,20113,19307,18464,17585,16671,15723,14743,13733,12695,11629,10539,9427,8294,7143,5976,4797,3607,2409,1206,0,-1206,-2409,-3607,-4797,-5976,-7143,-8294,-9427,-10539,-11629,-12695,-13733,-14743,-15723,-16671,-17585,-18464,-19307,-20113,-20881,-21610,-22300,-22950,-23561,-24131,-24661,-25152,-25603,-26016,-26391,-26729,-27031,-27298,-27532,-27733,-27904,-28045,-28159,-28246,-28310,-28352,-28373,-28376,-28363,-28335,-28294,-28243,-28183,-28116,-28044,-27969,-27892,-27814,-27739,-27666,-27597,-27533,-27476,-27425,-27383,-27350,-27325,-27311,-27306,-27311,-27325,-27350,-27383,-27425,-27476,-27533,-27597,-27666,-27739,-27814,-27892,-27969,-28044,-28116,-28183,-28243,-28294,-28335,-28363,-28376,-28373,-28352,-28310,-28246,-28159,-28045,-27904,-27733,-27532,-27298,-27031,-26729,-26391,-26016,-25603,-25152,-24661,-24131,-23561,-22950,-22300,-21610,-20881,-20113,-19307,-18464,-17585,-16671,-15723,-14743,-13733,-12695,-11629,-10539,-9427,-8294,-7143,-5976,-4797,-3607,-2409,-1206};   // sin θ + sin 3θ/6: third harmonic is common to all phases
static const uint16_t sect_angle[6] = {1707, 2389, 3072, 3755, 341, 1024};   // 150°+60°·k: six-step vector in sector k
volatile uint8_t         sine_on;             // mode is latched on TE 0→1 (global for the bench test)
static volatile uint16_t sine_theta_edge;     // angle at the last Hall edge
static volatile int8_t   sine_edge_dir;
static volatile uint32_t sine_step_q8;        // angle step per PWM period (Q8), computed on the edge; 0 = stopped
static volatile uint32_t sine_tick_lim;       // PWM periods after an edge after which the rotor is considered stopped
static volatile uint32_t sine_acc_q8, sine_ticks;
static uint32_t hall_t6[6];                   // times of the last 6 edges: period per electrical revolution is independent of Hall sensor placement
static uint8_t  hall_t6_i, hall_t6_n;
volatile uint32_t g_hall_period6_us;          // time of the last 6 steps (0 = none)
/* self-test / identification */
static uint8_t  st_state;
static uint32_t st_t0;
static int32_t  st_acc_i, st_acc_w, st_n, st_pos0, st_hall0;
static int32_t  st_u[3], st_i[3], st_w[3];
static int16_t  st_out;
static imp_cmd_t st_prev_act;                  // host command for the duration of the self-test
static int32_t  st_acc_d;
static int32_t  st_pos_h, st_hall_h;              // start point of the steady-state segment (step/unit, direction)
static uint8_t  st_bl_done;
volatile uint8_t g_latched;
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint32_t unl_until;                     // TEST_UNLIMITED: until which ms
static int32_t  iloop_integ;                   // current PI in torque mode, duty ×16
static uint32_t  imp_t_ms;          // when the active command was applied
volatile int32_t g_vel_cnt;
volatile uint32_t g_odo_steps;      // absolute Hall steps since startup         // speed in ADC units/s from Halls (in the direction of increasing position)

/* ---- v1 controller ---- */
static int32_t  p_cmd_q8;           // v1 profile: position, ADC units ×256
static int32_t  v_cmd;              // v1 profile: speed, ADC units/s
static int32_t  spd_integ;          // speed mode: integrator
static uint8_t  v1_active;

#define TAU_COOL_MS  60000u    // cooling time constant, ms
#define E_MAX_UJ     1600000000u   // thermal budget 1600 J [HYP]

uint32_t micros(void)
{
    uint32_t ms, val;
    do {
        ms = g_ms;
        val = SysTick->VAL;
    } while (ms != g_ms);
    return ms * 1000u + (((71999u - val) * 58255u) >> 22);   // /72 without division, exact over the whole range
}

/* ---------- timers ---------- */
void motor_tim1_init(void)
{
    TIM_CR1(TIM1_B) = (TIM_CR1(TIM1_B) & 0xFF8Fu & 0xFCFFu);
    TIM_ARR(TIM1_B) = 0x3FFu;
    TIM_PSC(TIM1_B) = 2u;
    TIM_RCR(TIM1_B) = 0;
    TIM_EGR(TIM1_B) = 1u;
    TIM_CCMR1(TIM1_B) = 0x6868u | (1u << 7) | (1u << 15);   // OC1/OC2: PWM1 + preload + OCxCE (OCREF_CLR enabled)
    TIM_CCMR2(TIM1_B) = 0x0068u | (1u << 7) | (7u << 12) | (1u << 11);   // OC3: PWM1 + preload + OC3CE; OC4: PWM2 + preload (ADC trigger)
    TIM_CCR1(TIM1_B) = 0;
    TIM_CCR2(TIM1_B) = 0;
    TIM_CCR3(TIM1_B) = 0;
    TIM_CCR4(TIM1_B) = 8;                                    // ADC sampling point, moved every tick
    TIM_CR2(TIM1_B) = 0;
    TIM_CCER(TIM1_B) = 0x555u | (1u << 12);   // CC4E: without it TIM1_OC4 gives no ADC trigger [MEAS]; not routed to a pin
    TIM_CR1(TIM1_B) |= 0x80u;
    TIM_BDTR(TIM1_B) = 0x042Fu;
    TIM_BDTR(TIM1_B) |= 0x8000u;
    TIM_CR1(TIM1_B) |= 1u;
}

void motor_tim6_init(void)
{
    TIM_ARR(TIM6_B) = 0xFFFFu;
    TIM_PSC(TIM6_B) = 0x8Fu;
    TIM_EGR(TIM6_B) = 1u;
    TIM_CR1(TIM6_B) = (TIM_CR1(TIM6_B) & ~8u) | 8u;
    TIM_SR(TIM6_B) = 0xFFFEu;
    TIM_DIER(TIM6_B) |= 1u;
}

void motor_tim14_init(void)
{
    TIM_CR1(TIM14_B) &= 0xFCFFu;
    TIM_ARR(TIM14_B) = 0x3E7u;
    TIM_PSC(TIM14_B) = 0x47u;
    TIM_EGR(TIM14_B) = 1u;
    TIM_SR(TIM14_B) &= ~1u;
    TIM_DIER(TIM14_B) |= 1u;
    TIM_CR1(TIM14_B) |= 1u;
}

void motor_tim15_init(void)
{
    TIM_CR1(TIM15_B) = (TIM_CR1(TIM15_B) & 0xFF8Fu & 0xFCFFu);
    TIM_ARR(TIM15_B) = 0xFFFFu;
    TIM_PSC(TIM15_B) = 0x47u;
    TIM_RCR(TIM15_B) = 0;
    TIM_EGR(TIM15_B) = 1u;
    TIM_CCER(TIM15_B) &= ~0x10u;
    TIM_CCMR1(TIM15_B) = (TIM_CCMR1(TIM15_B) & 0xCFFu) | (1u << 8);
    TIM_CCER(TIM15_B) = (TIM_CCER(TIM15_B) & 0xFF5Fu) | 0x10u;
    TIM_CCMR1(TIM15_B) &= 0xF3FFu;
    TIM_CCER(TIM15_B) &= ~1u;
    TIM_CCMR1(TIM15_B) = (TIM_CCMR1(TIM15_B) & 0xFF0Cu) | 2u;
    TIM_CCER(TIM15_B) = (TIM_CCER(TIM15_B) & 0xFFF5u) | 2u | 1u;
    TIM_CCMR1(TIM15_B) &= 0xFFF3u;
    TIM_CR1(TIM15_B) |= 0x10000u;
    TIM_CR1(TIM15_B) |= 0x80000u;
    TIM_SMCR(TIM15_B) = (TIM_SMCR(TIM15_B) & 0xFF8Fu) | 0x60u;
    TIM_SMCR(TIM15_B) = (TIM_SMCR(TIM15_B) & 0xFFF8u) | 4u;
    TIM_SMCR(TIM15_B) = (TIM_SMCR(TIM15_B) & 0xFF7Fu) | 0x80u;
    TIM_CR1(TIM15_B) |= 4u;
    TIM_SR(TIM15_B) &= ~7u;
    TIM_DIER(TIM15_B) |= 7u;
    TIM_CR1(TIM15_B) |= 1u;
}

static void selftest_start(void);
static void selftest_step(void);
static void impedance_control(void);

/* ---------- Halls ---------- */
static inline uint8_t hall_read(void)
{
    uint32_t idr = GPIO_IDR(GPIOA_B) & 0xFFFFu;
    return (uint8_t)(((idr >> 3) & 4u) | ((idr >> 5) & 2u) | ((idr >> 4) & 1u));
}

static void hall_edge(void)
{
    uint8_t h = hall_read();
    g_hall_code = h;
    int8_t sec = hall_sector[h];
    uint32_t now = micros();
    if (sec < 0) {
        g_status |= ST_HALL_BAD;
        return;
    }
    if (hall_prev_sector >= 0) {
        int d = (sec - hall_prev_sector + 6) % 6;
        if (d == 0) return;                                              // same sector: bounce/noise — not a period
        if ((now - g_hall_last_us) < 60u) return;                        // A→B→A bounce: faster than 16k steps/s does not happen
        if (d == 1)      { g_hall_count++; g_hall_dir = 1; g_odo_steps++; }
        else if (d == 5) { g_hall_count--; g_hall_dir = -1; g_odo_steps++; }
        if (d == 1 || d == 5) {
            uint32_t p = now - g_hall_last_us;
            if (p < 50u) p = 50u;                                        // ≤ 20000 steps/s
            g_hall_period_us = p;
            // average over 6 edges (electrical revolution) — free of Hall placement error
            uint32_t p6 = 0;
            if (hall_t6_n >= 6) { p6 = now - hall_t6[hall_t6_i]; if (p6 > 6u * 100000u) p6 = 0; }
            hall_t6[hall_t6_i] = now; hall_t6_i = (uint8_t)((hall_t6_i + 1u) % 6u);
            if (hall_t6_n < 6) hall_t6_n++;
            g_hall_period6_us = p6;
            uint32_t ps = p6 ? p6 / 6u : p;                              // period for the angle step
            // sine: angle step per 42.67 µs PWM period = 683·42.67/ps (Q8); stopped after 2 periods without an edge
            sine_step_q8 = (ps <= 20000u) ? 7460352u / ps : 0u;
            sine_tick_lim = (2u * p) / 43u + 1u;
        } else {
            hall_t6_n = 0;                                               // direction change/missed step — restart the average
        }
    }
    hall_prev_sector = sec;
    g_hall_last_us = now;
    // sine: moving +, enter the sector 30° before its center; moving −, 30° after
    sine_edge_dir = g_hall_dir;
    sine_theta_edge = (uint16_t)((sect_angle[sec] + (g_hall_dir < 0 ? 341u : 4096u - 341u)) & 0xFFFu);
    sine_acc_q8 = 0; sine_ticks = 0;
}

int8_t motor_out_sign(void)
{
    if (regs[REG_TORQUE_ENABLE] != 1 || c.duty == 0) return 0;
    return g_dir ? 1 : -1;
}

/* ---------- sinusoidal commutation: TIM1 update 23.4 kHz ---------- */
static inline int32_t g_a(uint32_t a) { return g_q15[(a >> 4) & 0xFFu]; }   // a: 0..4095 → sin a + sin 3a/6

void TIM1_NON_CC_IRQHandler(void)   // no divisions: ~150 cycles out of 3077 per PWM period
{
    TIM_SR(TIM1_B) &= ~1u;
    if (!sine_on) return;
    uint32_t amp = (uint32_t)(uint16_t)c.duty;
    if (amp == 0 || g_coast) {                               // brake (low-side switches) or coast
        TIM_CCR1(TIM1_B) = 0; TIM_CCR2(TIM1_B) = 0; TIM_CCR3(TIM1_B) = 0;
        return;
    }
    int8_t sec = hall_sector[g_hall_code];
    if (sec < 0) return;
    uint32_t theta;
    if (sine_step_q8 == 0 || ++sine_ticks > sine_tick_lim) {
        theta = sect_angle[sec];                             // slow/stopped: sector center, like six-step
    } else {
        sine_acc_q8 += sine_step_q8;
        if (sine_acc_q8 > (683u << 8)) sine_acc_q8 = 683u << 8;   // not past the end of the sector
        uint32_t adv = sine_acc_q8 >> 8;
        theta = sine_edge_dir < 0 ? (sine_theta_edge + 4096u - adv) : (sine_theta_edge + adv);
    }
    // phase advance — along the torque direction (g_dir), in units of 4096/360 ≈ 91/8 per degree
    int32_t lead = (int32_t)(int8_t)regs2[R2_SINE_ADV - REGS2_BASE] * 91 / 8;
    int32_t ofs = (int32_t)(int8_t)regs2[R2_SINE_OFS - REGS2_BASE] * 91 / 8;   // Hall sensor placement — independent of direction
    // (sin θ, sin θ−120°, sin θ+120°) gives a vector at θ−90° → +90°; g_dir = 1 (increasing position) = vector at 180°, like 7−h in six-step
    theta = (theta + 1024u + (uint32_t)ofs + (g_dir ? 2048u - (uint32_t)lead : (uint32_t)lead)) & 0xFFFu;
    // third harmonic 1/6 in the table → fundamental amplitude ×1.155: line voltage = Vbus·duty, same as six-step
    int32_t a = (int32_t)((amp * 591u) >> 10);
    TIM_CCR1(TIM1_B) = (uint32_t)(512 + ((a * g_a(theta)) >> 15));
    TIM_CCR2(TIM1_B) = (uint32_t)(512 + ((a * g_a(theta + 4096u - 1365u)) >> 15));
    TIM_CCR3(TIM1_B) = (uint32_t)(512 + ((a * g_a(theta + 1365u)) >> 15));
}

static void speed_update(void)
{
    uint32_t period = g_hall_period_us;
    uint32_t since = micros() - g_hall_last_us;
    if (since > 100000u || period == 0) {
        g_speed = 0;
        hall_t6_n = 0;
        return;
    }
    uint32_t p6 = g_hall_period6_us;
    if (p6 && since <= period * 2u) period = p6 / 6u;               // steady rotation: average over a revolution
    // rotor slowed down or stopped: speed no higher than 1/(time since last edge), otherwise the estimate
    // holds the old value for up to 100 ms and the PD D-term makes the drive oscillate at full duty
    if (since > period) period = since;
    int32_t s = (int32_t)(1000000u / period);
    g_speed = (g_hall_dir < 0) ? -s : s;
}

// speed in the direction of increasing position (ADC units): Hall sign relative to the potentiometer
static int32_t speed_pos_dir(void)
{
    int32_t s = -g_speed;                      // measured: as pos increases, the Hall counter decreases
    if (regs[REG_CFG_FLAGS] & CFG_SPEED_INV) s = -s;
    return s;
}

/* ---------- commutation ---------- */
static inline void cc_enable(unsigned shift, unsigned on)
{
    TIM_CCER(TIM1_B) = (TIM_CCER(TIM1_B) & ~(1u << shift) & 0xFFFFu) | ((on << shift) & 0xFFFFu);
}

static void commutate(void)
{
    if (g_coast) return;                       // coast: all switches off, no commutation needed
    __disable_irq();                           // read Halls and write CCR in one piece: EXTI cannot get in between
    uint8_t h = hall_read();
    c.hall = g_dir ? (uint8_t)(7 - h) : h;
    c.duty_copy = c.duty;
    uint32_t d = (uint32_t)(uint16_t)c.duty;
    TIM_CCER(TIM1_B) |= 0x444u;                // complementary outputs (may have been removed by coast)
    if (sine_on) {                             // sine: all three phases PWM, values written by TIM1_NON_CC_IRQHandler
        TIM_CCER(TIM1_B) |= 0x555u;
        __enable_irq();
        c.prev_hall = c.hall;
        return;
    }
    switch (c.hall) {
    case 6: TIM_CCR3(TIM1_B) = 0; TIM_CCR2(TIM1_B) = d; TIM_CCR1(TIM1_B) = 0; cc_enable(4, 1); cc_enable(0, 1); cc_enable(8, 0); break;
    case 2: TIM_CCR3(TIM1_B) = d; TIM_CCR2(TIM1_B) = 0; TIM_CCR1(TIM1_B) = 0; cc_enable(8, 1); cc_enable(0, 1); cc_enable(4, 0); break;
    case 3: TIM_CCR3(TIM1_B) = d; TIM_CCR2(TIM1_B) = 0; TIM_CCR1(TIM1_B) = 0; cc_enable(4, 1); cc_enable(8, 1); cc_enable(0, 0); break;
    case 1: TIM_CCR3(TIM1_B) = 0; TIM_CCR2(TIM1_B) = 0; TIM_CCR1(TIM1_B) = d; cc_enable(4, 1); cc_enable(0, 1); cc_enable(8, 0); break;
    case 5: TIM_CCR3(TIM1_B) = 0; TIM_CCR2(TIM1_B) = 0; TIM_CCR1(TIM1_B) = d; cc_enable(8, 1); cc_enable(0, 1); cc_enable(4, 0); break;
    case 4: TIM_CCR3(TIM1_B) = 0; TIM_CCR2(TIM1_B) = d; TIM_CCR1(TIM1_B) = 0; cc_enable(4, 1); cc_enable(8, 1); cc_enable(0, 0); break;
    default:
        g_status |= ST_HALL_BAD;
        if (hall_bad_count < 255) hall_bad_count++;
        break;
    }
    __enable_irq();
    if (hall_sector[h] >= 0) hall_bad_count = 0;
    c.prev_hall = c.hall;
}

void EXTI4_15_IRQHandler(void)
{
    for (uint32_t line = 0x10u; line <= 0x40u; line <<= 1) {
        if ((EXTI_PR & line) && (EXTI_IMR & line)) {
            EXTI_PR = line;
            g_last_hall_ms = g_ms;
            hall_edge();
            commutate();
            g_recommutate = 0;
        }
    }
}

void motor_new_goal(void)
{
    g_new_goal = 1;
    c.i_en = 1;
    c.integ = 0;
}

/* ---------- PWM input ---------- */
void TIM15_IRQHandler(void)
{
    if ((TIM_SR(TIM15_B) & 1u) && (TIM_DIER(TIM15_B) & 1u)) {
        TIM_SR(TIM15_B) &= ~1u;
    }
    if ((TIM_SR(TIM15_B) & 2u) && (TIM_DIER(TIM15_B) & 2u)) {
        TIM_SR(TIM15_B) &= ~2u;
        int16_t w = (int16_t)TIM_CCR1(TIM15_B);
        pwm_in[1] = w;
        int16_t lo = calib.pwm_min, hi = calib.pwm_max;
        if ((int32_t)w > (int32_t)lo - 50 && (int32_t)w < (int32_t)hi + 50) {
            if (w < lo) pwm_in[1] = lo;
            if (pwm_in[1] > hi) pwm_in[1] = hi;
            w = pwm_in[1];
            set_reg16(REG_GOAL, (uint16_t)w);
            if ((uint32_t)((int32_t)pwm_in[2] - (int32_t)w + 50) > 100u) {
                regs[REG_TORQUE_LIMIT] = regs[REG_MAX_TORQUE];
                regs[REG_TORQUE_LIMIT + 1] = regs[REG_MAX_TORQUE + 1];
                motor_new_goal();
            }
            pwm_in[2] = w;
            regs[REG_TORQUE_ENABLE] = 1;
        }
    }
    if ((TIM_SR(TIM15_B) & 4u) && (TIM_DIER(TIM15_B) & 4u)) {
        TIM_SR(TIM15_B) &= ~4u;
        pwm_in[0] = (int16_t)TIM_CCR2(TIM15_B);
    }
}

/* ---------- target: µs → ADC units (shared by both controllers, stock semantics) ---------- */
static int16_t goal_to_adc(void)
{
    int16_t goal_us = (int16_t)reg16(REG_GOAL);
    if (goal_us > 0x834) goal_us = 0x834;
    else if (goal_us < 900) goal_us = 900;
    calib.travel = reg16(REG_TRAVEL);
    c.kA = regs[REG_K_CW]  ? regs[REG_K_CW]  : (int32_t)calib.travel;
    c.kB = regs[REG_K_CCW] ? regs[REG_K_CCW] : (int32_t)calib.travel;
    // stock: 2048 ± (|g−1500|·K/460·4095/600) = ± num·4095/276000, truncation toward zero as with float
    int32_t g = goal_us, out;
    if (g > 1500) {
        uint32_t num = (uint32_t)(g - 1500) * (uint32_t)c.kB;
        out = 2048 + (int32_t)((num / 276000u) * 4095u + (num % 276000u) * 4095u / 276000u);
    } else {
        uint32_t num = (uint32_t)(1500 - g) * (uint32_t)c.kA;
        out = 2048 - (int32_t)((num / 276000u) * 4095u + ((num % 276000u) * 4095u + 275999u) / 276000u);
    }
    if (out < 0) out = 0;
    if (out > 0xFFF) out = 0xFFF;
    return (int16_t)out;
}

/* ---------- stock controller ---------- */
static void position_control_stock(void)
{
    uint8_t slope_cw = regs[REG_COMPL_SLOPE_CW], slope_ccw = regs[REG_COMPL_SLOPE_CCW];
    c.goal = goal_to_adc();
    c.goal_copy = c.goal;
    set_reg16(REG_GOAL_ADC, (uint16_t)c.goal);

    if (!c.running) {
        c.running = 1;
        c.vel_q8 = 0;
        c.start = g_present_pos;
        c.target = g_present_pos;
    }
    if (g_new_goal) {
        int16_t tg = c.target, st = c.start, gl = c.goal;
        if (((tg <= st) || (gl <= tg)) && ((st <= tg) || (tg <= gl)) && tg != st && gl != tg) {
            g_new_goal = 0;
            c.vel_q8 = 0;
            c.start = tg;
            c.cmd = tg;
        }
    }
    c.dist = (c.start < c.goal) ? (int16_t)(c.goal - c.start) : (int16_t)(c.start - c.goal);

    c.vel_q8 += (int32_t)c.speed * 65536 / (int32_t)(calib.speed_k_x256 ? calib.speed_k_x256 : 1u);   // vel += speed/speed_k
    if ((int32_t)c.dist * 256 > c.vel_q8) {
        if (c.goal > c.start) c.target = (int16_t)(c.start + (c.vel_q8 >> 8));                 // (int)(start + vel)
        else                  c.target = (int16_t)(c.start - ((c.vel_q8 + 255) >> 8));         // (int)(start − vel): truncation toward zero
        c.cmd = c.target;
    } else {
        c.vel_q8 = 0;
        c.start = c.goal;
        c.target = c.goal;
        c.cmd = c.goal;
    }

    int32_t err = (int16_t)(c.cmd - g_present_pos);
    c.err = (int16_t)err;
    if ((uint32_t)(err + 49) < 99u) {
        c.integ += err;
    }
    int32_t slope;
    int32_t margin = c.i_en ? 1 : 5;
    if (err < 0) {
        err += margin;
        if (err >= 0) err = 0;
        slope = slope_cw ? slope_cw : 1;
    } else {
        err -= margin;
        if (err <= 0) err = 0;
        slope = slope_ccw ? slope_ccw : 1;
    }
    c.err = (int16_t)err;
    if ((uint32_t)(err + 4) < 9u) {
        if ((uint32_t)(g_ms - c.t_settle) > 10u) {
            c.integ = 0;
            c.i_en = 0;
            g_new_goal = 0;
            c.vel_q8 = 0;
            c.start = c.target;
            c.cmd = c.target;
        }
    } else {
        c.t_settle = g_ms;
    }
    if (c.i_en != 1) {
        c.out = (int16_t)(err * 100 / slope);                               // err·(100/slope), truncation toward zero
    } else {
        c.out = (int16_t)((err * 10000 / slope + c.integ) / 100);           // + integ·0.01, single truncation
    }
}

static void wheel_mode_stock(void)
{
    if (c.speed & 0x400) {
        c.speed = (int16_t)-(c.speed & 0x3FF);
    }
    c.out = c.speed;
}

/* ---------- v1 controller ---------- */
static uint32_t isqrt32(uint32_t x)   // integer square root, bitwise
{
    uint32_t r = 0, bit = 1u << 30;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

static void position_control_v1(void)
{
    int16_t goal = goal_to_adc();
    c.goal = goal;
    set_reg16(REG_GOAL_ADC, (uint16_t)goal);
    int wrap = (regs[REG_CFG_FLAGS] & CFG_WRAP) != 0;

    if (!v1_active) {                       // start from the current position, no jerk
        v1_active = 1;
        p_cmd_q8 = (int32_t)g_present_pos << 8;
        v_cmd = 0;
        c.running = 1;
    }
    // profile: trapezoidal by vmax/acc, or instantaneous (position in Q8, speed in units/s, acceleration in units/s²)
    int32_t vmax = reg16(REG_PROFILE_VMAX);
    int32_t acc = (int32_t)reg16(REG_PROFILE_ACC) * 100;
    int32_t dist_q8 = ((int32_t)goal << 8) - p_cmd_q8;
    if (wrap) {
        while (dist_q8 > (2048 << 8)) dist_q8 -= 4096 << 8;
        while (dist_q8 < -(2048 << 8)) dist_q8 += 4096 << 8;
    }
    if (vmax <= 0 && acc <= 0) {
        p_cmd_q8 = (int32_t)goal << 8;
        v_cmd = 0;
    } else {
        int32_t ad = (dist_q8 < 0 ? -dist_q8 : dist_q8) >> 8;              // units
        int32_t vt;
        if (acc > 0) {
            vt = (int32_t)isqrt32((uint32_t)(acc / 32) * (uint32_t)ad) * 8;   // sqrt(2·a·d) = sqrt(a/32·d)·8
            if (vmax > 0 && vt > vmax) vt = vmax;
        } else {
            vt = vmax;
        }
        if (dist_q8 < 0) vt = -vt;
        int32_t dv = vt - v_cmd;
        int32_t dvmax = (acc > 0) ? (acc + 999) / 1000 : 0x7FFFFFFF;        // speed increment per 1 ms
        if (dv > dvmax) dv = dvmax;
        if (dv < -dvmax) dv = -dvmax;
        v_cmd += dv;
        int32_t step_q8 = v_cmd * 256 / 1000;
        if ((dist_q8 >= 0 && step_q8 > dist_q8) || (dist_q8 < 0 && step_q8 < dist_q8)) {   // do not overshoot the target
            step_q8 = dist_q8;
            v_cmd = 0;
        }
        p_cmd_q8 += step_q8;
        if (wrap) {
            while (p_cmd_q8 >= (4096 << 8)) p_cmd_q8 -= 4096 << 8;
            while (p_cmd_q8 < 0) p_cmd_q8 += 4096 << 8;
        }
    }
    c.cmd = (int16_t)(p_cmd_q8 >> 8);

    // position error
    int32_t err = (int32_t)(int16_t)(c.cmd - g_present_pos);
    if (wrap) {
        while (err > 2048) err -= 4096;
        while (err < -2048) err += 4096;
    }
    c.err = (int16_t)err;

    // PD: profile and actual speed — in Hall steps/s
    int32_t spc = regs[REG_STEPS_PER_CNT];                           // ×0.001 steps per unit
    int32_t v_cmd_steps = v_cmd * spc / 1000;
    int32_t v_err = v_cmd_steps - speed_pos_dir();
    int32_t out = err * (int32_t)regs[REG_KP] / 10 + v_err * (int32_t)regs[REG_KD] / 100;
    if (out > 1023) out = 1023;
    if (out < -1023) out = -1023;
    c.out = (int16_t)out;
}

/* mA of feedforward torque → duty: u = I·R + Ke·ω; R from R_EST (or R_REF), Ke from KV/PP */
static int32_t ff_duty(int32_t ff_ma)
{
    int32_t r_mohm = clampi(reg16(REG_R_REF), 50, 5000);   // model: R_REF (R_EST drifts under fast transients)
    int32_t kv = reg16(REG_KV); int32_t pp = regs[REG_PP] ? regs[REG_PP] : 1;
    ff_ma = clampi(ff_ma, -30000, 30000);
    int32_t u_mv = ff_ma * r_mohm / 1000;
    if (kv > 0) u_mv += speed_pos_dir() * 10 / pp * 1000 / kv;          // back-EMF, mV
    int32_t vb = reg16(REG_VBUS_MV); if (vb < 5000) vb = 5000;
    int32_t d = u_mv * 1024 / vb;
    // torque mode (Kp = Kd = 0) and CFG_ILOOP: slow integrator on measured current on top of the model (shunt filtered → ~50 Hz)
    if ((regs[REG_CFG_FLAGS] & CFG_ILOOP) && imp_act.kp == 0 && imp_act.kd == 0 && ff_ma != 0) {
        int32_t want = ff_ma < 0 ? -ff_ma : ff_ma;
        int32_t w = speed_pos_dir();
        // the shunt sees only current magnitude: integrate when current flows in the ff direction (rotor stopped or turning along ff)
        int ok_dir = (w > -200 && w < 200) || ((w > 0) == (ff_ma > 0));
        if (g_i_valid && want > 300 && ok_dir) {
            int32_t e = want - (int32_t)g_i_wind_ma;                     // mA
            iloop_integ += e / 32;                                       // ~0.002 duty/mA per ms (bandwidth below the shunt filter)
            if (iloop_integ > 200 * 16) iloop_integ = 200 * 16;
            if (iloop_integ < -200 * 16) iloop_integ = -200 * 16;
        }
        d += (ff_ma < 0 ? -(iloop_integ / 16) : (iloop_integ / 16));
    } else {
        iloop_integ = 0;
    }
    return d;
}

/* duty ceiling at which winding current does not exceed i_ma: u = I·R + E, E — back-EMF along the applied voltage
   (when rotating against it E < 0 and the ceiling drops). No shunt filter lag: limiting by the model, measurement is
   a safety net (cap_update below) */
static int32_t duty_for_current(uint32_t i_ma, int dir_pos)
{
    int32_t r = clampi(reg16(REG_R_REF), 50, 5000);
    if (i_ma > 30000u) i_ma = 30000u;
    int32_t kv = reg16(REG_KV); int32_t pp = regs[REG_PP] ? regs[REG_PP] : 1;
    int32_t w = speed_pos_dir(); if (!dir_pos) w = -w;
    int32_t u_mv = (int32_t)i_ma * r / 1000;
    if (kv > 0) u_mv += w * 10 / pp * 1000 / kv;
    int32_t vb = reg16(REG_VBUS_MV); if (vb < 5000) vb = 5000;
    int32_t d = u_mv * 1024 / vb;
    return d < 0 ? 0 : (d > 1023 ? 1023 : d);
}

static void imp_take_command(void)
{
    uint8_t seq = regs2[R2_CMD_SEQ - REGS2_BASE];
    if (g_cmd_new || seq != imp_last_seq) {   // block written over the bus (flag) or via SWD (sequence number changed)
        g_cmd_new = 0;
        imp_last_seq = seq;
        imp_pend.pos = (int16_t)r2_16(R2_CMD_POS); imp_pend.vel = (int16_t)r2_16(R2_CMD_VEL);
        imp_pend.kp = r2_16(R2_CMD_KP); imp_pend.kd = r2_16(R2_CMD_KD);
        imp_pend.ff = (int16_t)r2_16(R2_CMD_FF); imp_pend.tmax = r2_16(R2_CMD_TMAX);
        imp_pend.mode = regs2[R2_CMD_MODE - REGS2_BASE]; imp_pend.at = r2_16(R2_CMD_AT);
        imp_pend_valid = 1;
    }
    if (imp_pend_valid) {
        uint16_t now16 = (uint16_t)(g_ms + g_time_offset);
        int16_t dt = (int16_t)(imp_pend.at - now16);
        // new command — new stall/contact event: flags and counters reset
        stall_ms_cnt = 0; contact_cnt = 0;
        regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~(ST2_STALL | ST2_CONTACT);
        if (imp_pend.mode == 2 && imp_pend.at != 0 && dt > 0) {
            // interpolation: start from the current target and its speed so that 100 Hz from the host becomes a smooth 1 kHz setpoint
            int live = (regs[REG_TORQUE_ENABLE] == 1 && imp_act.mode != 0 && !st_state);
            imp_p0 = live ? imp_pt : g_present_pos;
            int32_t v0 = live ? imp_vt : 0;
            imp_T = (uint32_t)dt;
            imp_t0 = g_ms;
            imp_v0T = clampi(v0 * (int32_t)imp_T / 1000, -65536, 65536);
            imp_v1T = clampi((int32_t)imp_pend.vel * (int32_t)imp_T / 1000, -65536, 65536);
            imp_act = imp_pend;
            imp_interp = 1;
            imp_pend_valid = 0;
            regs[REG_TORQUE_ENABLE] = 1;
        } else if (imp_pend.at == 0 || dt <= 0) {
            imp_act = imp_pend;
            imp_t_ms = g_ms;
            imp_interp = 0;
            imp_pend_valid = 0;
            if (imp_act.mode != 0) regs[REG_TORQUE_ENABLE] = 1;
        }
    }
}

/* target and its speed for the current ms: cubic Hermite (Q10) or first-order hold 100 ms */
static void imp_target(void)
{
    if (imp_interp) {
        uint32_t el = g_ms - imp_t0;
        if (el >= imp_T) {
            imp_interp = 0;
            imp_t_ms = imp_t0 + imp_T;
        } else {
            int32_t s = (int32_t)(el * 1024u / imp_T);
            int32_t s2 = (s * s) >> 10, s3 = (s2 * s) >> 10;
            int32_t h00 = 2 * s3 - 3 * s2 + 1024, h10 = s3 - 2 * s2 + s, h11 = s3 - s2;
            int32_t p1 = imp_act.pos;
            if (regs[REG_CFG_FLAGS] & CFG_WRAP) {                        // ring: to the target along the shortest path
                int32_t dd = p1 - imp_p0;
                while (dd > 2048) dd -= 4096;
                while (dd < -2048) dd += 4096;
                p1 = imp_p0 + dd;
            }
            imp_pt = (h00 * imp_p0 + (1024 - h00) * p1 + h10 * imp_v0T + h11 * imp_v1T) >> 10;
            int32_t dnum = (6 * s2 - 6 * s) * (imp_p0 - p1) + (3 * s2 - 4 * s + 1024) * imp_v0T + (3 * s2 - 2 * s) * imp_v1T;
            int32_t q = clampi(dnum / (int32_t)imp_T, -10000000, 10000000);   // Q10 units/ms
            imp_vt = clampi((q * 125) >> 7, -32767, 32767);                   // ×1000/1024 → units/s
            return;
        }
    }
    uint32_t age = g_ms - imp_t_ms;
    if (age > 100u) age = 100u;
    imp_pt = (int32_t)imp_act.pos + (int32_t)imp_act.vel * (int32_t)age / 1000;
    imp_vt = (age < 100u) ? imp_act.vel : 0;
}

/* "soft" fault (bus timeout, collision): per FAULT_MODE — hold in place or torque off */
static void soft_fault(void)
{
    uint8_t fm = regs2[R2_FAULT_MODE - REGS2_BASE];
    if (fm == 2) {
        imp_act.pos = g_present_pos; imp_act.vel = 0;
        if (imp_act.mode == 0) { imp_act.kp = r2_16(R2_CMD_KP); imp_act.kd = r2_16(R2_CMD_KD); }
        imp_act.ff = 0; imp_act.tmax = r2_16(R2_HOLD_TMAX); imp_act.mode = 1;
        imp_interp = 0; imp_t_ms = g_ms;
        imp_pend_valid = 0;
        regs[REG_TORQUE_ENABLE] = 1;
        regs2[R2_STATUS2 - REGS2_BASE] |= ST2_HOLD;
    } else {
        regs[REG_TORQUE_ENABLE] = 0;
    }
}

static void bus_timeout_check(void)   // from TIM14
{
    uint16_t tmo = reg16(REG_BUS_TIMEOUT);
    if (tmo && regs[REG_TORQUE_ENABLE] && !(g_status & ST_BUS_TMO) && (uint32_t)(g_ms - g_last_rx_ms) > tmo) {
        g_status |= ST_BUS_TMO;
        soft_fault();
    }
}

static void vbus_check(void)   // from TIM14: 5 ms in a row past threshold, not earlier than 500 ms after startup (bus not up yet)
{
    static uint8_t hi_n, lo_n;
    if (g_ms < 500u) return;
    uint16_t vmax = reg16(REG_VBUS_MAX), vmin = reg16(REG_VBUS_MIN), vb = reg16(REG_VBUS_MV);
    if (vmax && vb > vmax) { if (hi_n < 255) hi_n++; } else hi_n = 0;
    if (vmin && vb < vmin) { if (lo_n < 255) lo_n++; } else lo_n = 0;
    if (hi_n >= 5) { regs[REG_TORQUE_ENABLE] = 0; g_status |= ST_VBUS_HIGH; if (regs[REG_CFG_FLAGS] & CFG_LATCH) g_latched = 1; }
    else if (hi_n == 0) g_status &= (uint8_t)~ST_VBUS_HIGH;
    if (lo_n >= 5) { regs[REG_TORQUE_ENABLE] = 0; g_status |= ST_VBUS_LOW;  if (regs[REG_CFG_FLAGS] & CFG_LATCH) g_latched = 1; }
    else if (lo_n == 0) g_status &= (uint8_t)~ST_VBUS_LOW;
}


static void motor_coast(void)   // coast: all six outputs removed → predriver keeps switches off
{
    __disable_irq();
    TIM_CCR1(TIM1_B) = 0; TIM_CCR2(TIM1_B) = 0; TIM_CCR3(TIM1_B) = 0;
    TIM_CCER(TIM1_B) = (TIM_CCER(TIM1_B) & ~0x555u) & 0xFFFFu;
    g_coast = 1;
    __enable_irq();
}

static void impedance_control(void)
{
    int wrap = (regs[REG_CFG_FLAGS] & CFG_WRAP) != 0;
    imp_target();
    int32_t p_t = imp_pt;
    uint8_t *st2 = &regs2[R2_STATUS2 - REGS2_BASE];
    *st2 = (uint8_t)((*st2 & ~(ST2_INTERP | ST2_SOFT)) | (imp_interp ? ST2_INTERP : 0));
    // soft limits: target does not go past them, beyond a limit — spring SOFT_KP (link does not hit the mechanical hard stop)
    int32_t smin = r2_16(R2_SOFT_MIN), smax = r2_16(R2_SOFT_MAX), soft = 0;
    if (smin && smax && smax <= smin) smin = smax = 0;                   // meaningless pair — ignore
    if (smin) {
        if (p_t < smin) p_t = smin;
        if (g_present_pos < smin) soft = (smin - g_present_pos) * regs2[R2_SOFT_KP - REGS2_BASE] / 10;
    }
    if (smax) {
        if (p_t > smax) p_t = smax;
        if (g_present_pos > smax) soft = (smax - g_present_pos) * regs2[R2_SOFT_KP - REGS2_BASE] / 10;
    }
    if (soft) *st2 |= ST2_SOFT;
    int32_t err = clampi(p_t - g_present_pos, -4095, 4095);
    if (wrap) { while (err > 2048) err -= 4096; while (err < -2048) err += 4096; }
    int32_t verr = clampi(imp_vt - g_vel_cnt, -30000, 30000);
    int32_t out = err * (int32_t)imp_act.kp / 100 + verr * (int32_t)imp_act.kd / 1000 + ff_duty(imp_act.ff) + soft;
    int32_t tmax = imp_act.tmax ? imp_act.tmax : 1023;
    if (tmax > 1023) tmax = 1023;
    if (out > tmax) out = tmax;
    if (out < -tmax) out = -tmax;
    c.out = (int16_t)out;
    c.cmd = (int16_t)p_t;
    c.err = (int16_t)err;
}

static void speed_control_v1(void)   // CW = CCW = 0: closed-loop speed mode, setpoint as in AX-12 (bit 10 — sign)
{
    int32_t sp = c.speed & 0x3FF;
    if (c.speed & 0x400) sp = -sp;
    int32_t target = sp * 2;                                          // steps/s (1023 → ~2046 ≈ 14 V on the winding)
    int32_t e = target - speed_pos_dir();
    spd_integ += e;
    if (spd_integ > 200000) spd_integ = 200000;
    if (spd_integ < -200000) spd_integ = -200000;
    int32_t out = e * (int32_t)regs[REG_KP] / 20 + spd_integ / 200;
    if (out > 1023) out = 1023;
    if (out < -1023) out = -1023;
    c.out = (int16_t)out;
}

/* ---------- thermal model and R/T estimate ---------- */
static void thermal_update(void)
{
    // per 1 ms: +I²R (µJ: (mA²/1000)·mΩ/1000) − E/τ
    uint32_t i = g_i_wind_ma;
    uint32_t r = reg16(REG_R_REF) ? reg16(REG_R_REF) : 550u;
    uint32_t heat = (i * i / 1000u) * r / 1000u;
    uint32_t cool = thermal_uj / TAU_COOL_MS;
    if (r > 5000u) r = 5000u;
    thermal_uj = (thermal_uj + heat > cool) ? thermal_uj + heat - cool : 0u;
    if (thermal_uj > 2u * E_MAX_UJ) thermal_uj = 2u * E_MAX_UJ;        // equilibrium at 15 A stall would overflow u32
    uint32_t pct = thermal_uj / (E_MAX_UJ / 100u);
    g_thermal_pct = (uint8_t)(pct > 255u ? 255u : pct);
    if (reg16(REG_I_CONT)) {
        if (thermal_uj > E_MAX_UJ) i2t_limiting = 1;
        else if (thermal_uj < E_MAX_UJ / 5u * 4u) i2t_limiting = 0;
    } else {
        i2t_limiting = 0;
    }
}

// R = (Vbus·D − Ke·ω) / I when current is significant; Ke from KV and PP. IIR averaging ~2 s. T = 25 + (R/R_ref − 1)/0.00393.
static void r_estimate_update(void)
{
    static uint16_t last_duty; static uint8_t steady_ms;
    uint32_t i_ma = g_i_wind_ma;
    uint16_t duty = (uint16_t)c.duty;
    int32_t dd = (int32_t)duty - (int32_t)last_duty; last_duty = duty;
    if (dd > 4 || dd < -4) steady_ms = 0; else if (steady_ms < 255) steady_ms++;
    if (steady_ms < 20) return;                                  // current lags through the shunt filter: compute only at steady duty
    if (!g_i_valid || i_ma < 800u || duty < 100u) return;
    int32_t u_mv = (int32_t)reg16(REG_VBUS_MV) * duty / 1024;
    int32_t sp = g_speed < 0 ? -g_speed : g_speed;
    int32_t kv = reg16(REG_KV);
    int32_t pp = regs[REG_PP] ? regs[REG_PP] : 1;
    if (kv > 0) u_mv -= sp * 10 / pp * 1000 / kv;            // back-EMF: steps/s × 60/(6·pp) / Kv, mV
    if (u_mv <= 50) return;
    int32_t r = u_mv * 1000 / (int32_t)i_ma;                 // mΩ
    if (r < 50 || r > 5000) return;
    int32_t r_q10 = r << 10;
    if (r_est_n < 2000) {
        r_est_n++;
        r_est_acc += (r_q10 - r_est_acc) / (int32_t)r_est_n;  // average over the first samples
    } else {
        r_est_acc += (r_q10 - r_est_acc) >> 11;              // then IIR ~2 s
    }
    g_r_est_mohm = (uint16_t)(r_est_acc >> 10);
    int32_t rref = reg16(REG_R_REF);
    if (rref > 0) {                                          // T = 25 + (R/R_ref − 1)/0.00393
        int32_t t = 25 + ((int32_t)g_r_est_mohm - rref) * 25445 / (rref * 100);
        if (t < -40) t = -40;
        if (t > 125) t = 125;
        g_t_est_c = (int8_t)t;
    }
}

/* ---------- limiters ---------- */
static uint8_t cur_over_ms, spd_over_ms;
static int32_t cap_update(int32_t cap, uint32_t value, uint32_t limit, int32_t duty, uint8_t *over_ms)
{
    if (!limit) { *over_ms = 0; return 1023; }
    if (value > limit) {
        if (*over_ms < 255) (*over_ms)++;
        if (*over_ms >= 5) {                                            // persists ≥ 5 ms — not an estimate spike on a transient
            int32_t want = (int32_t)((uint32_t)duty * limit / value);   // proportional reduction
            if (want < cap) cap = want;
            cap -= 1;
        }
    } else {
        *over_ms = 0;
        cap += 4;
    }
    if (cap < 8) cap = 8;
    if (cap > 1023) cap = 1023;
    return cap;
}

static void apply_output(void)
{
    int16_t punch = (int16_t)reg16(REG_PUNCH);
    int16_t tlim = (int16_t)reg16(REG_TORQUE_LIMIT);
    int32_t out = c.out;
    if (out > 0 && out < punch) {
        c.out = punch;
    } else if (out < 0 && out > -punch) {
        c.out = (int16_t)-punch;
    }
    out = c.out;
    if (out < 0) {
        out = -out;
        if (out > tlim) out = tlim;
        g_dir = 0;
    } else {
        if (out > tlim) out = tlim;
        g_dir = 1;
    }

    g_status &= (uint8_t)~(ST_CUR_LIMIT | ST_SPD_LIMIT | ST_I2T);
    uint8_t st_open = (uint8_t)(st_state && st_state < 6);        // self-test phases 1–5: known duty ≤ 300, no ceilings; return — with ceiling
    uint32_t ilim = (unl_until || st_open) ? 0u : reg16(REG_LIMIT_CURRENT);   // TEST_UNLIMITED and self-test: no ceilings
    if (i2t_limiting && !unl_until && !st_open) {                            // I²t also cuts the model ceiling
        uint32_t ic = reg16(REG_I_CONT);
        if (!ilim || ic < ilim) ilim = ic;
        g_status |= ST_I2T;
    }
    if (ilim) {
        int32_t cm = duty_for_current(ilim, g_dir);
        if (out > cm) { out = cm; g_status |= ST_CUR_LIMIT; }
    }
    // safety net: the model holds current at the limit, measurement overshoots on transients → threshold 1.5·limit; with an invalid estimate (duty < 80)
    // the ceiling grows rather than freezes (otherwise deadlock: collapsed below 80 → estimate invalid → does not grow)
    cur_cap = cap_update(cur_cap, g_i_valid ? g_i_wind_ma : 0, ilim + ilim / 2, c.duty, &cur_over_ms);
    uint32_t sp = (uint32_t)(g_speed < 0 ? -g_speed : g_speed);
    spd_cap = cap_update(spd_cap, sp, (unl_until || st_open) ? 0u : reg16(REG_LIMIT_SPEED), c.duty, &spd_over_ms);
    if (out > cur_cap) { out = cur_cap; g_status |= ST_CUR_LIMIT; }
    if (out > spd_cap) { out = spd_cap; g_status |= ST_SPD_LIMIT; }

    c.out = (int16_t)out;
    c.duty = (int16_t)out;
    // current sampling point: middle of the high-side on-window minus ~1.5 µs for the ADC channel queue
    {
        int32_t s = (out > 70) ? (out / 2 - 30) : 8;
        if (s < 8) s = 8;
        TIM_CCR4(TIM1_B) = (uint32_t)s;
    }
    if (c.duty == 0) {
        g_recommutate = 1;
    }
    if (g_recommutate) {
        commutate();
    } else if (!sine_on && !g_coast) {                            // six-step: new duty into the active phase every tick, do not wait for an edge
        switch (c.hall) {
        case 6: case 4: TIM_CCR2(TIM1_B) = (uint32_t)out; break;
        case 2: case 3: TIM_CCR3(TIM1_B) = (uint32_t)out; break;
        case 1: case 5: TIM_CCR1(TIM1_B) = (uint32_t)out; break;
        default: break;
        }
    }
}

/* ---------- self-test / identification with one command (CFG2_CMD = 2) ----------
   0 Halls; 1 +duty 120 150 ms → Hall direction vs position and steps/unit; 2–4 three operating points (+150, −300, +300)
   → R and Kv by the two-point method (U = I·R + Ke·ω); 5 friction: duty ramps 1/ms until breakaway; 6 return via impedance. */
static void selftest_start(void)
{
    int32_t p = g_present_pos;
    uint8_t *res = &regs2[R2_ST_RESULT - REGS2_BASE];
    *res = 0;
    if (p < 1000 || p > 3000) { *res = 0x40 | 0x80; return; }    // excursion up to +750/−200 units from start
    st_state = 1; st_t0 = g_ms; st_pos0 = p; st_hall0 = g_hall_count; st_acc_i = st_acc_w = st_n = 0;
    st_out = 120; st_bl_done = 0;
    st_prev_act = imp_act;
    if (hall_sector[g_hall_code] >= 0) *res |= 0x01;
    regs[REG_TORQUE_ENABLE] = 1;
    regs2[R2_STATUS2 - REGS2_BASE] |= ST2_SELFTEST;
}

static void selftest_point(unsigned k, int16_t out, uint32_t hold_ms)
{
    uint32_t el = g_ms - st_t0;
    c.out = out;
    if (el >= hold_ms / 2u) {                                    // averaging over the second half
        int32_t w = g_speed < 0 ? -g_speed : g_speed;
        st_acc_i += g_i_wind_ma; st_acc_w += w; st_acc_d += c.duty; st_n++;   // c.duty — applied on the previous tick (after ceilings and TL)
    }
    if (el >= hold_ms) {
        st_u[k] = st_n ? (int32_t)reg16(REG_VBUS_MV) * (st_acc_d / st_n) / 1024 : 0;
        st_i[k] = st_n ? st_acc_i / st_n : 0;
        st_w[k] = st_n ? st_acc_w / st_n : 0;
        st_acc_i = st_acc_w = st_acc_d = st_n = 0;
        st_t0 = g_ms;
        st_state++;
    }
}

static void selftest_step(void)
{
    uint8_t *res = &regs2[R2_ST_RESULT - REGS2_BASE];
    int32_t p = g_present_pos;
    if (p < 150 || p > 3950) { st_state = 0; c.out = 0; *res |= 0x40 | 0x80; regs[REG_TORQUE_ENABLE] = 0; regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_SELFTEST; return; }
    switch (st_state) {
    case 1: {                                                    // backlash on reversal: rotor already moving, output still stationary
        c.out = 120;
        if (!st_bl_done && ((p - st_pos0) > 5 || (st_pos0 - p) > 5)) {
            int32_t dh = g_hall_count - st_hall0;
            set_r2_16(R2_ST_BACKLASH, (uint16_t)(dh < 0 ? -dh : dh));
            st_bl_done = 1;
        }
        if (g_ms - st_t0 >= 150u) {
            if (!st_bl_done) set_r2_16(R2_ST_BACKLASH, 0xFFFF);        // output never moved
            st_t0 = g_ms; st_state = 2;
        }
        break; }
    case 2: {                                                    // point 0 (+150): its second half is the steady-state segment
        uint32_t el = g_ms - st_t0;
        if (el == 100u) { st_pos_h = p; st_hall_h = g_hall_count; }
        selftest_point(0, 150, 200u);
        if (st_state == 3) {                                     // direction and steps/unit from the 100 ms steady-state segment
            int32_t dp = p - st_pos_h, dh = g_hall_count - st_hall_h;
            int8_t dir = (dp > 0) == (dh > 0) ? 1 : -1;
            regs2[R2_ST_DIR - REGS2_BASE] = (uint8_t)dir;
            int8_t expect = (regs[REG_CFG_FLAGS] & CFG_SPEED_INV) ? 1 : -1;   // firmware assumes: pos increases — counter decreases
            if (dir == expect && dp > 20) *res |= 0x02;
            if (dp > 20) {
                uint32_t spc = (uint32_t)(dh < 0 ? -dh : dh) * 1000u / (uint32_t)dp;
                set_r2_16(R2_ST_SPC, (uint16_t)spc);
                uint32_t ref = regs[REG_STEPS_PER_CNT];
                if (ref && spc > ref * 7u / 10u && spc < ref * 13u / 10u) *res |= 0x10;
            }
        }
        break; }
    case 3: selftest_point(1, -300, 200u); break;
    case 4: selftest_point(2, 300, 150u);
        if (st_state == 5) {                                     // R, Kv from points 0 and 1 (different speeds)
            int32_t num = st_u[0] * st_w[1] - st_u[1] * st_w[0];
            int32_t den = (st_i[0] * st_w[1] - st_i[1] * st_w[0]) / 1000;
            int32_t di = st_i[1] - st_i[0];
            int32_t r = den ? num / den : 0;                     // mΩ
            // without load current barely depends on speed — the two points degenerate; then R from the journal (locked shaft)
            if ((di > -400 && di < 400) || r < 200 || r > 2000) { r = reg16(REG_R_REF); }
            else *res |= 0x04;
            set_r2_16(R2_ST_R, (uint16_t)r);
            int32_t pp = regs[REG_PP] ? regs[REG_PP] : 1;
            int32_t e = st_u[1] - st_i[1] * r / 1000;           // mV of back-EMF at the 2nd point
            int32_t kv = e > 0 ? st_w[1] * 10 / pp * 1000 / e : 0;   // rpm / V
            if (kv > 300 && kv < 5000) *res |= 0x08;
            set_r2_16(R2_ST_KV, (uint16_t)(kv < 0 ? 0 : (kv > 65535 ? 65535 : kv)));
            // no-load friction from points 0 (+150) and 2 (+300): I = IC + KV·ω — Coulomb + viscous
            if (st_w[2] > st_w[0] + 50) {
                int32_t kvf = (st_i[2] - st_i[0]) * 1000 / (st_w[2] - st_w[0]);          // µA per step/s
                if (kvf < 0) kvf = 0;
                int32_t icf = st_i[0] - kvf * st_w[0] / 1000;
                if (icf < 0) icf = 0;
                set_r2_16(R2_FRIC_KV, (uint16_t)(kvf > 65535 ? 65535 : kvf));
                set_r2_16(R2_FRIC_IC, (uint16_t)(icf > 65535 ? 65535 : icf));
            }
            st_out = 0;
        }
        break;
    case 5: {                                                    // friction: wait for stop, then duty +1/ms until breakaway
        int32_t v = g_vel_cnt < 0 ? -g_vel_cnt : g_vel_cnt;
        if (st_out == 0) { c.out = 0; if (v < 50 && g_ms - st_t0 > 200u) { st_out = 1; st_t0 = g_ms; } break; }
        c.out = st_out;
        if (v > 200 || st_out >= 400) {
            regs2[R2_ST_FRICTION - REGS2_BASE] = (uint8_t)(st_out > 255 ? 255 : st_out);
            st_t0 = g_ms; st_state = 6;
            imp_act.pos = (int16_t)st_pos0; imp_act.vel = 0; imp_act.kp = 310; imp_act.kd = 60; imp_act.ff = 0; imp_act.tmax = 300; imp_act.mode = 1;
            imp_interp = 0; imp_t_ms = g_ms;
        } else if ((g_ms - st_t0) >= 1u) { st_out++; st_t0 = g_ms; }
        break; }
    case 6: {                                                    // return to the starting point: until arrived (or 2 s)
        impedance_control();
        int32_t e = p - st_pos0; if (e < 0) e = -e;
        static uint8_t in_ms; if (e < 15) { if (in_ms < 255) in_ms++; } else in_ms = 0;
        if (in_ms >= 100 || g_ms - st_t0 >= 2000u) {
            in_ms = 0;
            st_state = 0; *res |= 0x80;
            regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_SELFTEST;
            regs[REG_TORQUE_ENABLE] = 0;
            imp_act = st_prev_act;                                   // restore the host command in full
        }
        break; }
    default: st_state = 0; break;
    }
}

void motor_selftest_request(void) { regs2[R2_CFG2_CMD - REGS2_BASE] = 2; }

/* 1 kHz */
void TIM14_IRQHandler(void)
{
    TIM_SR(TIM14_B) &= ~1u;
    speed_update();
    {
        uint8_t cc = regs2[R2_CFG2_CMD - REGS2_BASE];
        if (cc == 3) { g_latched = 0; g_status &= (uint8_t)~(ST_VBUS_HIGH | ST_VBUS_LOW | ST_HALL_BAD); regs2[R2_CFG2_CMD - REGS2_BASE] = 0; }
        if (cc == 4) { unl_until = g_ms + 10000u; regs2[R2_STATUS2 - REGS2_BASE] |= ST2_UNLIMITED; regs2[R2_CFG2_CMD - REGS2_BASE] = 0; }
        if (unl_until && (int32_t)(g_ms - unl_until) >= 0) {           // TEST_UNLIMITED expired: freeze the pre-trigger history
            unl_until = 0;
            regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_UNLIMITED;
            g_bb_freeze_req = 1;
        }
        if (cc == 2 && regs[REG_TORQUE_ENABLE] == 1) {               // self-test only from rest, do not defer
            regs2[R2_CFG2_CMD - REGS2_BASE] = 0; regs2[R2_ST_RESULT - REGS2_BASE] = 0x40 | 0x80;
        }
    }
    g_vel_cnt = speed_pos_dir() * 1000 / (int32_t)(regs[REG_STEPS_PER_CNT] ? regs[REG_STEPS_PER_CNT] : 222);
    thermal_update();
    r_estimate_update();
    imp_take_command();
    bus_timeout_check();
    vbus_check();
    // fault gate — after command reception, so a command cannot enable torque bypassing the latch/Vbus
    if (g_latched || (g_status & (ST_VBUS_HIGH | ST_VBUS_LOW))) regs[REG_TORQUE_ENABLE] = 0;
    if ((regs[REG_CFG_FLAGS] & CFG_HALL_OFF) && hall_bad_count >= 3) {
        regs[REG_TORQUE_ENABLE] = 0;
        if (regs[REG_CFG_FLAGS] & CFG_LATCH) g_latched = 1;
    }
    c.cw_limit = (int16_t)reg16(REG_CW_LIMIT);
    c.ccw_limit = (int16_t)reg16(REG_CCW_LIMIT);
    c.speed = (int16_t)reg16(REG_SPEED);
    uint8_t mode = regs[REG_CTRL_MODE];
    if (regs[REG_TORQUE_ENABLE] == 1 && g_coast) {           // leaving coast: restore commutation
        g_coast = 0;
        regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_COAST;
        g_recommutate = 1;
    }
    if (regs[REG_TORQUE_ENABLE] == 1 && st_state) {           // self-test drives the motor itself
        selftest_step();
    } else if (regs[REG_TORQUE_ENABLE] == 1 && imp_act.mode != 0) {
        impedance_control();
        v1_active = 0;
        spd_integ = 0;
    } else if (regs[REG_TORQUE_ENABLE] == 1) {
        if (c.cw_limit == 0 && c.ccw_limit == 0) {
            if (mode == 1) speed_control_v1(); else wheel_mode_stock();
            v1_active = 0;
        } else {
            if (c.speed == 0) c.speed = 0x3FF;
            if (mode == 1) position_control_v1(); else position_control_stock();
            spd_integ = 0;
        }
    } else {
        c.goal = 0;
        set_reg16(REG_GOAL, 0);
        c.running = 0;
        c.out = 0;
        cur_cap = spd_cap = 1023;
        v1_active = 0;
        spd_integ = 0;
        stall_ms_cnt = 0;
        regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~(ST2_STALL | ST2_HOLD | ST2_INTERP | ST2_SOFT);
        {   // commutation mode switches only with torque off
            uint8_t want = (regs[REG_CFG_FLAGS] & CFG_SINE) ? 1u : 0u;
            if (want != sine_on) {
                sine_on = want;
                if (want) { TIM_SR(TIM1_B) &= ~1u; TIM_DIER(TIM1_B) |= 1u; }
                else      { TIM_DIER(TIM1_B) &= ~1u; }
            }
        }
        if (st_state) {                                                // torque removed mid self-test — abort
            st_state = 0; regs2[R2_ST_RESULT - REGS2_BASE] |= 0x40 | 0x80; regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_SELFTEST;
        }
        if (regs2[R2_CFG2_CMD - REGS2_BASE] == 2) { regs2[R2_CFG2_CMD - REGS2_BASE] = 0; selftest_start(); }
        if (regs2[R2_FAULT_MODE - REGS2_BASE] == 1) {
            if (!g_coast) { motor_coast(); regs2[R2_STATUS2 - REGS2_BASE] |= ST2_COAST; }
        } else {
            if (g_coast) { g_coast = 0; regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_COAST; }
            g_recommutate = 1;                                             // CCR = 0 → low-side switches = brake
        }
    }
    // contact/impact: rotor decelerated sharply along the motion while the loop pushed the same way (3–5 ms, no IMU)
    {
        uint16_t cd = r2_16(R2_CONTACT_DECEL);
        int32_t v = g_vel_cnt, av = v < 0 ? -v : v, apv = v_prev_cnt < 0 ? -v_prev_cnt : v_prev_cnt;
        uint8_t *st2 = &regs2[R2_STATUS2 - REGS2_BASE];
        if (cd && regs[REG_TORQUE_ENABLE] == 1 && c.out != 0 && apv > 500 && (apv - av) > (int32_t)cd && ((c.out > 0) == (v_prev_cnt > 0))) {
            if (contact_cnt < 255) contact_cnt++;
        } else contact_cnt = 0;
        if (contact_cnt >= 2) { *st2 |= ST2_CONTACT; contact_hold = 50; }
        else if (contact_hold && --contact_hold == 0) *st2 &= (uint8_t)~ST2_CONTACT;
        v_prev_cnt = v;
    }
    // stall/collision detection: loop wants to move (|out| ≥ STALL_DUTY before ceilings), but the rotor is stationary
    if (regs[REG_TORQUE_ENABLE] == 1) {
        uint16_t sd = r2_16(R2_STALL_DUTY);
        int32_t o = c.out < 0 ? -c.out : c.out, v = g_vel_cnt < 0 ? -g_vel_cnt : g_vel_cnt;
        uint8_t *st2 = &regs2[R2_STATUS2 - REGS2_BASE];
        if (sd && o >= sd && v < 100) { if (stall_ms_cnt < 255) stall_ms_cnt++; } else stall_ms_cnt = 0;
        uint8_t sms = regs2[R2_STALL_MS - REGS2_BASE] ? regs2[R2_STALL_MS - REGS2_BASE] : 1;
        if (sd && stall_ms_cnt >= sms) {
            uint8_t act = regs2[R2_STALL_ACT - REGS2_BASE];
            if (!(*st2 & ST2_STALL)) { *st2 |= ST2_STALL; if (act == 2) soft_fault(); }
            if (act == 1) c.out = (int16_t)(c.out / 4);
        } else if (stall_ms_cnt == 0 && v >= 100) {
            *st2 &= (uint8_t)~ST2_STALL;
        }
    }
    // state machine for the host (CiA 402-like, indication only)
    {
        uint8_t s2 = regs2[R2_STATUS2 - REGS2_BASE], st;
        if (g_ms < 200u) st = 0;
        else if (g_latched) st = 6;
        else if (g_status & (ST_VBUS_HIGH | ST_VBUS_LOW | ST_HALL_BAD)) st = 5;
        else if (st_state) st = 4;
        else if (regs[REG_TORQUE_ENABLE] == 1 && (s2 & ST2_HOLD)) st = 3;
        else if (regs[REG_TORQUE_ENABLE] == 1) st = 2;
        else st = 1;
        regs2[R2_STATE - REGS2_BASE] = st;
    }
    apply_output();
}

void motor_stall_check(void)
{
    if ((uint32_t)(g_ms - g_last_hall_ms) > 10u) {
        g_recommutate = 1;
    }
}

uint16_t motor_duty(void)   { return (uint16_t)c.duty; }
uint16_t motor_pwm_in(void) { return (uint16_t)pwm_in[1]; }
