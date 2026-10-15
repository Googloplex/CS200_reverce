// Мотор: TIM1 → предрайвер, шестишаговая коммутация по Холлам, контуры управления 1 кГц, вход ШИМ.
// Режим CTRL_MODE = 0: контроллер стока (перенос родной, см. тег v0-clone).
// Режим CTRL_MODE = 1: трапецеидальный профиль + PD с обратной связью по скорости с Холлов, замкнутый режим
// скорости (CW = CCW = 0), кольцевая ошибка для датчика 360°.
// Общее: Холлы как датчик (счётчик, период, скорость), защиты (ток, обороты, I²t), оценка R и температуры обмотки.
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
    int32_t  vel_q8;            // скорость профиля стока, ед АЦП ×256
} ctl_t;

static ctl_t c;
static volatile uint8_t  g_recommutate;
static uint8_t           g_dir;
static volatile uint32_t g_last_hall_ms;
static volatile uint8_t  g_new_goal;
static int16_t pwm_in[3];

/* ---- Холлы как датчик ---- */
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
static uint32_t   thermal_uj;       // тепло в обмотке, мкДж
static uint8_t    i2t_limiting;
static int32_t    cur_cap = 1023, spd_cap = 1023;
static int32_t    r_est_acc;        // оценка R, мОм ×1024
static uint16_t   r_est_n;

/* ---- команда импеданса (v1.3) ---- */
typedef struct { int16_t pos, vel; uint16_t kp, kd; int16_t ff; uint16_t tmax; uint8_t mode; uint16_t at; } imp_cmd_t;
static imp_cmd_t imp_pend, imp_act;
static uint8_t   imp_pend_valid;
static uint8_t   imp_last_seq;
// кубическая интерполяция цели (mode 2): от (p0, v0) в t0 к (imp_act.pos, imp_act.vel) за T мс
static uint8_t   imp_interp;
static int32_t   imp_p0, imp_v0T, imp_v1T;   // ед; v·T/1000 — путь за T при скорости v
static uint32_t  imp_t0, imp_T;
static int32_t   imp_pt, imp_vt;              // текущая цель и её скорость (ед, ед/с)
static uint8_t   stall_ms_cnt, g_coast;
static int32_t   v_prev_cnt;                  // контакт по торможению
static uint8_t   contact_cnt, contact_hold;
/* синусоидальная коммутация (CFG_SINE): угол 0..4095 = 360° эл., вектор = центр сектора + интерполяция по времени */
static const int16_t g_q15[256] = {0,1206,2409,3607,4797,5976,7143,8294,9427,10539,11629,12695,13733,14743,15723,16671,17585,18464,19307,20113,20881,21610,22300,22950,23561,24131,24661,25152,25603,26016,26391,26729,27031,27298,27532,27733,27904,28045,28159,28246,28310,28352,28373,28376,28363,28335,28294,28243,28183,28116,28044,27969,27892,27814,27739,27666,27597,27533,27476,27425,27383,27350,27325,27311,27306,27311,27325,27350,27383,27425,27476,27533,27597,27666,27739,27814,27892,27969,28044,28116,28183,28243,28294,28335,28363,28376,28373,28352,28310,28246,28159,28045,27904,27733,27532,27298,27031,26729,26391,26016,25603,25152,24661,24131,23561,22950,22300,21610,20881,20113,19307,18464,17585,16671,15723,14743,13733,12695,11629,10539,9427,8294,7143,5976,4797,3607,2409,1206,0,-1206,-2409,-3607,-4797,-5976,-7143,-8294,-9427,-10539,-11629,-12695,-13733,-14743,-15723,-16671,-17585,-18464,-19307,-20113,-20881,-21610,-22300,-22950,-23561,-24131,-24661,-25152,-25603,-26016,-26391,-26729,-27031,-27298,-27532,-27733,-27904,-28045,-28159,-28246,-28310,-28352,-28373,-28376,-28363,-28335,-28294,-28243,-28183,-28116,-28044,-27969,-27892,-27814,-27739,-27666,-27597,-27533,-27476,-27425,-27383,-27350,-27325,-27311,-27306,-27311,-27325,-27350,-27383,-27425,-27476,-27533,-27597,-27666,-27739,-27814,-27892,-27969,-28044,-28116,-28183,-28243,-28294,-28335,-28363,-28376,-28373,-28352,-28310,-28246,-28159,-28045,-27904,-27733,-27532,-27298,-27031,-26729,-26391,-26016,-25603,-25152,-24661,-24131,-23561,-22950,-22300,-21610,-20881,-20113,-19307,-18464,-17585,-16671,-15723,-14743,-13733,-12695,-11629,-10539,-9427,-8294,-7143,-5976,-4797,-3607,-2409,-1206};   // sin θ + sin 3θ/6: третья гармоника общая для фаз
static const uint16_t sect_angle[6] = {1707, 2389, 3072, 3755, 341, 1024};   // 150°+60°·k: вектор шестишаговой в секторе k
volatile uint8_t         sine_on;             // режим защёлкивается при TE 0→1 (глобальный для стенда)
static volatile uint16_t sine_theta_edge;     // угол на последнем фронте Холлов
static volatile int8_t   sine_edge_dir;
static volatile uint32_t sine_step_q8;        // шаг угла за период ШИМ (Q8), считается на фронте; 0 = стоим
static volatile uint32_t sine_tick_lim;       // периодов ШИМ после фронта, после которых считаем, что ротор встал
static volatile uint32_t sine_acc_q8, sine_ticks;
static uint32_t hall_t6[6];                   // времена последних 6 фронтов: период за электрический оборот не зависит от расстановки Холлов
static uint8_t  hall_t6_i, hall_t6_n;
volatile uint32_t g_hall_period6_us;          // время 6 последних шагов (0 = нет)
/* самотест / идентификация */
static uint8_t  st_state;
static uint32_t st_t0;
static int32_t  st_acc_i, st_acc_w, st_n, st_pos0, st_hall0;
static int32_t  st_u[3], st_i[3], st_w[3];
static int16_t  st_out;
static imp_cmd_t st_prev_act;                  // команда хоста на время самотеста
static int32_t  st_acc_d;
static int32_t  st_pos_h, st_hall_h;              // точка начала установившегося участка (шаг/ед, направление)
static uint8_t  st_bl_done;
volatile uint8_t g_latched;
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint32_t unl_until;                     // TEST_UNLIMITED: до какой мс
static int32_t  iloop_integ;                   // PI тока в режиме момента, duty ×16
static uint32_t  imp_t_ms;          // когда применена активная команда
volatile int32_t g_vel_cnt;
volatile uint32_t g_odo_steps;      // шагов Холлов по модулю с запуска         // скорость в ед АЦП/с по Холлам (в направлении роста положения)

/* ---- контроллер v1 ---- */
static int32_t  p_cmd_q8;           // профиль v1: положение, ед АЦП ×256
static int32_t  v_cmd;              // профиль v1: скорость, ед АЦП/с
static int32_t  spd_integ;          // режим скорости: интегратор
static uint8_t  v1_active;

#define TAU_COOL_MS  60000u    // постоянная остывания, мс
#define E_MAX_UJ     1600000000u   // тепловой бюджет 1600 Дж [ГИП]

uint32_t micros(void)
{
    uint32_t ms, val;
    do {
        ms = g_ms;
        val = SysTick->VAL;
    } while (ms != g_ms);
    return ms * 1000u + (((71999u - val) * 58255u) >> 22);   // /72 без деления, точно на всём диапазоне
}

/* ---------- таймеры ---------- */
void motor_tim1_init(void)
{
    TIM_CR1(TIM1_B) = (TIM_CR1(TIM1_B) & 0xFF8Fu & 0xFCFFu);
    TIM_ARR(TIM1_B) = 0x3FFu;
    TIM_PSC(TIM1_B) = 2u;
    TIM_RCR(TIM1_B) = 0;
    TIM_EGR(TIM1_B) = 1u;
    TIM_CCMR1(TIM1_B) = 0x6868u | (1u << 7) | (1u << 15);   // OC1/OC2: PWM1 + preload + OCxCE (OCREF_CLR разрешён)
    TIM_CCMR2(TIM1_B) = 0x0068u | (1u << 7) | (7u << 12) | (1u << 11);   // OC3: PWM1 + preload + OC3CE; OC4: PWM2 + preload (триггер АЦП)
    TIM_CCR1(TIM1_B) = 0;
    TIM_CCR2(TIM1_B) = 0;
    TIM_CCR3(TIM1_B) = 0;
    TIM_CCR4(TIM1_B) = 8;                                    // точка выборки АЦП, двигается каждый тик
    TIM_CR2(TIM1_B) = 0;
    TIM_CCER(TIM1_B) = 0x555u | (1u << 12);   // CC4E: без него TIM1_OC4 не даёт триггер АЦП [ИЗМ]; на пин не выведен
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

/* ---------- Холлы ---------- */
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
        if (d == 0) return;                                              // тот же сектор: дребезг/помеха — не период
        if ((now - g_hall_last_us) < 60u) return;                        // дребезг A→B→A: быстрее 16 тыс. шаг/с не бывает
        if (d == 1)      { g_hall_count++; g_hall_dir = 1; g_odo_steps++; }
        else if (d == 5) { g_hall_count--; g_hall_dir = -1; g_odo_steps++; }
        if (d == 1 || d == 5) {
            uint32_t p = now - g_hall_last_us;
            if (p < 50u) p = 50u;                                        // ≤ 20000 шаг/с
            g_hall_period_us = p;
            // среднее по 6 фронтам (электрический оборот) — без ошибки расстановки Холлов
            uint32_t p6 = 0;
            if (hall_t6_n >= 6) { p6 = now - hall_t6[hall_t6_i]; if (p6 > 6u * 100000u) p6 = 0; }
            hall_t6[hall_t6_i] = now; hall_t6_i = (uint8_t)((hall_t6_i + 1u) % 6u);
            if (hall_t6_n < 6) hall_t6_n++;
            g_hall_period6_us = p6;
            uint32_t ps = p6 ? p6 / 6u : p;                              // период для шага угла
            // синус: шаг угла за период ШИМ 42.67 мкс = 683·42.67/ps (Q8); стоим после 2 периодов без фронта
            sine_step_q8 = (ps <= 20000u) ? 7460352u / ps : 0u;
            sine_tick_lim = (2u * p) / 43u + 1u;
        } else {
            hall_t6_n = 0;                                               // смена направления/пропуск — среднее заново
        }
    }
    hall_prev_sector = sec;
    g_hall_last_us = now;
    // синус: при движении + входим в сектор за 30° до его центра, при − — через 30° после
    sine_edge_dir = g_hall_dir;
    sine_theta_edge = (uint16_t)((sect_angle[sec] + (g_hall_dir < 0 ? 341u : 4096u - 341u)) & 0xFFFu);
    sine_acc_q8 = 0; sine_ticks = 0;
}

int8_t motor_out_sign(void)
{
    if (regs[REG_TORQUE_ENABLE] != 1 || c.duty == 0) return 0;
    return g_dir ? 1 : -1;
}

/* ---------- синусоидальная коммутация: TIM1 update 23.4 кГц ---------- */
static inline int32_t g_a(uint32_t a) { return g_q15[(a >> 4) & 0xFFu]; }   // a: 0..4095 → sin a + sin 3a/6

void TIM1_NON_CC_IRQHandler(void)   // без делений: ~150 тактов из 3077 на период ШИМ
{
    TIM_SR(TIM1_B) &= ~1u;
    if (!sine_on) return;
    uint32_t amp = (uint32_t)(uint16_t)c.duty;
    if (amp == 0 || g_coast) {                               // тормоз (нижние ключи) или свободный ход
        TIM_CCR1(TIM1_B) = 0; TIM_CCR2(TIM1_B) = 0; TIM_CCR3(TIM1_B) = 0;
        return;
    }
    int8_t sec = hall_sector[g_hall_code];
    if (sec < 0) return;
    uint32_t theta;
    if (sine_step_q8 == 0 || ++sine_ticks > sine_tick_lim) {
        theta = sect_angle[sec];                             // медленно/стоим: центр сектора, как шестишаговая
    } else {
        sine_acc_q8 += sine_step_q8;
        if (sine_acc_q8 > (683u << 8)) sine_acc_q8 = 683u << 8;   // не дальше конца сектора
        uint32_t adv = sine_acc_q8 >> 8;
        theta = sine_edge_dir < 0 ? (sine_theta_edge + 4096u - adv) : (sine_theta_edge + adv);
    }
    // опережение — вдоль направления момента (g_dir), в единицах 4096/360 ≈ 91/8 на градус
    int32_t lead = (int32_t)(int8_t)regs2[R2_SINE_ADV - REGS2_BASE] * 91 / 8;
    int32_t ofs = (int32_t)(int8_t)regs2[R2_SINE_OFS - REGS2_BASE] * 91 / 8;   // расстановка Холлов — не зависит от направления
    // (sin θ, sin θ−120°, sin θ+120°) даёт вектор на θ−90° → +90°; g_dir = 1 (рост положения) = вектор на 180°, как 7−h у шестишаговой
    theta = (theta + 1024u + (uint32_t)ofs + (g_dir ? 2048u - (uint32_t)lead : (uint32_t)lead)) & 0xFFFu;
    // третья гармоника 1/6 в таблице → амплитуда фундаментала ×1.155: линейное напряжение = Vbus·duty, как у шестишаговой
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
    if (p6 && since <= period * 2u) period = p6 / 6u;               // установившееся вращение: среднее за оборот
    // ротор замедлился или встал: скорость не выше 1/(время с последнего фронта), иначе оценка
    // держит старое значение до 100 мс и D-член PD раскачивает привод на полном duty
    if (since > period) period = since;
    int32_t s = (int32_t)(1000000u / period);
    g_speed = (g_hall_dir < 0) ? -s : s;
}

// скорость в направлении роста положения (ед АЦП): знак Холлов относительно потенциометра
static int32_t speed_pos_dir(void)
{
    int32_t s = -g_speed;                      // измерено: при росте pos счётчик Холлов убывает
    if (regs[REG_CFG_FLAGS] & CFG_SPEED_INV) s = -s;
    return s;
}

/* ---------- коммутация ---------- */
static inline void cc_enable(unsigned shift, unsigned on)
{
    TIM_CCER(TIM1_B) = (TIM_CCER(TIM1_B) & ~(1u << shift) & 0xFFFFu) | ((on << shift) & 0xFFFFu);
}

static void commutate(void)
{
    if (g_coast) return;                       // свободный ход: все ключи закрыты, коммутация не нужна
    __disable_irq();                           // чтение Холлов и запись CCR одним куском: EXTI не вклинится между ними
    uint8_t h = hall_read();
    c.hall = g_dir ? (uint8_t)(7 - h) : h;
    c.duty_copy = c.duty;
    uint32_t d = (uint32_t)(uint16_t)c.duty;
    TIM_CCER(TIM1_B) |= 0x444u;                // комплементарные выходы (могли быть сняты свободным ходом)
    if (sine_on) {                             // синус: все три фазы ШИМ, значения пишет TIM1_NON_CC_IRQHandler
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

/* ---------- вход ШИМ ---------- */
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

/* ---------- цель: мкс → единицы АЦП (общая для обоих контроллеров, семантика стока) ---------- */
static int16_t goal_to_adc(void)
{
    int16_t goal_us = (int16_t)reg16(REG_GOAL);
    if (goal_us > 0x834) goal_us = 0x834;
    else if (goal_us < 900) goal_us = 900;
    calib.travel = reg16(REG_TRAVEL);
    c.kA = regs[REG_K_CW]  ? regs[REG_K_CW]  : (int32_t)calib.travel;
    c.kB = regs[REG_K_CCW] ? regs[REG_K_CCW] : (int32_t)calib.travel;
    // сток: 2048 ± (|g−1500|·K/460·4095/600) = ± num·4095/276000, усечение к нулю как у float
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

/* ---------- контроллер стока ---------- */
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
        else                  c.target = (int16_t)(c.start - ((c.vel_q8 + 255) >> 8));         // (int)(start − vel): усечение к нулю
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
        c.out = (int16_t)(err * 100 / slope);                               // err·(100/slope), усечение к нулю
    } else {
        c.out = (int16_t)((err * 10000 / slope + c.integ) / 100);           // + integ·0.01, одно усечение
    }
}

static void wheel_mode_stock(void)
{
    if (c.speed & 0x400) {
        c.speed = (int16_t)-(c.speed & 0x3FF);
    }
    c.out = c.speed;
}

/* ---------- контроллер v1 ---------- */
static uint32_t isqrt32(uint32_t x)   // целочисленный корень, побитово
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

    if (!v1_active) {                       // старт из текущего положения, без рывка
        v1_active = 1;
        p_cmd_q8 = (int32_t)g_present_pos << 8;
        v_cmd = 0;
        c.running = 1;
    }
    // профиль: трапеция по vmax/acc, либо мгновенно (положение в Q8, скорость в ед/с, ускорение в ед/с²)
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
        int32_t ad = (dist_q8 < 0 ? -dist_q8 : dist_q8) >> 8;              // ед
        int32_t vt;
        if (acc > 0) {
            vt = (int32_t)isqrt32((uint32_t)(acc / 32) * (uint32_t)ad) * 8;   // sqrt(2·a·d) = sqrt(a/32·d)·8
            if (vmax > 0 && vt > vmax) vt = vmax;
        } else {
            vt = vmax;
        }
        if (dist_q8 < 0) vt = -vt;
        int32_t dv = vt - v_cmd;
        int32_t dvmax = (acc > 0) ? (acc + 999) / 1000 : 0x7FFFFFFF;        // прирост скорости за 1 мс
        if (dv > dvmax) dv = dvmax;
        if (dv < -dvmax) dv = -dvmax;
        v_cmd += dv;
        int32_t step_q8 = v_cmd * 256 / 1000;
        if ((dist_q8 >= 0 && step_q8 > dist_q8) || (dist_q8 < 0 && step_q8 < dist_q8)) {   // не проскакивать цель
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

    // ошибка положения
    int32_t err = (int32_t)(int16_t)(c.cmd - g_present_pos);
    if (wrap) {
        while (err > 2048) err -= 4096;
        while (err < -2048) err += 4096;
    }
    c.err = (int16_t)err;

    // PD: скорость профиля и реальная — в шагах Холлов/с
    int32_t spc = regs[REG_STEPS_PER_CNT];                           // ×0.001 шагов на единицу
    int32_t v_cmd_steps = v_cmd * spc / 1000;
    int32_t v_err = v_cmd_steps - speed_pos_dir();
    int32_t out = err * (int32_t)regs[REG_KP] / 10 + v_err * (int32_t)regs[REG_KD] / 100;
    if (out > 1023) out = 1023;
    if (out < -1023) out = -1023;
    c.out = (int16_t)out;
}

/* мА прямого момента → скважность: u = I·R + Ke·ω; R из R_EST (или R_REF), Ke из KV/PP */
static int32_t ff_duty(int32_t ff_ma)
{
    int32_t r_mohm = clampi(reg16(REG_R_REF), 50, 5000);   // модель: R_REF (R_EST под быстрыми переходами плывёт)
    int32_t kv = reg16(REG_KV); int32_t pp = regs[REG_PP] ? regs[REG_PP] : 1;
    ff_ma = clampi(ff_ma, -30000, 30000);
    int32_t u_mv = ff_ma * r_mohm / 1000;
    if (kv > 0) u_mv += speed_pos_dir() * 10 / pp * 1000 / kv;          // противоЭДС, мВ
    int32_t vb = reg16(REG_VBUS_MV); if (vb < 5000) vb = 5000;
    int32_t d = u_mv * 1024 / vb;
    // режим момента (Kp = Kd = 0) и CFG_ILOOP: медленный интегратор по измеренному току поверх модели (шунт отфильтрован → ~50 Гц)
    if ((regs[REG_CFG_FLAGS] & CFG_ILOOP) && imp_act.kp == 0 && imp_act.kd == 0 && ff_ma != 0) {
        int32_t want = ff_ma < 0 ? -ff_ma : ff_ma;
        int32_t w = speed_pos_dir();
        // шунт видит только модуль тока: интегрировать, когда ток идёт в сторону ff (ротор стоит или крутится вдоль ff)
        int ok_dir = (w > -200 && w < 200) || ((w > 0) == (ff_ma > 0));
        if (g_i_valid && want > 300 && ok_dir) {
            int32_t e = want - (int32_t)g_i_wind_ma;                     // мА
            iloop_integ += e / 32;                                       // ~0.002 duty/мА за мс (полоса ниже фильтра шунта)
            if (iloop_integ > 200 * 16) iloop_integ = 200 * 16;
            if (iloop_integ < -200 * 16) iloop_integ = -200 * 16;
        }
        d += (ff_ma < 0 ? -(iloop_integ / 16) : (iloop_integ / 16));
    } else {
        iloop_integ = 0;
    }
    return d;
}

/* потолок duty, при котором ток обмотки не выше i_ma: u = I·R + E, E — противоЭДС вдоль приложенного напряжения
   (при вращении навстречу E < 0 и потолок падает). Без запаздывания фильтра шунта: ограничение по модели, измерение —
   страховка (cap_update ниже) */
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
    if (g_cmd_new || seq != imp_last_seq) {   // блок записан по шине (флаг) или по SWD (сменился номер)
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
        // новая команда — новое событие упора/контакта: флаги и счётчики заново
        stall_ms_cnt = 0; contact_cnt = 0;
        regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~(ST2_STALL | ST2_CONTACT);
        if (imp_pend.mode == 2 && imp_pend.at != 0 && dt > 0) {
            // интерполяция: стартуем из текущей цели и её скорости, чтобы 100 Гц от хоста стали гладким заданием на 1 кГц
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

/* цель и её скорость на текущую мс: кубика Эрмита (Q10) или удержание 1-го порядка 100 мс */
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
            if (regs[REG_CFG_FLAGS] & CFG_WRAP) {                        // кольцо: к цели по короткому пути
                int32_t dd = p1 - imp_p0;
                while (dd > 2048) dd -= 4096;
                while (dd < -2048) dd += 4096;
                p1 = imp_p0 + dd;
            }
            imp_pt = (h00 * imp_p0 + (1024 - h00) * p1 + h10 * imp_v0T + h11 * imp_v1T) >> 10;
            int32_t dnum = (6 * s2 - 6 * s) * (imp_p0 - p1) + (3 * s2 - 4 * s + 1024) * imp_v0T + (3 * s2 - 2 * s) * imp_v1T;
            int32_t q = clampi(dnum / (int32_t)imp_T, -10000000, 10000000);   // Q10 ед/мс
            imp_vt = clampi((q * 125) >> 7, -32767, 32767);                   // ×1000/1024 → ед/с
            return;
        }
    }
    uint32_t age = g_ms - imp_t_ms;
    if (age > 100u) age = 100u;
    imp_pt = (int32_t)imp_act.pos + (int32_t)imp_act.vel * (int32_t)age / 1000;
    imp_vt = (age < 100u) ? imp_act.vel : 0;
}

/* отказ «мягкого» типа (таймаут шины, столкновение): по FAULT_MODE — удержание в точке или момент выкл */
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

static void bus_timeout_check(void)   // из TIM14
{
    uint16_t tmo = reg16(REG_BUS_TIMEOUT);
    if (tmo && regs[REG_TORQUE_ENABLE] && !(g_status & ST_BUS_TMO) && (uint32_t)(g_ms - g_last_rx_ms) > tmo) {
        g_status |= ST_BUS_TMO;
        soft_fault();
    }
}

static void vbus_check(void)   // из TIM14: 5 мс подряд за порогом, не раньше 500 мс после старта (шина ещё не поднялась)
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


static void motor_coast(void)   // свободный ход: все шесть выходов сняты → предрайвер держит ключи закрытыми
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
    // мягкие упоры: цель не выходит за них, за упором — пружина SOFT_KP (звено не бьёт в механический упор)
    int32_t smin = r2_16(R2_SOFT_MIN), smax = r2_16(R2_SOFT_MAX), soft = 0;
    if (smin && smax && smax <= smin) smin = smax = 0;                   // бессмысленная пара — игнорируем
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

static void speed_control_v1(void)   // CW = CCW = 0: замкнутый режим скорости, задание как в AX-12 (бит 10 — знак)
{
    int32_t sp = c.speed & 0x3FF;
    if (c.speed & 0x400) sp = -sp;
    int32_t target = sp * 2;                                          // шагов/с (1023 → ~2046 ≈ 14 В на обмотке)
    int32_t e = target - speed_pos_dir();
    spd_integ += e;
    if (spd_integ > 200000) spd_integ = 200000;
    if (spd_integ < -200000) spd_integ = -200000;
    int32_t out = e * (int32_t)regs[REG_KP] / 20 + spd_integ / 200;
    if (out > 1023) out = 1023;
    if (out < -1023) out = -1023;
    c.out = (int16_t)out;
}

/* ---------- тепловая модель и оценка R/T ---------- */
static void thermal_update(void)
{
    // за 1 мс: +I²R (мкДж: (мА²/1000)·мОм/1000) − E/τ
    uint32_t i = g_i_wind_ma;
    uint32_t r = reg16(REG_R_REF) ? reg16(REG_R_REF) : 550u;
    uint32_t heat = (i * i / 1000u) * r / 1000u;
    uint32_t cool = thermal_uj / TAU_COOL_MS;
    if (r > 5000u) r = 5000u;
    thermal_uj = (thermal_uj + heat > cool) ? thermal_uj + heat - cool : 0u;
    if (thermal_uj > 2u * E_MAX_UJ) thermal_uj = 2u * E_MAX_UJ;        // равновесие при 15 А стопа переполнило бы u32
    uint32_t pct = thermal_uj / (E_MAX_UJ / 100u);
    g_thermal_pct = (uint8_t)(pct > 255u ? 255u : pct);
    if (reg16(REG_I_CONT)) {
        if (thermal_uj > E_MAX_UJ) i2t_limiting = 1;
        else if (thermal_uj < E_MAX_UJ / 5u * 4u) i2t_limiting = 0;
    } else {
        i2t_limiting = 0;
    }
}

// R = (Vbus·D − Ke·ω) / I, когда ток заметный; Ke из KV и PP. Усреднение IIR ~2 с. T = 25 + (R/R_ref − 1)/0.00393.
static void r_estimate_update(void)
{
    static uint16_t last_duty; static uint8_t steady_ms;
    uint32_t i_ma = g_i_wind_ma;
    uint16_t duty = (uint16_t)c.duty;
    int32_t dd = (int32_t)duty - (int32_t)last_duty; last_duty = duty;
    if (dd > 4 || dd < -4) steady_ms = 0; else if (steady_ms < 255) steady_ms++;
    if (steady_ms < 20) return;                                  // ток через фильтр шунта отстаёт: считать только при ровном duty
    if (!g_i_valid || i_ma < 800u || duty < 100u) return;
    int32_t u_mv = (int32_t)reg16(REG_VBUS_MV) * duty / 1024;
    int32_t sp = g_speed < 0 ? -g_speed : g_speed;
    int32_t kv = reg16(REG_KV);
    int32_t pp = regs[REG_PP] ? regs[REG_PP] : 1;
    if (kv > 0) u_mv -= sp * 10 / pp * 1000 / kv;            // противоЭДС: шагов/с × 60/(6·pp) / Kv, мВ
    if (u_mv <= 50) return;
    int32_t r = u_mv * 1000 / (int32_t)i_ma;                 // мОм
    if (r < 50 || r > 5000) return;
    int32_t r_q10 = r << 10;
    if (r_est_n < 2000) {
        r_est_n++;
        r_est_acc += (r_q10 - r_est_acc) / (int32_t)r_est_n;  // среднее по первым выборкам
    } else {
        r_est_acc += (r_q10 - r_est_acc) >> 11;              // затем IIR ~2 с
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

/* ---------- ограничители ---------- */
static uint8_t cur_over_ms, spd_over_ms;
static int32_t cap_update(int32_t cap, uint32_t value, uint32_t limit, int32_t duty, uint8_t *over_ms)
{
    if (!limit) { *over_ms = 0; return 1023; }
    if (value > limit) {
        if (*over_ms < 255) (*over_ms)++;
        if (*over_ms >= 5) {                                            // держится ≥ 5 мс — не всплеск оценки на переходе
            int32_t want = (int32_t)((uint32_t)duty * limit / value);   // пропорциональное снижение
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
    uint8_t st_open = (uint8_t)(st_state && st_state < 6);        // фазы самотеста 1–5: известные duty ≤ 300, без потолков; возврат — с потолком
    uint32_t ilim = (unl_until || st_open) ? 0u : reg16(REG_LIMIT_CURRENT);   // TEST_UNLIMITED и самотест: без потолков
    if (i2t_limiting && !unl_until && !st_open) {                            // I²t режет и модельный потолок
        uint32_t ic = reg16(REG_I_CONT);
        if (!ilim || ic < ilim) ilim = ic;
        g_status |= ST_I2T;
    }
    if (ilim) {
        int32_t cm = duty_for_current(ilim, g_dir);
        if (out > cm) { out = cm; g_status |= ST_CUR_LIMIT; }
    }
    // страховка: модель держит ток у лимита, измерение в переходах завышено → порог 1.5·лимита; при невалидной оценке (duty < 80)
    // потолок растёт, а не замораживается (иначе взаимоблокировка: обрушился ниже 80 → оценка невалидна → не растёт)
    cur_cap = cap_update(cur_cap, g_i_valid ? g_i_wind_ma : 0, ilim + ilim / 2, c.duty, &cur_over_ms);
    uint32_t sp = (uint32_t)(g_speed < 0 ? -g_speed : g_speed);
    spd_cap = cap_update(spd_cap, sp, (unl_until || st_open) ? 0u : reg16(REG_LIMIT_SPEED), c.duty, &spd_over_ms);
    if (out > cur_cap) { out = cur_cap; g_status |= ST_CUR_LIMIT; }
    if (out > spd_cap) { out = spd_cap; g_status |= ST_SPD_LIMIT; }

    c.out = (int16_t)out;
    c.duty = (int16_t)out;
    // точка выборки тока: середина окна открытого верхнего ключа минус ~1.5 мкс на очередь каналов АЦП
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
    } else if (!sine_on && !g_coast) {                            // шестишаговая: новый duty в активную фазу каждый тик, не ждать фронта
        switch (c.hall) {
        case 6: case 4: TIM_CCR2(TIM1_B) = (uint32_t)out; break;
        case 2: case 3: TIM_CCR3(TIM1_B) = (uint32_t)out; break;
        case 1: case 5: TIM_CCR1(TIM1_B) = (uint32_t)out; break;
        default: break;
        }
    }
}

/* ---------- самотест / идентификация одной командой (CFG2_CMD = 2) ----------
   0 Холлы; 1 +duty 120 150 мс → направление Холлов vs положение и шагов/ед; 2–4 три рабочие точки (+150, −300, +300)
   → R и Kv по методу двух точек (U = I·R + Ke·ω); 5 трение: duty растёт 1/мс до страгивания; 6 возврат импедансом. */
static void selftest_start(void)
{
    int32_t p = g_present_pos;
    uint8_t *res = &regs2[R2_ST_RESULT - REGS2_BASE];
    *res = 0;
    if (p < 1000 || p > 3000) { *res = 0x40 | 0x80; return; }    // экскурсия до +750/−200 ед от старта
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
    if (el >= hold_ms / 2u) {                                    // усреднение по второй половине
        int32_t w = g_speed < 0 ? -g_speed : g_speed;
        st_acc_i += g_i_wind_ma; st_acc_w += w; st_acc_d += c.duty; st_n++;   // c.duty — применённый в прошлом тике (после потолков и TL)
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
    case 1: {                                                    // мёртвый ход при реверсе: ротор уже идёт, выход ещё стоит
        c.out = 120;
        if (!st_bl_done && ((p - st_pos0) > 5 || (st_pos0 - p) > 5)) {
            int32_t dh = g_hall_count - st_hall0;
            set_r2_16(R2_ST_BACKLASH, (uint16_t)(dh < 0 ? -dh : dh));
            st_bl_done = 1;
        }
        if (g_ms - st_t0 >= 150u) {
            if (!st_bl_done) set_r2_16(R2_ST_BACKLASH, 0xFFFF);        // выход так и не сдвинулся
            st_t0 = g_ms; st_state = 2;
        }
        break; }
    case 2: {                                                    // точка 0 (+150): в её второй половине — установившийся участок
        uint32_t el = g_ms - st_t0;
        if (el == 100u) { st_pos_h = p; st_hall_h = g_hall_count; }
        selftest_point(0, 150, 200u);
        if (st_state == 3) {                                     // направление и шагов/ед по установившемуся участку 100 мс
            int32_t dp = p - st_pos_h, dh = g_hall_count - st_hall_h;
            int8_t dir = (dp > 0) == (dh > 0) ? 1 : -1;
            regs2[R2_ST_DIR - REGS2_BASE] = (uint8_t)dir;
            int8_t expect = (regs[REG_CFG_FLAGS] & CFG_SPEED_INV) ? 1 : -1;   // прошивка считает: pos растёт — счётчик убывает
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
        if (st_state == 5) {                                     // R, Kv по точкам 0 и 1 (разные скорости)
            int32_t num = st_u[0] * st_w[1] - st_u[1] * st_w[0];
            int32_t den = (st_i[0] * st_w[1] - st_i[1] * st_w[0]) / 1000;
            int32_t di = st_i[1] - st_i[0];
            int32_t r = den ? num / den : 0;                     // мОм
            // без нагрузки ток почти не зависит от скорости — две точки вырождаются; тогда R из журнала (заблокированный вал)
            if ((di > -400 && di < 400) || r < 200 || r > 2000) { r = reg16(REG_R_REF); }
            else *res |= 0x04;
            set_r2_16(R2_ST_R, (uint16_t)r);
            int32_t pp = regs[REG_PP] ? regs[REG_PP] : 1;
            int32_t e = st_u[1] - st_i[1] * r / 1000;           // мВ противоЭДС во 2-й точке
            int32_t kv = e > 0 ? st_w[1] * 10 / pp * 1000 / e : 0;   // об/мин / В
            if (kv > 300 && kv < 5000) *res |= 0x08;
            set_r2_16(R2_ST_KV, (uint16_t)(kv < 0 ? 0 : (kv > 65535 ? 65535 : kv)));
            // трение без нагрузки по точкам 0 (+150) и 2 (+300): I = IC + KV·ω — Кулон + вязкое
            if (st_w[2] > st_w[0] + 50) {
                int32_t kvf = (st_i[2] - st_i[0]) * 1000 / (st_w[2] - st_w[0]);          // мкА на шаг/с
                if (kvf < 0) kvf = 0;
                int32_t icf = st_i[0] - kvf * st_w[0] / 1000;
                if (icf < 0) icf = 0;
                set_r2_16(R2_FRIC_KV, (uint16_t)(kvf > 65535 ? 65535 : kvf));
                set_r2_16(R2_FRIC_IC, (uint16_t)(icf > 65535 ? 65535 : icf));
            }
            st_out = 0;
        }
        break;
    case 5: {                                                    // трение: ждём остановки, потом duty +1/мс до страгивания
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
    case 6: {                                                    // возврат в исходную точку: пока не пришли (или 2 с)
        impedance_control();
        int32_t e = p - st_pos0; if (e < 0) e = -e;
        static uint8_t in_ms; if (e < 15) { if (in_ms < 255) in_ms++; } else in_ms = 0;
        if (in_ms >= 100 || g_ms - st_t0 >= 2000u) {
            in_ms = 0;
            st_state = 0; *res |= 0x80;
            regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_SELFTEST;
            regs[REG_TORQUE_ENABLE] = 0;
            imp_act = st_prev_act;                                   // вернуть команду хоста целиком
        }
        break; }
    default: st_state = 0; break;
    }
}

void motor_selftest_request(void) { regs2[R2_CFG2_CMD - REGS2_BASE] = 2; }

/* 1 кГц */
void TIM14_IRQHandler(void)
{
    TIM_SR(TIM14_B) &= ~1u;
    speed_update();
    {
        uint8_t cc = regs2[R2_CFG2_CMD - REGS2_BASE];
        if (cc == 3) { g_latched = 0; g_status &= (uint8_t)~(ST_VBUS_HIGH | ST_VBUS_LOW | ST_HALL_BAD); regs2[R2_CFG2_CMD - REGS2_BASE] = 0; }
        if (cc == 4) { unl_until = g_ms + 10000u; regs2[R2_STATUS2 - REGS2_BASE] |= ST2_UNLIMITED; regs2[R2_CFG2_CMD - REGS2_BASE] = 0; }
        if (unl_until && (int32_t)(g_ms - unl_until) >= 0) {           // TEST_UNLIMITED истёк: заморозить предысторию
            unl_until = 0;
            regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_UNLIMITED;
            g_bb_freeze_req = 1;
        }
        if (cc == 2 && regs[REG_TORQUE_ENABLE] == 1) {               // самотест только из покоя, не откладывать
            regs2[R2_CFG2_CMD - REGS2_BASE] = 0; regs2[R2_ST_RESULT - REGS2_BASE] = 0x40 | 0x80;
        }
    }
    g_vel_cnt = speed_pos_dir() * 1000 / (int32_t)(regs[REG_STEPS_PER_CNT] ? regs[REG_STEPS_PER_CNT] : 222);
    thermal_update();
    r_estimate_update();
    imp_take_command();
    bus_timeout_check();
    vbus_check();
    // шлюз отказов — после приёма команды, чтобы команда не включила момент в обход защёлки/Vbus
    if (g_latched || (g_status & (ST_VBUS_HIGH | ST_VBUS_LOW))) regs[REG_TORQUE_ENABLE] = 0;
    if ((regs[REG_CFG_FLAGS] & CFG_HALL_OFF) && hall_bad_count >= 3) {
        regs[REG_TORQUE_ENABLE] = 0;
        if (regs[REG_CFG_FLAGS] & CFG_LATCH) g_latched = 1;
    }
    c.cw_limit = (int16_t)reg16(REG_CW_LIMIT);
    c.ccw_limit = (int16_t)reg16(REG_CCW_LIMIT);
    c.speed = (int16_t)reg16(REG_SPEED);
    uint8_t mode = regs[REG_CTRL_MODE];
    if (regs[REG_TORQUE_ENABLE] == 1 && g_coast) {           // выход из свободного хода: вернуть коммутацию
        g_coast = 0;
        regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_COAST;
        g_recommutate = 1;
    }
    if (regs[REG_TORQUE_ENABLE] == 1 && st_state) {           // самотест ведёт мотор сам
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
        {   // режим коммутации переключается только без момента
            uint8_t want = (regs[REG_CFG_FLAGS] & CFG_SINE) ? 1u : 0u;
            if (want != sine_on) {
                sine_on = want;
                if (want) { TIM_SR(TIM1_B) &= ~1u; TIM_DIER(TIM1_B) |= 1u; }
                else      { TIM_DIER(TIM1_B) &= ~1u; }
            }
        }
        if (st_state) {                                                // момент сняли посреди самотеста — прервать
            st_state = 0; regs2[R2_ST_RESULT - REGS2_BASE] |= 0x40 | 0x80; regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_SELFTEST;
        }
        if (regs2[R2_CFG2_CMD - REGS2_BASE] == 2) { regs2[R2_CFG2_CMD - REGS2_BASE] = 0; selftest_start(); }
        if (regs2[R2_FAULT_MODE - REGS2_BASE] == 1) {
            if (!g_coast) { motor_coast(); regs2[R2_STATUS2 - REGS2_BASE] |= ST2_COAST; }
        } else {
            if (g_coast) { g_coast = 0; regs2[R2_STATUS2 - REGS2_BASE] &= (uint8_t)~ST2_COAST; }
            g_recommutate = 1;                                             // CCR = 0 → нижние ключи = тормоз
        }
    }
    // контакт/удар: ротор резко затормозил вдоль движения, а контур толкал в ту же сторону (3–5 мс, без IMU)
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
    // детект упора/столкновения: контур хочет ехать (|out| ≥ STALL_DUTY до потолков), а ротор стоит
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
    // автомат состояний для хоста (CiA 402-подобный, только индикация)
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
