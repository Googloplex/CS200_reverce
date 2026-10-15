// Таблица регистров (Dynamixel AX-12 + свои Kpower) и журнал настроек во Flash.
// Раскладка и смысл полей — docs/native_fw_config.md.
#pragma once
#include <stdint.h>

#define REG_MODEL          0x00   // u16 = 12 (AX-12)
#define REG_FW_VER         0x02
#define REG_ID             0x03
#define REG_BAUD           0x04   // скорость = 2 000 000 / (BAUD + 1)
#define REG_RETURN_DELAY   0x05   // × 2 мкс
#define REG_CW_LIMIT       0x06   // u16
#define REG_CCW_LIMIT      0x08   // u16
#define REG_TEMP_LIMIT     0x0B
#define REG_MIN_VOLT       0x0C
#define REG_MAX_VOLT       0x0D
#define REG_MAX_TORQUE     0x0E   // u16
#define REG_STATUS_RETURN  0x10
#define REG_ALARM_LED      0x11
#define REG_ALARM_SHUTDOWN 0x12
#define REG_POS_OFFSET     0x14   // s16: поправка к Present Position [ПРОШ]
#define REG_TORQUE_ENABLE  0x18
#define REG_LED            0x19
#define REG_COMPL_MARGIN_CW  0x1A
#define REG_COMPL_MARGIN_CCW 0x1B
#define REG_COMPL_SLOPE_CW   0x1C
#define REG_COMPL_SLOPE_CCW  0x1D
#define REG_GOAL           0x1E   // u16, мкс импульса 900..2100 [ПРОШ]
#define REG_SPEED          0x20   // u16, в режиме колеса: бит 10 — знак
#define REG_TORQUE_LIMIT   0x22   // u16
#define REG_PRESENT_POS    0x24   // s16, единицы АЦП (0..4095)
#define REG_PRESENT_LOAD   0x28   // u16, ≈ мА
#define REG_PRESENT_VOLT   0x2A   // 0.1 В, калибровано; насыщение 25.5 В (legacy u8)
#define REG_PRESENT_TEMP   0x2B   // не измеряется
#define REG_REGISTERED     0x2C
#define REG_LOCK           0x2F
#define REG_PUNCH          0x30   // u16, минимальный выход
#define REG_33             0x33   // байт из кода родной прошивки по 0x08002000 (= 0x12)
#define REG_PWM_MIN        0x40   // мкс (900)
#define REG_PWM_CENTER     0x42   // мкс (1500)
#define REG_PWM_MAX        0x44   // мкс (2100)
#define REG_46             0x46   // u16, = travel·2048/REG_4C (1068)
#define REG_48             0x48
#define REG_TRAVEL         0x4A   // u16, градусы (120)
#define REG_4C             0x4C   // u16 (230)
#define REG_K_CW           0x4E   // байт: масштаб хода вниз от центра (0 → travel)
#define REG_K_CCW          0x4F   // байт: масштаб хода вверх от центра (0 → travel)
#define REG_GOAL_ADC       0x5A   // u16, цель в единицах АЦП (только чтение)

/* ---- свои настройки v1 (0x50..0x5F, сохраняются в журнал; 0 = выключено) ---- */
#define REG_LIMIT_CURRENT  0x50   // u16 мА, предел оценки тока обмотки
#define REG_LIMIT_SPEED    0x52   // u16 шагов Холлов/с, предел оборотов
#define REG_BUS_TIMEOUT    0x54   // u16 мс без пакетов → момент выкл
#define REG_VBUS_MAX       0x56   // u16 мВ, выше → момент выкл
#define REG_I_CONT         0x58   // u16 мА, длительный ток для I²t
#define REG_LOAD_OFFSET    0x5C   // u16, смещение нуля датчика тока (автокалибровка при выкл. моменте)
#define REG_CFG_FLAGS      0x5E   // u8: 1 IWDG, 2 сбой Холлов → момент выкл, 4 кольцевая ошибка (0..4095), 8 инверсия знака скорости
#define REG_BUS_SLOT       0x5F   // u8 ×10 мкс, слот ответа в SYNC_READ (по умолчанию 30 → 300 мкс)
/* блок 0x13..0x17 (EEPROM-часть, Lock действует) и 0x34..0x3F */
#define REG_STEPS_PER_CNT  0x13   // u8 ×0.001 шагов Холлов на единицу АЦП (измерено 222)
#define REG_R_REF          0x16   // u16 мОм, сопротивление обмотки при 25 °C (измерено ~550) — для оценки температуры
#define REG_LIMIT_HW       0x32   // u8 ×100 мА: аппаратный лимит среднего тока шины (сторож АЦП → OCREF_CLR TIM1), 0 = выкл
#define REG_KV             0x34   // u16 об/мин/В (измерено 1470 при pp = 1)
#define REG_PP             0x36   // u8 пар полюсов
#define REG_CTRL_MODE      0x37   // u8: 0 — контроллер стока, 1 — v1 (трапеция + PD по скорости Холлов)
#define REG_PROFILE_VMAX   0x38   // u16 ед АЦП/с, 0 = без ограничения
#define REG_PROFILE_ACC    0x3A   // u16 ×100 ед АЦП/с², 0 = без ограничения
#define REG_KP             0x3C   // u8 ×0.1 скважности на единицу ошибки (сток ≈ 31)
#define REG_KD             0x3D   // u8 скважности на 100 шагов/с ошибки скорости
#define REG_VBUS_MIN       0x3E   // u16 мВ, ниже → момент выкл, флаг 0x40
#define CFG_WDT        0x01
#define CFG_HALL_OFF   0x02
#define CFG_WRAP       0x04
#define CFG_SPEED_INV  0x08
#define CFG_SINE      0x10   // синусоидальная коммутация с интерполяцией угла по Холлам (переключается при TE = 0)
#define CFG_LATCH     0x20   // жёсткие отказы (Vbus, Холлы) защёлкиваются: момент не включить до CFG2_CMD = 3
#define CFG_ILOOP     0x40   // в режиме момента (Kp = Kd = 0) ff в мА замыкается по измеренному току (медленный PI поверх модели)

/* ---- телеметрия v1 (0x60..0x77, только чтение) ---- */
#define REG_HALL_SPEED     0x60   // s16, шагов Холлов/с, знак = направление
#define REG_HALL_COUNT     0x62   // s32, накопленные шаги Холлов
#define REG_I_BUS          0x66   // u16 мА, ток шины по шунту
#define REG_VBUS_MV        0x68   // u16 мВ (калибровано по ЛБП: 1829 ед = 24.0 В)
#define REG_I_WIND         0x6A   // u16 мА, оценка тока обмотки = I_bus / скважность
#define REG_STATUS         0x6C   // u8 флаги: 1 лимит тока, 2 лимит оборотов, 4 таймаут шины, 8 Холлы, 0x10 Vbus, 0x20 I²t
#define REG_HALL_CODE      0x6D   // u8
#define REG_UPTIME         0x6E   // u16 с
#define REG_THERMAL        0x70   // u8 % теплового бюджета I²t
#define REG_I_WIND_MAX     0x72   // u16 мА, пик с запуска
#define REG_DUTY           0x74   // u16, скважность 0..1023
#define REG_R_EST          0x76   // u16 мОм, оценка сопротивления обмотки онлайн (0 = нет данных)
#define REG_T_EST          0x71   // s8 °C, оценка температуры обмотки по R_EST/R_REF

#define ST_CUR_LIMIT  0x01
#define ST_SPD_LIMIT  0x02
#define ST_BUS_TMO    0x04
#define ST_HALL_BAD   0x08
#define ST_VBUS_HIGH  0x10
#define ST_I2T        0x20
#define ST_VBUS_LOW   0x40
#define ST_WDT_GUARD  0x80   // IWDG отключён защитой от цикла перезагрузок

#define REGS_SIZE          0x80

/* ---- расширенная таблица 0x80..0xBF (RAM, в журнал не пишется) ---- */
#define REGS2_BASE         0x80
#define REGS2_SIZE         0x80
/* команда импеданса (пишется одним SYNC_WRITE: addr 0x80, 16 байт) */
#define R2_CMD_POS         0x80   // s16 ед АЦП — цель
#define R2_CMD_VEL         0x82   // s16 ед АЦП/с — скорость; цель едет с ней 100 мс после команды (удержание 1-го порядка)
#define R2_CMD_KP          0x84   // u16 ×0.01 скважности/ед (310 ≈ сток)
#define R2_CMD_KD          0x86   // u16 ×0.001 скважности/(ед/с)
#define R2_CMD_FF          0x88   // s16 мА — прямой момент (через R_EST и Ke·ω → скважность)
#define R2_CMD_TMAX        0x8A   // u16 предел скважности 0..1023 (0 = 1023)
#define R2_CMD_MODE        0x8C   // u8: 0 — старый контур по Goal, 1 — импеданс (включает момент), 2 — импеданс с кубической
                                  //     интерполяцией цели от текущей (p, v) к (CMD_POS, CMD_VEL) к моменту CMD_AT
#define R2_CMD_AT          0x8D   // u16 время хоста (мл. 16 бит мс), когда применить; 0 = сразу
#define R2_CMD_SEQ         0x8F   // u8: номер команды, ПОСЛЕДНИЙ байт блока — смена значения = команда записана целиком (SWD и шина)
/* быстрая телеметрия (SYNC_READ 0x90, 16 байт) */
#define R2_TLM_POS         0x90   // s16 ед АЦП
#define R2_TLM_VEL         0x92   // s16 ед АЦП/с (по Холлам)
#define R2_TLM_IBUS        0x94   // u16 мА
#define R2_TLM_TEMP        0x96   // s8 °C обмотки (оценка)
#define R2_TLM_STATUS      0x97   // u8 флаги
#define R2_TLM_VBUS        0x98   // u16 мВ
#define R2_TLM_DUTY        0x9A   // u16
#define R2_TLM_TIME        0x9C   // u32 мс по часам хоста (после TIME_SYNC), иначе локальные
#define R2_TIME_OFFSET     0xA0   // u32 смещение host − local (только чтение)
/* v1.5: мягкие упоры, политика отказа, детект упора/столкновения */
#define R2_SOFT_MIN        0xA4   // u16 ед АЦП — мягкий упор снизу (0 и 0 = выкл): цель режется, за упором пружина SOFT_KP
#define R2_SOFT_MAX        0xA6   // u16
#define R2_SOFT_KP         0xA8   // u8 ×0.1 duty/ед жёсткость пружины за упором (50)
#define R2_FAULT_MODE      0xA9   // u8: что делать без момента/при отказе: 0 тормоз (нижние ключи), 1 свободный ход (все ключи закрыты),
                                  //     2 удержание — при таймауте шины/столкновении импеданс в точке отказа (tmax = HOLD_TMAX)
#define R2_STALL_DUTY      0xAA   // u16: |выход контура| ≥ этого при скорости < 100 ед/с дольше STALL_MS → упор (0 = выкл)
#define R2_STALL_MS        0xAC   // u8 мс (50)
#define R2_STALL_ACT       0xAD   // u8: 0 только флаг, 1 ослабить выход до 25 % пока упор, 2 отказ по FAULT_MODE
#define R2_STATUS2         0xAE   // u8: 1 упор, 2 за мягким упором, 4 удержание по отказу, 8 идёт интерполяция, 16 свободный ход
#define R2_TAU_AVAIL       0xAF   // u8 % доступного момента сейчас: LIMIT_CURRENT с линейным снижением к I_CONT по мере расхода I²t (SOA)
#define R2_HOLD_TMAX       0xB0   // u16 предел duty удержания по отказу (300)

/* v1.5: паспорт, счётчики, температура кристалла, чёрный ящик */
#define R2_TEMP_MCU        0xB2   // s8 °C кристалла: ADC_IN16 по заводской калибровке (0x1FFFF580: T1, T2, отсчёты при 3.3 В)
#define R2_STATE           0xB3   // u8 автомат: 0 BOOT, 1 READY, 2 ENABLED, 3 HOLD (удержание по отказу), 4 SELFTEST, 5 FAULT, 6 LATCHED (ждёт CFG2_CMD = 3)
#define R2_UID             0xB4   // 12 байт — 96-битный UID чипа, копия 0x1FFFF5B0 [ГИП: адрес по виду данных]
#define R2_ENERGY_MJ       0xC0   // u32 мДж, взятых из шины с запуска (Vbus·I_bus; рекуперацию шунт не видит)
#define R2_ODO_STEPS       0xC4   // u32 шагов Холлов по модулю с запуска (одометр ротора)
#define R2_PKT_OK          0xC8   // u16 пакетов нам с верной контрольной суммой
#define R2_PKT_ERR         0xCA   // u16 пакетов нам с ошибкой контрольной суммы
#define R2_FRIC_IC         0xCC   // u16 мА — ток трения Кулона (из самотеста: точки +150/+300 duty без нагрузки) [ГИП]
#define R2_FRIC_KV         0xCE   // u16 мкА на шаг/с — вязкое трение; I_fric = IC + KV·|ω|/1000; вычитается из момента телеметрии
#define R2_BB_CTRL         0xD0   // u8 чёрный ящик: запись 0 — пуск, 1 — заморозить; чтение: 1 = заморожен
#define R2_BB_PAGE         0xD1   // u8 страница окна 0..16 (по 3 записи, от старой к новой)
#define R2_BB_HEAD         0xD2   // u16 число записей с запуска (младшие 16 бит)
#define R2_BB_TRIG         0xD4   // u8 что заморозило: биты STATUS, 0x80 = упор
#define R2_CFG2_CMD        0xD5   // u8 команда: 1 — сохранить настройки v1.5 во Flash 0x08007A00, 2 — самотест/идентификация,
                                  //    3 — снять защёлкнутый отказ, 4 — TEST_UNLIMITED: 10 с без потолков тока/оборотов/I²t, флаг 0x80 в STATUS2,
                                  //    по истечении чёрный ящик замораживается с предысторией
#define R2_CONTACT_DECEL   0xD6   // u16 ед/с за 1 мс: торможение ротора вдоль движения при толкающем контуре → контакт (0 = выкл)
#define R2_TLM_TORQUE      0xD8   // s16 мН·м на выходе: I_wind·Kt·N (без КПД редуктора), знак = направление выхода
#define R2_N_GEAR          0xDA   // u16 ×0.1 передаточное число (2230) [ГИП]
#define R2_KT              0xDC   // u16 мкН·м/А ротора; 0 = из KV: 9549296/KV (6496 при 1470)
#define R2_ST_RESULT       0xDE   // u8 самотест: 1 Холлы, 2 направление как в CFG, 4 R измерен (две точки с разным током — только под нагрузкой; иначе R_REF), 8 Kv, 16 шагов/ед, 0x40 прервано, 0x80 готово
#define R2_ST_FRICTION     0xDF   // u8 duty, при котором ротор стронулся
#define R2_ST_R            0xE0   // u16 мОм по двум рабочим точкам
#define R2_ST_KV           0xE2   // u16 об/мин/В
#define R2_ST_SPC          0xE4   // u16 ×0.001 шагов Холлов на единицу АЦП
#define R2_SINE_ADV        0xE6   // s8 градусов эл.: опережение вектора вдоль момента в синусном режиме
#define R2_SINE_OFS        0xD7   // s8 градусов эл.: постоянный сдвиг угла (расстановка Холлов относительно фаз; стенд: ≈ −6)
#define R2_ST_DIR          0xE7   // s8: −1 счётчик Холлов убывает при росте положения (как принято в прошивке), +1 наоборот
#define R2_BB_WIN          0xE8   // 16 Б: 2 записи × {pos s16, vel s16, I_bus u16, duty/4 u8, status u8}, шаг 5 мс, кольцо 260 мс
#define BB_N               52
#define BB_PER_PAGE        2
#define R2_ST_BACKLASH     0xF8   // u16 шагов Холлов мёртвого хода при реверсе (ротор идёт, выход стоит) — из самотеста
#define BB_PERIOD_MS       5
#define BB_POST            10     // записей после события до заморозки

#define INFO_T1_X10        (*(volatile uint32_t *)0x1FFFF580u)   // температура калибровки 1, ×0.1 °C (26.6)
#define INFO_T2_X10        (*(volatile uint32_t *)0x1FFFF584u)   // температура калибровки 2 (105.0)
#define INFO_VTS1          (*(volatile uint32_t *)0x1FFFF588u)   // отсчёт при T1 (мл. 16 бит — опора 3.3 В)
#define INFO_VTS2          (*(volatile uint32_t *)0x1FFFF58Cu)   // отсчёт при T2
#define INFO_UID           ((const volatile uint8_t *)0x1FFFF5B0u)
#define INFO_DATE          ((const volatile uint8_t *)0x1FFFF854u)   // год u16, месяц, день [ГИП]

#define ST2_STALL     0x01
#define ST2_SOFT      0x02
#define ST2_HOLD      0x04
#define ST2_INTERP    0x08
#define ST2_COAST     0x10
#define ST2_CONTACT   0x20   // удар/контакт: ротор резко затормозил, контур толкал
#define ST2_SELFTEST  0x40   // идёт самотест
#define ST2_UNLIMITED 0x80   // TEST_UNLIMITED активен
#define REGS_DATA_END      0x78   // далее служебная часть журнала: seq, флаг 'Z', CHK, 'K'

extern uint8_t regs[REGS_SIZE];
extern uint8_t regs2[REGS2_SIZE];
static inline uint16_t r2_16(unsigned a) { return (uint16_t)(regs2[a - REGS2_BASE] | (regs2[a - REGS2_BASE + 1] << 8)); }
static inline void set_r2_16(unsigned a, uint16_t v) { regs2[a - REGS2_BASE] = (uint8_t)v; regs2[a - REGS2_BASE + 1] = (uint8_t)(v >> 8); }
static inline void set_r2_32(unsigned a, uint32_t v) { set_r2_16(a, (uint16_t)v); set_r2_16(a + 2, (uint16_t)(v >> 16)); }

static inline uint16_t reg16(unsigned a) { return (uint16_t)(regs[a] | (regs[a + 1] << 8)); }
static inline void set_reg16(unsigned a, uint16_t v) { regs[a] = (uint8_t)v; regs[a + 1] = (uint8_t)(v >> 8); }

/* калибровочные величины, производные от регистров (RAM_20000072..8C в родной) */
typedef struct {
    int16_t  pwm_min, pwm_max;
    int16_t  r46, r48;
    uint16_t center;        // положение (АЦП) при центральном импульсе
    uint16_t travel;        // REG_TRAVEL
    uint16_t r4c;           // REG_4C
    uint32_t speed_k_x256;  // делитель скорости профиля стока ×256: 1023·87·r4c/(4095·60) = r4c·0.36224
} calib_t;
/* fixed point вместо soft-float (v1.5): у M0 нет FPU, библиотека float/double стоила 9.5 КБ Flash */
extern calib_t calib;

uint8_t dxl_checksum(const uint8_t *p, unsigned n);   // ~sum

void regs_factory_defaults(void);   // заводские значения EEPROM-части (FUN_080045dc)
void regs_ram_reset(void);          // RAM-часть после загрузки/сброса (FUN_0800464c)
void calib_from_regs(void);         // пересчёт calib по регистрам (FUN_08003908)

void journal_load(void);            // при старте: найти актуальную запись или создать заводскую
void journal_save(uint8_t boot_flag);   // записать текущие регистры в следующий слот
void cfg2_load(void);               // v1.5: настройки regs2 из сектора 0x08007A00
void cfg2_save(void);
