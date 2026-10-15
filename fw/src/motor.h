// Мотор: TIM1 → предрайвер, коммутация по Холлам, контур положения 1 кГц, вход ШИМ, защиты v1.
#pragma once
#include <stdint.h>

void motor_tim1_init(void);
void motor_tim6_init(void);
void motor_tim14_init(void);
void motor_tim15_init(void);
void motor_new_goal(void);
void motor_stall_check(void);
uint16_t motor_duty(void);      // текущая скважность 0..1023
int8_t   motor_out_sign(void);  // знак выхода контура: +1 к росту положения, −1, 0 = нет момента
void     motor_selftest_request(void);
uint16_t motor_pwm_in(void);    // последний импульс на PA3, мкс
uint32_t micros(void);

/* Холлы как датчик (v1) */
extern volatile int32_t  g_hall_count;
extern volatile uint32_t g_hall_period_us;
extern volatile uint8_t  g_hall_code;
extern volatile int32_t  g_speed;        // шагов Холлов/с со знаком
extern volatile uint8_t  g_status;       // флаги ST_*
extern volatile uint8_t  g_thermal_pct;
extern volatile uint16_t g_r_est_mohm;   // оценка сопротивления обмотки
extern volatile int8_t   g_t_est_c;      // оценка температуры обмотки
extern volatile int32_t  g_vel_cnt;      // скорость ед АЦП/с по Холлам
extern volatile uint32_t g_odo_steps;    // одометр ротора, шагов Холлов
extern volatile uint8_t  g_latched;      // защёлкнутый отказ (CFG_LATCH): момент не включить до CFG2_CMD = 3
