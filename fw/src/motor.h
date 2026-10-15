// Motor: TIM1 → predriver, Hall commutation, 1 kHz position loop, PWM input, v1 protections.
#pragma once
#include <stdint.h>

void motor_tim1_init(void);
void motor_tim6_init(void);
void motor_tim14_init(void);
void motor_tim15_init(void);
void motor_new_goal(void);
void motor_stall_check(void);
uint16_t motor_duty(void);      // current duty 0..1023
int8_t   motor_out_sign(void);  // loop output sign: +1 toward increasing position, −1, 0 = no torque
void     motor_selftest_request(void);
uint16_t motor_pwm_in(void);    // last pulse on PA3, µs
uint32_t micros(void);

/* Halls as a sensor (v1) */
extern volatile int32_t  g_hall_count;
extern volatile uint32_t g_hall_period_us;
extern volatile uint8_t  g_hall_code;
extern volatile int32_t  g_speed;        // Hall steps/s, signed
extern volatile uint8_t  g_status;       // ST_* flags
extern volatile uint8_t  g_thermal_pct;
extern volatile uint16_t g_r_est_mohm;   // winding resistance estimate
extern volatile int8_t   g_t_est_c;      // winding temperature estimate
extern volatile int32_t  g_vel_cnt;      // speed in ADC units/s from Halls
extern volatile uint32_t g_odo_steps;    // rotor odometer, Hall steps
extern volatile uint8_t  g_latched;      // latched fault (CFG_LATCH): torque cannot be enabled until CFG2_CMD = 3
