// Sensors: ADC via DMA (4 channels), position/current/voltage, feedback PWM on PA0.
#pragma once
#include <stdint.h>

#define ADC_IDX_PA1    0   // ADCIN1  — unused
#define ADC_IDX_VBUS   1   // ADCIN7  — DC bus voltage divider
#define ADC_IDX_CURR   2   // ADCIN11 — current (shunt op-amp)
#define ADC_IDX_POT    3   // ADCIN15 — potentiometer

extern volatile uint16_t adc_buf[4];
extern int16_t g_present_pos;      // RAM_2000006E, ADC units
extern volatile uint16_t g_i_wind_ma;   // v1.2: measured phase current, mA (synchronous sampling)
extern volatile uint8_t  g_i_valid;     // sample fell within the on-window of the switch
extern volatile uint16_t g_awd_count;   // hardware limit trips

void sense_adc_init(void);         // FUN_08003220
void sense_tim16_init(void);       // feedback PWM output (FUN_08004758)
void sense_update(void);           // every main loop pass (FUN_080041b0)
void sense_mcu_temp_poll(void);    // die temperature once per second (replaces the ADC sequence for ~100 µs)
extern volatile uint8_t g_bb_freeze_req;   // ISR requests flight recorder freeze
void sense_init_calib(void);       // v1: current zero offset from the journal
