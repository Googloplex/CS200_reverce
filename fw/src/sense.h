// Датчики: АЦП через DMA (4 канала), положение/ток/напряжение, ШИМ обратной связи на PA0.
#pragma once
#include <stdint.h>

#define ADC_IDX_PA1    0   // ADCIN1  — не используется
#define ADC_IDX_VBUS   1   // ADCIN7  — делитель напряжения шины
#define ADC_IDX_CURR   2   // ADCIN11 — ток (ОУ шунта)
#define ADC_IDX_POT    3   // ADCIN15 — потенциометр

extern volatile uint16_t adc_buf[4];
extern int16_t g_present_pos;      // RAM_2000006E, единицы АЦП
extern volatile uint16_t g_i_wind_ma;   // v1.2: измеренный ток фазы, мА (синхронная выборка)
extern volatile uint8_t  g_i_valid;     // выборка попала в окно открытого ключа
extern volatile uint16_t g_awd_count;   // срабатываний аппаратного лимита

void sense_adc_init(void);         // FUN_08003220
void sense_tim16_init(void);       // выход ШИМ обратной связи (FUN_08004758)
void sense_update(void);           // каждый проход главного цикла (FUN_080041b0)
void sense_mcu_temp_poll(void);    // температура кристалла раз в секунду (подменяет последовательность АЦП на ~100 мкс)
extern volatile uint8_t g_bb_freeze_req;   // ISR просит заморозить чёрный ящик
void sense_init_calib(void);       // v1: смещение нуля тока из журнала
