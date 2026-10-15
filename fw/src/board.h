// Kpower HB200T board: pin assignment (docs/pinmap.md) and low-level helpers.
#pragma once
#include <stdint.h>
#include "lcm32.h"

/* Pins [FW] */
#define PIN_BUS        2    // PA2  — wire S: UART1, AF1 = TX, AF7 = RX (half-duplex)
#define PIN_PWM_IN     3    // PA3  — PWM input 900..2100 µs: TIM15_CH2 (AF0)
#define PIN_FB_OUT     0    // PA0  — position feedback PWM output: TIM16_CH1 (AF3)
#define PIN_HALL_A     4    // PA4  — Halls with pull-up, EXTI4..6
#define PIN_HALL_B     5
#define PIN_HALL_C     6
/* analog: PA1 — ADCIN1 (unused), PA7 — ADCIN7 (Vbus), PA10 — ADCIN15 (potentiometer), PF6 — ADCIN11 (current) */
/* predriver: PF4 AF2 = TIM1_CH3 (HIN1), PB6 AF4 = TIM1_CH3N (LIN1), PF3 AF2 = TIM1_CH2 (HIN2),
   PB4 AF2 = TIM1_CH2N (LIN2), PB7 AF5 = TIM1_CH1 (HIN3), PB3 AF1 = TIM1_CH1N (LIN3) */

#define AF_BUS_TX      1
#define AF_BUS_RX      7

/* Interrupt priorities (2 bits) — as in stock firmware */
#define PRIO_HALL      1
#define PRIO_DMA       2
#define PRIO_TIM15     2
#define PRIO_TIM6      2
#define PRIO_TIM14     3

typedef struct {
    uint16_t pins;   // mask
    uint8_t  mode;   // 0 input, 1 output, 2 AF, 3 analog
    uint8_t  speed;  // 0/1 → SR_BSRR
    uint8_t  otype;  // 0 push-pull, 1 open-drain, 2 open-source
    uint8_t  pupd;   // 0 none, 1 up, 2 down, 3 both
} gpio_cfg_t;

void gpio_init(uint32_t port, const gpio_cfg_t *c);
void gpio_af(uint32_t port, unsigned pin, unsigned af);

void sc_clk0_enable(uint32_t mask);   // SYSCTRL ClkEnR0 |= mask (with key)
void sc_clk2_enable(uint32_t mask);   // SYSCTRL ClkEnR2 |= mask
void sc_dma_req_map(unsigned req, unsigned periph);   // EDU_CFG0: DMA request line ← peripheral
void cc_pwr_cfg_and(uint32_t mask);   // CHIPCTRL PWR_CFG &= mask
void sc_dbg_stop(uint32_t mask);      // SYSCTRL DBG_CFG |= mask (IWDG_STOP = bit 9)
void sc_edu_cfg4(uint32_t val);       // SYSCTRL EDU_CFG4 = val (TIM1 OCREF_CLR/break sources)

void dma_ch_deinit(unsigned ch);
void dma_ch_enable(unsigned ch, int on);

void board_clocks_init(void);         // peripheral clocks (FUN_08003f78)
void board_gpio_init(void);           // all board pins (FUN_0800346c)
void board_nvic_init(void);           // SysTick 1 kHz + interrupt priorities/enables
void board_reset(void);               // NVIC_SystemReset

extern volatile uint32_t g_ms;        // milliseconds since start (SysTick)
