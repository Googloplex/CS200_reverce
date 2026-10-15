// Регистры LCM32F037, которые использует прошивка. Смещения — из SVD пака (docs/datasheets/.../LCM32F037.svd).
// Свои определения вместо структур вендорского заголовка: там есть объединения с неочевидными именами.
#pragma once
#include <stdint.h>
#include "LCM32F037.h"   // IRQn_Type, core_cm0.h (NVIC, SysTick, SCB)

#define REG32(base, off) (*(volatile uint32_t *)((uintptr_t)(base) + (off)))

/* ---------------- CHIPCTRL 0x40017000: тактирование, питание ---------------- */
#define CC_BASE        0x40017000u
#define CC_CLK_CFG     REG32(CC_BASE, 0x00)   // clk_SRC_SEL[21:20] RCH_EN[19] OSCH_EN[17] PLL_EN[16] APB1_DIV[15:12] APB0_DIV[11:8] SYS_CLK_SEL[7:0]
#define CC_PWR_CFG     REG32(CC_BASE, 0x04)
#define CC_CTRL        REG32(CC_BASE, 0x08)
#define CC_PLL_CFG     REG32(CC_BASE, 0x0C)   // PLL_OD[12] PLL_DM[9:8] PLL_DN[6:2] PLL_LS[1] PLL_SRC[0]
#define CC_OSCH_CFG    REG32(CC_BASE, 0x14)
#define CC_STS         REG32(CC_BASE, 0x18)   // SYS_CLK_LOCK[22] PLL_LOCK[20] OSCH_STB[19]
#define CC_KEY         REG32(CC_BASE, 0x20)   // ключ записи
#define CC_BDCR        REG32(CC_BASE, 0x28)
#define CC_UNLOCK_KEY  0x87214365u

/* ---------------- SYSCTRL 0x48007000: клоки периферии, маршрутизация ---------------- */
#define SC_BASE        0x48007000u
#define SC_CLKEN0      REG32(SC_BASE, 0x00)
#define SC_CLKEN1      REG32(SC_BASE, 0x04)
#define SC_CLKEN2      REG32(SC_BASE, 0x08)
#define SC_DBG_CFG     REG32(SC_BASE, 0x14)   // *_STOP: остановка таймеров/сторожей при остановке ядра отладчиком
#define SC_EDU_CFG4    REG32(SC_BASE, 0x30)   // TIM1: OCCSE[29:24] (OCREF_CLR: ADC_AWD,TIM15,TIM3,ACMP2,ACMP1,ACMP0), BKSE[14:8], SP[6:0]
#define SC_EDU_CFG0    REG32(SC_BASE, 0x20)   // DMA_REQn_MAP — по 4 бита на линию запроса
#define SC_KEY         REG32(SC_BASE, 0x7C)
#define SC_UNLOCK_KEY  0x05FA659Au

#define SC_CLK_GPIOA   (1u << 0)
#define SC_CLK_GPIOB   (1u << 1)
#define SC_CLK_DMA     (1u << 8)
#define SC_CLK_ADC     (1u << 9)
#define SC_CLK_UART1   (1u << 14)
#define SC_CLK_EFLS    (1u << 16)
#define SC_CLK_RAM0    (1u << 17)
#define SC_CLK_RAM1    (1u << 18)
#define SC_CLK_TIM1    (1u << 19)
#define SC_CLK_GPIOF   (1u << 21)
#define SC_CLK_TIM6    (1u << 26)
#define SC_CLK_TIM14   (1u << 27)
#define SC_CLK_TIM15   (1u << 28)
#define SC_CLK_TIM16   (1u << 29)
#define SC_CLK2_ANA    (1u << 5)
#define SC_CLK2_EXTI   (1u << 6)

/* ---------------- GPIO ---------------- */
#define GPIOA_B        0x48000000u
#define GPIOB_B        0x48000200u
#define GPIOF_B        0x48000A00u
#define GPIO_MODER(p)   REG32(p, 0x00)   // 2 бита на пин: 0 вход, 1 выход, 2 AF, 3 аналог
#define GPIO_OD_BSRR(p) REG32(p, 0x04)   // open-drain: бит n — установить, бит n+16 — сбросить
#define GPIO_SR_BSRR(p) REG32(p, 0x08)   // slew rate
#define GPIO_PU_BSRR(p) REG32(p, 0x0C)   // подтяжка вверх
#define GPIO_PD_BSRR(p) REG32(p, 0x10)   // подтяжка вниз
#define GPIO_IDR(p)     REG32(p, 0x14)
#define GPIO_ODR(p)     REG32(p, 0x18)
#define GPIO_BSRR(p)    REG32(p, 0x1C)
#define GPIO_AFRL(p)    REG32(p, 0x24)   // ВНИМАНИЕ: у LCM32 AFRL по 0x24, AFRH по 0x28 (не как у STM32)
#define GPIO_AFRH(p)    REG32(p, 0x28)
#define GPIO_OS_BSRR(p) REG32(p, 0x38)   // open-source

/* ---------------- Таймеры (STM32-подобные) ---------------- */
#define TIM1_B         0x40010000u
#define TIM6_B         0x40009000u
#define TIM14_B        0x4000A000u
#define TIM15_B        0x4001B000u
#define TIM16_B        0x4001C000u
#define TIM_CR1(t)     REG32(t, 0x00)
#define TIM_CR2(t)     REG32(t, 0x04)
#define TIM_SMCR(t)    REG32(t, 0x08)
#define TIM_DIER(t)    REG32(t, 0x0C)
#define TIM_SR(t)      REG32(t, 0x10)
#define TIM_EGR(t)     REG32(t, 0x14)
#define TIM_CCMR1(t)   REG32(t, 0x18)
#define TIM_CCMR2(t)   REG32(t, 0x1C)
#define TIM_CCER(t)    REG32(t, 0x20)
#define TIM_CNT(t)     REG32(t, 0x24)
#define TIM_PSC(t)     REG32(t, 0x28)
#define TIM_ARR(t)     REG32(t, 0x2C)
#define TIM_RCR(t)     REG32(t, 0x30)
#define TIM_CCR1(t)    REG32(t, 0x34)
#define TIM_CCR2(t)    REG32(t, 0x38)
#define TIM_CCR3(t)    REG32(t, 0x3C)
#define TIM_CCR4(t)    REG32(t, 0x40)
#define TIM_BDTR(t)    REG32(t, 0x44)

/* ---------------- UART (ARM PL011) ---------------- */
#define UART1_B        0x40015000u
#define UART_DR(u)     REG32(u, 0x00)
#define UART_FR(u)     REG32(u, 0x18)
#define UART_IBRD(u)   REG32(u, 0x24)
#define UART_FBRD(u)   REG32(u, 0x28)
#define UART_LCR_H(u)  REG32(u, 0x2C)
#define UART_CR(u)     REG32(u, 0x30)
#define UART_IFLS(u)   REG32(u, 0x34)
#define UART_DMACR(u)  REG32(u, 0x48)
#define UART_FR_BUSY   (1u << 3)

/* ---------------- DMAC (DesignWare DW_ahb_dmac) 0x48004000 ---------------- */
#define DMAC_B           0x48004000u
#define DMA_CH(n)        (DMAC_B + 0x58u * (n))
#define DMA_SAR(c)       REG32(c, 0x00)
#define DMA_DAR(c)       REG32(c, 0x08)
#define DMA_LLP(c)       REG32(c, 0x10)
#define DMA_CTL_L(c)     REG32(c, 0x18)   // INT_EN[0] DST_W[3:1] SRC_W[6:4] DINC[8:7] SINC[10:9] DMS[13:11] SMS[16:14] TT_FC[22:20]
#define DMA_CTL_H(c)     REG32(c, 0x1C)   // BLOCK_TS[11:0]; при чтении во время обмена — сколько уже передано
#define DMA_CFG_L(c)     REG32(c, 0x40)   // CH_PRIOR[7:5] RELOAD_SRC[30] RELOAD_DST[31]
#define DMA_CFG_H(c)     REG32(c, 0x44)   // FIFO_MODE[1] SRC_PER[10:7] DEST_PER[14:11]
#define DMAC_RAWTFR      REG32(DMAC_B, 0x2C0)
#define DMAC_STATUSBLOCK REG32(DMAC_B, 0x2F0)
#define DMAC_MASKBLOCK   REG32(DMAC_B, 0x318)
#define DMAC_CLEARTFR    REG32(DMAC_B, 0x338)
#define DMAC_CLEARBLOCK  REG32(DMAC_B, 0x340)
#define DMAC_CLEARSRC    REG32(DMAC_B, 0x348)
#define DMAC_CLEARDST    REG32(DMAC_B, 0x350)
#define DMAC_CLEARERR    REG32(DMAC_B, 0x358)
#define DMAC_CFG         REG32(DMAC_B, 0x398)
#define DMAC_CHEN        REG32(DMAC_B, 0x3A0)   // бит n — состояние, бит n+8 — разрешение записи

/* ---------------- ADC 0x4001A000 ---------------- */
#define ADC_B          0x4001A000u
#define ADC_ISR        REG32(ADC_B, 0x00)
#define ADC_CR         REG32(ADC_B, 0x08)
#define ADC_CFGR1      REG32(ADC_B, 0x0C)
#define ADC_CFGR2      REG32(ADC_B, 0x10)
#define ADC_SMPR       REG32(ADC_B, 0x14)
#define ADC_CHSELR     REG32(ADC_B, 0x1C)
#define ADC_DR_ADDR    (ADC_B + 0x20u)
#define ADC_IER        REG32(ADC_B, 0x04)
#define ADC_TR         REG32(ADC_B, 0x18)   // HT[27:16] LT[11:0] — пороги аналогового сторожа
#define ADC_TRIM       REG32(ADC_B, 0x24)
#define ADC_EXTCFG     REG32(ADC_B, 0x28)
#define ADC_TESTCFG    REG32(ADC_B, 0x2C)
#define ADC_TRIM_FACTORY_WORD  (*(volatile uint32_t *)0x1FFFF5ECu)   // если не FF — TRIM уже загружен из инфо-зоны

/* ---------------- ANACTRL 0x40017C00: компараторы, DAC ---------------- */
#define ANA_B          0x40017C00u
#define ACMP2_CSR      REG32(ANA_B, 0x28)   // CP_EN[0] CP_FREN[1] CP_HYSEN[2] CP_IS[3] CP_FS[8] CP_DLY[10:9] CP_BLANKING[14:12] CP_PF[20:16]
#define ACMP2_MUXCR    REG32(ANA_B, 0xD0)   // N_AMUX[25:24]: 00=DAC0, P_AMUX[17:16]: 01=CPP21 (PF6)
#define DAC0_CR        REG32(ANA_B, 0x40)   // EN[0] TEN[2:1] TSEL[5:3] VREF_SEL[14]
#define DAC0_DHR       REG32(ANA_B, 0x48)   // 10 бит, VDDA
#define ANA_CSR        REG32(ANA_B, 0xF8)   // DAC0_BUF_EN[4] — буфер на PA4 держать выключенным (PA4 = Холл A)

/* ---------------- EXTI 0x40011000 ---------------- */
#define EXTI_B         0x40011000u
#define EXTI_IMR       REG32(EXTI_B, 0x00)
#define EXTI_EMR       REG32(EXTI_B, 0x04)
#define EXTI_RTSR      REG32(EXTI_B, 0x08)
#define EXTI_FTSR      REG32(EXTI_B, 0x0C)
#define EXTI_PR        REG32(EXTI_B, 0x14)

/* ---------------- IWDG 0x40017400 (STM32-подобный: KR/PR/RLR/SR) ---------------- */
#define IWDG_B         0x40017400u
#define IWDG_KR        REG32(IWDG_B, 0x00)
#define IWDG_PR        REG32(IWDG_B, 0x04)
#define IWDG_RLR       REG32(IWDG_B, 0x08)
#define IWDG_SR        REG32(IWDG_B, 0x0C)

/* ---------------- FLASH 0x4001D000 ---------------- */
#define FLASH_B        0x4001D000u
#define FLASH_ACR      REG32(FLASH_B, 0x00)
#define FLASH_KEYR     REG32(FLASH_B, 0x04)
#define FLASH_SR       REG32(FLASH_B, 0x0C)   // BSY[0] WRPRTERR[2]? PGERR[4]? EOP[5]
#define FLASH_CR       REG32(FLASH_B, 0x10)   // PG[0] PER[1] STRT[6] LOCK[7]
#define FLASH_AR       REG32(FLASH_B, 0x14)
#define FLASH_KEY1     0x45670123u
#define FLASH_KEY2     0xCDEF89ABu
#define FLASH_PAGE     512u
