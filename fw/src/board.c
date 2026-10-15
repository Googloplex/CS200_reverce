#include "board.h"

volatile uint32_t g_ms;

/* ---------- SYSCTRL / CHIPCTRL с ключами ---------- */
static void sc_write(volatile uint32_t *reg, uint32_t val)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    SC_KEY = SC_UNLOCK_KEY;
    *reg = val;
    __set_PRIMASK(pm);
}

void sc_clk0_enable(uint32_t mask) { sc_write(&SC_CLKEN0, SC_CLKEN0 | mask); }
void sc_clk2_enable(uint32_t mask) { sc_write(&SC_CLKEN2, SC_CLKEN2 | mask); }

void sc_dbg_stop(uint32_t mask) { sc_write(&SC_DBG_CFG, SC_DBG_CFG | mask); }
void sc_edu_cfg4(uint32_t val)  { sc_write(&SC_EDU_CFG4, val); }

void sc_dma_req_map(unsigned req, unsigned periph)
{
    sc_write(&SC_EDU_CFG0, SC_EDU_CFG0 | ((uint32_t)periph << ((req & 0x3Fu) << 2)));
}

void cc_pwr_cfg_and(uint32_t mask)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    CC_KEY = CC_UNLOCK_KEY;
    CC_PWR_CFG = CC_PWR_CFG & mask;
    __set_PRIMASK(pm);
}

/* ---------- GPIO (повтор вендорского GPIO_Init) ---------- */
void gpio_init(uint32_t port, const gpio_cfg_t *c)
{
    uint8_t pupd = (c->mode == 1) ? 0 : c->pupd;
    for (unsigned i = 0; i < 16; i++) {
        if (!(c->pins & (1u << i))) {
            continue;
        }
        GPIO_MODER(port) = (GPIO_MODER(port) & ~(3u << (2 * i))) | ((uint32_t)c->mode << (2 * i));
        GPIO_SR_BSRR(port) = c->speed ? (1u << i) : (1u << (i + 16));
        switch (c->otype) {
        case 1:  GPIO_OD_BSRR(port) = 1u << i; break;
        case 2:  GPIO_OS_BSRR(port) = 1u << i; break;
        default: GPIO_OD_BSRR(port) = 1u << (i + 16); GPIO_OS_BSRR(port) = 1u << (i + 16); break;
        }
        switch (pupd) {
        case 1:  GPIO_PD_BSRR(port) = 1u << (i + 16); GPIO_PU_BSRR(port) = 1u << i; break;
        case 2:  GPIO_PU_BSRR(port) = 1u << (i + 16); GPIO_PD_BSRR(port) = 1u << i; break;
        case 3:  GPIO_PU_BSRR(port) = 1u << i;        GPIO_PD_BSRR(port) = 1u << i; break;
        default: GPIO_PU_BSRR(port) = 1u << (i + 16); GPIO_PD_BSRR(port) = 1u << (i + 16); break;
        }
    }
}

void gpio_af(uint32_t port, unsigned pin, unsigned af)
{
    if (pin < 8) {
        GPIO_AFRL(port) = (GPIO_AFRL(port) & ~(0xFu << (4 * pin))) | ((uint32_t)af << (4 * pin));
    } else {
        GPIO_AFRH(port) = (GPIO_AFRH(port) & ~(0xFu << (4 * (pin - 8)))) | ((uint32_t)af << (4 * (pin - 8)));
    }
}

/* ---------- DMAC ---------- */
void dma_ch_deinit(unsigned ch)
{
    uint32_t c = DMA_CH(ch);
    uint32_t bit = 1u << ch;
    DMAC_CFG |= 1u;                                   // контроллер включён
    DMAC_CHEN = (DMAC_CHEN | (bit << 8)) & ~bit;      // канал выключен
    DMA_SAR(c) = 0;
    DMA_DAR(c) = 0;
    DMA_CTL_L(c) = 0;
    DMA_CTL_H(c) = 0;
    DMA_CFG_L(c) = 0;
    DMA_CFG_H(c) = 4;
    DMAC_CLEARTFR = bit;
    DMAC_CLEARBLOCK = bit;
    DMAC_CLEARSRC = bit;
    DMAC_CLEARDST = bit;
    DMAC_CLEARERR = bit;
}

void dma_ch_enable(unsigned ch, int on)
{
    DMAC_CHEN = on ? (0x101u << ch) : ((0x100u << ch) | (DMAC_CHEN & ~(1u << ch)));
}

/* ---------- клоки периферии (порядок как в родной) ---------- */
void board_clocks_init(void)
{
    sc_clk0_enable(SC_CLK_GPIOA);
    sc_clk0_enable(SC_CLK_GPIOB);
    sc_clk0_enable(SC_CLK_GPIOF);
    sc_clk0_enable(SC_CLK_DMA);
    sc_clk0_enable(SC_CLK_TIM6);
    sc_clk0_enable(SC_CLK_TIM14);
    sc_clk0_enable(SC_CLK_EFLS);
    sc_clk0_enable(SC_CLK_TIM1);
    sc_clk0_enable(SC_CLK_UART1);
    sc_clk0_enable(SC_CLK_TIM15);
    sc_clk0_enable(SC_CLK_TIM16);
    sc_clk0_enable(SC_CLK_ADC);
    sc_clk2_enable(SC_CLK2_EXTI);
    sc_clk2_enable(SC_CLK2_ANA);
}

/* ---------- выводы платы (FUN_0800346c) ---------- */
static void exti_line_both_edges(uint32_t line)
{
    EXTI_IMR &= ~line;
    EXTI_EMR &= ~line;
    EXTI_IMR |= line;
    EXTI_RTSR &= ~line;
    EXTI_FTSR &= ~line;
    EXTI_RTSR |= line;
    EXTI_FTSR |= line;
}

void board_gpio_init(void)
{
    gpio_cfg_t c;

    // PA2 — шина, по умолчанию приём (AF7 = UART1_RX). Отличие от родной: подтяжка вверх, чтобы линия
    // не лежала в нуле без мастера и работала с адаптером через диод (в родной pupd = 0).
    c = (gpio_cfg_t){ .pins = 1u << PIN_BUS, .mode = 2, .speed = 1, .otype = 0, .pupd = 1 };
    gpio_init(GPIOA_B, &c);
    gpio_af(GPIOA_B, PIN_BUS, AF_BUS_RX);

    // PA3 — вход ШИМ, TIM15_CH2 (AF0), подтяжка вниз
    c = (gpio_cfg_t){ .pins = 1u << PIN_PWM_IN, .mode = 2, .speed = 1, .otype = 0, .pupd = 2 };
    gpio_init(GPIOA_B, &c);
    gpio_af(GPIOA_B, PIN_PWM_IN, 0);

    // PB3/PB4/PB6/PB7 — входы предрайвера, AF, подтяжка вниз
    c = (gpio_cfg_t){ .pins = 0xD8, .mode = 2, .speed = 1, .otype = 0, .pupd = 2 };
    gpio_init(GPIOB_B, &c);
    gpio_af(GPIOB_B, 3, 1);   // TIM1_CH1N
    gpio_af(GPIOB_B, 4, 2);   // TIM1_CH2N
    gpio_af(GPIOB_B, 6, 4);   // TIM1_CH3N
    gpio_af(GPIOB_B, 7, 5);   // TIM1_CH1
    // PF3/PF4 — входы предрайвера
    c.pins = 0x18;
    gpio_init(GPIOF_B, &c);
    gpio_af(GPIOF_B, 3, 2);   // TIM1_CH2
    gpio_af(GPIOF_B, 4, 2);   // TIM1_CH3

    // PA4..PA6 — Холлы: вход, подтяжка вверх, EXTI по обоим фронтам
    c = (gpio_cfg_t){ .pins = 0x70, .mode = 0, .speed = 1, .otype = 0, .pupd = 1 };
    gpio_init(GPIOA_B, &c);
    exti_line_both_edges(1u << 4);
    exti_line_both_edges(1u << 5);
    exti_line_both_edges(1u << 6);

    // PA1, PA7, PA10 — аналоговые входы АЦП
    c = (gpio_cfg_t){ .pins = 0x482, .mode = 3, .speed = 1, .otype = 0, .pupd = 0 };
    gpio_init(GPIOA_B, &c);
    gpio_af(GPIOA_B, 1, 1);
    gpio_af(GPIOA_B, 7, 1);
    gpio_af(GPIOA_B, 10, 1);
    // PF6 — аналоговый вход (ток)
    c.pins = 0x40;
    gpio_init(GPIOF_B, &c);
    gpio_af(GPIOF_B, 6, 1);

    // PA0 — выход ШИМ обратной связи, TIM16_CH1 (AF3)
    c = (gpio_cfg_t){ .pins = 1u << PIN_FB_OUT, .mode = 2, .speed = 1, .otype = 0, .pupd = 0 };
    gpio_init(GPIOA_B, &c);
    gpio_af(GPIOA_B, PIN_FB_OUT, 3);
}

/* ---------- SysTick + NVIC ---------- */
void board_nvic_init(void)
{
    SysTick->LOAD = 71999u;          // 1 кГц при 72 МГц
    SCB->SHP[1] &= 0x00FFFFFFu;      // приоритет SysTick = 0
    SysTick->VAL = 0;
    SysTick->CTRL = 7;

    NVIC_SetPriority(EXTI4_15_IRQn, PRIO_HALL);   NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_SetPriority(TIM1_NON_CC_IRQn, PRIO_HALL); NVIC_EnableIRQ(TIM1_NON_CC_IRQn);   // синус: обновление CCR каждый период ШИМ (DIER.UIE включается с режимом)
    NVIC_SetPriority(DMAC_CH1_2_IRQn, PRIO_DMA);  NVIC_EnableIRQ(DMAC_CH1_2_IRQn);
    NVIC_SetPriority(TIM15_IRQn, PRIO_TIM15);     NVIC_EnableIRQ(TIM15_IRQn);
    NVIC_SetPriority(TIM6_DAC0_2_IRQn, PRIO_TIM6);NVIC_EnableIRQ(TIM6_DAC0_2_IRQn);
    NVIC_SetPriority(TIM14_IRQn, PRIO_TIM14);     NVIC_EnableIRQ(TIM14_IRQn);
}

void board_reset(void)
{
    __disable_irq();
    __DSB();
    SCB->AIRCR = 0x05FA0004u;        // SYSRESETREQ
    __DSB();
    while (1) {
    }
}

void SysTick_Handler(void)
{
    g_ms++;
}
