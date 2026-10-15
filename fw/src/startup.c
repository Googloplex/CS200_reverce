// Старт приложения. Таблица векторов лежит по 0x08001000 — родной загрузчик Kpower читает отсюда
// SP/PC и форвардит все прерывания трамплинами (ldr r0,=0x08001040; ldr r0,[r0,#n]; bx r0).
#include <stdint.h>
#include <string.h>
#include "LCM32F037.h"

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);
extern void SystemInit(void);

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK_IRQ(name) void name(void) __attribute__((weak, alias("Default_Handler")))
WEAK_IRQ(NMI_Handler);
WEAK_IRQ(HardFault_Handler);
WEAK_IRQ(SVC_Handler);
WEAK_IRQ(PendSV_Handler);
WEAK_IRQ(SysTick_Handler);
WEAK_IRQ(WWDG_IRQHandler);        // 0
WEAK_IRQ(WT_IRQHandler);          // 2
WEAK_IRQ(EFLS_IRQHandler);        // 3
WEAK_IRQ(CHIPCTRL_IRQHandler);    // 4
WEAK_IRQ(EXTI0_1_IRQHandler);     // 5
WEAK_IRQ(EXTI2_3_IRQHandler);     // 6
WEAK_IRQ(EXTI4_15_IRQHandler);    // 7
WEAK_IRQ(DMAC_CH0_IRQHandler);    // 9
WEAK_IRQ(DMAC_CH1_2_IRQHandler);  // 10
WEAK_IRQ(DMAC_CH3_5_IRQHandler);  // 11
WEAK_IRQ(ADC_IRQHandler);         // 12
WEAK_IRQ(TIM1_NON_CC_IRQHandler); // 13
WEAK_IRQ(TIM1_CC_IRQHandler);     // 14
WEAK_IRQ(TIM3_IRQHandler);        // 16
WEAK_IRQ(TIM6_DAC_IRQHandler);    // 17
WEAK_IRQ(TIM14_IRQHandler);       // 19
WEAK_IRQ(TIM15_IRQHandler);       // 20
WEAK_IRQ(TIM16_IRQHandler);       // 21
WEAK_IRQ(TIM17_IRQHandler);       // 22
WEAK_IRQ(I2C0_IRQHandler);        // 23
WEAK_IRQ(SPI_IRQHandler);         // 25
WEAK_IRQ(UART0_IRQHandler);       // 27
WEAK_IRQ(UART1_IRQHandler);       // 28
WEAK_IRQ(IWDG_IRQHandler);        // 30
WEAK_IRQ(DIV_IRQHandler);         // 31

typedef void (*vector_t)(void);

__attribute__((section(".isr_vector"), used))
const vector_t g_vectors[16 + 32] = {
    (vector_t)&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    0, 0, 0, 0, 0, 0, 0,
    SVC_Handler,
    0, 0,
    PendSV_Handler,
    SysTick_Handler,
    /* внешние прерывания, номера как в LCM32F037.h */
    WWDG_IRQHandler,        // 0
    0,                      // 1
    WT_IRQHandler,          // 2
    EFLS_IRQHandler,        // 3
    CHIPCTRL_IRQHandler,    // 4
    EXTI0_1_IRQHandler,     // 5
    EXTI2_3_IRQHandler,     // 6
    EXTI4_15_IRQHandler,    // 7
    0,                      // 8
    DMAC_CH0_IRQHandler,    // 9
    DMAC_CH1_2_IRQHandler,  // 10
    DMAC_CH3_5_IRQHandler,  // 11
    ADC_IRQHandler,         // 12
    TIM1_NON_CC_IRQHandler, // 13
    TIM1_CC_IRQHandler,     // 14
    0,                      // 15
    TIM3_IRQHandler,        // 16
    TIM6_DAC_IRQHandler,    // 17
    0,                      // 18
    TIM14_IRQHandler,       // 19
    TIM15_IRQHandler,       // 20
    TIM16_IRQHandler,       // 21
    TIM17_IRQHandler,       // 22
    I2C0_IRQHandler,        // 23
    0,                      // 24
    SPI_IRQHandler,         // 25
    0,                      // 26
    UART0_IRQHandler,       // 27
    UART1_IRQHandler,       // 28
    0,                      // 29
    IWDG_IRQHandler,        // 30
    DIV_IRQHandler,         // 31
};

void Default_Handler(void)   // любое необработанное прерывание или отказ → сброс (так делает и родная на HardFault)
{
    __DSB();
    SCB->AIRCR = 0x05FA0004u;
    __DSB();
    while (1) {
    }
}

void Reset_Handler(void)
{
    // Приходим сюда из загрузчика: у него могли остаться включённые прерывания, SysTick, DMA.
    __disable_irq();
    __set_MSP((uint32_t)&_estack);
    SysTick->CTRL = 0;
    NVIC->ICER[0] = 0xFFFFFFFFu;
    NVIC->ICPR[0] = 0xFFFFFFFFu;

    memcpy(&_sdata, &_sidata, (size_t)((uint8_t *)&_edata - (uint8_t *)&_sdata));
    memset(&_sbss, 0, (size_t)((uint8_t *)&_ebss - (uint8_t *)&_sbss));

    SystemInit();
    main();
    while (1) {
    }
}
