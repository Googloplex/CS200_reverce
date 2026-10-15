// Тактирование — перенос инициализации родной прошивки (re/decomp_named.c: FUN_08002a4c → FUN_080027cc,
// FUN_08002a20, FUN_080023e4). Кварц 16 МГц → PLL (×18/2/2) → 72 МГц, HCLK = PCLK = 72 МГц [ПРОШ].
#include "lcm32.h"

uint32_t SystemCoreClock = 72000000u;

static void cc_write(volatile uint32_t *reg, uint32_t val)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    CC_KEY = CC_UNLOCK_KEY;   // ключ действует на одну следующую запись
    *reg = val;
    __set_PRIMASK(pm);
}

static void sc_write(volatile uint32_t *reg, uint32_t val)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    SC_KEY = SC_UNLOCK_KEY;
    *reg = val;
    __set_PRIMASK(pm);
}

// Задержка родной прошивки: n итераций по ~719 тактов SysTick (на RC 16 МГц ≈ 45 мкс каждая).
static void systick_delay(uint32_t n)
{
    uint32_t load = SysTick->LOAD, val = SysTick->VAL, ctrl = SysTick->CTRL;
    SysTick->CTRL = 0;
    SysTick->LOAD = 0xFFFFFFu;
    SysTick->VAL = 0;
    SysTick->CTRL = 7;
    while (n--) {
        SysTick->VAL = 0;
        while (SysTick->VAL > 0xFFFD30u) {
        }
    }
    SysTick->LOAD = load;
    SysTick->VAL = val;
    SysTick->CTRL = ctrl;
}

void SystemInit(void)
{
    // --- сброс тактирования на внутренний RC (RCC_DeInit родной прошивки) ---
    cc_write(&CC_CLK_CFG, 0x80000u);              // RCH_EN, источник — RCH
    cc_write(&CC_CTRL, 0);
    cc_write(&CC_OSCH_CFG, 7);
    cc_write(&CC_BDCR, CC_BDCR | 0x800u);         // BDRST
    cc_write(&CC_BDCR, 0x7004u);
    sc_write(&SC_CLKEN0, SC_CLK_RAM0 | SC_CLK_RAM1);   // все клоки периферии выкл., кроме RAM

    // --- супервизор питания: LVRS = 3, LVREN ---
    cc_write(&CC_PWR_CFG, (CC_PWR_CFG & ~0xEu) | (3u << 1));
    cc_write(&CC_PWR_CFG, CC_PWR_CFG | 1u);
    systick_delay(1000);

    // --- PF0/PF1 (кварц): аналоговый режим, AF2 — как в родной ---
    sc_write(&SC_CLKEN0, SC_CLKEN0 | SC_CLK_GPIOF);
    GPIO_MODER(GPIOF_B) = (GPIO_MODER(GPIOF_B) & ~0xFu) | 0xFu;   // PF0, PF1 → 3 (аналог)
    GPIO_SR_BSRR(GPIOF_B) = (1u << 16) | (1u << 17);
    GPIO_OD_BSRR(GPIOF_B) = (1u << 16) | (1u << 17);
    GPIO_OS_BSRR(GPIOF_B) = (1u << 16) | (1u << 17);
    GPIO_PU_BSRR(GPIOF_B) = (1u << 16) | (1u << 17);
    GPIO_PD_BSRR(GPIOF_B) = (1u << 16) | (1u << 17);
    GPIO_AFRL(GPIOF_B) = (GPIO_AFRL(GPIOF_B) & ~0xFFu) | 0x22u;

    FLASH_ACR = 0x12u;                            // 2 такта ожидания Flash под 72 МГц

    // --- внешний кварц 16 МГц ---
    cc_write(&CC_CLK_CFG, 0xA0000u);              // RCH_EN | OSCH_EN
    while (!(CC_STS & (1u << 19))) {              // OSCH_STB
    }
    // --- PLL: DN=17 (×18), OD=1 (÷2), LS=1, SRC=OSCH → 16·18/2/2 = 72 МГц ---
    cc_write(&CC_PLL_CFG, 0x1047u);
    cc_write(&CC_CLK_CFG, CC_CLK_CFG | 0xB0000u); // + PLL_EN
    while (!(CC_STS & (1u << 20))) {              // PLL_LOCK
    }
    cc_write(&CC_CLK_CFG, CC_CLK_CFG | 0x2B0000u);// clk_SRC_SEL = 2 (PLL), делители APB = 1
    while (!(CC_STS & (1u << 22))) {              // SYS_CLK_LOCK
    }
}

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 72000000u;
}
