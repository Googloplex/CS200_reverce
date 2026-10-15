// Clone of Kpower HB200T stock firmware (application at 0x08001000, Kpower bootloader stays).
// Init order and main loop as in stock (re/decomp_named.c, FUN_080035e8).
#include "board.h"
#include "regs.h"
#include "dxl.h"
#include "sense.h"
#include "motor.h"

/* IWDG reboot-loop guard: counter in no-init RAM survives reset but not power-off */
__attribute__((section(".noinit"))) static uint32_t boot_magic;
__attribute__((section(".noinit"))) static uint32_t boot_count;
#define BOOT_MAGIC 0xB007C0DEu

static void iwdg_start(void)   // ~1 s at RCL 32768 Hz / 32
{
    sc_dbg_stop(1u << 9);       // watchdog stops while core is halted by debugger (flashing, halt)
    IWDG_KR = 0xCCCCu;
    IWDG_KR = 0x5555u;
    IWDG_PR = 3u;
    IWDG_RLR = 1024u;
    IWDG_KR = 0xAAAAu;
}

int main(void)
{
    if (boot_magic != BOOT_MAGIC) {
        boot_magic = BOOT_MAGIC;
        boot_count = 0;
    }
    boot_count++;
    uint8_t wdt_on = 0;
    board_clocks_init();
    journal_load();
    dxl_set_id(regs[REG_ID]);
    calib_from_regs();
    sense_init_calib();
    board_gpio_init();
    sense_adc_init();
    motor_tim1_init();
    motor_tim6_init();
    motor_tim14_init();
    motor_tim15_init();
    sense_tim16_init();
    dxl_uart_init();
    dxl_uart_setup();
    dxl_tx_buf_init();
    board_nvic_init();
    regs[REG_FW_VER] = 0x25;        // v1.5: + cubic interpolation by timestamps, soft limits, stall detection, failsafe policy
    if ((regs[REG_CFG_FLAGS] & CFG_WDT) && boot_count < 4) {
        iwdg_start();
        wdt_on = 1;
    } else if (boot_count >= 4) {
        g_status |= ST_WDT_GUARD;   // 3 fast resets in a row — do not enable watchdog
    }
    __enable_irq();                 // PRIMASK set since start until here

    while (1) {
        if (wdt_on) IWDG_KR = 0xAAAAu;
        if (boot_count && g_ms > 5000u) boot_count = 0;   // alive 5 s — no reboot loop
        motor_stall_check();
        dxl_poll();
        sense_update();
        sense_mcu_temp_poll();
        // v1: bus timeout and overvoltage → torque off
        {
            // v1.5: bus timeout and Vbus checked in TIM14 (motor.c), single context — no races on g_status/imp_act
        }
        if (g_baud_flag) {
            g_baud_flag = 0;
            while (g_tx_busy) {
            }
            dxl_uart_setup();
        }
        if (g_boot_flag == 'Z') {
            break;
        }
        // Flash write stalls the core for milliseconds (vectors and ISRs in Flash) — only with torque off, otherwise deferred
        if (regs[REG_TORQUE_ENABLE] == 0) {
            if (g_save_flag == 1) {
                g_save_flag = 0;
                journal_save(0);
            }
            if (regs2[R2_CFG2_CMD - REGS2_BASE] == 1) {
                regs2[R2_CFG2_CMD - REGS2_BASE] = 0;
                cfg2_save();
            }
        }
    }
    // command 0x64: save journal with flag 'Z' and jump to bootloader
    journal_save(g_boot_flag);
    while (g_tx_busy) {
    }
    board_reset();
    return 0;
}
