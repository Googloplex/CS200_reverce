// Клон родной прошивки Kpower HB200T (приложение с 0x08001000, загрузчик Kpower остаётся).
// Порядок инициализации и главный цикл — как в родной (re/decomp_named.c, FUN_080035e8).
#include "board.h"
#include "regs.h"
#include "dxl.h"
#include "sense.h"
#include "motor.h"

/* защита от цикла перезагрузок IWDG: счётчик в неинициализируемой RAM переживает сброс, но не выключение питания */
__attribute__((section(".noinit"))) static uint32_t boot_magic;
__attribute__((section(".noinit"))) static uint32_t boot_count;
#define BOOT_MAGIC 0xB007C0DEu

static void iwdg_start(void)   // ~1 с при RCL 32768 Гц / 32
{
    sc_dbg_stop(1u << 9);       // пёс стоит, пока ядро остановлено отладчиком (прошивка, halt)
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
    regs[REG_FW_VER] = 0x25;        // v1.5: + кубическая интерполяция по меткам времени, мягкие упоры, детект упора, политика отказа
    if ((regs[REG_CFG_FLAGS] & CFG_WDT) && boot_count < 4) {
        iwdg_start();
        wdt_on = 1;
    } else if (boot_count >= 4) {
        g_status |= ST_WDT_GUARD;   // 3 быстрых сброса подряд — сторожевой таймер не включаем
    }
    __enable_irq();                 // до этого момента PRIMASK установлен со старта

    while (1) {
        if (wdt_on) IWDG_KR = 0xAAAAu;
        if (boot_count && g_ms > 5000u) boot_count = 0;   // прожили 5 с — цикл перезагрузок не идёт
        motor_stall_check();
        dxl_poll();
        sense_update();
        sense_mcu_temp_poll();
        // v1: таймаут шины и перенапряжение → момент выкл
        {
            // v1.5: таймаут шины и Vbus проверяются в TIM14 (motor.c), один контекст — без гонок на g_status/imp_act
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
        // запись во Flash останавливает ядро на миллисекунды (векторы и ISR во Flash) — только без момента, иначе откладываем
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
    // команда 0x64: сохранить журнал с флагом 'Z' и уйти в загрузчик
    journal_save(g_boot_flag);
    while (g_tx_busy) {
    }
    board_reset();
    return 0;
}
