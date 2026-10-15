// Шина: UART1 на PA2 (полудуплекс), Dynamixel Protocol 1.0, приём через DMA в кольцо 255 Б,
// ответ через DMA с задержкой TIM6 (Return Delay Time). Перенос FUN_080042fc / 0800338c / 08003e50 и обработчиков.
#pragma once
#include <stdint.h>

#define DXL_INST_PING        0x01
#define DXL_INST_READ        0x02
#define DXL_INST_WRITE       0x03
#define DXL_INST_REG_WRITE   0x04
#define DXL_INST_ACTION      0x05
#define DXL_INST_RESET       0x06
#define DXL_INST_BOOTLOADER  0x64   // своя: сохранить журнал с флагом 'Z' и перезагрузиться в загрузчик
#define DXL_INST_SYNC_WRITE  0x83

#define DXL_ERR_RANGE        0x08   // запись в EEPROM при Lock
#define DXL_ERR_CHECKSUM     0x10
#define DXL_ERR_OVERLOAD     0x20
#define DXL_ERR_INSTRUCTION  0x40

void dxl_uart_init(void);     // UART1 + DMA (FUN_080044cc), вызывать один раз
void dxl_uart_setup(void);    // скорость по REG_BAUD (FUN_0800333c), можно вызывать повторно
void dxl_tx_buf_init(void);   // FF FF в начале буфера ответа (FUN_08004164)
void dxl_poll(void);          // разбор принятого, обработка команд, запуск ответа (вызывать из главного цикла)
void dxl_reg_write(uint8_t addr, const uint8_t *data, unsigned n);   // запись в таблицу с побочными эффектами
void dxl_set_id(uint8_t id);

/* флаги для главного цикла и регулятора */
extern volatile uint8_t g_tx_busy;      // идёт передача ответа
extern volatile uint8_t g_save_flag;    // надо сохранить журнал
extern volatile uint8_t g_baud_flag;    // сменилась скорость — переинициализировать UART
extern volatile uint8_t g_boot_flag;    // 'Z' — уйти в загрузчик
extern volatile uint8_t g_lock;         // EEPROM-часть заблокирована
extern volatile uint8_t g_err;          // байт ошибок статус-пакета
extern volatile uint32_t g_last_rx_ms;  // время последнего пакета для нас (для таймаута шины)
extern volatile uint8_t  g_cmd_new;     // записан блок команды импеданса
extern volatile uint32_t g_time_offset; // часы хоста − локальные, мс
extern volatile uint8_t  g_time_synced;
