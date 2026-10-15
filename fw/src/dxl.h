// Bus: UART1 on PA2 (half-duplex), Dynamixel Protocol 1.0, DMA receive into a 255 B ring,
// DMA reply delayed by TIM6 (Return Delay Time). Port of FUN_080042fc / 0800338c / 08003e50 and handlers.
#pragma once
#include <stdint.h>

#define DXL_INST_PING        0x01
#define DXL_INST_READ        0x02
#define DXL_INST_WRITE       0x03
#define DXL_INST_REG_WRITE   0x04
#define DXL_INST_ACTION      0x05
#define DXL_INST_RESET       0x06
#define DXL_INST_BOOTLOADER  0x64   // custom: save journal with flag 'Z' and reboot into bootloader
#define DXL_INST_SYNC_WRITE  0x83

#define DXL_ERR_RANGE        0x08   // EEPROM write while Lock
#define DXL_ERR_CHECKSUM     0x10
#define DXL_ERR_OVERLOAD     0x20
#define DXL_ERR_INSTRUCTION  0x40

void dxl_uart_init(void);     // UART1 + DMA (FUN_080044cc), call once
void dxl_uart_setup(void);    // baud from REG_BAUD (FUN_0800333c), may be called again
void dxl_tx_buf_init(void);   // FF FF at start of reply buffer (FUN_08004164)
void dxl_poll(void);          // parse received data, handle commands, start reply (call from main loop)
void dxl_reg_write(uint8_t addr, const uint8_t *data, unsigned n);   // table write with side effects
void dxl_set_id(uint8_t id);

/* flags for main loop and controller */
extern volatile uint8_t g_tx_busy;      // reply transmission in progress
extern volatile uint8_t g_save_flag;    // journal save pending
extern volatile uint8_t g_baud_flag;    // baud changed — reinit UART
extern volatile uint8_t g_boot_flag;    // 'Z' — jump to bootloader
extern volatile uint8_t g_lock;         // EEPROM area locked
extern volatile uint8_t g_err;          // status packet error byte
extern volatile uint32_t g_last_rx_ms;  // time of last packet addressed to us (for bus timeout)
extern volatile uint8_t  g_cmd_new;     // impedance command block written
extern volatile uint32_t g_time_offset; // host clock − local, ms
extern volatile uint8_t  g_time_synced;
