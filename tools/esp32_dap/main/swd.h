#pragma once

#include <stdint.h>
#include <stdbool.h>

// SWD ACK values as reported in CMSIS-DAP transfer responses
#define SWD_ACK_OK        0x01
#define SWD_ACK_WAIT      0x02
#define SWD_ACK_FAULT     0x04
#define SWD_ACK_NO_ACK    0x07
#define SWD_PARITY_ERROR  0x08

// Bits of the DAP_SWJ_Pins byte
#define SWJ_PIN_SWCLK   (1 << 0)
#define SWJ_PIN_SWDIO   (1 << 1)
#define SWJ_PIN_NRESET  (1 << 7)

typedef struct {
    uint8_t turnaround;   // 1..4 clock cycles
    bool data_phase;      // generate data phase on WAIT/FAULT
    uint8_t idle_cycles;  // idle cycles after each transfer
} swd_config_t;

extern swd_config_t swd_cfg;

void swd_init(void);
void swd_connect(void);
void swd_disconnect(void);
void swd_set_clock(uint32_t hz);

void swd_swdio_drive(bool drive);
void swd_seq_out(const uint8_t *data, uint32_t bits);
void swd_seq_in(uint8_t *data, uint32_t bits);

// One SWD transaction without WAIT retries. request bits: 0 APnDP, 1 RnW, 2 A2, 3 A3
uint8_t swd_transfer(uint8_t request, uint32_t *data);

void swd_pins_write(uint8_t value, uint8_t select);
uint8_t swd_pins_read(void);

void swd_led(bool on);
