#include <string.h>
#include "dxl.h"
#include "regs.h"
#include "board.h"
#include "motor.h"

#define RX_RING     255u          // receive ring (DMA block size)
#define DMA_CH_RX   0
#define DMA_CH_TX   1

static uint8_t rx_ring[RX_RING];  // RAM_200000A8
static uint8_t pkt[128];          // received packet: ID LEN INST params CHK (RAM_200001A7)
static uint8_t regw[128];         // deferred REG_WRITE (RAM_20000227)
static uint8_t txb[5 + 128 + 1];  // reply: FF FF ID LEN ERR params CHK (stock has 127 B — READ > 122 B corrupts the table)

static int16_t  rx_state;         // RAM_2000005C
static uint8_t  rx_proto;         // 0 — FF FF (Dynamixel), 1 — FD FD (bootloader packets, skipped)
static int16_t  rx_start;         // index of ID byte in the ring
static int16_t  rx_len;           // LEN field
static int16_t  rx_idx;           // read index
static int16_t  rx_cur;           // DMA write index
static uint8_t  my_id;            // RAM_20000049
static uint8_t  tx_n;             // checksum index in txb (= 5 + number of params)
static uint32_t tx_delay_us;      // v1: reply delay on top of Return Delay (SYNC_READ slot)
static uint8_t  force_reply;      // v1: reply despite broadcast ID (SYNC_READ)

volatile uint8_t g_tx_busy, g_save_flag, g_baud_flag, g_boot_flag, g_lock, g_err;
volatile uint32_t g_last_rx_ms;
volatile uint8_t g_cmd_new;          // impedance command block 0x80..0x8F written
volatile uint32_t g_time_offset;     // host_ms − g_ms
volatile uint8_t g_time_synced;

static unsigned read_any(unsigned addr, uint8_t *dst, unsigned n)   // read from regs/regs2, clipped to bounds
{
    unsigned k = 0;
    for (; k < n; k++) {
        unsigned a = addr + k;
        if (a < REGS_SIZE) dst[k] = regs[a];
        else if (a < REGS2_BASE + REGS2_SIZE) dst[k] = regs2[a - REGS2_BASE];
        else break;
    }
    return k;
}

/* ---------- UART / DMA ---------- */
void dxl_uart_init(void)
{
    UART_LCR_H(UART1_B) |= 0x10u;            // FIFO
    UART_IFLS(UART1_B) = 0;
    UART_DMACR(UART1_B) |= 2u;               // TX DMA
    UART_DMACR(UART1_B) |= 1u;               // RX DMA

    // channel 1: memory → UART (reply), interrupt on block
    dma_ch_deinit(DMA_CH_TX);
    DMA_CTL_L(DMA_CH(DMA_CH_TX)) |= 1u;      // INT_EN
    DMAC_MASKBLOCK = 0x101u << DMA_CH_TX;
    DMA_SAR(DMA_CH(DMA_CH_TX)) = (uint32_t)txb;
    DMA_DAR(DMA_CH(DMA_CH_TX)) = UART1_B;    // DR
    DMA_CTL_L(DMA_CH(DMA_CH_TX)) = 0x200181u;// TT_FC=2, DINC=no, SINC=increment, 8 bit, INT_EN
    DMA_CFG_L(DMA_CH(DMA_CH_TX)) = 2u << 5;  // priority 2
    DMA_CFG_H(DMA_CH(DMA_CH_TX)) = 4u | (1u << 11) | (1u << 7);   // DEST_PER = SRC_PER = 1
    sc_dma_req_map(1, 2);                    // request 1 ← UART1 TX

    // channel 0: UART → 255 B ring, auto-reload
    dma_ch_deinit(DMA_CH_RX);
    DMA_SAR(DMA_CH(DMA_CH_RX)) = UART1_B;
    DMA_DAR(DMA_CH(DMA_CH_RX)) = (uint32_t)rx_ring;
    DMA_CTL_L(DMA_CH(DMA_CH_RX)) = 0x200600u;// TT_FC=2, SINC=no, DINC=increment, 8 bit
    DMA_CTL_H(DMA_CH(DMA_CH_RX)) = RX_RING;
    DMA_CFG_L(DMA_CH(DMA_CH_RX)) = (1u << 30) | (1u << 31) | (3u << 5);   // reload src/dst, priority 3
    DMA_CFG_H(DMA_CH(DMA_CH_RX)) = 4u;       // DEST_PER = SRC_PER = 0
    sc_dma_req_map(0, 3);                    // request 0 ← UART1 RX
    dma_ch_enable(DMA_CH_RX, 1);
}

void dxl_uart_setup(void)
{
    uint32_t baud = 2000000u / ((uint32_t)regs[REG_BAUD] + 1u);
    uint32_t div = baud * 16u;
    uint32_t clk = SystemCoreClock;          // UART1 clocked from PCLK = 72 MHz
    uint32_t ibrd = clk / div;
    uint32_t fbrd = ((clk - ibrd * div) * 64u + (baud & 0x0FFFFFFFu) * 8u) / div;

    UART_CR(UART1_B) = 0;
    UART_IBRD(UART1_B) = ibrd;
    UART_FBRD(UART1_B) = fbrd;
    UART_LCR_H(UART1_B) |= 0x60u;            // 8 bit, 1 stop, no parity
    UART_CR(UART1_B) |= 0x300u;              // TXE | RXE
    UART_CR(UART1_B) |= 1u;                  // UARTEN
}

void dxl_tx_buf_init(void)
{
    txb[0] = 0xFF;
    txb[1] = 0xFF;
}

static void tx_start(void)   // FUN_08004174
{
    gpio_af(GPIOA_B, PIN_BUS, AF_BUS_TX);
    DMA_CTL_H(DMA_CH(DMA_CH_TX)) = (uint32_t)tx_n + 1u;
    DMA_SAR(DMA_CH(DMA_CH_TX)) = (uint32_t)txb;
    dma_ch_enable(DMA_CH_TX, 1);
}

void DMAC_CH1_2_IRQHandler(void)
{
    if (DMAC_STATUSBLOCK & (1u << DMA_CH_TX)) {
        DMAC_CLEARBLOCK = 1u << DMA_CH_TX;
        while (UART_FR(UART1_B) & UART_FR_BUSY) {
        }
        g_tx_busy = 0;
        gpio_af(GPIOA_B, PIN_BUS, AF_BUS_RX);
    }
}

void TIM6_DAC_IRQHandler(void)   // reply delay expired
{
    TIM_SR(TIM6_B) &= ~1u;
    tx_start();
}

/* ---------- register write with side effects (FUN_080049c8) ---------- */
void dxl_reg_write(uint8_t addr, const uint8_t *data, unsigned n)
{
    uint8_t cmd_touched = 0;
    __disable_irq();                                    // TIM14 must not see a half-written u16/block (≤128 bytes — a few µs)
    for (unsigned i = 0; i < n; i++) {
        unsigned a = (addr + i) & 0xFFu;
        if (a >= REGS2_BASE) {                          // RAM register block v1.3+
            if (a < 0x90) {                             // impedance command
                regs2[a - REGS2_BASE] = data[i];
                cmd_touched = 1;
            } else if ((a >= R2_SOFT_MIN && a < R2_STATUS2) || a == R2_HOLD_TMAX || a == R2_HOLD_TMAX + 1 || a == R2_BB_CTRL || a == R2_BB_PAGE ||
                       a == R2_CFG2_CMD || (a >= R2_CONTACT_DECEL && a <= R2_CONTACT_DECEL + 1) ||
                       (a >= R2_N_GEAR && a <= R2_KT + 1) || a == R2_SINE_ADV || a == R2_SINE_OFS) {
                regs2[a - REGS2_BASE] = data[i];        // v1.5 settings and flight recorder control
            }
            continue;
        }
        if (a == 10 || a < 3 || a >= 0x60 || (a >= 0x24 && a <= 0x2E) || a == 0x5A || a == 0x5B) {
            continue;                                   // read-only
        }
        regs[a] = data[i];
        if (a < 0x18) {                                 // EEPROM area
            if (g_lock == 0) {
                g_save_flag = 1;
                if (a == REG_BAUD) {
                    g_baud_flag = 1;
                }
            } else {
                g_err |= DXL_ERR_RANGE;
            }
            continue;
        }
        if (a == REG_GOAL || a == REG_GOAL + 1) {
            regs[REG_TORQUE_ENABLE] = 1;
            regs[REG_TORQUE_LIMIT] = regs[REG_MAX_TORQUE];
            regs[REG_TORQUE_LIMIT + 1] = regs[REG_MAX_TORQUE + 1];
            motor_new_goal();
            g_err &= (uint8_t)~DXL_ERR_OVERLOAD;
        }
        if (a == REG_TRAVEL || a == REG_TRAVEL + 1) {
            calib.travel = reg16(REG_TRAVEL);
            g_save_flag = 1;
        }
        if (a == REG_K_CW || a == REG_K_CCW) {
            g_save_flag = 1;
        }
        if (a == REG_SPEED || a == REG_SPEED + 1) {
            regs[REG_TORQUE_ENABLE] = 1;
            g_err &= (uint8_t)~DXL_ERR_OVERLOAD;
        }
        if (a == REG_LOCK && regs[REG_LOCK] == 1) {
            g_lock = 1;
        }
        if (a >= 0x50 && a < 0x60) {                   // custom settings — to journal
            g_save_flag = 1;
        }
    }
    if (cmd_touched) g_cmd_new = 1;                     // only after the whole block is written
    __enable_irq();
}

/* ---------- instruction handlers ---------- */
static void inst_ping(void)
{
    if (pkt[1] != 2) g_err |= DXL_ERR_INSTRUCTION;
}

static void inst_read(void)
{
    if (pkt[1] != 4) {
        g_err |= DXL_ERR_INSTRUCTION;
        return;
    }
    unsigned addr = pkt[3], n = pkt[4];
    if (n + 6 > sizeof txb) {
        n = sizeof txb - 6;
    }
    n = read_any(addr, &txb[5], n);
    tx_n = (uint8_t)(n + 5);
}

static void inst_write(void)
{
    if (pkt[1] > 3) dxl_reg_write(pkt[3], &pkt[4], pkt[1] - 3u);
    else g_err |= DXL_ERR_INSTRUCTION;
}

static void inst_reg_write(void)
{
    if (pkt[1] > 3) {
        memcpy(regw, pkt, pkt[1] + 2u);
        regs[REG_REGISTERED] = 1;
    } else {
        g_err |= DXL_ERR_INSTRUCTION;
    }
}

static void inst_action(void)
{
    if (pkt[1] != 2 || regs[REG_REGISTERED] != 1) {
        g_err |= DXL_ERR_INSTRUCTION;
        return;
    }
    dxl_reg_write(regw[3], &regw[4], regw[1] - 3u);
    regs[REG_REGISTERED] = 0;
}

static void inst_reset(void)
{
    if (pkt[1] != 2) {
        g_err |= DXL_ERR_INSTRUCTION;
        return;
    }
    regs_factory_defaults();
    regs_ram_reset();
    g_save_flag = 1;
}

static void inst_bootloader(void)
{
    if (pkt[1] != 2) g_err |= DXL_ERR_INSTRUCTION;
    else g_boot_flag = 'Z';
}

static void inst_sync_read(void)   // v1: FF FF FE LEN 0x84 addr len ID1..IDn CHK
{
    if (pkt[1] < 5) {
        g_err |= DXL_ERR_INSTRUCTION;
        return;
    }
    unsigned addr = pkt[3], n = pkt[4], ids = pkt[1] - 4u;
    for (unsigned k = 0; k < ids; k++) {
        if (pkt[5 + k] == my_id) {
            if (n + 6 > sizeof txb) n = sizeof txb - 6;
            n = read_any(addr, &txb[5], n);
            tx_n = (uint8_t)(n + 5);
            tx_delay_us = (uint32_t)k * (uint32_t)regs[REG_BUS_SLOT] * 10u;
            force_reply = 1;
            return;
        }
    }
}

static void inst_sync_write(void)
{
    if (pkt[1] < 6) {
        g_err |= DXL_ERR_INSTRUCTION;
        return;
    }
    unsigned addr = pkt[3], L = pkt[4];
    // Differs from stock: there the loop bound is computed wrong (pkt[3] − pkt[6] + 1); here — from LEN.
    for (unsigned i = 5; i + L <= pkt[1]; i += L + 1) {
        if (pkt[i] == my_id) {
            dxl_reg_write((uint8_t)addr, &pkt[i + 1], L);
            return;
        }
    }
}

static void inst_time_sync(void)   // v1.3: FF FF FE 06 0x86 t0 t1 t2 t3 CHK — host clock, ms
{
    if (pkt[1] != 6) {
        g_err |= DXL_ERR_INSTRUCTION;
        return;
    }
    uint32_t host = (uint32_t)pkt[3] | ((uint32_t)pkt[4] << 8) | ((uint32_t)pkt[5] << 16) | ((uint32_t)pkt[6] << 24);
    g_time_offset = host - g_ms;
    g_time_synced = 1;
    set_r2_32(R2_TIME_OFFSET, g_time_offset);
}

static void inst_set_id_by_uid(void)   // v1.5: FF FF FE 0F 0x87 uid[12] new_id CHK — ID by chip serial, without physical access to the servo
{
    if (pkt[1] != 15) {
        g_err |= DXL_ERR_INSTRUCTION;
        return;
    }
    for (unsigned i = 0; i < 12; i++) {
        if (pkt[3 + i] != regs2[R2_UID - REGS2_BASE + i]) return;   // not ours — stay silent
    }
    if (pkt[15] < 0xFE) {
        regs[REG_ID] = pkt[15];
        g_save_flag = 1;
    }
    force_reply = 1;
}

static void inst_uid_scan(void)   // v1.5: FF FF FE 02 0x88 CHK — each servo replies UID[12] in its own slot (by UID bytes)
{
    const uint8_t *u = &regs2[R2_UID - REGS2_BASE];
    for (unsigned i = 0; i < 12; i++) txb[5 + i] = u[i];
    tx_n = 5 + 12;
    tx_delay_us = (uint32_t)((u[8] ^ u[9] ^ u[10] ^ u[11]) & 0x3Fu) * (uint32_t)regs[REG_BUS_SLOT] * 10u;
    force_reply = 1;
}

static void dispatch(void)   // FUN_0800338c
{
    tx_n = 5;
    g_err &= 0xA7u;
    if (dxl_checksum(pkt, pkt[1] + 1u) != pkt[pkt[1] + 1]) {
        g_err |= DXL_ERR_CHECKSUM;
        set_r2_16(R2_PKT_ERR, (uint16_t)(r2_16(R2_PKT_ERR) + 1u));
        return;
    }
    set_r2_16(R2_PKT_OK, (uint16_t)(r2_16(R2_PKT_OK) + 1u));
    switch (pkt[2]) {
    case DXL_INST_PING:       inst_ping(); break;
    case DXL_INST_READ:       inst_read(); break;
    case DXL_INST_WRITE:      inst_write(); break;
    case DXL_INST_REG_WRITE:  inst_reg_write(); break;
    case DXL_INST_ACTION:     inst_action(); break;
    case DXL_INST_RESET:      inst_reset(); break;
    case DXL_INST_BOOTLOADER: inst_bootloader(); break;
    case DXL_INST_SYNC_WRITE: inst_sync_write(); break;
    case 0x84:                inst_sync_read(); break;
    case 0x86:                inst_time_sync(); break;
    case 0x87:                inst_set_id_by_uid(); break;
    case 0x88:                inst_uid_scan(); break;
    default:                  g_err |= DXL_ERR_INSTRUCTION; break;
    }
}

static void respond(void)   // FUN_08003e50
{
    int ok = force_reply;
    if (pkt[0] == my_id) {
        uint8_t lvl = regs[REG_STATUS_RETURN];
        if (lvl == 2 || pkt[2] == DXL_INST_PING) ok = 1;
        else if (lvl == 1 && pkt[2] == DXL_INST_READ) ok = 1;
    }
    if (ok) {
        txb[2] = my_id;
        txb[3] = (uint8_t)(tx_n - 3);
        txb[4] = g_err;
        txb[tx_n] = dxl_checksum(&txb[2], (unsigned)tx_n - 2u);
        uint32_t ticks = (uint32_t)regs[REG_RETURN_DELAY] + tx_delay_us / 2u;   // TIM6 tick = 2 µs
        if (ticks == 0) {
            tx_start();
        } else {
            TIM_ARR(TIM6_B) = ticks > 0xFFFFu ? 0xFFFFu : ticks;
            TIM_CR1(TIM6_B) |= 1u;
        }
        g_tx_busy = 1;
    }
    tx_delay_us = 0;
    force_reply = 0;
    my_id = regs[REG_ID];       // ID change takes effect after the reply
}

/* ---------- receive state machine (FUN_080042fc) ---------- */
static int16_t rx_avail(void)
{
    int16_t cur = (int16_t)(DMA_CTL_H(DMA_CH(DMA_CH_RX)) & 0xFFFu);
    rx_cur = cur;
    int d = cur - rx_idx;
    if (cur < rx_idx) d += 0xFF;
    return (int16_t)(d & 0xFFFF);
}

void dxl_poll(void)
{
    if (rx_avail() == 0) {
        return;
    }
    uint8_t hdr = rx_proto ? 0xFD : 0xFF;
    switch (rx_state) {
    case 0: {
        uint8_t b = rx_ring[rx_idx];
        if (b == 0xFF)      { rx_proto = 0; rx_state = 1; }
        else if (b == 0xFD) { rx_proto = 1; rx_state = 1; }
        rx_idx++;
        break;
    }
    case 1:
        rx_state = (rx_ring[rx_idx] == hdr) ? 2 : 0;
        rx_idx++;
        break;
    case 2:
        if (rx_ring[rx_idx] != hdr) {       // skip extra header bytes
            rx_start = rx_idx;
            rx_state = 3;
        }
        rx_idx++;
        break;
    case 3:
        if (!rx_proto) rx_len = rx_ring[rx_idx];
        rx_state = 4;
        rx_idx++;
        break;
    case 4:
        if (rx_proto) {                     // bootloader uses 16-bit length
            uint8_t lo = rx_idx ? rx_ring[rx_idx - 1] : rx_ring[RX_RING - 1];
            rx_len = (int16_t)((rx_ring[rx_idx] << 8) | lo);
        }
        rx_state = 5;
        break;
    default:
        if (rx_proto) {                     // bootloader packet: skip only
            if (rx_avail() < rx_len + 1) return;
            int16_t e = (int16_t)(rx_start + rx_len + 2);
            rx_idx = (e <= 254) ? (int16_t)(e + 1) : (int16_t)(e - 254);
            rx_state = 0;
            break;
        }
        if (rx_avail() < rx_len) return;
        {
            int16_t e = (int16_t)(rx_start + rx_len + 1);   // CHK index
            unsigned total = (unsigned)rx_len + 2u;          // ID LEN INST ... CHK
            if (total > sizeof pkt) {                        // differs from stock: packet longer than buffer — skip
                rx_idx = (e <= 254) ? (int16_t)(e + 1) : (int16_t)(e - 254);
                rx_state = 0;
                break;
            }
            if (e <= 254) {
                memcpy(pkt, &rx_ring[rx_start], total);
                rx_idx = (int16_t)(e + 1);
            } else {
                unsigned n1 = RX_RING - (unsigned)rx_start;
                if (n1 > total) n1 = total;
                memcpy(pkt, &rx_ring[rx_start], n1);
                memcpy(pkt + n1, rx_ring, total - n1);
                rx_idx = (int16_t)(e - 254);
            }
        }
        if (pkt[0] == my_id || pkt[0] == 0xFE) {
            g_last_rx_ms = g_ms;
            __disable_irq(); g_status &= (uint8_t)~ST_BUS_TMO; __enable_irq();
            dispatch();
            respond();
        }
        rx_state = 0;
        break;
    }
    if (rx_idx >= (int16_t)RX_RING) {
        rx_idx = (int16_t)(rx_idx - RX_RING);
    }
}

void dxl_set_id(uint8_t id) { my_id = id; }
