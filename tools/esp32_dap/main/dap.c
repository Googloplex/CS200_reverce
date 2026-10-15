// CMSIS-DAP v2 command processor (SWD only)

#include <string.h>
#include <stdbool.h>
#include "dap.h"
#include "swd.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define ID_DAP_INFO               0x00
#define ID_DAP_HOST_STATUS        0x01
#define ID_DAP_CONNECT            0x02
#define ID_DAP_DISCONNECT         0x03
#define ID_DAP_TRANSFER_CONFIGURE 0x04
#define ID_DAP_TRANSFER           0x05
#define ID_DAP_TRANSFER_BLOCK     0x06
#define ID_DAP_TRANSFER_ABORT     0x07
#define ID_DAP_WRITE_ABORT        0x08
#define ID_DAP_DELAY              0x09
#define ID_DAP_RESET_TARGET       0x0A
#define ID_DAP_SWJ_PINS           0x10
#define ID_DAP_SWJ_CLOCK          0x11
#define ID_DAP_SWJ_SEQUENCE       0x12
#define ID_DAP_SWD_CONFIGURE      0x13
#define ID_DAP_SWD_SEQUENCE       0x1D
#define ID_DAP_EXECUTE_COMMANDS   0x7F

#define DAP_OK     0x00
#define DAP_ERROR  0xFF

#define DAP_PORT_DISABLED  0
#define DAP_PORT_SWD       1

#define XFER_APnDP        (1 << 0)
#define XFER_RnW          (1 << 1)
#define XFER_MATCH_VALUE  (1 << 4)
#define XFER_MATCH_MASK   (1 << 5)
#define XFER_MISMATCH     (1 << 4)  // in response

#define DP_RDBUFF_READ    0x0E      // DP, read, A[3:2] = 0b11
#define DP_ABORT_WRITE    0x00

static const char *s_serial = "";
static uint16_t s_wait_retry = 100;
static uint16_t s_match_retry = 0;
static uint32_t s_match_mask = 0xFFFFFFFF;

static inline uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void wr32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

void dap_init(const char *serial)
{
    s_serial = serial;
    swd_init();
}

static uint8_t transfer_retry(uint8_t request, uint32_t *data)
{
    uint8_t ack;
    uint32_t retry = s_wait_retry;
    do {
        ack = swd_transfer(request, data);
    } while (ack == SWD_ACK_WAIT && retry--);
    return ack;
}

static uint32_t info_string(uint8_t *resp, const char *str)
{
    uint32_t len = str ? strlen(str) + 1 : 0;
    resp[1] = len;
    if (len) {
        memcpy(&resp[2], str, len);
    }
    return 2 + len;
}

static uint32_t cmd_info(const uint8_t *req, uint8_t *resp)
{
    switch (req[1]) {
    case 0x01: return info_string(resp, "Espressif");
    case 0x02: return info_string(resp, "ESP32-S3 CMSIS-DAP");
    case 0x03: return info_string(resp, s_serial);
    case 0x04: return info_string(resp, "2.1.0");
    case 0x09: return info_string(resp, "1.0");
    case 0xF0:  // capabilities: SWD only
        resp[1] = 1;
        resp[2] = 0x01;
        return 3;
    case 0xFE:
        resp[1] = 1;
        resp[2] = DAP_PACKET_COUNT;
        return 3;
    case 0xFF:
        resp[1] = 2;
        wr16(&resp[2], DAP_PACKET_SIZE);
        return 4;
    default:
        return info_string(resp, NULL);
    }
}

// Size of request data that follows the request byte of a DAP_Transfer item
static inline uint32_t transfer_item_data(uint8_t r)
{
    return (!(r & XFER_RnW) || (r & XFER_MATCH_VALUE)) ? 4 : 0;
}

static uint32_t cmd_transfer(const uint8_t *req, uint32_t req_len, uint8_t *resp, uint32_t *used)
{
    const uint8_t *p = &req[3];
    const uint8_t *end = req + req_len;
    uint8_t *out = &resp[3];
    uint8_t *out_end = resp + DAP_PACKET_SIZE;
    uint32_t count = req[2];
    uint32_t done = 0;
    uint8_t ack = SWD_ACK_OK;
    bool post_read = false;
    bool check_write = false;
    uint32_t data;

    for (; done < count && p < end; done++) {
        uint8_t r = *p;
        const uint8_t *item = p + 1;
        p += 1 + transfer_item_data(r);

        if (r & XFER_RnW) {
            bool ap_plain_read = (r & XFER_APnDP) && !(r & XFER_MATCH_VALUE);
            if (post_read && !ap_plain_read) {
                // Collect the pending posted AP read before doing anything else
                ack = transfer_retry(DP_RDBUFF_READ, &data);
                if (ack != SWD_ACK_OK) {
                    break;
                }
                wr32(out, data);
                out += 4;
                post_read = false;
            }

            if (r & XFER_MATCH_VALUE) {
                uint32_t match = rd32(item);
                if (r & XFER_APnDP) {
                    ack = transfer_retry(r, NULL);
                    if (ack != SWD_ACK_OK) {
                        break;
                    }
                }
                uint32_t retry = s_match_retry;
                do {
                    ack = transfer_retry(r, &data);
                } while (ack == SWD_ACK_OK && (data & s_match_mask) != match && retry--);
                if (ack == SWD_ACK_OK && (data & s_match_mask) != match) {
                    ack |= XFER_MISMATCH;
                }
                if (ack != SWD_ACK_OK) {
                    break;
                }
            } else if (r & XFER_APnDP) {
                if (out + 4 > out_end) {
                    break;
                }
                if (!post_read) {
                    ack = transfer_retry(r, NULL);
                    if (ack != SWD_ACK_OK) {
                        break;
                    }
                    post_read = true;
                } else {
                    ack = transfer_retry(r, &data);
                    if (ack != SWD_ACK_OK) {
                        break;
                    }
                    wr32(out, data);
                    out += 4;
                }
            } else {
                if (out + 4 > out_end) {
                    break;
                }
                ack = transfer_retry(r, &data);
                if (ack != SWD_ACK_OK) {
                    break;
                }
                wr32(out, data);
                out += 4;
            }
            check_write = false;
        } else {
            if (post_read) {
                ack = transfer_retry(DP_RDBUFF_READ, &data);
                if (ack != SWD_ACK_OK) {
                    break;
                }
                wr32(out, data);
                out += 4;
                post_read = false;
            }
            data = rd32(item);
            if (r & XFER_MATCH_MASK) {
                s_match_mask = data;
                ack = SWD_ACK_OK;
            } else {
                ack = transfer_retry(r, &data);
                if (ack != SWD_ACK_OK) {
                    break;
                }
                check_write = true;
            }
        }
    }

    if (ack == SWD_ACK_OK) {
        if (post_read) {
            ack = transfer_retry(DP_RDBUFF_READ, &data);
            if (ack == SWD_ACK_OK) {
                wr32(out, data);
                out += 4;
            }
        } else if (check_write) {
            ack = transfer_retry(DP_RDBUFF_READ, NULL);
        }
    }

    // Skip items after the failed one so that the next command in a batch is found
    for (uint32_t i = done + 1; i < count && p < end; i++) {
        uint8_t r = *p++;
        p += transfer_item_data(r);
    }
    *used = (p > end ? end : p) - req;

    resp[1] = done;
    resp[2] = ack;
    return out - resp;
}

static uint32_t cmd_transfer_block(const uint8_t *req, uint32_t req_len, uint8_t *resp, uint32_t *used)
{
    uint32_t count = rd16(&req[2]);
    uint8_t r = req[4];
    const uint8_t *p = &req[5];
    uint8_t *out = &resp[4];
    uint32_t done = 0;
    uint8_t ack = SWD_ACK_OK;
    uint32_t data;

    if (r & XFER_RnW) {
        *used = 5;
        uint32_t max_words = (DAP_PACKET_SIZE - 4) / 4;
        if (count > max_words) {
            count = max_words;
        }
        if (count && (r & XFER_APnDP)) {
            ack = transfer_retry(r, NULL);  // first AP read is posted
        }
        while (done < count && ack == SWD_ACK_OK) {
            uint8_t this_req = ((r & XFER_APnDP) && done == count - 1) ? DP_RDBUFF_READ : r;
            ack = transfer_retry(this_req, &data);
            if (ack != SWD_ACK_OK) {
                break;
            }
            wr32(out, data);
            out += 4;
            done++;
        }
    } else {
        uint32_t avail = (req_len - 5) / 4;
        *used = 5 + count * 4;
        if (count > avail) {
            count = avail;
        }
        while (done < count) {
            data = rd32(p);
            p += 4;
            ack = transfer_retry(r, &data);
            if (ack != SWD_ACK_OK) {
                break;
            }
            done++;
        }
        if (ack == SWD_ACK_OK && done) {
            ack = transfer_retry(DP_RDBUFF_READ, NULL);
        }
    }

    wr16(&resp[1], done);
    resp[3] = ack;
    return out - resp;
}

static uint32_t cmd_swd_sequence(const uint8_t *req, uint32_t req_len, uint8_t *resp, uint32_t *used)
{
    uint32_t seqs = req[1];
    const uint8_t *p = &req[2];
    const uint8_t *end = req + req_len;
    uint8_t *out = &resp[2];
    resp[1] = DAP_OK;

    for (uint32_t i = 0; i < seqs && p < end; i++) {
        uint8_t info = *p++;
        uint32_t bits = info & 0x3F;
        if (bits == 0) {
            bits = 64;
        }
        uint32_t bytes = (bits + 7) / 8;
        if (info & 0x80) {
            if (out + bytes > resp + DAP_PACKET_SIZE) {
                resp[1] = DAP_ERROR;
                break;
            }
            swd_swdio_drive(false);
            swd_seq_in(out, bits);
            out += bytes;
        } else {
            swd_swdio_drive(true);
            swd_seq_out(p, bits);
            p += bytes;
        }
    }
    swd_swdio_drive(true);
    *used = p - req;
    return out - resp;
}

static uint32_t process_one(const uint8_t *req, uint32_t req_len, uint8_t *resp, uint32_t *used);

static uint32_t cmd_execute(const uint8_t *req, uint32_t req_len, uint8_t *resp, uint32_t *used)
{
    uint32_t n = req[1];
    uint32_t in = 2, out = 2;
    resp[1] = 0;
    for (uint32_t i = 0; i < n && in < req_len; i++) {
        uint32_t u = 0;
        out += process_one(&req[in], req_len - in, &resp[out], &u);
        in += u ? u : 1;
        resp[1]++;
    }
    *used = in;
    return out;
}

static uint32_t process_one(const uint8_t *req, uint32_t req_len, uint8_t *resp, uint32_t *used)
{
    resp[0] = req[0];
    *used = 1;

    switch (req[0]) {
    case ID_DAP_INFO:
        *used = 2;
        return cmd_info(req, resp);

    case ID_DAP_HOST_STATUS:
        *used = 3;
        if (req[1] == 0) {
            swd_led(req[2]);
        }
        resp[1] = DAP_OK;
        return 2;

    case ID_DAP_CONNECT:
        *used = 2;
        if (req[1] <= DAP_PORT_SWD) {
            swd_connect();
            resp[1] = DAP_PORT_SWD;
        } else {
            resp[1] = DAP_PORT_DISABLED;
        }
        return 2;

    case ID_DAP_DISCONNECT:
        swd_disconnect();
        resp[1] = DAP_OK;
        return 2;

    case ID_DAP_TRANSFER_CONFIGURE:
        *used = 6;
        swd_cfg.idle_cycles = req[1];
        s_wait_retry = rd16(&req[2]);
        s_match_retry = rd16(&req[4]);
        resp[1] = DAP_OK;
        return 2;

    case ID_DAP_TRANSFER:
        return cmd_transfer(req, req_len, resp, used);

    case ID_DAP_TRANSFER_BLOCK:
        return cmd_transfer_block(req, req_len, resp, used);

    case ID_DAP_TRANSFER_ABORT:
        // Commands run to completion before the next one is read, nothing to abort
        return 0;

    case ID_DAP_WRITE_ABORT: {
        *used = 6;
        uint32_t data = rd32(&req[2]);
        resp[1] = swd_transfer(DP_ABORT_WRITE, &data) == SWD_ACK_OK ? DAP_OK : DAP_ERROR;
        return 2;
    }

    case ID_DAP_DELAY:
        *used = 3;
        esp_rom_delay_us(rd16(&req[1]));
        resp[1] = DAP_OK;
        return 2;

    case ID_DAP_RESET_TARGET:
        resp[1] = DAP_OK;
        resp[2] = 0;  // no device-specific reset sequence
        return 3;

    case ID_DAP_SWJ_PINS: {
        *used = 7;
        uint8_t value = req[1], select = req[2];
        uint32_t wait_us = rd32(&req[3]);
        swd_pins_write(value, select);
        if (wait_us) {
            int64_t deadline = esp_timer_get_time() + wait_us;
            while (((swd_pins_read() ^ value) & select) && esp_timer_get_time() < deadline) {
            }
        }
        resp[1] = swd_pins_read();
        return 2;
    }

    case ID_DAP_SWJ_CLOCK:
        *used = 5;
        swd_set_clock(rd32(&req[1]));
        resp[1] = DAP_OK;
        return 2;

    case ID_DAP_SWJ_SEQUENCE: {
        uint32_t bits = req[1] ? req[1] : 256;
        *used = 2 + (bits + 7) / 8;
        swd_swdio_drive(true);
        swd_seq_out(&req[2], bits);
        resp[1] = DAP_OK;
        return 2;
    }

    case ID_DAP_SWD_CONFIGURE:
        *used = 2;
        swd_cfg.turnaround = (req[1] & 0x03) + 1;
        swd_cfg.data_phase = req[1] & 0x04;
        resp[1] = DAP_OK;
        return 2;

    case ID_DAP_SWD_SEQUENCE:
        return cmd_swd_sequence(req, req_len, resp, used);

    case ID_DAP_EXECUTE_COMMANDS:
        return cmd_execute(req, req_len, resp, used);

    default:
        resp[0] = 0xFF;
        return 1;
    }
}

uint32_t dap_process(const uint8_t *req, uint32_t req_len, uint8_t *resp)
{
    uint32_t used;
    if (req_len == 0) {
        return 0;
    }
    return process_one(req, req_len, resp, &used);
}
