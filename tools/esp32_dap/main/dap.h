#pragma once

#include <stdint.h>

#define DAP_PACKET_SIZE   64
#define DAP_PACKET_COUNT  1

void dap_init(const char *serial);

// Processes one CMSIS-DAP request. Returns response length; 0 means no response is sent.
uint32_t dap_process(const uint8_t *req, uint32_t req_len, uint8_t *resp);
