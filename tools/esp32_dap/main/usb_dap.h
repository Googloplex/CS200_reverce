#pragma once

#include "esp_err.h"

// Starts TinyUSB with a CMSIS-DAP v2 (WinUSB bulk) interface and the DAP processing task
esp_err_t usb_dap_start(void);
