#include "esp_err.h"
#include "usb_dap.h"

void app_main(void)
{
    ESP_ERROR_CHECK(usb_dap_start());
}
