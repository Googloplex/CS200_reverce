// USB side of the probe: CMSIS-DAP v2 vendor interface with MS OS 2.0 descriptors,
// so Windows binds WinUSB automatically and no driver installation is needed.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "usb_dap.h"
#include "dap.h"

static const char *TAG = "usb_dap";

#define USB_VID  0x303A  // Espressif VID; replace with your own for a product
#define USB_PID  0x4D41

#define VENDOR_REQUEST_MICROSOFT  0x01
#define EP_OUT   0x01
#define EP_IN    0x81

enum { STR_LANGID, STR_MANUFACTURER, STR_PRODUCT, STR_SERIAL, STR_INTERFACE, STR_COUNT };

static char s_serial[13];

static const char *s_strings[STR_COUNT] = {
    [STR_LANGID] = (const char[]) { 0x09, 0x04 },
    [STR_MANUFACTURER] = "Espressif",
    // Host tools find the probe by the "CMSIS-DAP" substring
    [STR_PRODUCT] = "ESP32-S3 CMSIS-DAP",
    [STR_SERIAL] = s_serial,
    [STR_INTERFACE] = "CMSIS-DAP v2",
};

static const tusb_desc_device_t s_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210,  // 2.1 so that the host asks for the BOS descriptor
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

static const uint8_t s_config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_VENDOR_DESCRIPTOR(0, STR_INTERFACE, EP_OUT, EP_IN, DAP_PACKET_SIZE),
};

// MS OS 2.0 descriptor set: WinUSB compatible ID + CMSIS-DAP v2 interface GUID
#define MS_OS_20_DESC_LEN  162

static const uint8_t s_ms_os_20_desc[MS_OS_20_DESC_LEN] = {
    // Set header: length, type, Windows version (8.1+), total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Compatible ID: WINUSB
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // Registry property: DeviceInterfaceGUIDs (REG_MULTI_SZ)
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
    'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0, 'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    U16_TO_U8S_LE(0x0050),
    '{', 0, 'C', 0, 'D', 0, 'B', 0, '3', 0, 'B', 0, '5', 0, 'A', 0, 'D', 0, '-', 0,
    '2', 0, '9', 0, '3', 0, 'B', 0, '-', 0, '4', 0, '6', 0, '6', 0, '3', 0, '-', 0,
    'A', 0, 'A', 0, '3', 0, '6', 0, '-', 0, '1', 0, 'A', 0, 'A', 0, 'E', 0, '4', 0,
    '6', 0, '4', 0, '6', 0, '3', 0, '7', 0, '7', 0, '6', 0, '}', 0, 0, 0, 0, 0,
};

#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t s_bos_desc[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

uint8_t const *tud_descriptor_bos_cb(void)
{
    return s_bos_desc;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
            request->bRequest == VENDOR_REQUEST_MICROSOFT && request->wIndex == 7) {
        return tud_control_xfer(rhport, request, (void *)s_ms_os_20_desc, MS_OS_20_DESC_LEN);
    }
    return false;
}

static SemaphoreHandle_t s_rx_sem;

#if CFG_TUD_API_V0_19_COMPAT
void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint16_t bufsize)
#else
void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint32_t bufsize)
#endif
{
    (void)idx;
    (void)buffer;
    (void)bufsize;
    xSemaphoreGive(s_rx_sem);
}

static void dap_task(void *arg)
{
    static uint8_t req[DAP_PACKET_SIZE];
    static uint8_t resp[DAP_PACKET_SIZE * 2];  // headroom for DAP_ExecuteCommands

    while (true) {
        xSemaphoreTake(s_rx_sem, pdMS_TO_TICKS(100));
        while (tud_vendor_available()) {
            // RX FIFO holds exactly one packet, so one read is one command
            memset(req, 0, sizeof(req));
            uint32_t n = tud_vendor_read(req, sizeof(req));
            uint32_t len = dap_process(req, n, resp);
            if (len > DAP_PACKET_SIZE) {
                len = DAP_PACKET_SIZE;
            }
            if (len == 0) {
                continue;
            }
            while (tud_vendor_write_available() < len && tud_vendor_mounted()) {
                vTaskDelay(1);
            }
            tud_vendor_write(resp, len);
            tud_vendor_write_flush();
        }
    }
}

esp_err_t usb_dap_start(void)
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    dap_init(s_serial);

    s_rx_sem = xSemaphoreCreateBinary();
    if (!s_rx_sem) {
        return ESP_ERR_NO_MEM;
    }

    tinyusb_config_t cfg = TINYUSB_DEFAULT_CONFIG();
    cfg.descriptor.device = &s_device_desc;
    cfg.descriptor.full_speed_config = s_config_desc;
    cfg.descriptor.string = s_strings;
    cfg.descriptor.string_count = STR_COUNT;
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&cfg), TAG, "TinyUSB install failed");

    if (xTaskCreate(dap_task, "dap", 4096, NULL, 6, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "CMSIS-DAP v2 probe ready, serial %s", s_serial);
    return ESP_OK;
}
