idf_component_register(SRCS "main.c" "usb_dap.c" "dap.c" "swd.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio esp_hw_support esp_timer)
