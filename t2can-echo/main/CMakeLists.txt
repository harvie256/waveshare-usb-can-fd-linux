idf_component_register(SRCS "echo.c" "mcp2518fd.c"
                       REQUIRES driver esp_timer esp_driver_usb_serial_jtag esp_driver_spi vfs)
