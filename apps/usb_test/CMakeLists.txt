include_directories(.)
set(usb_test_SOURCES example.c usb_cdc.c usb_desc_cdc.c usb_device.c usbf_hw.c usb_uart.c)
add_application(usb_test "${usb_test_SOURCES}")
