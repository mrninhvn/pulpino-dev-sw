#include <stdint.h>
#include <unistd.h>
#include "usb_timer.h"
#include "usb_uart.h"
#include "usb_cdc.h"
#include "usb_device.h"
#include "usbf_hw.h"

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
#define USB_DEV_BASE   0x1A140000

//-----------------------------------------------------------------
// main
//-----------------------------------------------------------------
int main(int argc, char *argv[])
{
    printf("USB Device Test\n");
    
    // USB init
    usbf_init(USB_DEV_BASE, 0, usb_cdc_process_request);
    usb_uart_init();

    // Force detach
    usbhw_attach(0);
    printf("USB dettached\n");
    //    timer_sleep(100);
    timer_sleep(1);
    usbhw_attach(1);
    printf("USB attached\n");

    while (1)
    {
        usbhw_service();

        // Loopback
        if (usb_uart_haschar())
        {
            int ch = usb_uart_getchar();
            printf("Received %c\n", ch);
            usb_uart_putchar(ch);
        }
    }

    return 0;
}
