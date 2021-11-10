#include "usb_control.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/usb/usbd.h>

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define UNUSED __attribute__ ((unused))

#define VENDOR_ID 0x26BA
#define PRODUCT_ID 0x8004

#include "usb_descriptors.h"

#include <stdlib.h>

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

volatile uint8_t waiting_tx = 0;

uint8_t usb_ready = 0;

static enum usbd_request_return_codes hid_control_request( UNUSED usbd_device *usbd_dev,
                                                           struct usb_setup_data *req,
                                                           uint8_t **buf,
                                                           uint16_t *len,
                                                           UNUSED usbd_control_complete_callback *complete )
{
    switch(req->bmRequestType){
        case 0x81:
        switch(req->bRequest){
            case USB_REQ_GET_DESCRIPTOR:
            if(req->wValue==0x2200){
                *buf = (uint8_t *)hid_report_descriptor;
                *len = sizeof(hid_report_descriptor);
                if(usb_ready==0) usb_ready=1;
                return 1;
            }else if(req->wValue==0x2100){
                *buf = (uint8_t *)USBD_HID_Desc;
                *len = sizeof(USBD_HID_Desc);
                return 1;
            }
            return 0;
            default:
            return 0;
        }
        break;
        default:
        return 0;
    }

}

static void hid_set_config(UNUSED usbd_device *usbd_dev, UNUSED uint16_t wValue)
{
    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);

    usbd_register_control_callback(
                usbd_dev,
                USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                hid_control_request);

    usb_ready_callback(usbd_dev);
}

usbd_device *g_usbd_dev;

void usb_send_packet(const void *buf, int len)
{
    waiting_tx = 1;
    usbd_ep_write_packet(g_usbd_dev, 0x81, buf, len);
    while (waiting_tx);
}

void usb_device_init(void)
{
    // Set up USB peripheral
    nvic_enable_irq(NVIC_USB_IRQ);

    g_usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev, &config, usb_strings, COUNT_OF(usb_strings), usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(g_usbd_dev, hid_set_config);
}

/**
 * USB interrupt handler
 */
void usb_isr(void)
{
    // Grab the USB endpoint register before it's modified by the USB driver
    // CTR_TX: "This bit is set by the hardware when an IN transaction is
    // successfully completed on this endpoint"
    const uint16_t ep_register = GET_REG(USB_EP_REG(1));

    // Handle USB requests as they come through
    usbd_poll(g_usbd_dev);

    if (ep_register & USB_EP_TX_CTR) {
        waiting_tx = 0;
    }
}
