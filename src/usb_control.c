#include "usb_control.h"

#include "sk6812.h"

#include <libopencm3/cm3/nvic.h>

#include <stdlib.h>

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define USB_VENDOR_ID 0x26BA
#define USB_PRODUCT_ID 0x8002

static usbd_device *g_usbd_dev;

static const char * const usb_strings[] = {
    "Stecman",
    "Status Light",
    USB_SERIALNUM,
};

const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0xFF,
    .bDeviceSubClass = 0XFF,
    .bDeviceProtocol = 0XFF,
    .bMaxPacketSize0 = 64,
    .idVendor = USB_VENDOR_ID,
    .idProduct = USB_PRODUCT_ID,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

const struct usb_endpoint_descriptor endpoint_descriptors[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
        .wMaxPacketSize = 64,
        .bInterval = 0x20,
    },
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x81,
        .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
        .wMaxPacketSize = 64,
        .bInterval = 0x20,
    },
};

const struct usb_interface_descriptor hid_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = 0xFF,
    .bInterfaceSubClass = 0xFF,
    .bInterfaceProtocol = 0xFF,
    .iInterface = 0,
    .endpoint = endpoint_descriptors,
};

const struct usb_interface ifaces[] = {
    {
        .num_altsetting = 1,
        .altsetting = &hid_iface,
    }
};

const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0xE0, // Bus powered and Support Remote Wake-up
    .bMaxPower = 0x64, // Max power = 200mA (With all LEDs on the total draw is ~150mA)

    .interface = ifaces,
};

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

/**
 * Callback for the interrupt-driven OUT endpoint
 *
 * This gets called whenever a new OUT packet has arrived.
 */
static void usb_rx_callback(usbd_device * usbd_dev, uint8_t ep)
{
    (void)ep;

    // Read the packet to clear the FIFO and make room for a new packet
    char buf[64] __attribute__ ((aligned(4)));
    usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    sk6812_reset();

    // Dump the recieve buffer to the leds as a test
    for (uint8_t i = 0; i < 12; i += 3) {
        sk6812_write_rgb( *((uint32_t*)(buf + i)) );
    }
}

/**
 * Callback for the interrupt-driven IN endpoint
 *
 * This gets called whenever an IN packet has been successfully transmitted.
 */
static void usb_tx_callback(usbd_device * usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64] __attribute__ ((aligned(4)));

    /* Keep sending packets */
    usbd_ep_write_packet(usbd_dev, 0x81, buf, 64);
}

static void usb_set_config(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;

    // EP OUT
    usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_INTERRUPT, 64, usb_rx_callback);

    // EP IN
    usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 64, usb_tx_callback);
}

void usb_device_init(void)
{
    // Set up USB peripheral
    nvic_enable_irq(NVIC_USB_IRQ);

    g_usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev_descr, &config, usb_strings, COUNT_OF(usb_strings), usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(g_usbd_dev, usb_set_config);

    usbd_register_set_config_callback(g_usbd_dev, &usb_set_config_callback);
}

/**
 * USB interrupt handler
 */
void usb_isr(void)
{
    // Handle USB requests as they come through
    usbd_poll(g_usbd_dev);
}
