#include "sk6812.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/usb/usbd.h>

#include <stdint.h>
#include <stdlib.h>

#define USB_VENDOR_ID 0x26BA
#define USB_PRODUCT_ID 0x8002

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

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
uint8_t usbd_control_buffer[128];

static void blink(void)
{
    sk6812_reset();

    // Write an initial state so it can be seen to be working
    static uint8_t current = 0;

    for (uint8_t i = 0; i < 4; ++i) {
        if (i == current) {
            sk6812_write_rgb(0x110000);
        } else {
            sk6812_write_rgb(0x001100);
        }
    }

    current++;
    if (current >= 4) {
        current = 0;
    }
}

/*
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

/*
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

/**
 * Configure the 12MHz HSE to give a 48MHz PLL clock for the USB peripheral to function
 *
 * This is based on a clock configuration calculated in STM32 Cube. Note that
 * the STM32F070 requires an external oscillator for USB as its HSI is not
 * precise enough per the datasheet:
 *
 * > [USB] requires a precise 48 MHz clock which can be generated from the
 * > internal main PLL (the clock source must use an HSE crystal oscillator).
 */
static void clock_setup_12mhz_hse_out_48mhz(void)
{
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);
    rcc_set_sysclk_source(RCC_HSE);

    rcc_set_hpre(RCC_CFGR_HPRE_NODIV);  // AHB Prescaler
    rcc_set_ppre(RCC_CFGR_PPRE_NODIV); // APB1 Prescaler

    flash_prefetch_enable();
    flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

    // PLL for USB: (12MHz * 8) / 2 = 48MHz
    rcc_set_prediv(RCC_CFGR2_PREDIV_DIV2);
    rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL8);
    rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
    rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2);

    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    rcc_set_sysclk_source(RCC_PLL);

    rcc_set_usbclk_source(RCC_PLL);

    rcc_apb1_frequency = 48000000;
    rcc_ahb_frequency = 48000000;
}

int main(void)
{
    // Turn on the SYSCFG module and switch out PA9/PA10 for PA11/PA12
    // The USB peripheral is not phsyically connected without this and won't be enumerated
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

    clock_setup_12mhz_hse_out_48mhz();

    sk6812_init();

    // Configure SysTick at 1ms intervals
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload((rcc_ahb_frequency/1000) - 1); // (48MHz/1000)
    systick_interrupt_enable();
    systick_counter_enable();

    // Set up USB peripheral
    nvic_enable_irq(NVIC_USB_IRQ);

    g_usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev_descr, &config, usb_strings, COUNT_OF(usb_strings), usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(g_usbd_dev, usb_set_config);

    while (1) {

    }
}

/**
 * USB interrupt handler
 */
void usb_isr(void)
{
    // Handle USB requests as they come through
    usbd_poll(g_usbd_dev);
}

/**
 * SysTick interrupt handler
 */
void sys_tick_handler(void)
{

}