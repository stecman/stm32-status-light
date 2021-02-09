#include "delay.h"
#include "sk6812.h"
#include "usb_control.h"
#include "usb_hid_keys.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>

#include <stdint.h>
#include <stdlib.h>

#define UNUSED __attribute__ ((unused))

static const uint8_t NUM_LEDS = 4;

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

static void set_all_leds(uint32_t rgb)
{
    sk6812_reset();

    for (uint8_t i = 0; i < NUM_LEDS; ++i) {
        sk6812_write_rgb(rgb);
    }
}

static uint8_t keys[8] = {
    KEY_NONE, // Modified keys
    0x0, // Reserved
    KEY_NONE,
    KEY_NONE,
    KEY_NONE,
    KEY_NONE,
    KEY_NONE,
    KEY_NONE,
};

/**
 * Callback for USB device ready state
 */
void usb_ready_callback(UNUSED usbd_device *usbd_dev)
{
    usb_send_packet(keys, 8);
}


int main(void)
{
    // Turn on the SYSCFG module and switch out PA9/PA10 for PA11/PA12
    // The USB peripheral is not phsyically connected without this and won't be enumerated
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

    // Clock setup
    clock_setup_12mhz_hse_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);

    // Init sub-systems
    delay_init();
    sk6812_init();

    // Enable EXT0 interrupt
    nvic_enable_irq(NVIC_EXTI0_1_IRQ);
    nvic_enable_irq(NVIC_EXTI2_3_IRQ);

    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO3);

    // Configure exterinal interrupt subsystem
    exti_select_source(EXTI1 | EXTI2 | EXTI3, GPIOA);
    exti_set_trigger(EXTI1 | EXTI2 | EXTI3, EXTI_TRIGGER_FALLING);
    exti_reset_request(EXTI1 | EXTI2 | EXTI3);
    exti_enable_request(EXTI1 | EXTI2 | EXTI3);

    set_all_leds(0x0);

    // Configure SysTick at 1ms intervals
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload((rcc_ahb_frequency/1000) - 1); // (48MHz/1000)
    systick_interrupt_enable();
    systick_counter_enable();

    usb_device_init();

    while (1) {

    }
}

static void handlekey(void)
{
    delay_ms(50);
    exti_reset_request(EXTI1 | EXTI2 | EXTI3);

    if (!usb_ready) {
        return;
    }

    const uint16_t input = GPIOA_IDR;

    const bool mutePressed = (input & GPIO1) == 0;
    const bool volDownPressed = (input & GPIO2) == 0;
    const bool volUpPressed = (input & GPIO3) == 0;

    if (volUpPressed && volDownPressed) {
        keys[2] = KEY_MEDIA_REFRESH;
        usb_send_packet(keys, 8);
        delay_ms(50);
        keys[2] = KEY_NONE;
        usb_send_packet(keys, 8);
    }
    else if (mutePressed) {
        keys[2] = KEY_MEDIA_MUTE;
        usb_send_packet(keys, 8);
        delay_ms(50);
        keys[2] = KEY_NONE;
        usb_send_packet(keys, 8);
    }
    else if (volUpPressed) {
        keys[2] = KEY_MEDIA_VOLUMEUP;
        usb_send_packet(keys, 8);
        delay_ms(50);
        keys[2] = KEY_NONE;
        usb_send_packet(keys, 8);
    }
    else if (volDownPressed) {
        keys[2] = KEY_MEDIA_VOLUMEDOWN;
        usb_send_packet(keys, 8);
        delay_ms(50);
        keys[2] = KEY_NONE;
        usb_send_packet(keys, 8);
    }
    else {
        keys[2] = KEY_NONE;
        usb_send_packet(keys, 8);
    }
}

void exti0_1_isr(void)
{
    handlekey();
}

void exti2_3_isr(void)
{
    handlekey();
}

/**
 * SysTick interrupt handler
 */
void sys_tick_handler(void)
{

}
