#include "sk6812.h"
#include "usb_control.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
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

/**
 * Callback for USB device ready state
 */
void usb_set_config_callback(UNUSED usbd_device *usbd_dev, UNUSED uint16_t wValue)
{
    set_all_leds(0x000500);
}

int main(void)
{
    // Turn on the SYSCFG module and switch out PA9/PA10 for PA11/PA12
    // The USB peripheral is not phsyically connected without this and won't be enumerated
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

    clock_setup_12mhz_hse_out_48mhz();

    sk6812_init();

    set_all_leds(0x0);

    // Configure SysTick at 1ms intervals
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload((rcc_ahb_frequency/1000) - 1); // (48MHz/1000)
    systick_interrupt_enable();
    systick_counter_enable();

    usb_device_init();

    set_all_leds(0x050000);

    while (1) {

    }
}

/**
 * SysTick interrupt handler
 */
void sys_tick_handler(void)
{

}