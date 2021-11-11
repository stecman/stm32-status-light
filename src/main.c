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
#include <string.h>

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

enum MediaKey {
    kMedia_PlayPause = (1<<0),
    kMedia_Stop = (1<<1),
    kMedia_PrevTrack = (1<<2),
    kMedia_NextTrack = (1<<3),
    kMedia_Mute = (1<<4),
    kMedia_VolumeDown = (1<<5),
    kMedia_VolumeUp = (1<<6),
    kMedia_Refresh = (1<<7),
};

static uint8_t usb_keys[9] = {
    1, // Report ID
    KEY_NONE, // Modified keys
    0x0, // Reserved
    KEY_NONE,
    KEY_NONE,
    KEY_NONE,
    KEY_NONE,
    KEY_NONE,
    KEY_NONE,
};

static uint8_t usb_media_keys[2] = {
    2, // Report ID
    0x0, // Media keys bit mask
};

static uint16_t changed_mask = 0;

/**
 * Send updates for all reports
 */
static void usb_send_updates(void)
{
    if (usb_ready) {
        if (changed_mask & 0b111111111111) {
            usb_send_packet(usb_keys, sizeof(usb_keys));
        }

        if (changed_mask & 0b1111000000000000) {
            usb_send_packet(usb_media_keys, sizeof(usb_media_keys));
        }

        changed_mask = 0;
    }
}

/**
 * Callback for USB device ready state
 */
void usb_ready_callback(UNUSED usbd_device *usbd_dev)
{
    // Always output all reports on connect
    // Hosts seem to ignore the device if it doesn't do this
    changed_mask = 0xFFFF;
    usb_send_updates();
}

// Key codes each button should send (left to right for the user)
static const uint8_t kKeyMap[12] = {
    0x68, // F13
    0x69, // F14
    0x6a, // F15
    0x6b, // F16
    0x6c, // F17
    0x6d, // F18
    0x6e, // F19
    0x6f, // F20
    0x70, // F21
    0x71, // F22
    0x72, // F23
    0x73, // F24
};

int main(void)
{
    // Turn on the SYSCFG module and switch out PA9/PA10 for PA11/PA12
    // The USB peripheral is not phsyically connected without this and won't be enumerated
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

    // Clock setup
    clock_setup_12mhz_hse_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    // Init sub-systems
    delay_init();
    sk6812_init();

    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1 | GPIO2 | GPIO3 | GPIO4);
    gpio_set(GPIOA, GPIO1 | GPIO2 | GPIO3 | GPIO4);

    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO1);

    set_all_leds(0x0);

    // Configure SysTick at 1ms intervals
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload((rcc_ahb_frequency/1000) - 1); // (48MHz/1000)
    systick_interrupt_enable();
    systick_counter_enable();

    usb_device_init();

    uint16_t state = 0x0;
    uint16_t lastState = 0x0;

    uint8_t debounce[16];

    const uint32_t moder_base = GPIOA_MODER;

    while (1) {
        // Scan key matrix
        for (uint8_t row = 0; row < 4; ++row) {
            gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1 | GPIO2 | GPIO3 | GPIO4);
            gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, (1<<(row+1)));

            // This would be better, but I'm having trouble getting it to work with breaking
            // GPIOA_MODER &= ~0x3FC;
            // GPIOA_MODER |= (0b0100 << (row*2));

            // Read all columns (split across GPIO A & B which makes this slightly messy)
            const uint8_t rowState = ((GPIOA_IDR & 0xE0) >> 5) | ((GPIOB_IDR & 0x02) << 2);

            // Debounce keys by requiring repeated reads before transitioning state
            for (uint8_t i = 0; i < 4; ++i) {
                const uint8_t keyIndex = (row*4) + i;

                if (rowState & (1<<i)) {
                    // Trigger key on after a fixed number of high reads
                    if (debounce[keyIndex] < 50) {
                        debounce[keyIndex]++;
                    } else {
                        state |= (1<<keyIndex);
                    }
                } else {
                    // Trigger key off after a fixed number of low reads
                    if (debounce[keyIndex] > 0) {
                        debounce[keyIndex]--;
                    } else {
                        state &= ~(1<<keyIndex);
                    }
                }
            }
        }

        if (state != lastState) {
            changed_mask = state ^ lastState;

            const uint8_t kMaxUsbIndex = 9;
            uint8_t usbIndex = 3;

            // Clear key map
            for (uint8_t i = usbIndex; i < kMaxUsbIndex - usbIndex; i++) {
                usb_keys[i] = KEY_NONE;
            }

            // Keys from fixed mapping
            for (uint8_t i = 0; i < 12; i++) {
                if (state & (1<<i)) {
                    usb_keys[usbIndex] = kKeyMap[i];
                    usbIndex++;

                    if (usbIndex == kMaxUsbIndex) {
                        // No more places to send pressed keys
                        break;
                    }
                }
            }

            // Key 13
            if (state & 0x1000) {
                usb_media_keys[1] |= kMedia_NextTrack;
            } else {
                usb_media_keys[1] &= ~kMedia_NextTrack;
            }

            // Key 14
            if (state & 0x2000) {
                usb_media_keys[1] |= kMedia_Mute;
            } else {
                usb_media_keys[1] &= ~kMedia_Mute;
            }

            // Key 15
            if (state & 0x4000) {
                usb_media_keys[1] |= kMedia_VolumeDown;
            } else {
                usb_media_keys[1] &= ~kMedia_VolumeDown;
            }

            // Key 16
            if (state & 0x8000) {
                usb_media_keys[1] |= kMedia_VolumeUp;
            } else {
                usb_media_keys[1] &= ~kMedia_VolumeUp;
            }

            usb_send_updates();
        }

        lastState = state;
    }

    while (1) {}
}

/**
 * SysTick interrupt handler
 */
void sys_tick_handler(void)
{

}
