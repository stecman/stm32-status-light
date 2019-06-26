#include "sk6812.h"

#include "stm32f0xx_hal.h"

static void _tim17_run_blocking()
{
    // Update settings and clear the prescaler counter
    TIM17->EGR |= TIM_EGR_UG;

    // Clear the overflow flag
    TIM17->SR = ~TIM_FLAG_UPDATE;

    // Start the timer
    TIM17->CR1 |= TIM_CR1_CEN;

    // Wait until the timer overflows
    while ((TIM17->SR & TIM_FLAG_UPDATE) == 0);

    // Stop the timer
    TIM17->CR1 &= ~TIM_CR1_CEN;
}

void delay_ms(uint16_t milliseconds)
{
    // Set period to requested tickets
    TIM17->ARR = milliseconds;

    // Set prescaler for 1 tick = 1ms
    TIM17->PSC = 48000;

    _tim17_run_blocking();
}

void delay_us(uint16_t microseconds)
{
    // Set period to requested tickets
    TIM17->ARR = microseconds;

    // Set prescaler for 1 tick = 1us
    TIM17->PSC = 48;

    _tim17_run_blocking();
}

/**
 * Transmission delays for SK6812 one-wire transmissio
 */
__attribute__((always_inline))
static inline void delay_300ns()
{
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
}

__attribute__((always_inline))
static inline void delay_600ns()
{
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP();
}

__attribute__((always_inline))
static inline void delay_900ns()
{
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP();
}

/**
 * Write a 24-bit colour on the LED data line
 */
static void sk6812_write_grb(uint32_t colour)
{
    // Disable interrupts from messing with the tight time-based LED data signal
    __disable_irq();

    uint8_t bits = 24;
    while (bits != 0) {

        // Check if the 24th bit is set
        if ((colour & 0x800000) == 0) {
            // Send "zero" bit

            // Write high
            LED_DATA_GPIO_Port->BSRR = LED_DATA_Pin;
            delay_300ns();

            // Write low
            LED_DATA_GPIO_Port->BRR = LED_DATA_Pin;
            delay_900ns();

        } else {
            // Send "one" bit

            // Write high
            LED_DATA_GPIO_Port->BSRR = LED_DATA_Pin;
            delay_600ns();

            // Write low
            LED_DATA_GPIO_Port->BRR = LED_DATA_Pin;
        }

        colour <<= 1;
        --bits;
    }

    __enable_irq();
}

static uint32_t rgbTogrb(uint32_t colour)
{
    return ((colour & 0xFF00) << 8) |
           ((colour & 0xFF0000) >> 8) |
           (colour & 0xFF);
}

void sk6812_reset(void)
{
    // Pull LED data line low for longer than the reset period
    LED_DATA_GPIO_Port->BRR = LED_DATA_Pin;
    delay_us(80);
}

void sk6812_write_rgb(uint32_t colour)
{
    sk6812_write_grb(rgbTogrb(colour));
}

uint32_t applyCieBrightness(uint32_t colour)
{
    // CIE1931 luminance correction table
    // Generated from code here: http://jared.geek.nz/2013/feb/linear-led-pwm
    static const uint8_t brightness[256] = {
        0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
        3, 4, 4, 4, 4, 4, 4, 5, 5, 5,
        5, 5, 6, 6, 6, 6, 6, 7, 7, 7,
        7, 8, 8, 8, 8, 9, 9, 9, 10, 10,
        10, 10, 11, 11, 11, 12, 12, 12, 13, 13,
        13, 14, 14, 15, 15, 15, 16, 16, 17, 17,
        17, 18, 18, 19, 19, 20, 20, 21, 21, 22,
        22, 23, 23, 24, 24, 25, 25, 26, 26, 27,
        28, 28, 29, 29, 30, 31, 31, 32, 32, 33,
        34, 34, 35, 36, 37, 37, 38, 39, 39, 40,
        41, 42, 43, 43, 44, 45, 46, 47, 47, 48,
        49, 50, 51, 52, 53, 54, 54, 55, 56, 57,
        58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
        68, 70, 71, 72, 73, 74, 75, 76, 77, 79,
        80, 81, 82, 83, 85, 86, 87, 88, 90, 91,
        92, 94, 95, 96, 98, 99, 100, 102, 103, 105,
        106, 108, 109, 110, 112, 113, 115, 116, 118, 120,
        121, 123, 124, 126, 128, 129, 131, 132, 134, 136,
        138, 139, 141, 143, 145, 146, 148, 150, 152, 154,
        155, 157, 159, 161, 163, 165, 167, 169, 171, 173,
        175, 177, 179, 181, 183, 185, 187, 189, 191, 193,
        196, 198, 200, 202, 204, 207, 209, 211, 214, 216,
        218, 220, 223, 225, 228, 230, 232, 235, 237, 240,
        242, 245, 247, 250, 252, 255,
    };

    return (brightness[(colour & 0xFF)]) |
           (brightness[(colour & 0xFF00) >> 8] << 8) |
           (brightness[(colour & 0xFF0000) >> 16] << 16);
}
