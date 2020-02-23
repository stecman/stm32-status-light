#pragma once

#include <stdint.h>

/**
 * Blocking delay for a period of milliseconds
 */
void delay_ms(uint16_t milliseconds);

/**
 * Blocking delay for a period of microseconds
 */
void delay_us(uint16_t microseconds);

/**
 * Configure perihperals needed to drive the SK6812 LEDS
 */
void sk6812_init(void);

/**
 * Issue a reset period on the LED data line
 * This is only needed if there will not be a natural 50uS delay between updates
 */
void sk6812_reset(void);

/**
 * Write an 8bpp RGB value to the next LED in the connected sequence
 * @param colour [description]
 */
void sk6812_write_rgb(uint32_t colour);

/**
 * Modify the passed 8bpp RGB colour to have a linear percieved brightness on a scale from 0-255
 *
 * The LEDs have a logarithmic brightness output (the input value are effectively "output power")
 */
uint32_t applyCieBrightness(uint32_t colour);
