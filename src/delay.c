#include "delay.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

void delay_init(void)
{
    rcc_periph_clock_enable(RCC_TIM17);

    // Configure Timer17 for delay functions
    timer_set_prescaler(TIM17, 1);
    timer_direction_up(TIM17);
    timer_set_period(TIM17, 0);
    timer_set_clock_division(TIM17, 1);
    timer_set_repetition_counter(TIM17, 0);
    timer_disable_preload(TIM17);
    timer_one_shot_mode(TIM17);
}

static void _tim17_run_blocking(void)
{
    // Update settings and clear the prescaler counter
    TIM_EGR(TIM17) |= TIM_EGR_UG;

    // Clear the overflow flag
    TIM_SR(TIM17) = ~TIM_SR_UIF;

    // Start the timer
    TIM_EGR(TIM17) = TIM_EGR_UG;
    TIM_CR1(TIM17) |= TIM_CR1_CEN;

    // Wait until the timer overflows
    // while ((TIM_SR(TIM17) & TIM_SR_UIF) == 0);
    while (TIM_CR1(TIM17) & TIM_CR1_CEN);

    // Stop the timer
    TIM_CR1(TIM17) &= ~TIM_CR1_CEN;
}

void delay_ms(uint16_t milliseconds)
{
    // Set period to requested tickets
    TIM_ARR(TIM17) = milliseconds;

    // Set prescaler for 1 tick = 1ms
    TIM_PSC(TIM17) = 48000;

    _tim17_run_blocking();
}

void delay_us(uint16_t microseconds)
{
    // Set period to requested tickets
    TIM_ARR(TIM17) = microseconds;

    // Set prescaler for 1 tick = 1us
    TIM_PSC(TIM17) = 48;

    _tim17_run_blocking();
}