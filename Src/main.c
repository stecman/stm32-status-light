/**
    ******************************************************************************
    * File Name          : main.c
    * Description        : Main program body
    ******************************************************************************
    * This notice applies to any and all portions of this file
    * that are not between comment pairs USER CODE BEGIN and
    * USER CODE END. Other portions of this file, whether
    * inserted by the user or by software development tools
    * are owned by their respective copyright owners.
    *
    * Copyright (c) 2019 STMicroelectronics International N.V.
    * All rights reserved.
    *
    * Redistribution and use in source and binary forms, with or without
    * modification, are permitted, provided that the following conditions are met:
    *
    * 1. Redistribution of source code must retain the above copyright notice,
    *    this list of conditions and the following disclaimer.
    * 2. Redistributions in binary form must reproduce the above copyright notice,
    *    this list of conditions and the following disclaimer in the documentation
    *    and/or other materials provided with the distribution.
    * 3. Neither the name of STMicroelectronics nor the names of other
    *    contributors to this software may be used to endorse or promote products
    *    derived from this software without specific written permission.
    * 4. This software, including modifications and/or derivative works of this
    *    software, must execute solely and exclusively on microcontroller or
    *    microprocessor devices manufactured by or for STMicroelectronics.
    * 5. Redistribution and use of this software other than as permitted under
    *    this license is void and will automatically terminate your rights under
    *    this license.
    *
    * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
    * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
    * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
    * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
    * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
    * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
    * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
    * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
    * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    *
    ******************************************************************************
    */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);

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

static void delay_ms(uint16_t milliseconds)
{
    // Set period to requested tickets
    TIM17->ARR = milliseconds;

    // Set prescaler for 1 tick = 1ms
    TIM17->PSC = 48000;

    _tim17_run_blocking();
}

static void delay_us(uint16_t microseconds)
{
    // Set period to requested tickets
    TIM17->ARR = microseconds;

    // Set prescaler for 1 tick = 1us
    TIM17->PSC = 48;

    _tim17_run_blocking();
}

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

static void sk6812_reset(void)
{
    // Pull LED data line low for longer than the reset period
    LED_DATA_GPIO_Port->BRR = LED_DATA_Pin;
    delay_us(80);
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

            // Delay including loop overhead to 600ns
            __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
            __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
            __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
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

static void sk6812_write_rgb(uint32_t colour)
{
    sk6812_write_grb(rgbTogrb(colour));
}

static uint32_t applyCieBrightness(uint32_t colour)
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

int main(void)
{
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    // MX_IWDG_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    // MX_USB_DEVICE_Init();

    uint32_t colour = 0;
    uint8_t rgb[3] = {0, 0, 0};

    uint8_t mask = 1;

    uint32_t colours[] = {
        0x550000,
        0x005500,
        0x000055,
        0x550055,
        0x555500,
        0x005555,
        0x125512,
        0xf99d1e,
        0xf9f61e,
        0x1ef9bc,
        0x971ef9,
    };

    {
        uint8_t colour_index = 0;

        for (int i = 0; i < 255; ++i) {
            delay_ms(100);

            sk6812_reset();
            for (int j = 0; j < 4; ++j) {
                sk6812_write_rgb(colours[colour_index]);
                ++colour_index;
                if (colour_index == (sizeof(colours)/sizeof(uint32_t))) colour_index = 0;
            }
        }
    }

    while (1)
    {
        for (int i = 240; i != 0; --i) {
            delay_ms(1);
            if ((mask & 0x1) && rgb[0] < 240) rgb[0] += 1; else if (rgb[0] > 0) rgb[0] -= 1;
            if ((mask & 0x2) && rgb[1] < 240) rgb[1] += 1; else if (rgb[1] > 0) rgb[1] -= 1;
            if ((mask & 0x4) && rgb[2] < 240) rgb[2] += 1; else if (rgb[2] > 0) rgb[2] -= 1;

            colour = (rgb[1] << 16) | (rgb[0] << 8) | rgb[2];
            colour = applyCieBrightness(colour);

            // Write colour to all LEDs
            sk6812_reset();
            sk6812_write_rgb(colour);
            sk6812_write_rgb(colour);
            sk6812_write_rgb(colour);
            sk6812_write_rgb(colour);
        }

        ++mask;
        if (mask == 0x7) {
            mask = 1;
        }
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    // Configure the Systick
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 1;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 0;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

    htim17.Instance = TIM17;
    htim17.Init.Prescaler = 1;
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = 0;
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.RepetitionCounter = 0;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    TIM17->CR1 |= TIM_CR1_OPM;
}

/** Configure pins as
                * Analog
                * Input
                * Output
                * EVENT_OUT
                * EXTI
*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED_DATA_Pin */
    GPIO_InitStruct.Pin = LED_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(LED_DATA_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
    * @brief  This function is executed in case of error occurrence.
    * @param  None
    * @retval None
    */
void _Error_Handler(char * file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

/**
    * @}
    */

/**
    * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
