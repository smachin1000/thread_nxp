/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if !defined(__BOARD_H__)
#define __BOARD_H__

#include <stdint.h>
#include "pin_mux.h"
#include "gpio_pins.h"

/* The board name */
#define BOARD_NAME                      "TWR-K21D50M"

/* The UART to use for debug messages. */
#ifndef BOARD_DEBUG_UART_INSTANCE
    #define BOARD_DEBUG_UART_INSTANCE   2
    #define BOARD_DEBUG_UART_BASEADDR   UART2_BASE
#endif
#ifndef BOARD_DEBUG_UART_BAUD
    #define BOARD_DEBUG_UART_BAUD       115200
#endif

#define BOARD_USE_UART

/* Define feature for the low_power_demo */
#define FSL_FEATURE_HAS_VLLS2 (1)

/* Define the port interrupt number for the board switches */
#define BOARD_SW_IRQ_NUM        PORTC_IRQn

/* Define print statement to inform user which switch to press for 
 * low_power_demo 
 */
#define PRINT_INT_SW_NUM \
  printf("SW2")

#define PRINT_LLWU_SW_NUM \
  printf("SW3")

/* Defines the llwu pin number for board switch which is used in power_manager_demo. */
#define BOARD_SW_HAS_LLWU_PIN           1
#define BOARD_SW_LLWU_EXT_PIN           kLlwuWakeupPin10
/* Switch port base address and IRQ handler name. Used by power_manager_demo */
#define BOARD_SW_LLWU_PIN               6
#define BOARD_SW_LLWU_BASE              PORTC_BASE
#define BOARD_SW_LLWU_IRQ_HANDLER       PORTC_IRQHandler
#define BOARD_SW_LLWU_IRQ_NUM           PORTC_IRQn

/* The MMA8451 i2c instance and slave address */
#define BOARD_MMA8451_I2C_INSTANCE      1
#define BOARD_SGTL_I2C_INSTANCE         1
#define BOARD_ACCEL_ADDR_ACTUAL         (0x1C)
#define BOARD_ACCEL_I2C_ADDR            (BOARD_ACCEL_ADDR_ACTUAL << 1)

#define BOARD_I2C_GPIO_SCL              GPIO_MAKE_PIN(HW_GPIOC, 10)
#define BOARD_I2C_GPIO_SDA              GPIO_MAKE_PIN(HW_GPIOC, 11)
#define BOARD_I2C_DELAY \
    do \
    { \
        int32_t i; \
        for (i = 0; i < 800; i++) \
        { \
            __asm("nop"); \
        } \
    } while (0)

/* The instances of peripherals used for dac_adc_demo */
#define BOARD_DAC_DEMO_DAC_INSTANCE     0U
#define BOARD_DAC_DEMO_ADC_INSTANCE     0U
#define BOARD_DAC_DEMO_ADC_CHANNEL      13U // Elevator B30

/* The i2c instance used for i2c DAC demo */
#define BOARD_DAC_I2C_INSTANCE          1

/* The i2c instance used for i2c communication demo */
#define BOARD_I2C_COMM_INSTANCE         1

/* The Flextimer instance/channel used for board */
#define BOARD_FTM_INSTANCE              0
#define BOARD_FTM_CHANNEL               6

/* ADC0 input channel */
#define BOARD_ADC0_INPUT_CHAN           12

/* board led color mapping */
#define BOARD_GPIO_LED_GREEN            kGpioLED1
#define BOARD_GPIO_LED_YELLOW           kGpioLED2
#define BOARD_GPIO_LED_ORANGE           kGpioLED3
#define BOARD_GPIO_LED_BLUE             kGpioLED4
#define BOARD_GPIO_LED_RED              BOARD_GPIO_LED_ORANGE // Workaround to fix RED LED issue

#define DISABLE_DEBUG_CONSOLE_TX PORT_HAL_SetMuxMode(PORTE_BASE, 16, kPortPinDisabled)
#define DISABLE_DEBUG_CONSOLE_RX PORT_HAL_SetMuxMode(PORTE_BASE, 17, kPortPinDisabled)

#define DISABLE_SW_INTERRUPT PORT_HAL_SetPinIntMode(PORTC_BASE, 6, kPortIntDisabled)
#define DISABLE_SW_PIN PORT_HAL_SetMuxMode(PORTC_BASE, 6, kPortPinDisabled)
#define ENABLE_SW_PIN PORT_HAL_SetMuxMode(PORTC_BASE, 6, kPortMuxAsGpio) 

#define LED1_EN (PORT_HAL_SetMuxMode(PORTD_BASE, 4, kPortMuxAsGpio))    /*!< Enable target LED0 */
#define LED2_EN (PORT_HAL_SetMuxMode(PORTD_BASE, 5, kPortMuxAsGpio))    /*!< Enable target LED1 */
#define LED3_EN (PORT_HAL_SetMuxMode(PORTD_BASE, 6, kPortMuxAsGpio))    /*!< Enable target LED2 */
#define LED4_EN (PORT_HAL_SetMuxMode(PORTD_BASE, 7, kPortMuxAsGpio))    /*!< Enable target LED3 */

#define LED1_DIS (PORT_HAL_SetMuxMode(PORTD_BASE, 4, kPortIntDisabled)) /*!< Enable target LED0 */
#define LED2_DIS (PORT_HAL_SetMuxMode(PORTD_BASE, 5, kPortIntDisabled)) /*!< Enable target LED1 */
#define LED3_DIS (PORT_HAL_SetMuxMode(PORTD_BASE, 6, kPortIntDisabled)) /*!< Enable target LED2 */
#define LED4_DIS (PORT_HAL_SetMuxMode(PORTD_BASE, 7, kPortIntDisabled)) /*!< Enable target LED3 */

#define LED1_OFF (GPIO_DRV_WritePinOutput(ledPins[0].pinName, 1))       /*!< Turn off target LED0 */
#define LED2_OFF (GPIO_DRV_WritePinOutput(ledPins[1].pinName, 1))       /*!< Turn off target LED1 */
#define LED3_OFF (GPIO_DRV_WritePinOutput(ledPins[2].pinName, 1))       /*!< Turn off target LED2 */
#define LED4_OFF (GPIO_DRV_WritePinOutput(ledPins[3].pinName, 1))       /*!< Turn off target LED3 */

#define LED1_ON (GPIO_DRV_WritePinOutput(ledPins[0].pinName, 0))        /*!< Turn on target LED0 */
#define LED2_ON (GPIO_DRV_WritePinOutput(ledPins[1].pinName, 0))        /*!< Turn on target LED1 */
#define LED3_ON (GPIO_DRV_WritePinOutput(ledPins[2].pinName, 0))        /*!< Turn on target LED2 */
#define LED4_ON (GPIO_DRV_WritePinOutput(ledPins[3].pinName, 0))        /*!< Turn on target LED3 */

/* The CMP instance used for board. */
#define BOARD_CMP_INSTANCE              0
/* The CMP channel used for board. */
#define BOARD_CMP_CHANNEL               0

/* The i2c instance used for sai demo */
#define BOARD_SAI_DEMO_I2C_INSTANCE     1

/* The rtc instance used for rtc_func */
#define BOARD_RTC_FUNC_INSTANCE         0

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

void hardware_init(void);
void dbg_uart_init(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __BOARD_H__ */
