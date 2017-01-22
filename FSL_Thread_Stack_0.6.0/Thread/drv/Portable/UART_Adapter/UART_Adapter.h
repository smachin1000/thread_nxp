/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file UART.h
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

#ifndef __UART_ADAPTER_H__
#define __UART_ADAPTER_H__

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "fsl_device_registers.h"

#if defined(HW_LPUART_INSTANCE_COUNT)
  #include "fsl_lpuart_driver.h"
  #include "fsl_lpuart_hal.h"
#else
  #include "fsl_uart_driver.h"
  #include "fsl_uart_hal.h"
#endif

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#if defined(HW_LPUART_INSTANCE_COUNT)
  #define uart_state_t               lpuart_state_t
  #define gUartInstanceCount_c       HW_LPUART_INSTANCE_COUNT
  #define gaUartBaseAddr             g_lpuartBaseAddr

#else 
  #define gUartInstanceCount_c       HW_UART_INSTANCE_COUNT
  #define gaUartBaseAddr             g_uartBaseAddr
#endif

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
typedef void (*pfUARTx_TxCB_t)(uint32_t);
typedef void (*pfUARTx_RxCB_t)(uint32_t);

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
void Uart_Init(uint32_t instance, uart_state_t * pUartState, pfUARTx_TxCB_t txCB, pfUARTx_RxCB_t rxCB);
void Uart_SetBaudrate(uint32_t instance, uint32_t baud);
void Uart_SendData(uint32_t instance, uint8_t * txBuff, uint32_t txSize);
void Uart_ReceiveData(uint32_t instance, uint8_t * rxBuff, uint32_t rxSize);

void Uart_EnableLowPowerWakeup(void);
void Uart_DisableLowPowerWakeup(void);
bool_t Uart_IsWakeUpSource(void);

#endif /* __UART_ADAPTER_H__ */