/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file UART_Adapter.c
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


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "UART_Adapter.h"
#include "fsl_clock_manager.h"
#include "pin_mux.h"


/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#if defined(HW_LPUART_INSTANCE_COUNT)
  #define gUartHwInstances_c HW_LPUART_INSTANCE_COUNT
#else
  #define gUartHwInstances_c HW_UART_INSTANCE_COUNT
#endif


/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */
typedef void (*pfUARTx_ISR_t)(void);

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
static void UARTx_ISR(void);
#if defined(HW_LPUART_INSTANCE_COUNT)
extern void LPUART_DRV_IrqHandler(uint32_t instance);
#else
extern void UART_DRV_IRQHandler(uint32_t instance);
#endif

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
#if defined(HW_LPUART_INSTANCE_COUNT)
    const lpuart_user_config_t defaultLpuartCfg = {
        .clockSource = kClockLpuartSrcOsc0erClk,
        .baudRate = 115200,
        .parityMode = kLpuartParityDisabled,
        .stopBitCount = kLpuartOneStopBit,
        .bitCountPerChar = kLpuart8BitsPerChar
    };
    
    extern void * g_lpuartStatePtr[HW_LPUART_INSTANCE_COUNT];
#else
    const uart_user_config_t defaultUartCfg = {
        .baudRate = 115200,
        .parityMode = kUartParityDisabled,
        .stopBitCount = kUartOneStopBit,
        .bitCountPerChar = kUart8BitsPerChar
    };
    
    extern void * g_uartStatePtr[HW_UART_INSTANCE_COUNT];
#endif

static pfUARTx_TxCB_t mUartTxCb[gUartHwInstances_c];
static pfUARTx_RxCB_t mUartRxCb[gUartHwInstances_c];
static uint8_t mUartIsrOffest = 1;

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

void Uart_Init(uint32_t instance, uart_state_t * pUartState, pfUARTx_TxCB_t txCB, pfUARTx_RxCB_t rxCB)
{
    IRQn_Type irq;

    if( instance >= gUartHwInstances_c )
        return;
    
#if (gUartHwInstances_c > 1)
#if defined(HW_LPUART_INSTANCE_COUNT)
    mUartIsrOffest = g_lpuartRxTxIrqId[1] - g_lpuartRxTxIrqId[0];
#else
    mUartIsrOffest = g_uartRxTxIrqId[1] - g_uartRxTxIrqId[0];
#endif
#endif

#if defined(HW_LPUART_INSTANCE_COUNT)
    configure_lpuart_pins(instance);
    LPUART_DRV_Init(instance, pUartState, &defaultLpuartCfg);
    irq = g_lpuartRxTxIrqId[instance];
#else
    configure_uart_pins(instance);
    UART_DRV_Init(instance, pUartState, &defaultUartCfg);
    irq = g_uartRxTxIrqId[instance];
#endif
    NVIC_SetPriority(irq, 0x40 >> (8 - __NVIC_PRIO_BITS)); // make UART ISR higher than normal
    /* Overwrite old ISR */
    OSA_InstallIntHandler(irq, UARTx_ISR);
    mUartTxCb[instance] = txCB;
    mUartRxCb[instance] = rxCB;
}

void Uart_SendData(uint32_t instance, uint8_t * txBuff, uint32_t txSize)
{
    if( instance >= gUartHwInstances_c )
        return;

#if defined(HW_LPUART_INSTANCE_COUNT)
    (void)LPUART_DRV_SendData(instance, txBuff, txSize);
    LPUART_HAL_SetTxDataRegEmptyIntCmd(gaUartBaseAddr[instance], TRUE);
#else
    (void)UART_DRV_SendData(instance, txBuff, txSize);
    UART_HAL_SetTxDataRegEmptyIntCmd(gaUartBaseAddr[instance], TRUE);
#endif
}

void Uart_ReceiveData(uint32_t instance, uint8_t * rxBuff, uint32_t rxSize)
{
    if( instance >= gUartHwInstances_c )
        return;

#if defined(HW_LPUART_INSTANCE_COUNT)
    (void)LPUART_DRV_ReceiveData(instance, rxBuff, rxSize);
#else
    (void)UART_DRV_ReceiveData(instance, rxBuff, rxSize);
#endif
}

void Uart_SetBaudrate(uint32_t instance, uint32_t baud)
{
    uint32_t uartSourceClock;
    uint32_t baseAddr = gaUartBaseAddr[instance];

    if( instance >= gUartHwInstances_c )
        return;

#if defined(HW_LPUART_INSTANCE_COUNT)
    uartSourceClock = CLOCK_SYS_GetLpuartFreq(instance);
    (void)LPUART_HAL_SetBaudRate(baseAddr, uartSourceClock, baud);
#else
    uartSourceClock = CLOCK_SYS_GetUartFreq(instance);
    (void)UART_HAL_SetBaudRate(baseAddr, uartSourceClock, baud);
#endif
}

void Uart_EnableLowPowerWakeup(void)
{
    uint32_t i;
    
    for( i=0; i<gUartInstanceCount_c; i++ )
    {
        uint32_t baseAddr = gaUartBaseAddr[i];
#if defined(HW_LPUART_INSTANCE_COUNT)
        LPUART_HAL_SetIntMode(baseAddr, kLpuartIntRxActiveEdge, FALSE);
        LPUART_HAL_ClearStatusFlag(baseAddr, kLpuartLineBreakDetect);
        LPUART_HAL_SetIntMode(baseAddr, kLpuartIntRxActiveEdge, TRUE);
#else
        UART_HAL_SetIntMode(baseAddr, kUartIntRxActiveEdge, FALSE);
        UART_HAL_ClearStatusFlag(baseAddr, kUartLineBreakDetect);
        UART_HAL_SetIntMode(baseAddr, kUartIntRxActiveEdge, TRUE);
#endif
    }
}

void Uart_DisableLowPowerWakeup(void)
{
    uint32_t i;
    
    for( i=0; i<gUartInstanceCount_c; i++ )
    {
        uint32_t baseAddr = gaUartBaseAddr[i];
#if defined(HW_LPUART_INSTANCE_COUNT)
        LPUART_HAL_SetIntMode(baseAddr, kLpuartIntRxActiveEdge, FALSE);
        LPUART_HAL_ClearStatusFlag(baseAddr, kLpuartLineBreakDetect);
#else
        UART_HAL_SetIntMode(baseAddr, kUartIntRxActiveEdge, FALSE);
        UART_HAL_ClearStatusFlag(baseAddr, kUartLineBreakDetect);
#endif
    }
}

bool_t Uart_IsWakeUpSource(void)
{
    uint32_t i;

    for( i=0; i<gUartInstanceCount_c; i++ )
    {
#if defined(HW_LPUART_INSTANCE_COUNT)
        if( LPUART_HAL_GetStatusFlag(gaUartBaseAddr[i], kLpuartLineBreakDetect) )
#else
        if( UART_HAL_GetStatusFlag(gaUartBaseAddr[i], kUartLineBreakDetect) )
#endif
        {
            return TRUE;
        }
    }
    return FALSE;
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************* */
static void UARTx_ISR(void)
{
    uint32_t instance = ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) >> SCB_ICSR_VECTACTIVE_Pos) - 16;
    uint32_t rxSize;
	uart_state_t * pState;

#if defined(HW_LPUART_INSTANCE_COUNT)
    instance = (instance - g_lpuartRxTxIrqId[0])/mUartIsrOffest;
    pState = (uart_state_t *)g_lpuartStatePtr[instance];
    rxSize = pState->rxSize;

    LPUART_DRV_IrqHandler(instance);
#else
    instance = (instance - g_uartRxTxIrqId[0])/mUartIsrOffest;
    pState = (uart_state_t *)g_uartStatePtr[instance];
    rxSize = pState->rxSize;
    
    UART_DRV_IRQHandler(instance);
#endif

    if( mUartTxCb[instance] && pState->txBuff && !pState->isTxBusy )
    {
        pState->txBuff = NULL;
#if defined(HW_LPUART_INSTANCE_COUNT)
        LPUART_HAL_SetTxDataRegEmptyIntCmd(gaUartBaseAddr[instance], FALSE);
#else
        UART_HAL_SetTxDataRegEmptyIntCmd(gaUartBaseAddr[instance], FALSE);
#endif
        mUartTxCb[instance](instance);
    }

    if(mUartRxCb[instance] && rxSize && (pState->rxSize != rxSize))
    {
        mUartRxCb[instance](instance);
    }
}