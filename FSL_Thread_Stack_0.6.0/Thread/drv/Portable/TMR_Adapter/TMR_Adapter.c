/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file LED.c
* LED implementation file for ARM CORTEX-M4 processor
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

#include "TMR_Adapter.h"

#include "fsl_lptmr_driver.h"
#include "fsl_lptmr_hal.h"

#include "fsl_os_abstraction.h"
#include "fsl_clock_manager.h"
#include "pin_mux.h"


/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
static void LPTMR_ISR(void);

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
static lptmr_state_t gLptmrUserState;
extern const IRQn_Type g_lptmrIrqId[HW_LPTMR_INSTANCE_COUNT];

#if defined(HW_FTM_INSTANCE_COUNT)
extern const IRQn_Type g_ftmIrqId[];
#else
extern const IRQn_Type g_tpmIrqId[];
#endif

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */
void StackTimer_Init(void (*cb)(void))
{
    IRQn_Type irqId;
#if defined(HW_FTM_INSTANCE_COUNT)
    uint32_t ftmBaseAddr = g_ftmBaseAddr[gStackTimerInstance_c];

    CLOCK_SYS_EnableFtmClock(gStackTimerInstance_c);

    FTM_HAL_Reset(ftmBaseAddr);
    FTM_HAL_Enable(ftmBaseAddr, TRUE);
    FTM_HAL_SetClockPs(ftmBaseAddr, kFtmDividedBy128);
    FTM_HAL_SetClockSource(ftmBaseAddr, kClock_source_FTM_None);
    FTM_HAL_SetTofFreq(ftmBaseAddr, 0);
    FTM_HAL_SetWriteProtectionCmd(ftmBaseAddr, 0);
    FTM_HAL_SetCounterInitVal(ftmBaseAddr, 0);
    FTM_HAL_SetCounter(ftmBaseAddr, 0);
    FTM_HAL_SetMod(ftmBaseAddr, 0xFFFF);
    /* Configure channel to toggle on compare match */
    FTM_HAL_SetChnMSnBAMode(ftmBaseAddr, gStackTimerChannel_c, 1);
    FTM_HAL_SetChnCountVal(ftmBaseAddr, gStackTimerChannel_c, 0x01);

    /* Install ISR */
    irqId = g_ftmIrqId[gStackTimerInstance_c];
    FTM_HAL_EnableTimerOverflowInt(ftmBaseAddr);
    FTM_HAL_EnableChnInt(ftmBaseAddr, gStackTimerChannel_c);
#else
    uint32_t tpmBaseAddr = g_tpmBaseAddr[gStackTimerInstance_c];

    CLOCK_SYS_EnableTpmClock(gStackTimerInstance_c);

    TPM_HAL_Reset(tpmBaseAddr, gStackTimerInstance_c);
    TPM_DRV_SetClock((uint8_t)gStackTimerInstance_c, kTpmClockSourceModuleOSCERCLK, kTpmDividedBy128);

    TPM_HAL_SetClockMode(tpmBaseAddr, kTpmClockSourceNoneClk);
    TPM_HAL_ClearCounter(tpmBaseAddr);
    TPM_HAL_SetMod(tpmBaseAddr, 0xFFFF); //allready done by TPM_HAL_Reset()
    /* Configure channel to Software compare; output pin not used */
    TPM_HAL_SetChnMsnbaElsnbaVal(tpmBaseAddr, gStackTimerChannel_c, BM_TPM_CnSC_MSA);
    TPM_HAL_SetChnCountVal(tpmBaseAddr, gStackTimerChannel_c, 0x01);

    /* Install ISR */
    irqId = g_tpmIrqId[gStackTimerInstance_c];
    TPM_HAL_EnableTimerOverflowInt(tpmBaseAddr);
    TPM_HAL_EnableChnInt(tpmBaseAddr, gStackTimerChannel_c);
#endif
    /* Overwrite old ISR */
    OSA_InstallIntHandler(irqId, cb);
    /* set interrupt priority */
    NVIC_SetPriority(irqId, gStackTimer_IsrPrio_c >> (8 - __NVIC_PRIO_BITS));
    NVIC_ClearPendingIRQ(irqId);
    NVIC_EnableIRQ(irqId);
}

void StackTimer_Enable(void)
{
#if defined(HW_FTM_INSTANCE_COUNT)
    FTM_HAL_SetClockSource(g_ftmBaseAddr[gStackTimerInstance_c], kClock_source_FTM_SystemClk);
#else
    TPM_HAL_SetClockMode(g_tpmBaseAddr[gStackTimerInstance_c], kTpmClockSourceModuleClk);
#endif
}

void StackTimer_Disable(void)
{
#if defined(HW_FTM_INSTANCE_COUNT)
    FTM_HAL_SetClockSource(g_ftmBaseAddr[gStackTimerInstance_c], kClock_source_FTM_None);
#else
    TPM_HAL_SetClockMode(g_tpmBaseAddr[gStackTimerInstance_c], kTpmClockSourceNoneClk);
#endif
}

uint32_t StackTimer_GetInputFrequency(void)
{
    uint32_t prescaller;
    uint32_t refClk;

#if defined(HW_FTM_INSTANCE_COUNT)
    CLOCK_SYS_GetFreq(kBusClock, &refClk);
    prescaller = FTM_HAL_GetClockPs(g_ftmBaseAddr[gStackTimerInstance_c]);
#else
    refClk = CLOCK_SYS_GetTpmFreq(gStackTimerInstance_c);
    prescaller = TPM_HAL_GetClockDiv(g_tpmBaseAddr[gStackTimerInstance_c]);
#endif
    return refClk / (1 << prescaller);
}

uint32_t StackTimer_GetCounterValue(void)
{
#if defined(HW_FTM_INSTANCE_COUNT)
    return FTM_HAL_GetCounter(g_ftmBaseAddr[gStackTimerInstance_c]);
#else
    return TPM_HAL_GetCounterVal(g_tpmBaseAddr[gStackTimerInstance_c]);
#endif
}

void StackTimer_SetOffsetTicks(uint32_t offset)
{
#if defined(HW_FTM_INSTANCE_COUNT)
    FTM_HAL_SetChnCountVal(g_ftmBaseAddr[gStackTimerInstance_c], gStackTimerChannel_c, offset);
#else
    TPM_HAL_SetChnCountVal(g_tpmBaseAddr[gStackTimerInstance_c], gStackTimerChannel_c, offset);
#endif
}

void StackTimer_ClearIntFlag(void)
{
#if defined(HW_FTM_INSTANCE_COUNT)
    uint32_t ftmBaseAddr = g_ftmBaseAddr[gStackTimerInstance_c];

    if( FTM_HAL_HasChnEventOccurred(ftmBaseAddr, gStackTimerChannel_c) )
    {
        BW_FTM_CnSC_CHF(ftmBaseAddr, gStackTimerChannel_c, 0);
    }
    if( FTM_HAL_IsOverflowIntEnabled(ftmBaseAddr) )
    {
        FTM_HAL_ClearTimerOverflow(ftmBaseAddr);
    }
#else
    TPM_DRV_IRQHandler(gStackTimerInstance_c);
#endif
}


void LPTMR_Init(void (*cb)(void))
{
    const lptmr_user_config_t userConfig = {
        .timerMode = kLptmrTimerModeTimeCounter,
        .prescalerClockSource = kClockLptmrSrcEr32kClk,
        .prescalerValue = kLptmrPrescalerDivide32GlichFiltch16,
        .freeRunningEnable = 1,
        .isInterruptEnabled = 1,
        .pinSelect = kLptmrPinSelectInput0,
        .pinPolarity = kLptmrPinPolarityActiveHigh
    };
    
    LPTMR_DRV_Init(gLptmrInstance_c, &userConfig, &gLptmrUserState);
    LPTMR_DRV_InstallCallback(gLptmrInstance_c, cb);
    LPTMR_DRV_Stop(gLptmrInstance_c);
    /* Overwrite old ISR */
    OSA_InstallIntHandler(g_lptmrIrqId[gLptmrInstance_c], LPTMR_ISR);
}

void LPTMR_Enable(void)
{
    LPTMR_DRV_Start(gLptmrInstance_c);
}

void LPTMR_Disable(void)
{
    LPTMR_DRV_Stop(gLptmrInstance_c);
}

uint32_t LPTMR_GetInputFrequency(void)
{
    return gLptmrUserState.prescalerClockHz;
}

uint32_t LPTMR_GetCounterValue(void)
{
    uint32_t baseAddr = g_lptmrBaseAddr[gLptmrInstance_c];
    
    return LPTMR_HAL_GetCounterValue(baseAddr);
}

void LPTMR_SetOffsetTicks(uint32_t offset)
{
    uint32_t baseAddr = g_lptmrBaseAddr[gLptmrInstance_c];
    
    LPTMR_HAL_SetCompareValue(baseAddr, offset);
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************* */
static void LPTMR_ISR(void)
{
    LPTMR_DRV_IRQHandler(gLptmrInstance_c);
}