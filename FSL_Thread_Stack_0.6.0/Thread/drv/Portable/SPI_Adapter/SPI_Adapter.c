/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file SPI.c
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
#include "SPI_Adapter.h"
#include "pin_mux.h"
#include "panic.h"

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#ifndef gXcvrSpiInstance_c
#define gXcvrSpiInstance_c (0xFF)
#endif

/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */
typedef void (*pfSPIx_ISR_t)(void);


/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
static void SPIx_ISR(void);
#if BOARD_USE_DSPI
extern void DSPI_DRV_IRQHandler(uint32_t instance);
#else
extern void SPI_DRV_IRQHandler(uint32_t instance);
#endif

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
#if BOARD_USE_DSPI
static const dspi_master_user_config_t defaultSpiMasterCfg = {
    .isChipSelectContinuous = false,
    .isSckContinuous = false,
    .pcsPolarity = kDspiPcs_ActiveLow,
    .whichCtar = kDspiCtar0,
    .whichPcs = kDspiPcs0,
};
extern const IRQn_Type g_dspiIrqId[HW_SPI_INSTANCE_COUNT];
extern void * g_dspiStatePtr[HW_SPI_INSTANCE_COUNT];
#else
extern const IRQn_Type g_spiIrqId[HW_SPI_INSTANCE_COUNT];
extern void * g_spiStatePtr[HW_SPI_INSTANCE_COUNT];
#endif

static pfSPIx_TxCB_t mSpiTxCb[HW_SPI_INSTANCE_COUNT];
static pfSPIx_RxCB_t mSpiRxCb[HW_SPI_INSTANCE_COUNT];


/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */
void SpiMaster_Init(uint32_t instance, spiMasterState_t* pSpiState)
{
    IRQn_Type spiIrq;

    if( instance == gXcvrSpiInstance_c )
    {
        panic(0,(uint32_t)SpiMaster_Init,0,0);
    }

#if BOARD_USE_DSPI
    spiIrq = g_dspiIrqId[instance];
    DSPI_DRV_MasterInit(instance, pSpiState, &defaultSpiMasterCfg);
#else
    spiIrq = g_spiIrqId[instance];
    SPI_DRV_MasterInit (instance, pSpiState);
#endif

    configure_spi_pins(instance);
    mSpiTxCb[instance] = NULL;
    mSpiRxCb[instance] = NULL;
    /* Overwrite old ISR */
    OSA_InstallIntHandler(spiIrq, SPIx_ISR);
    /* set interrupt priority */
    NVIC_SetPriority(spiIrq, gSpi_IsrPrio_c >> (8 - __NVIC_PRIO_BITS));
    NVIC_ClearPendingIRQ(spiIrq);
    NVIC_EnableIRQ(spiIrq);
}

void SpiMaster_Configure(uint32_t instance, spiBusConfig_t* pConfig)
{
    uint32_t baudRate;
#if BOARD_USE_DSPI
    dspi_device_t dspiCfg = {
        .bitsPerSec                 = pConfig->bitsPerSec,
        .dataBusConfig.bitsPerFrame = pConfig->bitsPerFrame,
        .dataBusConfig.clkPhase     = (dspi_clock_phase_t)pConfig->clkPhase,
        .dataBusConfig.clkPolarity  = (dspi_clock_polarity_t)pConfig->clkPolarity,
        .dataBusConfig.direction    = (dspi_shift_direction_t )pConfig->direction
    };

    DSPI_DRV_MasterConfigureBus(instance, &dspiCfg, &baudRate);
#else
    spi_master_user_config_t spiCfg = {
        .bitsPerSec = pConfig->bitsPerSec,
        .polarity   = (spi_clock_polarity_t)pConfig->clkPolarity,
        .phase      = (spi_clock_phase_t)pConfig->clkPhase,
        .direction  = (spi_shift_direction_t)pConfig->direction
    };

    SPI_DRV_MasterConfigureBus (instance, &spiCfg, &baudRate);
#endif
    (void)baudRate;
}

void SpiMaster_SyncTransfer(uint32_t instance, uint8_t* pTxData, uint8_t* pRxData, uint16_t size)
{
#if BOARD_USE_DSPI
    DSPI_DRV_MasterTransferBlocking(instance, NULL, pTxData, pRxData, size, OSA_WAIT_FOREVER);
#else
    SPI_DRV_MasterTransferBlocking(instance, NULL, pTxData, pRxData, size, OSA_WAIT_FOREVER);
#endif
}

void SpiMaster_AsyncTransfer(uint32_t instance, uint8_t* pTxData, uint8_t* pRxData, uint16_t size)
{
#if BOARD_USE_DSPI
    DSPI_DRV_MasterTransfer(instance, NULL, pTxData, pRxData, size);
#else
    SPI_DRV_MasterTransfer(instance, NULL, pTxData, pRxData, size);
#endif
}

uint8_t SpiMaster_GetStatus(uint32_t instance)
{
#if BOARD_USE_DSPI
    if( kStatus_DSPI_Busy == DSPI_DRV_MasterGetTransferStatus(instance, NULL) )
        return 1;
#else
    if( kStatus_SPI_Busy == SPI_DRV_MasterGetTransferStatus(instance, NULL) )
        return 1;
#endif
    return 0;
}

void SpiSlave_Init(uint32_t instance, spiSlaveStare_t* pSpiState, pfSPIx_RxCB_t pfIn, pfSPIx_TxCB_t pfOut)
{
    IRQn_Type spiIrq;
#if BOARD_USE_DSPI
    dspi_slave_user_config_t spiSlaveCfg = {
        .dataConfig.bitsPerFrame = 8,
        .dataConfig.clkPhase     = gSpiClkPhase_FirstEdge_d,
        .dataConfig.clkPolarity  = gSpiClk_ActiveHigh_d,
        .dataConfig.direction    = gSpiMsbFirst_d,
    };

    spiIrq = g_dspiIrqId[instance];
    DSPI_DRV_SlaveInit(instance, pSpiState, &spiSlaveCfg);
#else
    spi_slave_user_config_t spiSlaveCfg = {
        .phase                   = kSpiClockPhase_FirstEdge,
        .polarity                = kSpiClockPolarity_ActiveHigh,
        .direction               = kSpiMsbFirst
    };

    spiIrq = g_spiIrqId[instance];
    SPI_DRV_SlaveInit(instance, pSpiState, &spiSlaveCfg);
#endif
    configure_spi_pins(instance);
    mSpiTxCb[instance] = pfOut;
    mSpiRxCb[instance] = pfIn;
    /* Overwrite old ISR */
    OSA_InstallIntHandler(spiIrq, SPIx_ISR);
    /* set interrupt priority */
    NVIC_SetPriority(spiIrq, gSpi_IsrPrio_c >> (8 - __NVIC_PRIO_BITS));
    NVIC_ClearPendingIRQ(spiIrq);
    NVIC_EnableIRQ(spiIrq);
}


/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************* */
static void SPIx_ISR(void)
{
    uint32_t instance = ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) >> SCB_ICSR_VECTACTIVE_Pos) - 16;
    uint32_t rxSize, txSize;
    spiMasterState_t *pState;

#if BOARD_USE_DSPI
    instance = instance - g_dspiIrqId[0];
    pState = (spiMasterState_t*)g_dspiStatePtr[instance];
#else
    instance = instance - g_spiIrqId[0];
    pState = (spiMasterState_t*)g_spiStatePtr[instance];
#endif
    txSize = pState->remainingSendByteCount;
    rxSize = pState->remainingReceiveByteCount;

#if BOARD_USE_DSPI
    DSPI_DRV_IRQHandler(instance);
#else
    SPI_DRV_IRQHandler(instance);
#endif

    if( mSpiTxCb[instance] && txSize && !(pState->remainingSendByteCount) )
    {
        mSpiTxCb[instance](instance);
    }

    if(mSpiRxCb[instance] && rxSize && (rxSize != pState->remainingReceiveByteCount))
    {
        mSpiRxCb[instance](instance);
    }
}
