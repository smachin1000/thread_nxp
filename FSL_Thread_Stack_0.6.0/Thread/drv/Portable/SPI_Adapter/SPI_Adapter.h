/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file SPI.h
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

#ifndef __SPI_ADAPTER_H__
#define __SPI_ADAPTER_H__

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "board.h"
//#include "fsl_device_registers.h"

#if BOARD_USE_DSPI
  #include "fsl_dspi_master_driver.h"
  #include "fsl_dspi_slave_driver.h"

#else
  #include "fsl_spi_master_driver.h"
  #include "fsl_spi_slave_driver.h"
#endif

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#define gSpi_IsrPrio_c (0x80)

#if BOARD_USE_DSPI
  #define spiMasterState_t dspi_master_state_t
  #define spiSlaveStare_t  dspi_slave_state_t
  #define spiStatus_t      dspi_status_t

  #define gSpiClkPhase_FirstEdge_d kDspiClockPhase_FirstEdge
  #define gSpiClkPhase_SecondEdge_d kDspiClockPhase_SecondEdge
  #define gSpiClk_ActiveHigh_d kDspiClockPolarity_ActiveHigh
  #define gSpiClk_ActiveLow_d kDspiClockPolarity_ActiveLow
  #define gSpiMsbFirst_d kDspiMsbFirst
  #define gSpiLsbFirst_d kDspiLsbFirst

#else
  #define spiMasterState_t spi_master_state_t
  #define spiSlaveStare_t  spi_slave_state_t
  #define spiStatus_t      spi_status_t

  #define gSpiClkPhase_FirstEdge_d kSpiClockPhase_FirstEdge
  #define gSpiClkPhase_SecondEdge_d kSpiClockPhase_SecondEdge
  #define gSpiClk_ActiveHigh_d kSpiClockPolarity_ActiveHigh 
  #define gSpiClk_ActiveLow_d kSpiClockPolarity_ActiveLow
  #define gSpiMsbFirst_d kSpiMsbFirst 
  #define gSpiLsbFirst_d kSpiLsbFirst
#endif

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
typedef void (*pfSPIx_TxCB_t)(uint32_t);
typedef void (*pfSPIx_RxCB_t)(uint32_t);

typedef struct spiBusConfig_tag{
    uint32_t bitsPerSec;
    uint8_t  bitsPerFrame;
    uint8_t  clkPolarity;
    uint8_t  clkPhase;
    uint8_t  direction;
}spiBusConfig_t;

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
void SpiSlave_Init          (uint32_t instance, spiSlaveStare_t* pSpiState, pfSPIx_RxCB_t pfIn, pfSPIx_TxCB_t pfOut);
void SpiMaster_Init         (uint32_t instance, spiMasterState_t* pSpiState);
void SpiMaster_Configure    (uint32_t instance, spiBusConfig_t* pConfig);
void SpiMaster_SyncTransfer (uint32_t instance, uint8_t* pTxData, uint8_t* pRxData, uint16_t size);
void SpiMaster_AsyncTransfer(uint32_t instance, uint8_t* pTxData, uint8_t* pRxData, uint16_t size);
uint8_t SpiMaster_GetStatus (uint32_t instance);

#endif /* __SPI_ADAPTER_H__ */