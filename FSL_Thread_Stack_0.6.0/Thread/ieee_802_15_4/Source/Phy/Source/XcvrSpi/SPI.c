/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MCR20Drv.h
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


/*****************************************************************************
*                               INCLUDED HEADERS                            *
*---------------------------------------------------------------------------*
* Add to this section all the headers that this module needs to include.    *
* Note that it is not a good practice to include header files into header   *
* files, so use this section only if there is no other better solution.     *
*---------------------------------------------------------------------------*
*****************************************************************************/

#include "EmbeddedTypes.h"
#include "SPI.h"
#include "fsl_clock_manager.h"
#include "pin_mux.h"

#if BOARD_USE_DSPI
    #include "fsl_dspi_hal.h"
#else
    #include "fsl_spi_hal.h"
    #include "fsl_spi_master_driver.h"
#endif

/*****************************************************************************
*                             PRIVATE MACROS                                *
*---------------------------------------------------------------------------*
* Add to this section all the access macros, registers mappings, bit access *
* macros, masks, flags etc ...
*---------------------------------------------------------------------------*
*****************************************************************************/
#if BOARD_USE_DSPI
dspi_command_config_t mSpiCommand = {
    .isChipSelectContinuous = FALSE,
    .whichCtar = kDspiCtar0,
    .whichPcs = kDspiPcs0,
    .isEndOfQueue = TRUE,
    .clearTransferCount = TRUE
};
extern const uint32_t g_dspiBaseAddr[];
#endif

/*****************************************************************************/
/*****************************************************************************/
void spi_master_init(uint32_t instance)
{
#if BOARD_USE_DSPI
    uint32_t baseAddr = g_dspiBaseAddr[instance];
    const dspi_data_format_config_t cfg = {
        .bitsPerFrame = 8,
        .clkPolarity = kDspiClockPolarity_ActiveHigh,
        .clkPhase = kDspiClockPhase_FirstEdge,
        .direction = kDspiMsbFirst
    };

    configure_spi_pins(instance);
    /* Enable SPI clock */
    CLOCK_SYS_EnableSpiClock(instance);

    /* Initialize SPI module */
    DSPI_HAL_Init(baseAddr);
    DSPI_HAL_SetMasterSlaveMode(baseAddr, kDspiMaster);
    DSPI_HAL_SetContinuousSckCmd(baseAddr, FALSE);
    DSPI_HAL_SetPcsPolarityMode(baseAddr, kDspiPcs0, kDspiPcs_ActiveLow);
    DSPI_HAL_SetFifoCmd(baseAddr, TRUE, TRUE);

    DSPI_HAL_SetDataFormat(baseAddr, kDspiCtar0, &cfg);
    DSPI_HAL_SetDataFormat(baseAddr, kDspiCtar1, &cfg);

    DSPI_HAL_SetBaudRate(baseAddr, kDspiCtar0,  8000000, CLOCK_SYS_GetSpiFreq(instance));
    DSPI_HAL_SetBaudRate(baseAddr, kDspiCtar1, 16000000, CLOCK_SYS_GetSpiFreq(instance));

    DSPI_HAL_Enable(baseAddr);
    DSPI_HAL_StartTransfer(baseAddr);
#else
    uint32_t baseAddr = g_spiBaseAddr[instance];

    configure_spi_pins(instance);

    CLOCK_SYS_EnableSpiClock(instance);

    SPI_HAL_Init(baseAddr);
    SPI_HAL_SetMasterSlave(baseAddr, kSpiMaster);
    SPI_HAL_Enable(baseAddr);
#endif
}

/*****************************************************************************/
/*****************************************************************************/
void spi_master_configure_speed(uint32_t instance, uint32_t freq)
{
#if BOARD_USE_DSPI
    if( freq > 8000000 )
        mSpiCommand.whichCtar = kDspiCtar1;
    else
        mSpiCommand.whichCtar = kDspiCtar0;
#endif
}

/*****************************************************************************/
/*****************************************************************************/
void spi_master_transfer(uint32_t instance,
                         uint8_t * sendBuffer,
                         uint8_t * receiveBuffer,
                         size_t transferByteCount)
{
    volatile uint8_t dummy;
#if BOARD_USE_DSPI
    uint32_t baseAddr = g_dspiBaseAddr[instance];
#else
    uint32_t baseAddr = g_spiBaseAddr[instance];
#endif

    if( !transferByteCount )
        return;

    if( !sendBuffer && !receiveBuffer )
        return;

#if BOARD_USE_DSPI
    DSPI_HAL_SetFlushFifoCmd(baseAddr, true, true);
#endif

    while( transferByteCount-- )
    {
        if( sendBuffer )
        {
            dummy = *sendBuffer;
            sendBuffer++;
        }
        else
        {
            dummy = 0xFF;
        }

#if BOARD_USE_DSPI
        DSPI_HAL_WriteDataMastermodeBlocking(baseAddr, &mSpiCommand, dummy);
        dummy = DSPI_HAL_ReadData(baseAddr);
#else
  #if FSL_FEATURE_SPI_16BIT_TRANSFERS
        SPI_HAL_WriteDataBlocking(baseAddr, kSpi8BitMode, 0, dummy);
        while(!SPI_HAL_IsReadBuffFullPending(baseAddr));
        dummy = SPI_HAL_ReadDataLow(baseAddr);
  #else
        SPI_HAL_WriteDataBlocking(baseAddr, dummy);
        dummy = SPI_HAL_ReadData(baseAddr);
  #endif
#endif

        if( receiveBuffer )
        {
            *receiveBuffer = dummy;
            receiveBuffer++;
        }
    }
}

/*****************************************************************************/
/*****************************************************************************/
inline void spi_master_configure_serialization_lsb(uint32_t instance)
{
#if BOARD_USE_DSPI
//TODO DSPI version
#else
    BW_SPI_C1_LSBFE(g_spiBaseAddr[instance], kSpiLsbFirst);
#endif
}

/*****************************************************************************/
/*****************************************************************************/
inline void spi_master_configure_serialization_msb(uint32_t instance)
{
#if BOARD_USE_DSPI
//TODO DSPI version
#else
    BW_SPI_C1_LSBFE(g_spiBaseAddr[instance], kSpiMsbFirst);
#endif
}
