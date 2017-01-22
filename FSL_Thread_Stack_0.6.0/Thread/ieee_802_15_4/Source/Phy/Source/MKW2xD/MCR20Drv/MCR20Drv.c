/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MCR20Drv.c
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
*---------------------------------------------------------------------------*
*****************************************************************************/

#include "MCR20Drv.h"
#include "MCR20Reg.h"

#include "SPI.h"
#include "fsl_gpio_driver.h"
#include "fsl_os_abstraction.h"


/*****************************************************************************
*                               PRIVATE VARIABLES                           *
*---------------------------------------------------------------------------*
* Add to this section all the variables and constants that have local       *
* (file) scope.                                                             *
* Each of this declarations shall be preceded by the 'static' keyword.      *
* These variables / constants cannot be accessed outside this module.       *
*---------------------------------------------------------------------------*
*****************************************************************************/
static uint32_t mPhyIrqDisableCnt = 1;

/* GPIO configuration */
const gpio_output_pin_user_config_t mXcvrSpiCsCfg = {
    .pinName = kGpioXcvrSpiCsPin_d,
    .config.outputLogic = 1,
    .config.slewRate = kPortFastSlewRate,
    .config.isOpenDrainEnabled = false,
    .config.driveStrength = kPortLowDriveStrength,
};

const gpio_input_pin_user_config_t mXcvrIrqPinCfg = {
    .pinName = kGpioXcvrIrqPin_d,
    .config.isPullEnable = false,
    .config.pullSelect = kPortPullDown,
    .config.isPassiveFilterEnabled = false,
    .config.interrupt = kPortIntDisabled
};


/*****************************************************************************
*                               PUBLIC VARIABLES                            *
*---------------------------------------------------------------------------*
* Add to this section all the variables and constants that have global      *
* (project) scope.                                                          *
* These variables / constants can be accessed outside this module.          *
* These variables / constants shall be preceded by the 'extern' keyword in  *
* the interface header.                                                     *
*---------------------------------------------------------------------------*
*****************************************************************************/

/*****************************************************************************
*                           PRIVATE FUNCTIONS PROTOTYPES                    *
*---------------------------------------------------------------------------*
* Add to this section all the functions prototypes that have local (file)   *
* scope.                                                                    *
* These functions cannot be accessed outside this module.                   *
* These declarations shall be preceded by the 'static' keyword.             *
*---------------------------------------------------------------------------*
*****************************************************************************/

/*****************************************************************************
*                                PRIVATE FUNCTIONS                          *
*---------------------------------------------------------------------------*
* Add to this section all the functions that have local (file) scope.       *
* These functions cannot be accessed outside this module.                   *
* These definitions shall be preceded by the 'static' keyword.              *
*---------------------------------------------------------------------------*
*****************************************************************************/


/*****************************************************************************
*                             PUBLIC FUNCTIONS                              *
*---------------------------------------------------------------------------*
* Add to this section all the functions that have global (project) scope.   *
* These functions can be accessed outside this module.                      *
* These functions shall have their declarations (prototypes) within the     *
* interface header file and shall be preceded by the 'extern' keyword.      *
*---------------------------------------------------------------------------*
*****************************************************************************/

/*---------------------------------------------------------------------------
* Name: MCR20Drv_Init
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_Init
(
void
)
{
    spi_master_init(gXcvrSpiInstance_c);
    spi_master_configure_speed(gXcvrSpiInstance_c, 8000000);
    
    /* Override SPI CS pin function. Set pin as GPIO */
    PORT_HAL_SetMuxMode(g_portBaseAddr[GPIO_EXTRACT_PORT(kGpioXcvrSpiCsPin_d)],
                        GPIO_EXTRACT_PIN(kGpioXcvrSpiCsPin_d), 
                        kPortMuxAsGpio);    
    GPIO_DRV_OutputPinInit(&mXcvrSpiCsCfg);
    gXcvrDeassertCS_d();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_DirectAccessSPIWrite
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_DirectAccessSPIWrite
(
uint8_t address,
uint8_t value
)
{
    uint16_t txData;
    
    ProtectFromMCR20Interrupt();
    
    spi_master_configure_speed(gXcvrSpiInstance_c, 16000000);
    
    gXcvrAssertCS_d();
    
    txData = (address & TransceiverSPI_DirectRegisterAddressMask);
    txData |= value << 8;
    
    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t *)&txData, NULL, sizeof(txData));
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_DirectAccessSPIMultiByteWrite
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_DirectAccessSPIMultiByteWrite
(
uint8_t startAddress,
uint8_t * byteArray,
uint8_t numOfBytes
)
{
    uint8_t txData;
    
    if( (numOfBytes == 0) || (byteArray == NULL) )
    {
        return;
    }
    
    ProtectFromMCR20Interrupt();

    spi_master_configure_speed(gXcvrSpiInstance_c, 16000000);

    gXcvrAssertCS_d();
    
    txData = (startAddress & TransceiverSPI_DirectRegisterAddressMask);

    spi_master_transfer(gXcvrSpiInstance_c, &txData, NULL, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, byteArray, NULL, numOfBytes);
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_PB_SPIByteWrite
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_PB_SPIByteWrite
(
uint8_t address,
uint8_t value
)
{
    uint32_t txData;
    
    ProtectFromMCR20Interrupt();

    spi_master_configure_speed(gXcvrSpiInstance_c, 16000000);

    gXcvrAssertCS_d();
    
    txData  = TransceiverSPI_WriteSelect            |
        TransceiverSPI_PacketBuffAccessSelect |
            TransceiverSPI_PacketBuffByteModeSelect;
    txData |= (address) << 8;
    txData |= (value)   << 16;
    
    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t*)&txData, NULL, 3);
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_PB_SPIBurstWrite
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_PB_SPIBurstWrite
(
uint8_t * byteArray,
uint8_t numOfBytes
)
{
    uint8_t txData;
    
    if( (numOfBytes == 0) || (byteArray == NULL) )
    {
        return;
    }
    
    ProtectFromMCR20Interrupt();

    spi_master_configure_speed(gXcvrSpiInstance_c, 16000000);

    gXcvrAssertCS_d();
    
    txData = TransceiverSPI_WriteSelect            |
        TransceiverSPI_PacketBuffAccessSelect |
            TransceiverSPI_PacketBuffBurstModeSelect;
    
    spi_master_transfer(gXcvrSpiInstance_c, &txData, NULL, 1);
    spi_master_transfer(gXcvrSpiInstance_c, byteArray, NULL, numOfBytes);
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_DirectAccessSPIRead
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/

uint8_t MCR20Drv_DirectAccessSPIRead
(
uint8_t address
)
{
    uint8_t txData;
    uint8_t rxData;
    
    ProtectFromMCR20Interrupt();
    
    spi_master_configure_speed(gXcvrSpiInstance_c, 8000000);
    
    gXcvrAssertCS_d();
    
    txData = (address & TransceiverSPI_DirectRegisterAddressMask) |
        TransceiverSPI_ReadSelect;
    
    spi_master_transfer(gXcvrSpiInstance_c, &txData, NULL, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, NULL, &rxData, sizeof(rxData));
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
    
    return rxData;
    
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_DirectAccessSPIMultyByteRead
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
uint8_t MCR20Drv_DirectAccessSPIMultiByteRead
(
uint8_t startAddress,
uint8_t * byteArray,
uint8_t numOfBytes
)
{
    uint8_t  txData;
    uint8_t  phyIRQSTS1;
    
    if( (numOfBytes == 0) || (byteArray == NULL) )
    {
        return 0;
    }
    
    ProtectFromMCR20Interrupt();

    spi_master_configure_speed(gXcvrSpiInstance_c, 8000000);

    gXcvrAssertCS_d();
    
    txData = (startAddress & TransceiverSPI_DirectRegisterAddressMask) |
        TransceiverSPI_ReadSelect;

    spi_master_transfer(gXcvrSpiInstance_c, &txData, &phyIRQSTS1, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, NULL, byteArray, numOfBytes);
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
    
    return phyIRQSTS1;
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_PB_SPIBurstRead
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
uint8_t MCR20Drv_PB_SPIBurstRead
(
uint8_t * byteArray,
uint8_t numOfBytes
)
{
    uint8_t  txData;
    uint8_t  phyIRQSTS1;
    
    if( (numOfBytes == 0) || (byteArray == NULL) )
    {
        return 0;
    }
    
    ProtectFromMCR20Interrupt();

    spi_master_configure_speed(gXcvrSpiInstance_c, 8000000);

    gXcvrAssertCS_d();
    
    txData = TransceiverSPI_ReadSelect |
        TransceiverSPI_PacketBuffAccessSelect |
            TransceiverSPI_PacketBuffBurstModeSelect;
    
    spi_master_transfer(gXcvrSpiInstance_c, &txData, &phyIRQSTS1, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, NULL, byteArray, numOfBytes);
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
    
    return phyIRQSTS1;
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IndirectAccessSPIWrite
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_IndirectAccessSPIWrite
(
uint8_t address,
uint8_t value
)
{
    uint32_t  txData;
    
    ProtectFromMCR20Interrupt();

    spi_master_configure_speed(gXcvrSpiInstance_c, 16000000);

    gXcvrAssertCS_d();
    
    txData = TransceiverSPI_IARIndexReg;
    txData |= (address) << 8;
    txData |= (value)   << 16;

    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t*)&txData, NULL, 3);
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IndirectAccessSPIMultiByteWrite
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_IndirectAccessSPIMultiByteWrite
(
uint8_t startAddress,
uint8_t * byteArray,
uint8_t numOfBytes
)
{
    uint16_t  txData;
    
    if( (numOfBytes == 0) || (byteArray == NULL) )
    {
        return;
    }
    
    ProtectFromMCR20Interrupt();
    
    spi_master_configure_speed(gXcvrSpiInstance_c, 16000000);
    
    gXcvrAssertCS_d();
    
    txData = TransceiverSPI_IARIndexReg;
    txData |= (startAddress)  << 8;
    
    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t*)&txData, NULL, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t*)byteArray, NULL, numOfBytes);
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IndirectAccessSPIRead
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
uint8_t MCR20Drv_IndirectAccessSPIRead
(
uint8_t address
)
{
    uint16_t  txData;
    uint8_t   rxData;
    
    ProtectFromMCR20Interrupt();

    spi_master_configure_speed(gXcvrSpiInstance_c, 8000000);
    
    gXcvrAssertCS_d();
    
    txData = TransceiverSPI_IARIndexReg | TransceiverSPI_ReadSelect;
    txData |= (address) << 8;

    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t*)&txData, NULL, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, NULL, &rxData, sizeof(rxData));
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
    
    return rxData;
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IndirectAccessSPIMultiByteRead
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_IndirectAccessSPIMultiByteRead
(
uint8_t startAddress,
uint8_t * byteArray,
uint8_t numOfBytes
)
{
    uint16_t  txData;
    
    if( (numOfBytes == 0) || (byteArray == NULL) )
    {
        return;
    }
    
    ProtectFromMCR20Interrupt();

    spi_master_configure_speed(gXcvrSpiInstance_c, 8000000);

    gXcvrAssertCS_d();
    
    txData = (TransceiverSPI_IARIndexReg | TransceiverSPI_ReadSelect);
    txData |= (startAddress) << 8;

    spi_master_transfer(gXcvrSpiInstance_c, (uint8_t*)&txData, NULL, sizeof(txData));
    spi_master_transfer(gXcvrSpiInstance_c, NULL, byteArray, numOfBytes);
    
    gXcvrDeassertCS_d();
    UnprotectFromMCR20Interrupt();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IRQ_PortConfig
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_IRQ_PortConfig
(
void
)
{
    PORT_HAL_SetMuxMode(g_portBaseAddr[GPIO_EXTRACT_PORT(kGpioXcvrIrqPin_d)],
                        GPIO_EXTRACT_PIN(kGpioXcvrIrqPin_d), 
                        kPortMuxAsGpio);  
    GPIO_DRV_InputPinInit(&mXcvrIrqPinCfg);
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IsIrqPending
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
uint32_t  MCR20Drv_IsIrqPending
(
void
)
{
    if( GPIO_DRV_ReadPinInput(kGpioXcvrIrqPin_d) )
    {
        return FALSE;
    }
    return TRUE;
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IRQ_Disable
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_IRQ_Disable
(
void
)
{
    OSA_EnterCritical(kCriticalDisableInt);

    if( mPhyIrqDisableCnt == 0 )
    {
        PORT_HAL_SetPinIntMode(g_portBaseAddr[GPIO_EXTRACT_PORT(kGpioXcvrIrqPin_d)], 
                               GPIO_EXTRACT_PIN(kGpioXcvrIrqPin_d), 
                               kPortIntDisabled);
    }
    
    mPhyIrqDisableCnt++;

    OSA_ExitCritical(kCriticalDisableInt);
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IRQ_Enable
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_IRQ_Enable
(
void
)
{
    OSA_EnterCritical(kCriticalDisableInt);

    if( mPhyIrqDisableCnt )
    {
        mPhyIrqDisableCnt--;
        
        if( mPhyIrqDisableCnt == 0 )
        {
            PORT_HAL_SetPinIntMode(g_portBaseAddr[GPIO_EXTRACT_PORT(kGpioXcvrIrqPin_d)], 
                                   GPIO_EXTRACT_PIN(kGpioXcvrIrqPin_d), 
                                   kPortIntLogicZero);
        }
    }

    OSA_ExitCritical(kCriticalDisableInt);
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IRQ_IsEnabled
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
uint32_t MCR20Drv_IRQ_IsEnabled
(
void
)
{
    port_interrupt_config_t mode;
    
    mode = PORT_HAL_GetPinIntMode(g_portBaseAddr[GPIO_EXTRACT_PORT(kGpioXcvrIrqPin_d)], 
                                  GPIO_EXTRACT_PIN(kGpioXcvrIrqPin_d));
    return (mode != kPortIntDisabled);
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_IRQ_Clear
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_IRQ_Clear
(
void
)
{
    GPIO_DRV_ClearPinIntFlag(kGpioXcvrIrqPin_d);
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_RST_Assert
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_RST_B_Assert
(
void
)
{
    GPIO_DRV_ClearPinOutput(kGpioXcvrResetPin_d);
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_RST_Deassert
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_RST_B_Deassert
(
void
)
{
    GPIO_DRV_SetPinOutput(kGpioXcvrResetPin_d);
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_SoftRST_Assert
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_SoftRST_Assert
(
void
)
{
    MCR20Drv_IndirectAccessSPIWrite(SOFT_RESET, (0x80));
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_SoftRST_Deassert
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_SoftRST_Deassert
(
void
)
{
    MCR20Drv_IndirectAccessSPIWrite(SOFT_RESET, (0x00));
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_Soft_RESET
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_Soft_RESET
(
void
)
{
    //assert SOG_RST
    MCR20Drv_IndirectAccessSPIWrite(SOFT_RESET, (0x80));
    
    //deassert SOG_RST
    MCR20Drv_IndirectAccessSPIWrite(SOFT_RESET, (0x00));
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_RESET
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_RESET
(
void
)
{
    volatile uint32_t delay = 1000;
    //assert RST_B
    MCR20Drv_RST_B_Assert();
    
    // TODO
    while(delay--);
    
    //deassert RST_B
    MCR20Drv_RST_B_Deassert();
}

/*---------------------------------------------------------------------------
* Name: MCR20Drv_Set_CLK_OUT_Freq
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
void MCR20Drv_Set_CLK_OUT_Freq
(
uint8_t freqDiv
)
{
    uint8_t clkOutCtrlReg = (freqDiv & cCLK_OUT_DIV_Mask) | cCLK_OUT_EN | cCLK_OUT_EXTEND;

    if(freqDiv == gCLK_OUT_FREQ_DISABLE)
    {
        clkOutCtrlReg = (cCLK_OUT_EXTEND | gCLK_OUT_FREQ_4_MHz); //reset value with clock out disabled
    }

    MCR20Drv_DirectAccessSPIWrite((uint8_t) CLK_OUT_CTRL, clkOutCtrlReg);
}
