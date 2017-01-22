/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file ASP.c
* This is the source file for the ASP module.
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

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"

#include "Phy.h"
#include "PhyInterface.h"
#include "MpmInterface.h"
#include "AspInterface.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "Panic.h"

#include "MCR20Drv.h"
#include "MCR20Reg.h"

#if gFsciIncluded_c
#include "FsciInterface.h"
#include "FsciCommands.h"
#include "FsciCommunication.h"
#endif

#if gAspCapability_d

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
#define mFAD_THR_ResetValue         0x82
#define mANT_AGC_CTRL_ResetValue    0x40

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
//MCR20 DTS modes
enum {
  gDtsNormal_c,
  gDtsTxOne_c,
  gDtsTxZero_c,
  gDtsTx2Mhz_c,
  gDtsTx200Khz_c,
  gDtsTx1MbpsPRBS9_c,
  gDtsTxExternalSrc_c,
  gDtsTxRandomSeq_c
};

/************************************************************************************
*************************************************************************************
* Private functions prototype
*************************************************************************************
************************************************************************************/
phyStatus_t AspSetDtsMode( uint8_t mode );
phyStatus_t AspEnableBER( void );
void AspDisableBER( void );

#if gFsciIncluded_c
static void fsciAspReqHandler(void *pData, void* param, uint32_t interfaceId);
static void AspSapMonitor(void *pData, void* param, uint32_t interfaceId);
#endif

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
//2405   2410    2415    2420    2425    2430    2435    2440    2445    2450    2455    2460    2465    2470    2475    2480
static const uint16_t asp_pll_frac[16] = {0x2400, 0x4C00, 0x7400, 0x9C00, 0xC400, 0xEC00, 0x1400, 0x3C00, 0x6400, 0x8C00, 0xB400, 0xDC00, 0x0400, 0x2C00, 0x5400, 0x7C00};

#if gFsciIncluded_c
static uint8_t mAspFsciBinding[gPhyInstancesCnt_c];
#endif

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  Initialize the ASP module
*
* \param[in]  phyInstance The instance of the PHY
* \param[in]  interfaceId The Serial Manager interface used
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 0
void ASP_Init( instanceId_t phyInstance, uint8_t interfaceId )
{
#if gFsciIncluded_c
    if( phyInstance < gPhyInstancesCnt_c )
    {
        mAspFsciBinding[phyInstance] = interfaceId;
        FSCI_RegisterOpGroup( gFSCI_AppAspOpcodeGroup_c, gFsciMonitorMode_c, fsciAspReqHandler, NULL, gAspInterfaceId);
        FSCI_RegisterOpGroup( gFSCI_AspSapId_c,          gFsciMonitorMode_c, AspSapMonitor,     NULL, gAspInterfaceId);
    }
#endif
}

/*! *********************************************************************************
* \brief  ASP SAP handler.
*
* \param[in]  pMsg        Pointer to the request message
* \param[in]  instanceId  The instance of the PHY
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 1
AspStatus_t APP_ASP_SapHandler(AppToAspMessage_t *pMsg, instanceId_t instanceId)
{
    AspStatus_t status = gAspSuccess_c;
#if gFsciIncluded_c
    FSCI_Monitor( gFSCI_AspSapId_c,
                  pMsg,
                  NULL,
                  gAspInterfaceId );
#endif
    switch( pMsg->msgType )
    {
    case aspMsgTypeGetTimeReq_c:
        Asp_GetTimeReq((uint32_t*)&pMsg->msgData.aspGetTimeReq.time);
        break;
    case aspMsgTypeXcvrWriteReq_c:
        status = Asp_XcvrWriteReq( pMsg->msgData.aspXcvrData.mode,
                                   pMsg->msgData.aspXcvrData.addr,
                                   pMsg->msgData.aspXcvrData.len,
                                   pMsg->msgData.aspXcvrData.data);
        break;
    case aspMsgTypeXcvrReadReq_c:
        status = Asp_XcvrReadReq( pMsg->msgData.aspXcvrData.mode,
                                  pMsg->msgData.aspXcvrData.addr,
                                  pMsg->msgData.aspXcvrData.len,
                                  pMsg->msgData.aspXcvrData.data);
        break;
    case aspMsgTypeSetFADState_c:
        status = Asp_SetFADState(pMsg->msgData.aspFADState);
        break;
    case aspMsgTypeSetFADThreshold_c:
        status = Asp_SetFADThreshold(pMsg->msgData.aspFADThreshold);
        break;
    case aspMsgTypeSetANTXState_c:
        status = Asp_SetANTXState(pMsg->msgData.aspANTXState);
        break;
    case aspMsgTypeGetANTXState_c:
        *((uint8_t*)&status) = Asp_GetANTXState();
        break;
    case aspMsgTypeSetPowerLevel_c:
        status = Asp_SetPowerLevel(pMsg->msgData.aspSetPowerLevelReq.powerLevel);
        break;
    case aspMsgTypeGetPowerLevel_c:
        *((uint8_t*)&status) = Asp_GetPowerLevel(); //remove compiler warning
        break;
    case aspMsgTypeTelecSetFreq_c:
        status = ASP_TelecSetFreq(pMsg->msgData.aspTelecsetFreq.channel);
        break;
    case aspMsgTypeTelecSendRawData_c:
        status = ASP_TelecSendRawData((uint8_t*)&pMsg->msgData.aspTelecSendRawData);
        break;
    case aspMsgTypeTelecTest_c:
        status = ASP_TelecTest(pMsg->msgData.aspTelecTest.mode);
        break;
    case aspMsgTypeSetLQIMode_c:
        status = Asp_SetLQIMode(pMsg->msgData.aspLQIMode);
        break;
    case aspMsgTypeGetRSSILevel_c:
        *((uint8_t*)&status) = Asp_GetRSSILevel(); //remove compiler warning
        break;
#if gMpmIncluded_d
    case aspMsgTypeSetMpmConfig_c:
        MPM_SetConfig(&pMsg->msgData.MpmConfig);
        break;
    case aspMsgTypeGetMpmConfig_c:
        MPM_GetConfig(&pMsg->msgData.MpmConfig);
        break;
#endif
    default:
        status = gAspInvalidRequest_c;// OR gAspInvalidParameter_c
        break;
    }
#if gFsciIncluded_c
    FSCI_Monitor( gFSCI_AspSapId_c,
                  pMsg,
                  (void*)&status,
                  gAspInterfaceId );
#endif
    return status;
}

/*! *********************************************************************************
* \brief  Returns the current PHY time
*
* \param[in]  time  location where the PHY time will be stored
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 2
void Asp_GetTimeReq(uint32_t *time)
{
    PhyTimeReadClock( time );
}

/*! *********************************************************************************
* \brief  Write XCVR registers
*
* \param[in]  mode   Direct/Indirect access
* \param[in]  addr   XCVR address
* \param[in]  len    number of bytes to write
* \param[in]  pData  data o be written
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 3
AspStatus_t Asp_XcvrWriteReq (uint8_t mode, uint16_t addr, uint8_t len, uint8_t* pData)
{
    if (mode)
        MCR20Drv_IndirectAccessSPIMultiByteWrite((uint8_t)addr, pData, len);
    else
        MCR20Drv_DirectAccessSPIMultiByteWrite((uint8_t)addr, pData, len);

    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Read XCVR registers
*
* \param[in]  mode   Direct/Indirect access
* \param[in]  addr   XCVR address
* \param[in]  len    number of bytes to read
* \param[in]  pData  location where data will be stored
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 4
AspStatus_t Asp_XcvrReadReq  (uint8_t mode, uint16_t addr, uint8_t len, uint8_t* pData)
{
    if (mode)
        MCR20Drv_IndirectAccessSPIMultiByteRead((uint8_t)addr, pData, len);
    else
        MCR20Drv_DirectAccessSPIMultiByteRead((uint8_t)addr, pData, len);

    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Set Tx output power level
*
* \param[in]  powerLevel   The new power level: 0x03-0x1F (see documentation for details)
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 5
AspStatus_t Asp_SetPowerLevel( uint8_t powerLevel )
{
    if(powerLevel > gAspPowerLevel_16dBm)
        return gAspInvalidParameter_c;

    {
        uint8_t res;

        res = PhyPlmeSetPwrLevelRequest(powerLevel);

        if( res == gPhySuccess_c )
        {
            return gAspSuccess_c;
        }
        else
        {
            return gAspDenied_c;
        }
    }
}

/*! *********************************************************************************
* \brief  Read the current Tx power level
*
* \return  power level
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 6
uint8_t Asp_GetPowerLevel()
{
    return MCR20Drv_DirectAccessSPIRead(PA_PWR);
}

/*! *********************************************************************************
* \brief  Set the state of Active Promiscuous functionality
*
* \param[in]  state  new state 
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 7
AspStatus_t Asp_SetActivePromState(bool_t state)
{
    PhySetActivePromiscuous(state);
    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Set the state of Fast Antenna Diversity functionality
*
* \param[in]  state  new state 
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 8
AspStatus_t Asp_SetFADState(bool_t state)
{
    if( gPhySuccess_c != PhyPlmeSetFADStateRequest(state) )
    {
        return gAspDenied_c;
    }
    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Set the Fast Antenna Diversity threshold
*
* \param[in]  threshold 
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 9
AspStatus_t Asp_SetFADThreshold(uint8_t threshold)
{
    if( gPhySuccess_c != PhyPlmeSetFADThresholdRequest(threshold) )
    {
        return gAspDenied_c;
    }
    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Set the ANTX functionality
*
* \param[in]  state 
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 10
AspStatus_t Asp_SetANTXState(bool_t state)
{
    if( gPhySuccess_c != PhyPlmeSetANTXStateRequest(state) )
    {
        return gAspDenied_c;
    }
    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Get the ANTX functionality
*
* \return  current state
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 11
uint8_t Asp_GetANTXState(void)
{
  return PhyPlmeGetANTXStateRequest();
}

/*! *********************************************************************************
* \brief  Set the ANTX pad state
*
* \param[in]  antAB_on 
* \param[in]  rxtxSwitch_on 
*
* \return  status
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 12
uint8_t Asp_SetANTPadStateRequest(bool_t antAB_on, bool_t rxtxSwitch_on)
{
    return PhyPlmeSetANTPadStateRequest(antAB_on, rxtxSwitch_on);
}

/*! *********************************************************************************
* \brief  Set the ANTX pad strength
*
* \param[in]  hiStrength 
*
* \return  status
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 13
uint8_t Asp_SetANTPadStrengthRequest(bool_t hiStrength)
{
    return PhyPlmeSetANTPadStrengthRequest(hiStrength);
}

/*! *********************************************************************************
* \brief  Set the ANTX inverted pads
*
* \param[in]  invAntA  invert Ant_A pad
* \param[in]  invAntB  invert Ant_B pad
* \param[in]  invTx    invert Tx pad
* \param[in]  invRx    invert Rx pad
*
* \return  status
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 14
uint8_t Asp_SetANTPadInvertedRequest(bool_t invAntA, bool_t invAntB, bool_t invTx, bool_t invRx)
{
    return PhyPlmeSetANTPadInvertedRequest(invAntA, invAntB, invTx, invRx);
}

/*! *********************************************************************************
* \brief  Set the LQI mode
*
* \param[in]  mode 
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 15
AspStatus_t Asp_SetLQIMode(bool_t mode)
{
    if( gPhySuccess_c != PhyPlmeSetLQIModeRequest(mode) )
    {
        return gAspDenied_c;
    }
    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Get the last RSSI level
*
* \return  RSSI
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 16
uint8_t Asp_GetRSSILevel(void)
{
  return PhyPlmeGetRSSILevelRequest();
}

/*! *********************************************************************************
* \brief  Set current channel
*
* \param[in]  channel  channel number (11-26)
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 17
AspStatus_t ASP_TelecSetFreq(uint8_t channel)
{
    PhyPlmeForceTrxOffRequest();
    if( gPhySuccess_c != PhyPlmeSetCurrentChannelRequest(channel,0) )
    {
        return gAspInvalidParameter_c;
    }

    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Send a raw data frame OTA
*
* \param[in]  dataPtr  raw data
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 18
AspStatus_t ASP_TelecSendRawData(uint8_t* dataPtr)
{
    uint8_t phyReg;

    dataPtr[0] += 2; /* Add FCS length to PSDU Length*/

    // Validate the length
    if(dataPtr[0] > gMaxPHYPacketSize_c)
        return gAspTooLong_c;

    //Force Idle
    PhyPlmeForceTrxOffRequest();
    AspSetDtsMode(gDtsNormal_c);
    AspDisableBER();
    // Load the TX PB: load the PSDU Lenght byte but not the FCS bytes
    MCR20Drv_PB_SPIBurstWrite(dataPtr, dataPtr[0] + 1 - 2);
    // Program a Tx sequence
    phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
    phyReg |=  gTX_c;
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Set Telec test mode
*
* \param[in]  mode  Telec test mode
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 19
AspStatus_t ASP_TelecTest(uint8_t mode)
{
    uint8_t phyReg;
	static uint8_t aTxContModPattern[2];
    uint8_t channel;
    static bool_t fracSet = FALSE;

    // Get current channel number
    channel = PhyPlmeGetCurrentChannelRequest(0);

    if( fracSet )
    {
        ASP_TelecSetFreq(channel);
        fracSet = FALSE;
    }

    switch( mode )
    {
    case gTestForceIdle_c:  //ForceIdle();
        PhyPlmeForceTrxOffRequest();

        AspSetDtsMode(gDtsNormal_c);
        AspDisableBER();
        break;

    case gTestPulseTxPrbs9_c:   // Continuously transmit a PRBS9 pattern.
        // PLME_PRBS9_Load (); // Load the TX RAM
        AspSetDtsMode(gDtsTxRandomSeq_c);
        //Enable continuous TX mode
        AspEnableBER();
        // Start Tx packet mode with no interrupt on end
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousRx_c: // Sets the device into continuous RX mode
        AspSetDtsMode(gDtsNormal_c);
        //Enable continuous RX mode
        AspEnableBER();
        // Set length of data in DUAL_PAN_DWELL register
        MCR20Drv_IndirectAccessSPIWrite(DUAL_PAN_DWELL, 127);
        // Start Rx packet mode with no interrupt on end
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gRX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousTxMod_c: // Sets the device to continuously transmit a 10101010 pattern
        AspSetDtsMode(gDtsNormal_c);
        //Enable continuous TX mode
        AspEnableBER();
        //Prepare TX operation
        aTxContModPattern[0] = 1;
        aTxContModPattern[1] = 0xAA;
        // Load the TX PB
        MCR20Drv_PB_SPIBurstWrite(aTxContModPattern, aTxContModPattern[0] + 1);
        // Program a Tx sequence
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousTxNoMod_c: // Sets the device to continuously transmit an unmodulated CW
        //Enable unmodulated TX
        AspSetDtsMode(gDtsTxOne_c);
        //Enable continuous TX mode
        AspEnableBER();
        MCR20Drv_DirectAccessSPIMultiByteWrite(PLL_FRAC0_LSB, (uint8_t *) &asp_pll_frac[channel - 11], 2);
        fracSet = TRUE;
        // Program a Tx sequence
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousTx2Mhz_c:
        AspSetDtsMode(gDtsTx2Mhz_c);
        //Enable continuous TX mode
        AspEnableBER();
        // Program a Tx sequence
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousTx200Khz_c:
        AspSetDtsMode(gDtsTx200Khz_c);
        //Enable continuous TX mode
        AspEnableBER();
        // Program a Tx sequence
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousTx1MbpsPRBS9_c:
        AspSetDtsMode(gDtsTx1MbpsPRBS9_c);
        //Enable continuous TX mode
        AspEnableBER();
        // Program a Tx sequence
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousTxExternalSrc_c:
        AspSetDtsMode(gDtsTxExternalSrc_c);
        //Enable continuous TX mode
        AspEnableBER();
        // Program a Tx sequence
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousTxNoModZero_c:
        //Enable unmodulated TX
        AspSetDtsMode(gDtsTxZero_c);
        //Enable continuous TX mode
        AspEnableBER();
        // Program a Tx sequence
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;

    case gTestContinuousTxNoModOne_c:
        //Enable unmodulated TX
        AspSetDtsMode(gDtsTxOne_c);
        //Enable continuous TX mode
        AspEnableBER();
        // Program a Tx sequence
        phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
        phyReg |=  gTX_c;
        MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);
        break;
    }

    return gAspSuccess_c;
}

/*! *********************************************************************************
* \brief  Return the instance of the PHY associated with the FSCI interface
*
* \param[in]  interfaceId  FSCI interface
*
* \return  insance
*
********************************************************************************** */
#if gFsciIncluded_c
#undef mFuncId_c
#define mFuncId_c 20
static uint32_t getPhyInstance( uint32_t interfaceId )
{
    uint32_t i;

    for( i=0; i<gPhyInstancesCnt_c; i++ )
        if( mAspFsciBinding[i] == interfaceId )
            return i;

    return 0;
}

/*! *********************************************************************************
* \brief  Handle ASP requests received from FSCI
*
* \param[in]  pData        monitored message
* \param[in]  param        
* \param[in]  interfaceId  FSCI interface 
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 21
static void fsciAspReqHandler(void *pData, void* param, uint32_t interfaceId)
{
    clientPacket_t *pClientPacket = ((clientPacket_t*)pData);
    uint8_t *pMsg = pClientPacket->structured.payload;

    pMsg -= sizeof(AppAspMsgType_t);
    ((AppToAspMessage_t*)pMsg)->msgType = (AppAspMsgType_t)pClientPacket->structured.header.opCode;

    APP_ASP_SapHandler( (AppToAspMessage_t*)pMsg, getPhyInstance( interfaceId ) );
    MEM_BufferFree(pData);
}

/*! *********************************************************************************
* \brief  Monitor the ASP Requests and Responses
*
* \param[in]  pData        monitored message
* \param[in]  param        
* \param[in]  interfaceId  FSCI interface 
*
* \return  AspStatus_t
*
********************************************************************************** */
#undef mFuncId_c
#define mFuncId_c 22
static void AspSapMonitor(void *pData, void* param, uint32_t interfaceId)
{
    clientPacket_t *pFsciPacket = MEM_BufferAlloc( sizeof(clientPacket_t) );
    AppToAspMessage_t *pReq = (AppToAspMessage_t*)pData;
    uint8_t *p;

    if( NULL == pFsciPacket )
    {
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
        return;
    }

    p = pFsciPacket->structured.payload;

    if( NULL == param ) // Requests
    {
        pFsciPacket->structured.header.opGroup = gFSCI_AppAspOpcodeGroup_c;
        pFsciPacket->structured.header.opCode = pReq->msgType;

        switch( pReq->msgType )
        {
        case aspMsgTypeGetTimeReq_c:
            break;
        case aspMsgTypeXcvrWriteReq_c:
        case aspMsgTypeXcvrReadReq_c:
            *p++ = pReq->msgData.aspXcvrData.mode;
            *((uint16_t*)p) = pReq->msgData.aspXcvrData.addr;
            p += sizeof(uint16_t);
            *p++ = pReq->msgData.aspXcvrData.len;
            if( pReq->msgType == aspMsgTypeXcvrWriteReq_c )
            {
                FLib_MemCpy( p, pReq->msgData.aspXcvrData.data,
                             pReq->msgData.aspXcvrData.len );
                p += pReq->msgData.aspXcvrData.len;
            }
            break;
        case aspMsgTypeSetFADState_c:
            FLib_MemCpy( p, &pReq->msgData.aspFADState, sizeof(pReq->msgData.aspFADState) );
            p += sizeof(pReq->msgData.aspFADState);
            break;
        case aspMsgTypeSetFADThreshold_c:
            FLib_MemCpy( p, &pReq->msgData.aspFADThreshold, sizeof(pReq->msgData.aspFADThreshold) );
            p += sizeof(pReq->msgData.aspFADThreshold);
            break;
        case aspMsgTypeSetANTXState_c:
            FLib_MemCpy( p, &pReq->msgData.aspANTXState, sizeof(pReq->msgData.aspANTXState) );
            p += sizeof(pReq->msgData.aspANTXState);
            break;
        case aspMsgTypeGetANTXState_c:
            /* Nothing to do here */
            break;

        case aspMsgTypeSetPowerLevel_c:
            FLib_MemCpy( p, &pReq->msgData.aspSetPowerLevelReq, sizeof(pReq->msgData.aspSetPowerLevelReq) );
            p += sizeof(pReq->msgData.aspSetPowerLevelReq);
            break;
        case aspMsgTypeGetPowerLevel_c:
            /* Nothing to do here */
            break;
        case aspMsgTypeTelecSetFreq_c:
            FLib_MemCpy( p, &pReq->msgData.aspTelecsetFreq, sizeof(pReq->msgData.aspTelecsetFreq) );
            p += sizeof(pReq->msgData.aspTelecsetFreq);
            break;
        case aspMsgTypeTelecSendRawData_c:
            FLib_MemCpy( p, &pReq->msgData.aspTelecSendRawData, sizeof(pReq->msgData.aspTelecSendRawData) );
            p += sizeof(pReq->msgData.aspTelecSendRawData);
            break;
        case aspMsgTypeTelecTest_c:
            FLib_MemCpy( p, &pReq->msgData.aspTelecTest, sizeof(pReq->msgData.aspTelecTest) );
            p += sizeof(pReq->msgData.aspTelecTest);
            break;
        case aspMsgTypeSetLQIMode_c:
            FLib_MemCpy(p, &pReq->msgData.aspLQIMode, sizeof(pReq->msgData.aspLQIMode) );
            p += sizeof(pReq->msgData.aspLQIMode);
            break;
        case aspMsgTypeGetRSSILevel_c:
            /* Nothing to do here */
            break;
        }
    }
    else // Confirms / Indications
    {
        pFsciPacket->structured.header.opGroup = gFSCI_AspAppOpcodeGroup_c;
        pFsciPacket->structured.header.opCode = pReq->msgType;

        *p++ = *((uint8_t*)param);/* copy status */

        switch( pReq->msgType )
        {
        case aspMsgTypeGetTimeReq_c:
            FLib_MemCpy( p, &pReq->msgData.aspGetTimeReq.time , sizeof(aspEventReq_t) );
            p += sizeof(aspEventReq_t);
            break;
        case aspMsgTypeGetMpmConfig_c:
            FLib_MemCpy( p, &pReq->msgData.MpmConfig , sizeof(mpmConfig_t) );
            p += sizeof(mpmConfig_t);
            break;
        case aspMsgTypeXcvrReadReq_c:
            *p++ = pReq->msgData.aspXcvrData.len; /* copy length */
            FLib_MemCpy( p, pReq->msgData.aspXcvrData.data, pReq->msgData.aspXcvrData.len );
            p += pReq->msgData.aspXcvrData.len;
            break;
        }

    }

    /* Send data over the serial interface */
    pFsciPacket->structured.header.len = (fsciLen_t)(p - pFsciPacket->structured.payload);

    if ( pFsciPacket->structured.header.len )
        FSCI_transmitFormatedPacket( pFsciPacket, interfaceId );
    else
        MEM_BufferFree( pFsciPacket );
}

#endif /* gFsciIncluded_c */


 /*! *********************************************************************************
* \brief  Set the Tx data source selector
*
* \param[in]  mode 
*
* \return  AspStatus_t
*
********************************************************************************** */
phyStatus_t AspSetDtsMode(uint8_t mode)
{
  uint8_t phyReg;

  phyReg = MCR20Drv_IndirectAccessSPIRead(TX_MODE_CTRL);
  phyReg &= ~cTX_MODE_CTRL_DTS_MASK;   // Clear DTS_MODE
  phyReg |= mode; // Set new DTS_MODE
  MCR20Drv_IndirectAccessSPIWrite(TX_MODE_CTRL, phyReg);

  return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  Enable XCVR test mode
*
* \return  AspStatus_t
*
********************************************************************************** */
phyStatus_t AspEnableBER()
{
  uint8_t phyReg;

  phyReg = MCR20Drv_IndirectAccessSPIRead(DTM_CTRL1);
  phyReg |= cDTM_CTRL1_DTM_EN;
  MCR20Drv_IndirectAccessSPIWrite(DTM_CTRL1, phyReg);

  phyReg = MCR20Drv_IndirectAccessSPIRead(TESTMODE_CTRL);
  phyReg |= cTEST_MODE_CTRL_CONTINUOUS_EN | cTEST_MODE_CTRL_IDEAL_PFC_EN;
  MCR20Drv_IndirectAccessSPIWrite(TESTMODE_CTRL, phyReg);

  return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  Disable XCVR test mode
*
********************************************************************************** */
void AspDisableBER()
{
  uint8_t phyReg;

  phyReg = MCR20Drv_IndirectAccessSPIRead(DTM_CTRL1);
  phyReg &= ~cDTM_CTRL1_DTM_EN;
  MCR20Drv_IndirectAccessSPIWrite(DTM_CTRL1, phyReg);

  phyReg = MCR20Drv_IndirectAccessSPIRead(TESTMODE_CTRL);
  phyReg &= ~(cTEST_MODE_CTRL_CONTINUOUS_EN | cTEST_MODE_CTRL_IDEAL_PFC_EN);
  MCR20Drv_IndirectAccessSPIWrite(TESTMODE_CTRL, phyReg);
}


#endif /* gAspCapability_d */