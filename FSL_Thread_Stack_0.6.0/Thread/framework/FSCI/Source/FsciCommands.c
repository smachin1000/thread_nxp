/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
* \file FsciCommands.c
* This is a source file which implements the FSCI commands received from the host.
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
#include "FsciInterface.h"
#include "FsciCommands.h"
#include "FsciCommunication.h"
#include "FunctionLib.h"
#include "MemManager.h"
#include "ModuleInfo.h"

#if gFSCI_IncludeMacCommands_c
    #include "FsciMacCommands.h"
#endif

#if gFSCI_IncludeLpmCommands_c
    #include "PWR_Interface.h"
#endif

#include "fsl_device_registers.h"

#if gBeeStackIncluded_d
#include "ZigbeeTask.h"
#endif

#if gFsciIncluded_c
/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define mRamStartAddress_c   (0x1FFF8000)
#define mRamEndAddress_c     (0x20007FF0)
#define mFlashStartAddress_c (0x00000000)
#define mFlashEndAddress_c   (0x0007FFFF)

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
extern void ResetMCU(void);
extern void Mac_GetExtendedAddress(uint8_t *pAddr, instanceId_t instanceId);
extern void Mac_SetExtendedAddress(uint8_t *pAddr, instanceId_t instanceId);
extern uint8_t PhyGetLastRxLqiValue(void);
extern gFsciOpGroup_t *FSCI_GetReqOpGroup(opGroup_t OG, uint8_t fsciInterface);

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef PACKED_STRUCT FsciWakeUpConfig_tag
{
    bool_t   signalWhenWakeUpFlag; /* Flag used to send or not a WakeUp.Ind message */
    uint32_t deepSleepDuration;    /* The deep sleep duration in 802.15.4 phy symbols (16 us) */
}FsciWakeUpConfig_t;

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
/* Set to TRUE when FSCI_Error() is called. */
uint8_t mFsciErrorReported;
bool_t (*pfFSCI_OtaSupportCalback)(clientPacket_t*) = NULL;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
extern uint8_t gNumberOfOG;
extern gFsciOpGroup_t gReqOpGroupTable[];
extern uint16_t gFreeMessagesCount;

/* FSCI Error message */
static gFsciErrorMsg_t mFsciErrorMsg = {
    gFSCI_StartMarker_c,
    gFSCI_CnfOpcodeGroup_c,
    mFsciMsgError_c,
    sizeof(clientPacketStatus_t),
    gFsciSuccess_c,
    0,
    0
};

/* FSCI OpCodes and coresponding handler functions */
static const gFsciOpCode_t FSCI_ReqOCtable[] =
{
    {mFsciMsgModeSelectReq_c,                FSCI_MsgModeSelectReqFunc},
    {mFsciMsgGetModeReq_c,                   FSCI_MsgGetModeReqFunc},
    {mFsciMsgResetCPUReq_c,                  FSCI_MsgResetCPUReqFunc},

#if gBeeStackIncluded_d
    {mFsciMsgAFResetReq_c,                   ZbFsciHandler},
    {mFsciMsgAPSResetReq_c,                  ZbFsciHandler},
    {mFsciMsgAPSReadyReq_c,                  ZbFsciHandler},
    {mFsciMsgDeregisterEndPoint_c,           ZbFsciHandler},
    {mFsciMsgRegisterEndPoint_c,             ZbFsciHandler},
    {mFsciMsgGetNumberOfEndPoints_c,         ZbFsciHandler},
    {mFsciMsgGetEndPointDescription_c,       ZbFsciHandler},
    {mFsciMsgGetEndPointIdList_c,            ZbFsciHandler},
    {mFsciMsgSetEpMaxWindowSize_c,           ZbFsciHandler},

    {mFsciMsgGetICanHearYouList_c,           ZbFsciHandler},
    {mFsciMsgSetICanHearYouList_c,           ZbFsciHandler},

    {mFsciMsgGetChannelReq_c,                ZbFsciHandler},
    {mFsciMsgSetChannelReq_c,                ZbFsciHandler},
    {mFsciMsgGetPanIDReq_c,                  ZbFsciHandler},
    {mFsciMsgSetPanIDReq_c,                  ZbFsciHandler},
    {mFsciMsgGetPermissionsTable_c,          ZbFsciHandler},
    {mFsciMsgSetPermissionsTable_c,          ZbFsciHandler},
    {mFsciMsgRemoveFromPermissionsTable_c,   ZbFsciHandler},
    {mFsciMsgAddDeviceToPermissionsTable_c,  ZbFsciHandler},
    {mFsciMsgGetNumOfMsgsReq_c,              ZbFsciHandler},

    {mFsciMsgApsmeGetIBReq_c,                ZbFsciHandler},
    {mFsciMsgApsmeSetIBReq_c,                ZbFsciHandler},
    {mFsciMsgNlmeGetIBReq_c,                 ZbFsciHandler},
    {mFsciMsgNlmeSetIBReq_c,                 ZbFsciHandler},
    {mFsciMsgFreeDiscoveryTablesReq_c,       ZbFsciHandler},
    {mFsciMsgSetJoinFilterFlagReq_c,         ZbFsciHandler},
    {mFsciMsgGetMaxApplicationPayloadReq_c,  ZbFsciHandler},

    {mFsciMsgGetApsDeviceKeyPairSet_c,       ZbFsciHandler},
    {mFsciMsgGetApsDeviceKey_c,              ZbFsciHandler},
    {mFsciMsgSetApsDeviceKey_c,              ZbFsciHandler},
    {mFsciMsgSetApsDeviceKeyPairSet_c,       ZbFsciHandler},
    {mFsciMsgClearApsDeviceKeyPairSet_c,     ZbFsciHandler},
     /* TBD */
    {mFsciMsgSetApsDeviceKeyPairSetKeyInfo,  ZbFsciHandler}, //not used by MAC
    {mFsciMsgSetApsOverrideApsEncryption,    ZbFsciHandler},
    {mFsciMsgSetPollRate,                    ZbFsciHandler},
    {mFsciMsgSetRejoinConfigParams,          ZbFsciHandler},
    {mFsciMsgSetFaMaxIncomingErrorReports_c, ZbFsciHandler},
    {mFsciMsgSetHighLowRamConcentrator,      ZbFsciHandler},
    
    {mFsciMsgEZModeComissioningStart,                    ZbFsciHandler}, //not used by MAC  
    {mFsciMsgZllTouchlinkCommissioningStart_c,           ZbFsciHandler},
    {mFsciMsgZllTouchlinkCommissioningConfigure_c,       ZbFsciHandler},
    {mFsciMsgZllTouchlinkGetListOfCommissionedDevices_c, ZbFsciHandler},
    {mFsciMsgZllTouchlinkRemoveEntry_c,                  ZbFsciHandler},    
    
    {mFsciMsgClearNeighborTableEntry_c,      ZbFsciHandler},
    
    {mFsciMsgAddToAddressMapPermanent_c,     ZbFsciHandler},
    {mFsciMsgRemoveFromAddressMap_c,         ZbFsciHandler},

    {mFsciOtaSupportQueryImageReq_c,         ZbFsciHandler}, //not used by MAC
    {mFsciOtaSupportQueryImageRsp_c,         ZbFsciHandler},
    {mFsciOtaSupportImageNotifyReq_c,        ZbFsciHandler},

    {mFsciMsgReadNwkMngAddressReq_c,         ZbFsciHandler},
    {mFsciMsgWriteNwkMngAddressReq_c,        ZbFsciHandler},
    {mFsciMsgStopNwkReq_c,                   ZbFsciHandler},
    {mFsciMsgStartNwkReq_c,                  ZbFsciHandler},
    {mFsciMsgRestartNwkReq_c,                ZbFsciHandler},
    {mFsciMsgStartNwkExReq_c,                ZbFsciHandler},
    {mFsciMsgStopNwkExReq_c,                 ZbFsciHandler},
    {mFsciMsgReadExtendedAdrReq_c,           ZbFsciHandler},
    {mFsciMsgWriteExtendedAdrReq_c,          ZbFsciHandler},
#else
    {mFsciMsgReadExtendedAdrReq_c,           FSCI_MsgReadExtendedAdrReqFunc},
    {mFsciMsgWriteExtendedAdrReq_c,          FSCI_MsgWriteExtendedAdrReqFunc},
#endif

    {mFsciLowLevelMemoryWriteBlock_c,        FSCI_WriteMemoryBlock},
    {mFsciLowLevelMemoryReadBlock_c,         FSCI_ReadMemoryBlock},
    {mFsciLowLevelPing_c,                    FSCI_Ping},

    {mFsciOtaSupportImageNotifyReq_c,        FSCI_OtaSupportHandlerFunc},
    {mFsciOtaSupportStartImageReq_c,         FSCI_OtaSupportHandlerFunc},
    {mFsciOtaSupportSetModeReq_c,            FSCI_OtaSupportHandlerFunc},
    {mFsciOtaSupportQueryImageRsp_c,         FSCI_OtaSupportHandlerFunc},
    {mFsciOtaSupportPushImageChunkReq_c,     FSCI_OtaSupportHandlerFunc},
    {mFsciOtaSupportCommitImageReq_c,        FSCI_OtaSupportHandlerFunc},
    {mFsciOtaSupportCancelImageReq_c,        FSCI_OtaSupportHandlerFunc},
    {mFsciOtaSupportSetFileVerPoliciesReq_c, FSCI_OtaSupportHandlerFunc},
    {mFsciOtaSupportAbortOTAUpgradeReq_c,    FSCI_OtaSupportHandlerFunc},

#if gFSCI_IncludeLpmCommands_c
    {mFsciMsgAllowDeviceToSleepReq_c,        FSCI_MsgAllowDeviceToSleepReqFunc},
#endif

    {mFsciGetUniqueId_c,                     FSCI_ReadUniqueId},
    {mFsciGetMcuId_c,                        FSCI_ReadMCUId},
    {mFsciGetSwVersions_c,                   FSCI_ReadModVer},
};

/* Used for maintaining backward compatibillity */
static const opGroup_t mFsciModeSelectSAPs[] =
{
    gFSCI_McpsSapId_c,
    gFSCI_MlmeSapId_c,
    gFSCI_AspSapId_c,
    gFSCI_NldeSapId_c,
    gFSCI_NlmeSapId_c,
    gFSCI_AspdeSapId_c,
    gFSCI_AfdeSapId_c,
    gFSCI_ApsmeSapId_c,
    gFSCI_ZdpSapId_c,
};

#if gFSCI_IncludeLpmCommands_c
uint8_t mFsciInterfaceToSendWakeUp;
static FsciWakeUpConfig_t  mFsciWakeUpConfig =
{
    FALSE,       /* WakeUp.Ind message is NOT sent when wake up */
    0x0003D090   /* deep sleep duration 4 seconds */
};
#endif

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief   This is the handler function for the FSCI OpGroup.
*          It calls the handler function for the received OpCode.
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
********************************************************************************** */
void fsciMsgHandler( void* pData, uint32_t fsciInterface )
{
    uint32_t i;

    /* Call the handler function for the received OpCode */
    for ( i = 0; i < NumberOfElements(FSCI_ReqOCtable); i++ )
        if( FSCI_ReqOCtable[i].opCode == ((clientPacket_t*)pData)->structured.header.opCode )
        {
            if( FSCI_ReqOCtable[i].pfOpCodeHandle( pData, fsciInterface ) )
            {
                /* Reuse received message */
                ((clientPacket_t*)pData)->structured.header.opGroup = gFSCI_CnfOpcodeGroup_c;
                FSCI_transmitFormatedPacket( pData, fsciInterface );
            }
            break;
        }

    /* If handler function was not found, send error message */
    if( i >= NumberOfElements(FSCI_ReqOCtable) )
    {
        MEM_BufferFree( pData );
        FSCI_Error( gFsciUnknownOpcode_c, fsciInterface );
    }
}

/*! *********************************************************************************
* \brief  Send an error message back to the external client.
*         This function should not block even if there is no more dynamic memory available
*
* \param[in] errorCode the erros encountered
* \param[in] fsciInterface the interface on which the packet was received
*
*
********************************************************************************** */
void FSCI_Error(uint8_t errorCode, uint32_t fsciInterface)
{
    uint32_t virtInterface = FSCI_GetVirtualInterface(fsciInterface);
    uint8_t size = sizeof(mFsciErrorMsg)-1;

    /* Don't cascade error messages. */
    if( mFsciErrorReported )
        return;

    mFsciErrorMsg.status = errorCode;
    mFsciErrorMsg.checksum = FSCI_computeChecksum( &mFsciErrorMsg.header.opGroup, size-2 );

    if( virtInterface )
    {
#if gFsciMaxVirtualInterfaces_c
        mFsciErrorMsg.checksum2  = mFsciErrorMsg.checksum;
        mFsciErrorMsg.checksum  += virtInterface;
        mFsciErrorMsg.checksum2 ^= mFsciErrorMsg.checksum;
        size++;
#else
        (void)virtInterface;
#endif
    }

    Serial_SyncWrite( gFsciInterfaces[fsciInterface], (uint8_t*)&mFsciErrorMsg, size );

    mFsciErrorReported = TRUE;
}

/*! *********************************************************************************
* \brief   Set FSCI operating mode for certain OpGroups
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
********************************************************************************** */
bool_t FSCI_MsgModeSelectReqFunc(void* pData, uint32_t fsciInterface)
{
    uint8_t i;
    uint8_t payloadIndex = 0;
    gFsciOpGroup_t *p;

    fsciLen_t dataLen = ((clientPacket_t*)pData)->structured.header.len;
    if( dataLen > 0 )
    {
        //gFsciTxBlocking = ((clientPacket_t*)pData)->structured.payload[payloadIndex++];
    }

    dataLen -= sizeof(((clientPacket_t*)pData)->structured.payload[0]);

    for( i = 0; i < dataLen; i++ )
    {
        p = FSCI_GetReqOpGroup(mFsciModeSelectSAPs[i], fsciInterface);
        if( NULL != p )
        {
            p->mode= ((clientPacket_t*)pData)->structured.payload[payloadIndex + i];
        }
    }

    ((clientPacket_t*)pData)->structured.payload[0] = gFsciSuccess_c;
    ((clientPacket_t*)pData)->structured.header.len = sizeof(uint8_t);
    return TRUE;
}

/*! *********************************************************************************
* \brief   Returns FSCI operating mode for certain OpGroups
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
********************************************************************************** */
bool_t FSCI_MsgGetModeReqFunc(void* pData, uint32_t fsciInterface)
{
    uint8_t i;
    uint8_t payloadIndex = 0;
    gFsciOpGroup_t *p;

    ((clientPacket_t*)pData)->structured.payload[payloadIndex++] = gFsciSuccess_c;
    ((clientPacket_t*)pData)->structured.payload[payloadIndex++] = gFsciTxBlocking;

    for( i = 0; i < NumberOfElements(mFsciModeSelectSAPs); i++ )
    {
        p = FSCI_GetReqOpGroup(mFsciModeSelectSAPs[i], fsciInterface);
        if( NULL != p )
        {
            ((clientPacket_t*)pData)->structured.payload[payloadIndex++] = p->mode;
        }
        else
        {
            ((clientPacket_t*)pData)->structured.payload[payloadIndex++] = gFsciInvalidMode;
        }
    }

    ((clientPacket_t*)pData)->structured.header.len = payloadIndex;
    return TRUE;
}

/*! *********************************************************************************
* \brief   Function used for writing to RAM memory.
*          Payload contains the packet received over the serial interface
*          bytes 0-3 --> start address for writing
*          byte  4   --> number of bytes to be written
*          bytes 5+  --> data to be written starting with start address.
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
********************************************************************************** */
bool_t FSCI_WriteMemoryBlock(void* pData, uint32_t fsciInterface)
{
    uint16_t len;
    uint8_t *addr;

    FLib_MemCpy(&addr, ((clientPacket_t*)pData)->structured.payload, sizeof(uint8_t*));
    len = ((clientPacket_t*)pData)->structured.payload[sizeof(uint8_t*)];

    /* Check RAM boundaries */
    if ( mRamStartAddress_c <= (uint32_t)addr && mRamEndAddress_c >= (uint32_t)addr)
    {
        FLib_MemCpy(addr, &((clientPacket_t*)pData)->structured.payload[sizeof(uint8_t*) + 1], len);
    }
    else
    {
        FSCI_Error(gFsciError_c, fsciInterface);
        len = 0;
    }

    ((clientPacket_t*)pData)->structured.header.len = sizeof(len);
    ((clientPacket_t*)pData)->structured.payload[0] = len;
    return TRUE;
}

/*! *********************************************************************************
* \brief   Function used for reading from RAM memory.
*          Payload contains the packet received over the serial interface
*          bytes 0-3 --> start address for reading
*          byte  4   --> number of bytes to read
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
********************************************************************************** */
bool_t FSCI_ReadMemoryBlock(void* pData, uint32_t fsciInterface)
{
    clientPacket_t *pPkt;
    uint16_t len;
    uint8_t *addr;

    FLib_MemCpy(&addr, ((clientPacket_t*)pData)->structured.payload, sizeof(uint8_t*));
    len = ((clientPacket_t*)pData)->structured.payload[sizeof(uint8_t*)];

    if( MEM_BufferGetSize(pData) >= (sizeof(clientPacketHdr_t) + len + 2) )
        pPkt = pData;
    else
        pPkt = MEM_BufferAlloc( sizeof(clientPacketHdr_t) + len + 2 );

    if( !pPkt )
    {
        FSCI_Error( gFsciOutOfMessages_c, fsciInterface );
        MEM_BufferFree(pData);
        return FALSE;
    }

    /* Check RAM and FLASH boundaries */
    if ((mRamStartAddress_c <= (uint32_t)addr && (uint32_t)addr <= mRamEndAddress_c) ||
        (mFlashStartAddress_c <  (uint32_t)addr && (uint32_t)addr <= mFlashEndAddress_c))
    {
        FLib_MemCpy(pPkt->structured.payload, addr, len);
    }
    else
    {
        FSCI_Error(gFsciError_c, fsciInterface);
        MEM_BufferFree(pData);
        return FALSE;
    }

    pPkt->structured.header.len = len;

    /* Check if the received buffer was reused. */
    if( pPkt == pData )
        return TRUE;

    /* A new buffer was allocated. Fill with aditional information */
    pPkt->structured.header.opGroup = gFSCI_CnfOpcodeGroup_c;
    pPkt->structured.header.opCode = mFsciLowLevelMemoryReadBlock_c;
    FSCI_transmitFormatedPacket( pPkt, fsciInterface );
    MEM_BufferFree(pData);

    return FALSE;
}

/*! *********************************************************************************
* \brief  This function simply echoes back the payload
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
* \remarks Remarks: if USB communication is used, the connection will be lost
*
********************************************************************************** */
bool_t  FSCI_Ping(void* pData, uint32_t fsciInterface)
{
    /* Nothing to do here */
    return TRUE;
}

/*! *********************************************************************************
* \brief  This function resets the MCU
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
* \remarks Remarks: if USB communication is used, the connection will be lost
*
********************************************************************************** */
bool_t FSCI_MsgResetCPUReqFunc(void* pData, uint32_t fsciInterface)
{
#if !gSerialMgrUseUSB_c
    ResetMCU();
#endif
    MEM_BufferFree(pData);
    return FALSE;
}

/*! *********************************************************************************
* \brief  This function writes the MAC Extended Address into the MAC layer
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
* \remarks Remarks: this function is legacy
*
********************************************************************************** */
bool_t FSCI_MsgWriteExtendedAdrReqFunc(void* pData, uint32_t fsciInterface)
{
#if gFSCI_IncludeMacCommands_c
    Mac_SetExtendedAddress( ((clientPacket_t*)pData)->structured.payload, fsciGetMacInstanceId(fsciInterface) );

#if gRF4CEIncluded_d
    extern uint8_t aExtendedAddress[8];
    FLib_MemCpy(aExtendedAddress,((clientPacket_t*)pData)->structured.payload, 8);
#endif

    ((clientPacket_t*)pData)->structured.payload[0] = gFsciSuccess_c;
#else
    ((clientPacket_t*)pData)->structured.payload[0] = gFsciRequestIsDisabled_c;
#endif
    ((clientPacket_t*)pData)->structured.header.len = sizeof(clientPacketStatus_t);
    return TRUE;
}

/*! *********************************************************************************
* \brief  This function sends the MAC Extended Address over the serial interface
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
* \remarks Remarks: this function is legacy
*
********************************************************************************** */
bool_t FSCI_MsgReadExtendedAdrReqFunc(void* pData, uint32_t fsciInterface)
{
#if gFSCI_IncludeMacCommands_c
    Mac_GetExtendedAddress( &((clientPacket_t*)pData)->structured.payload[sizeof(clientPacketStatus_t)], fsciGetMacInstanceId(fsciInterface) );

    ((clientPacket_t*)pData)->structured.payload[0] = gFsciSuccess_c;
    ((clientPacket_t*)pData)->structured.header.len = sizeof(clientPacketStatus_t)
                                                      + sizeof(uint64_t);
#else
    ((clientPacket_t*)pData)->structured.payload[0] = gFsciRequestIsDisabled_c;
    ((clientPacket_t*)pData)->structured.header.len = sizeof(clientPacketStatus_t);
#endif
    return TRUE;
}

/*! *********************************************************************************
* \brief  This function sends the LQI value for the last received packet over
*         the serial interface
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE in order to recycled the received message
*
********************************************************************************** */
bool_t FSCI_GetLastLqiValue(void* pData, uint32_t fsciInterface)
{
    ((clientPacket_t*)pData)->structured.payload[0] = PhyGetLastRxLqiValue();
    ((clientPacket_t*)pData)->structured.header.len = sizeof(clientPacketStatus_t);
    return TRUE;
}

#if gFSCI_IncludeLpmCommands_c
/*! *********************************************************************************
* \brief
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE if the received message was recycled, FALSE if it must be deleted
*
********************************************************************************** */
bool_t FSCI_MsgAllowDeviceToSleepReqFunc(void* pData, uint32_t fsciInterface)
{
    mFsciInterfaceToSendWakeUp = (uint8_t)fsciInterface;
    /* Set the new configuration */
    FLib_MemCpy(&mFsciWakeUpConfig,
                ((clientPacket_t*)pData)->structured.payload,
                sizeof(mFsciWakeUpConfig));

    if( mFsciWakeUpConfig.deepSleepDuration < 10 )
        ((clientPacket_t*)pData)->structured.payload[0] = gFsciError_c;
    else
        ((clientPacket_t*)pData)->structured.payload[0] = gFsciSuccess_c;

    ((clientPacket_t*)pData)->structured.header.opGroup = gFSCI_CnfOpcodeGroup_c;
    ((clientPacket_t*)pData)->structured.header.len = sizeof(clientPacketStatus_t);

    // Perform a Sync Tx
    gFsciTxBlocking = TRUE;
    FSCI_transmitFormatedPacket(pData, fsciInterface);
    gFsciTxBlocking = FALSE;

    if( mFsciWakeUpConfig.deepSleepDuration >= 10 )
    {
        PWR_SetDeepSleepTimeInSymbols(mFsciWakeUpConfig.deepSleepDuration);
        PWR_AllowDeviceToSleep();  /* Allow device to sleep */
    }

    return FALSE;
}

/*! *********************************************************************************
* \brief
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE if the received message was recycled, FALSE if it must be deleted
*
********************************************************************************** */
void FSCI_SendWakeUpIndication( void )
{
    clientPacket_t *pPkt;

    PWR_DisallowDeviceToSleep();  /* Disallow device to sleep */

    if( mFsciWakeUpConfig.signalWhenWakeUpFlag )
    {
        pPkt = MEM_BufferAlloc(sizeof(clientPacketHdr_t) +
                               sizeof(clientPacketStatus_t) + 2);

        if( !pPkt )
            return;

        pPkt->structured.header.opGroup = gFSCI_CnfOpcodeGroup_c;
        pPkt->structured.header.opCode = mFsciMsgWakeUpIndication_c;
        pPkt->structured.header.len = sizeof(clientPacketStatus_t);
        pPkt->structured.payload[0] = gFsciSuccess_c;
        FSCI_transmitFormatedPacket( pPkt, mFsciInterfaceToSendWakeUp );
    }
}
#endif

/*! *********************************************************************************
* \brief  This function sends the content of the SIM_UID registers over the
*         serial interface
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE if the received message was recycled, FALSE if it must be deleted
*
********************************************************************************** */
bool_t FSCI_ReadUniqueId(void* pData, uint32_t fsciInterface)
{
    clientPacket_t *pPkt = (clientPacket_t*)pData;
    uint32_t *p = (uint32_t*)pPkt->structured.payload;

    pPkt->structured.header.len = 4*sizeof(uint32_t);
#if defined(SIM_UIDH)
    *p++ = SIM_UIDH;
#else
    *p++ = 0;
#endif
    *p++ = HW_SIM_UIDMH_RD(SIM_BASE);
    *p++ = HW_SIM_UIDML_RD(SIM_BASE);
    *p++ = HW_SIM_UIDL_RD(SIM_BASE);

    return TRUE;
}

/*! *********************************************************************************
* \brief  This function sends the content of the SIM_SDID register over the
*         serial interface
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE if the received message was recycled, FALSE if it must be deleted
*
********************************************************************************** */
bool_t FSCI_ReadMCUId(void* pData, uint32_t fsciInterface)
{
    clientPacket_t *pPkt = (clientPacket_t*)pData;
    uint32_t *p = (uint32_t*)pPkt->structured.payload;

    pPkt->structured.header.len = sizeof(uint32_t);
    *p++ = HW_SIM_SDID_RD(SIM_BASE);

    return TRUE;
}

/*! *********************************************************************************
* \brief  This function reads all module information located in section VERSION_TAGS
*         and sends this information over the serial interface
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE if the received message was recycled, FALSE if it must be deleted
*
********************************************************************************** */
bool_t FSCI_ReadModVer(void* pData, uint32_t fsciInterface)
{
    clientPacket_t *pPkt;
    moduleInfo_t *pInfo = gVERSION_TAGS_startAddr_d;
    uint16_t size = sizeof(clientPacketHdr_t) + 2 +
                    gVERSION_TAGS_entries_d * gVERSION_TAGS_entrySizeNoPadding_d;

    /* Check if the received buffer is large enough to be reused */
    if( MEM_BufferGetSize(pData) >= size )
        pPkt = pData;
    else
        pPkt = MEM_BufferAlloc( size );

    if( !pPkt )
    {
        FSCI_Error( gFsciOutOfMessages_c, fsciInterface );
        MEM_BufferFree(pData);
        return FALSE;
    }

    pPkt->structured.payload[0] = gVERSION_TAGS_entries_d;
    size = sizeof(uint8_t);

    while( pInfo < gVERSION_TAGS_endAddr_d )
    {
        FLib_MemCpy( &pPkt->structured.payload[size],
                     &pInfo->moduleId,
                     gVERSION_TAGS_entrySizeNoPadding_d -
                     GetSizeOfMember(moduleInfo_t, moduleString) );
        size += gVERSION_TAGS_entrySizeNoPadding_d;
        pInfo++;
    }

    pPkt->structured.header.len = (uint8_t)size;

    /* Check if the received buffer was reused. */
    if( pPkt == pData )
        return TRUE;

    /* A new buffer was allocated. Fill with aditional information */
    pPkt->structured.header.opGroup = gFSCI_CnfOpcodeGroup_c;
    pPkt->structured.header.opCode = mFsciGetSwVersions_c;
    FSCI_transmitFormatedPacket( pPkt, fsciInterface );
    MEM_BufferFree(pData);

    return FALSE;
}

/*! *********************************************************************************
* \brief  This function handles the requests for the OTA OpCodes
*
* \param[in] pData pointer to location of the received data
* \param[in] fsciInterface the interface on which the packet was received
*
* \return  TRUE if the received message was recycled, FALSE if it must be deleted
*
********************************************************************************** */
bool_t FSCI_OtaSupportHandlerFunc(void* pData, uint32_t fsciInterface)
{
    if( pfFSCI_OtaSupportCalback )
    {
        if( pfFSCI_OtaSupportCalback((clientPacket_t*)pData) )
            return TRUE;
    }

    MEM_BufferFree(pData);
    return FALSE;
}


#endif /* gFsciIncluded_c */