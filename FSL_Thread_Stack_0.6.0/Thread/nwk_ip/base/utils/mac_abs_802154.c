/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

/*!=================================================================================================
\file       mac_abs_802154.c
\brief      This is a public source file for the MAC abstraction module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* Basic types */
#include "EmbeddedTypes.h"

#include "MemManager.h"
#include "FunctionLib.h"

/* MAC interface */
#include "MacInterface.h"
#include "MacConfig.h"

/* Other */
#include "mac_abs_802154.h"
#include "sixlowpan_interface.h"
#include "LED.h"
#include "mac_filtering.h"

#include "TimersManager.h"
/*==================================================================================================
Private macros
==================================================================================================*/

/* If defined, the MSDU is copyed to the end of the MCPS_Data.Req structure */
#define gMacFreeRequestInline       1

#define MAC_GET_MAX_MSDU_SIZE       1
#define MAC_MAX_MSDU_SIZE_802154    88U

#define MAC_DEFAULT_KEY_SOURCE      0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

#define MAC_I_CAN_HEAR_YOU_TABLE_SIZE   5

#ifdef _DEBUG
#define NWKDBG_STATS_ENABLED
//#define NWKDBG_TRACKTIME
#endif

#ifdef NWKDBG_TRACKTIME
#define CODE_TIME_MAX_ENTRIES       100U
#endif


#define MacTxLedToggle  if(gEnable802154TxLed)\
                        Led2Toggle
#define MacTxLedOn      if(gEnable802154TxLed)\
                        Led2On                          
#define MacTxLedOff     if(gEnable802154TxLed)\
                        Led2Off 
/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/
static resultType_t MCPS_NWK_SapHandlerCB(mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);
static resultType_t MLME_NWK_SapHandlerCB(nwkMessage_t* pMsg, instanceId_t instanceId);

/* MAC Requests */
/* MCPS Services */
static void                 MAC_McpsDataReq(macAbsMcpsDataReq_t * pParam, instanceId_t instanceId);

/* MLME Services */
static macAbsMlmeGetCnf_t * MAC_MlmeGetReq(macAbsMlmeGetReq_t * pParams, instanceId_t instanceId);
static macAbsMlmeResetCnf_t MAC_MlmeResetReq(macAbsMlmeResetReq_t * pParam, instanceId_t instanceId);
static macAbsMlmeScanCnf_t  MAC_MlmeScanReq(macAbsMlmeScanReq_t * pParam, instanceId_t instanceId);
static macAbsMlmeSetCnf_t   MAC_MlmeSetReq(macAbsMlmeSetReq_t * pParam, instanceId_t instanceId);
static macAbsMlmeStartCnf_t MAC_MlmeStartReq(macAbsMlmeStartReq_t * pParam, instanceId_t instanceId);
static void MAC_MlmePollReq(macAbsMlmePollReq_t * pParam, instanceId_t instanceId);

static bool_t               MAC_SetPib(uint32_t pibAttribute, void* pPibValue, instanceId_t instanceId);
static bool_t               MAC_GetPib(uint32_t pibAttribute, void* pPibValue, instanceId_t instanceId);
static bool_t               MAC_SetPibUint8Val(uint32_t pibAttribute, uint8_t pibValue, instanceId_t instanceId);

static uint32_t             MAC_GetMaxMsduSize(macAbsMcpsDataReq_t * pParam, instanceId_t instanceId);
static bool_t               MAC_SetPANId(uint16_t panId, instanceId_t instanceId);
static uint16_t             MAC_GetPANId(instanceId_t instanceId);
static bool_t               MAC_SetShortAddress(uint16_t shortAddress, instanceId_t instanceId);
static uint16_t             MAC_GetShortAddress(instanceId_t instanceId);
static bool_t               MAC_SetExtendedAddress(uint64_t extendedAddress, instanceId_t instanceId);
static uint64_t             MAC_GetExtendedAddress(instanceId_t instanceId);
static bool_t               MAC_SetKey(uint8_t idxInKeyTable, uint8_t* key, uint8_t keyIndex, instanceId_t instanceId);
static bool_t               MAC_GetKey(uint8_t idxInKeyTable, uint8_t* key, instanceId_t instanceId);
static bool_t               MAC_SetChannel(uint8_t channel, instanceId_t instanceId);
static uint8_t              MAC_GetChannel(instanceId_t instanceId);
static bool_t               MAC_SetRxOnWhenIdle(bool_t bRxOnWhenIdle, instanceId_t instanceId);
static bool_t               MAC_GetRxOnWhenIdle(instanceId_t instanceId);
static bool_t               MAC_SetFrameCounter(uint32_t frameCounter, instanceId_t instanceId);
static uint32_t             MAC_GetFrameCounter(instanceId_t instanceId);
static bool_t               MAC_GetNeighborFrameCounter(uint32_t       *pFrameCtr,
                                                                           uint8_t neighborIdx, 
                                                                           uint8_t maxNeighbors,
                                                                           uint8_t macKeyIdx,
                                                                           uint8_t maxMacKeys,
                                                                           instanceId_t instanceId);
static void                 MAC_SetPanSecurity(uint8_t maxNbOfKeys, uint8_t maxNbOfNeighbors, uint8_t index, instanceId_t instanceId);
static void                 MAC_SetNeighborSecurity(uint8_t neighborIdx, uint8_t maxNeighbors, uint8_t macKeyIdx, uint8_t maxMacKeys, uint64_t extAddr, uint16_t shortAddr, uint16_t panId, uint32_t frameCounter, instanceId_t instanceId);
static void                 MAC_SetMlmeCallbacks(macAbsCallbacks_t* mlmeCallbacks, instanceId_t instanceId);
#if 0
static bool_t               MAC_SetNeighborTableEntry(macNeighborTableEntry_T * nt_entry, instanceId_t instanceId);
static bool_t               MAC_GetNeighborTableEntry(macNeighborTableEntry_T * nt_entry, instanceId_t instanceId);
static bool_t               MAC_GetNeighborTableEntryByAddress(uint16_t shortAddress, macNeighborTableEntry_T* pNeighborTableEntry, instanceId_t instanceId);
#endif

/* MAC Callbacks */
static void MAC_McpsDataIndCB(mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);
static void MAC_McpsDataCnfCB(mcpsToNwkMessage_t* pMsg, instanceId_t instanceId);


static void MAC_MlmeBeaconNotifyIndCB(mlmeBeaconNotifyInd_t * pPayload, instanceId_t instanceId);
static void MAC_MlmeScanCnfCB(mlmeScanCnf_t * pPayload, instanceId_t instanceId);
static void MAC_MlmeCommStatusIndCB(mlmeCommStatusInd_t *pPayload, instanceId_t instanceId);
static void MAC_MlmePollCnfCB(mlmePollCnf_t * pPayload, instanceId_t instanceId);
static void MAC_MlmePollIndCB(mlmePollNotifyInd_t * pPayload, instanceId_t instanceId);

#if 0
static void MAC_MlmeSyncLossIndCB(mlmeSyncLossInd_t * pPayload, instanceId_t instanceId);
#endif

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

static const macAbsRequests_t macAbsReq =
{
    .MCPS_DataReq = MAC_McpsDataReq,
    .MLME_SetReq = MAC_MlmeSetReq,
    .MLME_GetReq = MAC_MlmeGetReq,
    .MLME_ResetReq = MAC_MlmeResetReq,
    .MLME_StartReq = MAC_MlmeStartReq,
    .MLME_PollReq = MAC_MlmePollReq,
    .MLME_ScanReq = MAC_MlmeScanReq,
    .GetMaxMsduSize = MAC_GetMaxMsduSize,
    .SetPANId = MAC_SetPANId,
    .GetPANId = MAC_GetPANId,
    .SetShortAddress = MAC_SetShortAddress,
    .GetShortAddress = MAC_GetShortAddress,
    .SetExtendedAddress = MAC_SetExtendedAddress,
    .GetExtendedAddress = MAC_GetExtendedAddress,
    .SetKey = MAC_SetKey,
    .GetKey = MAC_GetKey,
    .SetChannel = MAC_SetChannel,
    .GetChannel = MAC_GetChannel,
    .SetRxOnWhenIdle = MAC_SetRxOnWhenIdle,
    .GetRxOnWhenIdle = MAC_GetRxOnWhenIdle,
    .SetPanSecurity = MAC_SetPanSecurity,
    .SetNeighborSecurity = MAC_SetNeighborSecurity,
    .SetMlmeCallbacks = MAC_SetMlmeCallbacks,
    .SetFrameCounter = MAC_SetFrameCounter,
    .GetFrameCounter = MAC_GetFrameCounter,
    .GetNeighborFrameCounter = MAC_GetNeighborFrameCounter
};

static macAbsCallbacks_t mMacCallbackFunctions = {NULL};

#ifdef NWKDBG_STATS_ENABLED
macAbsStats_t macAbsStats = {0};

#ifdef NWKDBG_TRACKTIME
static uint32_t time_idx = 0;
static code_time_t code_time[CODE_TIME_MAX_ENTRIES] = {0};
#endif

#endif

bool_t gEnable802154TxLed=FALSE;
/*==================================================================================================
Public global variables declarations
==================================================================================================*/

///TODO: Will be removed when this function is added to the MAC interface.
extern uint16_t mlmeGetSizeOfPIB(pibId_t  pib);

/*==================================================================================================
Public functions
==================================================================================================*/
instanceId_t MAC_RegisterAbsCb_802154(instanceId_t macInstanceId)
{
    Mac_RegisterSapHandlers(MCPS_NWK_SapHandlerCB, MLME_NWK_SapHandlerCB, macInstanceId);

    return (instanceId_t)&macAbsReq;
}

static void MAC_McpsDataReq
(
    macAbsMcpsDataReq_t * pParam,
    instanceId_t instanceId
)
{
    nwkToMcpsMessage_t *pNwkToMcpsMsg;
    resultType_t result;

#ifdef NWKDBG_STATS_ENABLED
    macAbsStats.mNbDataReq++;

#ifdef NWKDBG_TRACKTIME
    code_time[time_idx].dataReqTime = TMR_GetTimestamp();
    code_time[time_idx].deltaTime = code_time[time_idx].dataReqTime - code_time[time_idx].dataIndTime;
    time_idx++;
    if(time_idx > CODE_TIME_MAX_ENTRIES-1) time_idx = 0;
#endif

#endif

#if gMacFreeRequestInline
    pNwkToMcpsMsg = MEM_BufferAlloc(sizeof(nwkToMcpsMessage_t) + pParam->msduLength);

    pNwkToMcpsMsg->msgData.dataReq.pMsdu = (uint8_t*)pNwkToMcpsMsg + sizeof(nwkToMcpsMessage_t);
    FLib_MemCpy(pNwkToMcpsMsg->msgData.dataReq.pMsdu, pParam->pMsdu, pParam->msduLength);

    /* do not free the pMcpsDataReq->pMsdu here because it needs
       to be keept in the TX queue in the 6LoWPAN module */
    //MEM_BufferFree(pParam->pMsdu);
#else
    pNwkToMcpsMsg = MEM_BufferAlloc(sizeof(nwkToMcpsMessage_t));
    pNwkToMcpsMsg->msgData.dataReq.pMsdu = pParam->pMsdu;
#endif

    pNwkToMcpsMsg->msgType = gMcpsDataReq_c;
    pNwkToMcpsMsg->msgData.dataReq.srcPanId = MAC_GetPANId(instanceId);

    if (gMacAbsAddrModeShortAddress_c == pParam->srcAddrMode)
    {
        pNwkToMcpsMsg->msgData.dataReq.srcAddr = MAC_GetShortAddress(instanceId);
    }
    else if (gMacAbsAddrModeExtendedAddress_c == pParam->srcAddrMode)
    {
        pNwkToMcpsMsg->msgData.dataReq.srcAddr = MAC_GetExtendedAddress(instanceId);
    }

    pNwkToMcpsMsg->msgData.dataReq.dstAddr       = pParam->dstAddr;
    pNwkToMcpsMsg->msgData.dataReq.dstAddrMode   = (addrModeType_t)pParam->dstAddrMode;
    pNwkToMcpsMsg->msgData.dataReq.dstPanId      = pParam->dstPANId;
    pNwkToMcpsMsg->msgData.dataReq.keyIdMode     = (keyIdModeType_t)pParam->keyIdMode;
    pNwkToMcpsMsg->msgData.dataReq.keyIndex      = pParam->keyIndex;
    pNwkToMcpsMsg->msgData.dataReq.msduHandle    = pParam->msduHandle;
    pNwkToMcpsMsg->msgData.dataReq.msduLength    = pParam->msduLength;
    pNwkToMcpsMsg->msgData.dataReq.securityLevel = (macSecurityLevel_t)pParam->securityLevel;
    pNwkToMcpsMsg->msgData.dataReq.srcAddrMode   = (addrModeType_t)pParam->srcAddrMode;
    pNwkToMcpsMsg->msgData.dataReq.txOptions     = (macTxOptions_t)pParam->txOptions;
    pNwkToMcpsMsg->msgData.dataReq.keySource     = pParam->keySource;

    /* do not free the pMcpsDataReq here because it needs
       to be keept in the TX queue in the 6LoWPAN module */
    //MEM_BufferFree(pParam);

    result = NWK_MCPS_SapHandler(pNwkToMcpsMsg, instanceId);

    /* Since the MAC abstraction function doesn`t return a synchronous status, a DataCnf is called. */
    if (gSuccess_c != result)
    {

#ifdef NWKDBG_STATS_ENABLED
        macAbsStats.mNbDataReqFail++;
#endif

        /* Free MCPS Data Request */
        MEM_BufferFree(pNwkToMcpsMsg);

        /* Send MCPS Data Confirm */
        macAbsMcpsDataCnf_t *pMcpsDataCnf = MEM_BufferAlloc(sizeof(macAbsMcpsDataCnf_t));

        pMcpsDataCnf->instanceId = instanceId;
        pMcpsDataCnf->msduHandle = pNwkToMcpsMsg->msgData.dataReq.msduHandle;
        pMcpsDataCnf->status     = (macAbsResultType_t)result;
        pMcpsDataCnf->timestamp  = 0;                                   /*Timestamp unavailable. */

        SLWP_McpsDataCnfCB(pMcpsDataCnf, instanceId);
    }
    else
    {
#ifdef NWKDBG_STATS_ENABLED
        macAbsStats.mNbDataReqSuccess++;
#endif
        
#if gUSBKW24D512Dongle
        MacTxLedToggle();
#else
        MacTxLedOn();
#endif        
    }
}

static macAbsMlmeSetCnf_t MAC_MlmeSetReq(macAbsMlmeSetReq_t * pParam, instanceId_t instanceId)
{
    mlmeMessage_t mlmeMsg = {.msgType = gMlmeSetReq_c};
    resultType_t result;

    mlmeMsg.msgData.setReq.pibAttribute = pParam->pibAttribute;
    mlmeMsg.msgData.setReq.pibAttributeIndex = pParam->pibAttributeIndex;
    mlmeMsg.msgData.setReq.pibAttributeValue = pParam->pPibAttributeValue;
    result = NWK_MLME_SapHandler(&mlmeMsg, instanceId);

    return (macAbsMlmeSetCnf_t)result;
}

static macAbsMlmeGetCnf_t * MAC_MlmeGetReq
(
    macAbsMlmeGetReq_t * pParams,
    instanceId_t instanceId
)
{
    mlmeMessage_t mlmeMsg = {.msgType = gMlmeGetReq_c};
    macAbsMlmeGetCnf_t * pMacAdaptMlmeGetCnf;
    uint8_t * pAttributeValue;
    resultType_t result;

    pMacAdaptMlmeGetCnf = (macAbsMlmeGetCnf_t *)MEM_BufferAlloc(sizeof(macAbsMlmeGetCnf_t));
    pAttributeValue = (uint8_t *)MEM_BufferAlloc(mlmeGetSizeOfPIB(pParams->pibAttribute));

    mlmeMsg.msgData.getReq.pibAttributeIndex = pParams->pibAttributeIndex;
    mlmeMsg.msgData.getReq.pibAttribute = pParams->pibAttribute; ///TODO: Verify if the lists of PIB Attributes are in sync.
    mlmeMsg.msgData.getReq.pibAttributeValue = pAttributeValue;
    result = NWK_MLME_SapHandler(&mlmeMsg, instanceId);

    pMacAdaptMlmeGetCnf->pibAttribute = pParams->pibAttribute;
    pMacAdaptMlmeGetCnf->pibAttributeIndex = pParams->pibAttributeIndex;
    pMacAdaptMlmeGetCnf->pPibAttributeValue = pAttributeValue;
    pMacAdaptMlmeGetCnf->status = (macAbsResultType_t)result;

    return pMacAdaptMlmeGetCnf;
}

static macAbsMlmeResetCnf_t MAC_MlmeResetReq
(
    macAbsMlmeResetReq_t * pParam,
    instanceId_t instanceId
)
{
    macAbsMlmeResetCnf_t mlmeResetCnf = gMacAbsSuccess_c;


    return (macAbsMlmeResetCnf_t)mlmeResetCnf;
}

static macAbsMlmeScanCnf_t MAC_MlmeScanReq
(
    macAbsMlmeScanReq_t * pParam,
    instanceId_t instanceId
)
{

    resultType_t result;
    macAbsMlmeScanCnf_t scanCnf;
    mlmeMessage_t* mlmeMsg = MEM_BufferAlloc(sizeof(mlmeMessage_t));
    mlmeScanReq_t *pScanReq;
    

    mlmeMsg->msgType = gMlmeScanReq_c;
    pScanReq = &mlmeMsg->msgData.scanReq;

    pScanReq->scanType = gScanModeActive_c;
    pScanReq->scanChannels = 1UL << pParam->scanChannel;
    pScanReq->scanDuration = pParam->scanDuration;
    pScanReq->securityLevel = gMacSecurityNone_c;

    result = NWK_MLME_SapHandler(mlmeMsg, instanceId);

    scanCnf.instanceId = instanceId;
    scanCnf.status = (macAbsResultType_t)result;
    
    return scanCnf;
}

static macAbsMlmeStartCnf_t MAC_MlmeStartReq
(
    macAbsMlmeStartReq_t * pParam,
    instanceId_t instanceId
)
{

    resultType_t result;
    mlmeMessage_t* mlmeMsg = MEM_BufferAlloc(sizeof(mlmeMessage_t));
    mlmeStartReq_t *pStartReq;

    mlmeMsg->msgType = gMlmeStartReq_c;
    pStartReq = &mlmeMsg->msgData.startReq;

    pStartReq->beaconOrder = 0x0F;
    pStartReq->superframeOrder = 0x0F;
    pStartReq->panCoordinator = FALSE;
    pStartReq->beaconSecurityLevel = gMacSecurityNone_c;
    pStartReq->batteryLifeExtension = FALSE;
    pStartReq->coordRealignment = FALSE;
    pStartReq->logicalChannel = pParam->channel;
    pStartReq->channelPage = gChannelPageId0_c;
    pStartReq->panId = pParam->panId;

    result = NWK_MLME_SapHandler(mlmeMsg, instanceId);

    return (macAbsMlmeStartCnf_t)result;
}

static void MAC_MlmePollReq
(
    macAbsMlmePollReq_t * pParam,
    instanceId_t instanceId
)
{
    resultType_t result;
    mlmeMessage_t *pMlmeMsg = MEM_BufferAlloc(sizeof(mlmeMessage_t));
    mlmePollReq_t *pPollReq;

    pMlmeMsg->msgType = gMlmePollReq_c;
    pPollReq = &pMlmeMsg->msgData.pollReq;

    pPollReq->coordAddrMode = (addrModeType_t)pParam->coordAddrMode;
    pPollReq->coordPanId = pParam->coordPanId;
    pPollReq->coordAddress = pParam->coordAddress;
    pPollReq->securityLevel = (macSecurityLevel_t)pParam->securityLevel;
    pPollReq->keyIdMode = (keyIdModeType_t)pParam->keyIdMode;
    pPollReq->keySource = pParam->keySource;
    pPollReq->keyIndex = pParam->keyIndex;

    result = NWK_MLME_SapHandler(pMlmeMsg, instanceId);

    if (gSuccess_c != result)
    {
        /* Free Request */
        MEM_BufferFree(pMlmeMsg);

        /* Send Confirm */
        if (mMacCallbackFunctions.mlmePollCnf)
        {
            macAbsMlmePollCnf_t *pMlmePollCnf = MEM_BufferAlloc(sizeof(macAbsMlmePollCnf_t));

            pMlmePollCnf->status = (macAbsResultType_t)result;

            mMacCallbackFunctions.mlmePollCnf(pMlmePollCnf);
        }
    }
    else
    {
#if gUSBKW24D512Dongle
        MacTxLedToggle();
#else
        MacTxLedOn();
#endif 
    }
}

/*!*************************************************************************************************
\fn    void MAC_SetMlmeCallbacks(uint16_t shortAddress)
\brief Public interface function for the MAC abstraction module. This function sets the callbacks for
       the MLME asynchronous indication and confirm functions.

\param [in]  mlmeCallbacks       Structure congaing MLME callbacks
\param [in]  instanceId          Mac instance ID

\param [in]  none
***************************************************************************************************/
void MAC_SetMlmeCallbacks
(
    macAbsCallbacks_t* pMlmeCallbacks,
    instanceId_t instanceId
)
{
    if (pMlmeCallbacks->mlmeBeaconNotifyInd)
    {
        mMacCallbackFunctions.mlmeBeaconNotifyInd = pMlmeCallbacks->mlmeBeaconNotifyInd;
    }
    if (pMlmeCallbacks->mlmeCommStatusInd)
    {
        mMacCallbackFunctions.mlmeCommStatusInd = pMlmeCallbacks->mlmeCommStatusInd;
    }
    if (pMlmeCallbacks->mlmeScanCnf)
    {
        mMacCallbackFunctions.mlmeScanCnf = pMlmeCallbacks->mlmeScanCnf;
    }
    if (pMlmeCallbacks->mlmeSyncLossInd)
    {
        mMacCallbackFunctions.mlmeSyncLossInd = pMlmeCallbacks->mlmeSyncLossInd;
    }
    if (pMlmeCallbacks->mlmePollCnf)
    {
        mMacCallbackFunctions.mlmePollCnf = pMlmeCallbacks->mlmePollCnf;
    }
}

/*!*************************************************************************************************
\fn    void MAC_SetPANId(uint16_t panId)
\brief Public interface function for the MAC abstraction module. This function sets the PAN Id.

\param [in]   panId             New PAN Id of the node.
***************************************************************************************************/
bool_t MAC_SetPANId
(
    uint16_t panId,
    instanceId_t instanceId
)
{
    return MAC_SetPib(gMPibPanId_c, &panId, instanceId);
}

/*!*************************************************************************************************
\fn    void MAC_GetPANId(uint16_t panId)
\brief Public interface function for the MAC abstraction module. This function gets the PAN Id.

\return       uint16_t          The PAN identifier of the device.
***************************************************************************************************/
uint16_t MAC_GetPANId
(
    instanceId_t instanceId
)
{
    uint16_t panId = 0;

    (void)MAC_GetPib(gMPibPanId_c, &panId, instanceId);

    return panId;
}

/*!*************************************************************************************************
\fn    void MAC_SetShortAddress(uint16_t shortAddress)
\brief Public interface function for the MAC abstraction module. This function sets the node short
       address.

\param [in]  shortAddress       Short address of the node.
***************************************************************************************************/
bool_t MAC_SetShortAddress
(
    uint16_t shortAddress,
    instanceId_t instanceId
)
{
    return MAC_SetPib(gMPibShortAddress_c, &shortAddress, instanceId);
}

/*!*************************************************************************************************
\fn    uint16_t MAC_GetShortAddress(void)
\brief Public interface function for the MAC abstraction module. This function gets the short
       address.

\return       uint16_t          The short link layer address of the device.
***************************************************************************************************/
uint16_t MAC_GetShortAddress
(
    instanceId_t instanceId
)
{
    uint16_t shortAddress = 0;

    (void)MAC_GetPib(gMPibShortAddress_c, &shortAddress, instanceId);

    return shortAddress;
}

/*!*************************************************************************************************
\fn    bool_t MAC_GetExtendedAddress(uint64_t * pExtAddress)
\brief Public interface function for the MAC abstraction module. This function gets the extended
       address.

\param [out]  pExtAddress             Extended address of the node. Invalid if FALSE is returned.

\return       bool_t                  Returns FALSE if operation has failed.
***************************************************************************************************/
bool_t MAC_SetExtendedAddress
(
    uint64_t extendedAddress,
    instanceId_t instanceId
)
{
    return MAC_SetPib(gMacPibExtendedAddress_c, &extendedAddress, instanceId);
}

/*!*************************************************************************************************
\fn    bool_t MAC_GetExtendedAddress(uint64_t * pExtAddress)
\brief Public interface function for the MAC abstraction module. This function gets the extended
       address.

\param [in] instanceId      mac instance id

\return     uint64_t        mac extended address
***************************************************************************************************/
uint64_t MAC_GetExtendedAddress
(
    instanceId_t instanceId
)
{
    uint64_t extendedAddress = 0;

    (void)MAC_GetPib(gMacPibExtendedAddress_c, &extendedAddress, instanceId);

    return extendedAddress;
}

/*!*************************************************************************************************
\fn     void MAC_SetKey(uint8_t idxInKeyTable, uint8_t* key, uint8_t keyIndex, instanceId_t instanceId)
\brief  This function is used for setting the Security Key into MAC security PIB

\param [in] idxInKeyTable   index in Key table
\param [in] key             pointer to Key
\param [in] keyIndex        key index
\param [in] instanceId      mac Instance

\return     bool_t         returns FALSE if the operation failed
***************************************************************************************************/
bool_t MAC_SetKey
(
    uint8_t      idxInKeyTable,
    uint8_t*     key,
    uint8_t      keyIndex,
    instanceId_t instanceId
)
{
    uint8_t lookupData[]    = {MAC_DEFAULT_KEY_SOURCE, 0x00};
    bool_t  result          = MAC_SetPib(gMPibiKeyTableCrtEntry_c, &idxInKeyTable, instanceId);

    if(TRUE == result)
    {
        result = MAC_SetPib(gMPibKey_c, key, instanceId);
    }

    if(TRUE == result)
    {
        lookupData[8]   = keyIndex;
        result          = MAC_SetPib(gMPibKeyIdLookupData_c, lookupData, instanceId);
    }
    
    if(TRUE == result)
    {
        MAC_SetPib(gMPibAutoRequestKeyIndex_c, &keyIndex, instanceId);
    }

    return result;
}

/*!*************************************************************************************************
\fn     void MAC_GetKey(uint8_t idxInKeyTable, uint8_t* key, instanceId_t instanceId)
\brief  This function is used for getting the Security Key from MAC security PIB

\param [in] idxInKeyTable   index in Key table
\param [in] key             pointer to put the key value
\param [in] instanceId      mac Instance

\return     bool_t         returns FALSE if the operation failed
***************************************************************************************************/
bool_t MAC_GetKey
(
    uint8_t      idxInKeyTable,
    uint8_t*     key,
    instanceId_t instanceId
)
{
    bool_t result = MAC_SetPib(gMPibiKeyTableCrtEntry_c, &idxInKeyTable, instanceId);

    if(TRUE == result)
    {
        result = MAC_GetPib(gMPibKey_c, key, instanceId);
    }

    return result;
}

/*!*************************************************************************************************
\fn    bool_t MAC_SetChannel(uint8_t channel, instanceId_t instanceId)
\brief Public interface function for the MAC abstraction module. This function sets the channel.

\param [in] channel         logical channel
\param [in] instanceId      mac Instance

\return     bool_t         returns FALSE if the operation failed
***************************************************************************************************/
bool_t MAC_SetChannel
(
    uint8_t channel,
    instanceId_t instanceId
)
{
    return MAC_SetPib(gMPibLogicalChannel_c, &channel, instanceId);
}

/*!*************************************************************************************************
\fn    bool_t MAC_GetChannel(instanceId_t instanceId)
\brief Public interface function for the MAC abstraction module. This function gets the channel.

\param [in] instanceId      mac instance id

\return     uint8_t         logical channel
***************************************************************************************************/
uint8_t MAC_GetChannel
(
    instanceId_t instanceId
)
{
    uint8_t channel = 0;

    (void)MAC_GetPib(gMPibLogicalChannel_c, &channel, instanceId);

    return channel;
}

/*!*************************************************************************************************
\fn    bool_t MAC_SetRxOnWhenIdle(bool_t bRxOnWhenIdle, instanceId_t instanceId)
\brief Public interface function for the MAC abstraction module. This function sets the receiver
       state.

\param [in] bRxOnWhenIdle   receiver state
\param [in] instanceId      mac Instance

\return     bool_t         returns FALSE if the operation failed
***************************************************************************************************/
bool_t MAC_SetRxOnWhenIdle
(
    bool_t bRxOnWhenIdle,
    instanceId_t instanceId
)
{
    return MAC_SetPib(gMPibRxOnWhenIdle_c, &bRxOnWhenIdle, instanceId);
}

/*!*************************************************************************************************
\fn    bool_t MAC_GetRxOnWhenIdle(instanceId_t instanceId)
\brief Public interface function for the MAC abstraction module. This function gets the receiver
state.

\param [in] instanceId      mac instance id

\return     uint8_t         logical channel
***************************************************************************************************/
bool_t MAC_GetRxOnWhenIdle
(
    instanceId_t instanceId
)
{
    bool_t bRxOnWhenIdle = FALSE;

    (void)MAC_GetPib(gMPibRxOnWhenIdle_c, &bRxOnWhenIdle, instanceId);

    return bRxOnWhenIdle;
}


/*!*************************************************************************************************
\fn     void MAC_GetMaxMsduSize(macAbsGetMaxMsduSizeReq_t *pParam, instanceId_t instanceId)
\brief  This function is used for getting the maximum available MAC payload for a given set of
        transmission parameters

\param [in]   pParam    pointer to the data request settings
\param [in]   pGmk      mac instance id
***************************************************************************************************/
uint32_t MAC_GetMaxMsduSize
(
    macAbsMcpsDataReq_t *pParam,
    instanceId_t instanceId
)
{
#if (MAC_GET_MAX_MSDU_SIZE)
    mcpsDataReq_t * pMcpsDataReq = MEM_BufferAlloc(sizeof(mcpsDataReq_t));

    pMcpsDataReq->srcPanId      = MAC_GetPANId(instanceId);
    pMcpsDataReq->srcAddrMode   = (addrModeType_t)pParam->srcAddrMode;
    pMcpsDataReq->dstAddrMode   = (addrModeType_t)pParam->dstAddrMode;
    pMcpsDataReq->dstPanId      = pParam->dstPANId;
    pMcpsDataReq->dstAddr       = pParam->dstAddr;
    //pMcpsDataReq->msduLength    = pParam->msduLength;
    //pMcpsDataReq->msduHandle    = pParam->msduHandle;
    pMcpsDataReq->txOptions     = (macTxOptions_t)pParam->txOptions;
    pMcpsDataReq->securityLevel = (macSecurityLevel_t)pParam->securityLevel;
    pMcpsDataReq->keyIdMode     = (keyIdModeType_t)pParam->keyIdMode;
    pMcpsDataReq->keySource     = pParam->keySource;
    pMcpsDataReq->keyIndex      = pParam->keyIndex;

    uint32_t maxMsduSize = Mac_GetMaxMsduLength(pMcpsDataReq);

    MEM_BufferFree(pMcpsDataReq);

    return maxMsduSize;
#else
    return (uint32_t)MAC_MAX_MSDU_SIZE_802154;
#endif
}

/*!*************************************************************************************************
\fn     void MAC_SetPanSecurity(uint8_t maxNbOfKeys, uint8_t maxNbOfNeighbors, instanceId_t instanceId)
\brief  This function is used for setting the Pan security information in MAC security PIBs

\param [in] maxNbOfKeys         maximum number of keys
\param [in] maxNbOfNeighbors    maximum number of neighbors
\param [in] index               index in the macKeyTable where the security material should be added
\param [in] instanceId          mac Instance

\return     none
***************************************************************************************************/
void MAC_SetPanSecurity
(
     uint8_t      maxNbOfKeys,
     uint8_t      maxNbOfNeighbors,
     uint8_t      index,
     instanceId_t instanceId
)
{
    uint8_t     defaultKeySource[]  = {MAC_DEFAULT_KEY_SOURCE};
    uint32_t    frameCounter        = 0;
    uint8_t     keyIdx, neighborIdx;
    uint8_t     pibValue;

    /* General Security PIBs */

    (void)MAC_SetPibUint8Val(gMPibSecurityEnabled_c,                         1,                  instanceId);
    (void)MAC_SetPib(gMPibDefaultKeySource_c,                                &defaultKeySource,  instanceId);
    (void)MAC_SetPib(gMPibFrameCounter_c,                                    &frameCounter,      instanceId);
    
    /* Auto Request security settings */
    /*
        Setting the gMPibAutoRequestKeySource_c is not required since Key Id
        mode 1 uses the macDefaultKeySource.
    */
#if MAC_SEC_POLL_ENABLED
    pibValue = gMacAbsMacSecurityEncMic32_c;
#else
    pibValue = gMacAbsMacSecurityDisabled_c;
#endif
    MAC_SetPib(gMPibAutoRequestSecurityLevel_c,                             &pibValue,            instanceId);
    pibValue = gMacAbsKeyIdMode1_c;
    MAC_SetPib(gMPibAutoRequestKeyIdMode_c,                                 &pibValue,            instanceId);

    /* Key Table */

    for(keyIdx = 0; keyIdx < maxNbOfKeys; keyIdx++)
    {
      /* cosmin: Check if the other entries have to be updated, otherwise remove the for */

        if(keyIdx != index)
        {
          continue;
        }

        (void)MAC_SetPib(gMPibiKeyTableCrtEntry_c,                           &keyIdx,            instanceId);

        /* KeyDescriptor - KeyIdLookupList */

        (void)MAC_SetPibUint8Val(gMPibiKeyIdLookuplistCrtEntry_c,            0,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibKeyIdLookupDataSize_c,                 1,                  instanceId);

        /* KeyDescriptor - KeyDeviceList */

        for(neighborIdx = 0; neighborIdx < maxNbOfNeighbors; neighborIdx++)
        {
            /* Index in the macDeviceTable */
            uint8_t devDescHandle = index * maxNbOfNeighbors + neighborIdx;
            uint8_t frameCounter = 0;

            (void)MAC_SetPib(gMPibiKeyDeviceListCrtEntry_c,                  &neighborIdx,       instanceId);
            (void)MAC_SetPib(gMPibKeyDeviceDescriptorHandle_c,               &devDescHandle,     instanceId);
            (void)MAC_SetPibUint8Val(gMPibUniqueDevice_c,                    0,                  instanceId);
            (void)MAC_SetPibUint8Val(gMPibBlackListed_c,                     0,                  instanceId);

            /* Reset the frame counter in the macDeviceTable */
            (void)MAC_SetPib(gMPibiDeviceTableCrtEntry_c,           &devDescHandle,    instanceId);
            (void)MAC_SetPib(gMPibDeviceDescriptorFrameCounter_c,   &frameCounter,      instanceId);
        }

        /* KeyDescriptor - KeyUsageList */
        
        /* Data Frames entry */
        (void)MAC_SetPibUint8Val(gMPibiKeyUsageListCrtEntry_c,               0,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibKeyUsageFrameType_c,                   1,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibKeyUsageCommnadFrameIdentifier_c,      0,                  instanceId);
        
        /* Data Request command entry */
        (void)MAC_SetPibUint8Val(gMPibiKeyUsageListCrtEntry_c,               1,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibKeyUsageFrameType_c,                   3,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibKeyUsageCommnadFrameIdentifier_c,      4,                  instanceId);
    
        /* Key Table - Security Level Table */

        (void)MAC_SetPibUint8Val(gMPibiSecurityLevelTableCrtEntry_c,          0,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibSecLevFrameType_c,                      1,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibSecLevCommnadFrameIdentifier_c,         0,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibSecLevSecurityMinimum_c,                0,                  instanceId);
        (void)MAC_SetPibUint8Val(gMPibSecLevDeviceOverrideSecurityMinimum_c,  1,                  instanceId);

    }

}


/*!*************************************************************************************************
\fn     void MAC_SetNeighborSecurity(uint8_t neighborIdx, uint64_t extAddr, uint16_t shortAddr, uint16_t panId, uint32_t frameCounter, instanceId_t instanceId)
\brief  This function is used for setting the neighbor information in MAC security PIBs

\param [in] neighborIdx     Index in deviceTable
\param [in] maxNeighbors    Maximum number of neighbors
\param [in] macKeyIdx       Index of the active MAC key
\param [in] maxMacKeys      Maximum number of MAC keys
\param [in] extAddr         neighbor extended address
\param [in] shortAddr       neighbor short address
\param [in] panId           neighbor pan identifier
\param [in] frameCounter    neighbor frame counter
\param [in] instanceId      mac Instance

\return     none
***************************************************************************************************/
void MAC_SetNeighborSecurity
(
    uint8_t         neighborIdx,
    uint8_t         maxNeighbors,
    uint8_t         macKeyIdx,
    uint8_t         maxMacKeys,
    uint64_t        extAddr,
    uint16_t        shortAddr,
    uint16_t        panId,
    uint32_t        frameCounter,
    instanceId_t    instanceId
)
{
    uint8_t i, devDescIdx;

    for(i = 0; i < maxMacKeys; i++)
    {
        devDescIdx = i * maxNeighbors + neighborIdx;

        (void)MAC_SetPib(gMPibiDeviceTableCrtEntry_c,           &devDescIdx,    instanceId);
        (void)MAC_SetPib(gMPibDeviceDescriptorExtAddress_c,     &extAddr,           instanceId);
        (void)MAC_SetPib(gMPibDeviceDescriptorShortAddress_c,   &shortAddr,         instanceId);
        (void)MAC_SetPib(gMPibDeviceDescriptorPanId_c,          &panId,             instanceId);
        (void)MAC_SetPibUint8Val(gMPibDeviceDescriptorExempt,   1,                  instanceId);

        if(i !=  macKeyIdx)
        {
            continue;
        }

        (void)MAC_SetPib(gMPibDeviceDescriptorFrameCounter_c,   &frameCounter,      instanceId);
    }
}


#if 0
bool_t MAC_GetNeighborTableEntry(macNeighborTableEntry_T * nt_entry, instanceId_t instanceId)
{
    return TRUE;
}

bool_t MAC_SetNeighborTableEntry(macNeighborTableEntry_T * nt_entry, instanceId_t instanceId)
{
    return FALSE;
}

/*!*************************************************************************************************
\fn     bool MAC_GetNeighborTableEntryByAddress(uint16_t shortAddress,
                                                macNeighborTableEntry_T* pNeighborTableEntry)
\brief  This function is used to verify if a device is present in the MAC Neighbor Table. If it is
present, the corresponding Neighbor Table Entry is returned

\param  [in]   shortAddress             Short address of the device.
\param  [out]  pNeighborTableEntry      Pointer to Neighbor Table Entry to be written (must be
                                        already allocated)


\retval        true                     Device is in the MAC neighbor table.
\retval        FALSE                    Device is not present in the MAC neighbor table.
***************************************************************************************************/
bool_t MAC_GetNeighborTableEntryByAddress
(
    uint16_t shortAddress,
    macNeighborTableEntry_T* pNeighborTableEntry,
    instanceId_t instanceId
)
{
    return FALSE;
}
#endif
/*==================================================================================================
Private functions
==================================================================================================*/

static resultType_t MCPS_NWK_SapHandlerCB
(
    mcpsToNwkMessage_t* pMsg,
    instanceId_t instanceId
)
{
    switch (pMsg->msgType)
    {
        case gMcpsDataCnf_c:
            ///TODO: Possible Missalignement Issue !!!
            MAC_McpsDataCnfCB(pMsg, instanceId);
            break;

        case gMcpsDataInd_c:
            ///TODO: Possible Missalignement Issue !!!
            MAC_McpsDataIndCB(pMsg, instanceId);
            break;

        default:
            break;
    }

    ///TODO: Possible issue -> may not free all the allocated memory for unhandled messages.
    /* Free Message */
    MEM_BufferFree(pMsg);

    return gSuccess_c;
}

static resultType_t MLME_NWK_SapHandlerCB
(
    nwkMessage_t* pMsg,
    instanceId_t instanceId
)
{
    switch (pMsg->msgType)
    {
        case gMlmeScanCnf_c:
            MAC_MlmeScanCnfCB(&pMsg->msgData.scanCnf, instanceId);
            break;

        case gMlmeBeaconNotifyInd_c:
            MAC_MlmeBeaconNotifyIndCB(&pMsg->msgData.beaconNotifyInd, instanceId);
            break;

        case gMlmeCommStatusInd_c:
            MAC_MlmeCommStatusIndCB(&pMsg->msgData.commStatusInd, instanceId);

        case gMlmePollCnf_c:
            MAC_MlmePollCnfCB(&pMsg->msgData.pollCnf, instanceId);
            break;

        case gMlmePollNotifyInd_c:
            MAC_MlmePollIndCB(&pMsg->msgData.pollNotifyInd, instanceId);
            break;

        default:
            break;
    }

    ///TODO: Possible issue -> may not free all the allocated memory for unhandled messages.
    /* Free Message */
    MEM_BufferFree(pMsg);

    return gSuccess_c;
}

static void MAC_McpsDataIndCB
(
    mcpsToNwkMessage_t* pMsg,
    instanceId_t instanceId
)
{
#ifdef NWKDBG_TRACKTIME
    code_time[time_idx].dataIndTime = TMR_GetTimestamp();
#endif

    if(TRUE == MacFiltering_KeepPacket((macAbsAddrModeType_t)pMsg->msgData.dataInd.srcAddrMode,
                                        pMsg->msgData.dataInd.srcAddr, &pMsg->msgData.dataInd.mpduLinkQuality))
    {
        macAbsMcpsDataInd_t * pMcpsDataInd = MEM_BufferAlloc(sizeof(macAbsMcpsDataInd_t));

        /* Populate the MAC abstraction structure */
        pMcpsDataInd->instanceId       = instanceId;
        pMcpsDataInd->srcAddrMode      = (macAbsAddrModeType_t)pMsg->msgData.dataInd.srcAddrMode;
        pMcpsDataInd->dstAddrMode      = (macAbsAddrModeType_t)pMsg->msgData.dataInd.dstAddrMode;
        pMcpsDataInd->dstPANId         = pMsg->msgData.dataInd.dstPanId;
        pMcpsDataInd->dstAddr          = pMsg->msgData.dataInd.dstAddr;
        pMcpsDataInd->srcAddr          = pMsg->msgData.dataInd.srcAddr;
        pMcpsDataInd->srcPANId         = pMsg->msgData.dataInd.srcPanId;
        pMcpsDataInd->msduLength       = pMsg->msgData.dataInd.msduLength;
        pMcpsDataInd->timestamp        = pMsg->msgData.dataInd.timestamp;
        pMcpsDataInd->mpduLinkQuality  = pMsg->msgData.dataInd.mpduLinkQuality;
        pMcpsDataInd->dsn              = pMsg->msgData.dataInd.dsn;
        pMcpsDataInd->securityLevel    = (macAbsSecurityLevel_t)pMsg->msgData.dataInd.securityLevel;
        pMcpsDataInd->keyIdMode        = (macAbsKeyIdModeType_t)pMsg->msgData.dataInd.keyIdMode;
        pMcpsDataInd->keySource        = pMsg->msgData.dataInd.keySource;
        pMcpsDataInd->keyIndex         = pMsg->msgData.dataInd.keyIndex;
        pMcpsDataInd->qualityOfService = (macAbsQoS_t)0;
        pMcpsDataInd->pMsdu             = MEM_BufferAlloc(pMcpsDataInd->msduLength);

        FLib_MemCpy(pMcpsDataInd->pMsdu, pMsg->msgData.dataInd.pMsdu, pMcpsDataInd->msduLength);

        SLWP_McpsDataIndCB(pMcpsDataInd, instanceId);
    }
}

static void MAC_McpsDataCnfCB
(
    mcpsToNwkMessage_t* pMsg,
    instanceId_t instanceId
)
{
    macAbsMcpsDataCnf_t *pAbsMcpsDataCnf = MEM_BufferAlloc(sizeof(macAbsMcpsDataCnf_t));

    pAbsMcpsDataCnf->instanceId = instanceId;
    pAbsMcpsDataCnf->msduHandle = pMsg->msgData.dataCnf.msduHandle;
    pAbsMcpsDataCnf->status     = (macAbsResultType_t)pMsg->msgData.dataCnf.status;
    pAbsMcpsDataCnf->timestamp  = pMsg->msgData.dataCnf.timestamp;

#ifdef NWKDBG_STATS_ENABLED
    if(gMacAbsSuccess_c != pAbsMcpsDataCnf->status)
    {
        macAbsStats.mNbDataCnfFail++;
    }
    else
    {
        macAbsStats.mNbDataCnfSuccess++;
    }
#endif

    SLWP_McpsDataCnfCB(pAbsMcpsDataCnf, instanceId);

#if gUSBKW24D512Dongle
    MacTxLedToggle();
#else
    MacTxLedOff();
#endif 
}

static void MAC_MlmeBeaconNotifyIndCB
(
    mlmeBeaconNotifyInd_t * pPayload,
    instanceId_t instanceId
)
{   
    macAbsMlmeBeaconNotifyInd_t *pMlmeBeaconInd = MEM_BufferAlloc(pPayload->sduLength +
                                                                  sizeof(macAbsMlmeBeaconNotifyInd_t));

    /* Populate the MAC abstraction structure */
    pMlmeBeaconInd->instanceId = instanceId;
    pMlmeBeaconInd->panDescriptor.shortAddress  = pPayload->pPanDescriptor->coordAddress;
    pMlmeBeaconInd->panDescriptor.coordPANId    = pPayload->pPanDescriptor->coordPanId;
    pMlmeBeaconInd->panDescriptor.linkQuality   = pPayload->pPanDescriptor->linkQuality;
    pMlmeBeaconInd->panDescriptor.rcCoord       = 0;
    pMlmeBeaconInd->beaconPloadSize             = pPayload->sduLength;
    FLib_MemCpy(pMlmeBeaconInd->aBeaconPload, pPayload->pSdu, pMlmeBeaconInd->beaconPloadSize);

    /* Free internal elements */
    MEM_BufferFree(pPayload->pBufferRoot);

    if(mMacCallbackFunctions.mlmeBeaconNotifyInd)
    {
        /* Call the registered callback */
        mMacCallbackFunctions.mlmeBeaconNotifyInd(pMlmeBeaconInd);
    }
}

static void MAC_MlmeScanCnfCB
(
    mlmeScanCnf_t *pPayload,
    instanceId_t instanceId
)
{
    macAbsMlmeScanCnf_t mlmeScanCnf;

    /* Populate the MAC abstraction structure */
    mlmeScanCnf.status = (macAbsResultType_t)pPayload->status;
    mlmeScanCnf.instanceId = instanceId;

    if(mMacCallbackFunctions.mlmeScanCnf)
    {
        /* Call the registered callback */
        mMacCallbackFunctions.mlmeScanCnf(&mlmeScanCnf);
    }

    /* Free the PAN Descriptor list */
    panDescriptorBlock_t *pNextDescriptor = pPayload->resList.pPanDescriptorBlockList;
    while (pNextDescriptor)
    {
        panDescriptorBlock_t *pTempNextDescriptor = pNextDescriptor->pNext;
        MEM_BufferFree(pNextDescriptor);
        pNextDescriptor = pTempNextDescriptor;
    }
}

static bool_t MAC_SetPib
(
    uint32_t pibAttribute,
    void* pPibValue,
    instanceId_t instanceId
)
{
    mlmeMessage_t mlmeMsg = {.msgType = gMlmeSetReq_c};

    mlmeMsg.msgData.setReq.pibAttribute         = pibAttribute;
    mlmeMsg.msgData.setReq.pibAttributeIndex    = 0;
    mlmeMsg.msgData.setReq.pibAttributeValue    = pPibValue;
    return (gSuccess_c == NWK_MLME_SapHandler(&mlmeMsg, instanceId)) ? TRUE : FALSE;
}

static bool_t MAC_GetPib
(
    uint32_t pibAttribute,
    void* pPibValue,
    instanceId_t instanceId
)
{
    mlmeMessage_t mlmeMsg = {.msgType = gMlmeGetReq_c};

    mlmeMsg.msgData.setReq.pibAttribute         = pibAttribute;
    mlmeMsg.msgData.setReq.pibAttributeIndex    = 0;
    mlmeMsg.msgData.setReq.pibAttributeValue    = pPibValue;
    return (gSuccess_c == NWK_MLME_SapHandler(&mlmeMsg, instanceId)) ? TRUE : FALSE;
}


static bool_t MAC_SetPibUint8Val
(
    uint32_t pibAttribute,
    uint8_t pibValue,
    instanceId_t instanceId
)
{
    return MAC_SetPib(pibAttribute, &pibValue, instanceId);
}

static void MAC_MlmeCommStatusIndCB
(
    mlmeCommStatusInd_t * pPayload,
    instanceId_t instanceId
)
{
    /* Allocate memory for data. MUST be FREED by Upper Layer */
    macAbsMlmeCommStatusInd_t *pMlmeCommStatusInd = MEM_BufferAlloc(sizeof(macAbsMlmeCommStatusInd_t));
    
    /* Populate the MAC abstraction structure */
    pMlmeCommStatusInd->dstAddr        = pPayload->destAddress;
    pMlmeCommStatusInd->dstAddrMode    = (macAbsAddrModeType_t)pPayload->destAddrMode;
    pMlmeCommStatusInd->keyIdMode      = (macAbsKeyIdModeType_t)pPayload->keyIdMode;
    pMlmeCommStatusInd->keyIndex       = pPayload->keyIndex;
    pMlmeCommStatusInd->keySource      = pPayload->keySource;
    pMlmeCommStatusInd->panId          = pPayload->panId;
    pMlmeCommStatusInd->securityLevel  = (macAbsSecurityLevel_t)pPayload->securityLevel;
    pMlmeCommStatusInd->srcAddr        = pPayload->srcAddress;
    pMlmeCommStatusInd->srcAddrMode    = (macAbsAddrModeType_t)pPayload->srcAddrMode;
    pMlmeCommStatusInd->status         = (macAbsResultType_t)pPayload->status;
    
    pMlmeCommStatusInd->macInstanceId  = instanceId;

    /* Call the registered callback */
    if (mMacCallbackFunctions.mlmeCommStatusInd)
    {
        mMacCallbackFunctions.mlmeCommStatusInd(pMlmeCommStatusInd);
    }
}

static void MAC_MlmePollIndCB
(
    mlmePollNotifyInd_t *pPayload,
    instanceId_t instanceId
)
{
    macAbsMlmePollNotifyInd_t mlmePollInd;

    /* Fix bug in MAC */
    if (gAddrModeShortAddress_c == pPayload->srcAddrMode)
    {
        pPayload->srcAddr &= (uint64_t)0xFFFF;
    }

    mlmePollInd.instanceId = instanceId;
    mlmePollInd.srcAddr = pPayload->srcAddr;
    mlmePollInd.srcAddrMode = (macAbsAddrModeType_t)pPayload->srcAddrMode;
    mlmePollInd.srcPanId = pPayload->srcPanId;

    SLWP_MlmePollIndCB(&mlmePollInd, instanceId);
}

static void MAC_MlmePollCnfCB
(
    mlmePollCnf_t * pPayload,
    instanceId_t instanceId
)
{
    macAbsMlmePollCnf_t mlmePollCnf;

    mlmePollCnf.status = (macAbsResultType_t)pPayload->status;

    if (mMacCallbackFunctions.mlmePollCnf)
    {
        mMacCallbackFunctions.mlmePollCnf(&mlmePollCnf);
    }
    
#if gUSBKW24D512Dongle
    MacTxLedToggle();
#else
    MacTxLedOff();
#endif
}

#if 0
static void MAC_MlmeSyncLossIndCB(mlmeSyncLossInd_t * pPayload, instanceId_t instanceId)
{

    macAbsMlmeSyncLossInd_t mlmeSyncLossInd;

    /* Populate the MAC abstraction structure */
    mlmeSyncLossInd.lossReason = (macAbsResultType_t)pPayload->lossReason;
    mlmeSyncLossInd.panId      = pPayload->panId;

    /* Call the registered callback */
    mMacCallbackFunctions.pMacAbsMlmeSyncLossInd(&mlmeSyncLossInd);

}
#endif

/*!*************************************************************************************************
\fn     bool_t MAC_SetFrameCounter(uint8_t channel, instanceId_t instanceId)
\private
\brief  Public interface function for the MAC abstraction module. This function sets the frame
        counter.

\param  [in]    frameCounter    link layer frame counter
\param  [in]    instanceId      mac Instance

\return         bool_t          returns FALSE if the operation failed
***************************************************************************************************/
bool_t MAC_SetFrameCounter
(
    uint32_t frameCounter,
    instanceId_t instanceId
)
{
    return MAC_SetPib(gMPibFrameCounter_c, &frameCounter, instanceId);
}

/*!*************************************************************************************************
\fn     uint32_t MAC_GetFrameCounter(instanceId_t instanceId)
\private
\brief  Public interface function for the MAC abstraction module. This function gets the frame
        counter.

\param  [in]    instanceId  mac instance id

\return         uint8_t     frame counter
***************************************************************************************************/
uint32_t MAC_GetFrameCounter
(
    instanceId_t instanceId
)
{
    uint32_t frameCounter = 0;

    (void)MAC_GetPib(gMPibFrameCounter_c, &frameCounter, instanceId);

    return frameCounter;
}

static bool_t MAC_GetNeighborFrameCounter
(
    uint32_t       *pFrameCtr,
    uint8_t         neighborIdx,
    uint8_t         maxNeighbors,
    uint8_t         macKeyIdx,
    uint8_t         maxMacKeys,
    instanceId_t    instanceId
)
{
    bool_t bRetValue = FALSE;
    uint8_t devDescIdx;

    devDescIdx = macKeyIdx * maxNeighbors + neighborIdx;

    bRetValue = MAC_SetPib(gMPibiDeviceTableCrtEntry_c, &devDescIdx, instanceId);

    if (bRetValue)
    {
        ///TODO: Does this function return FALSE if no Frame Counter is set for that device?
        bRetValue = MAC_GetPib(gMPibDeviceDescriptorFrameCounter_c, pFrameCtr, instanceId);
    }

    return bRetValue;
}
/*================================================================================================*/

/*==================================================================================================
Private debug functions
==================================================================================================*/
