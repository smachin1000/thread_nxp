/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file FsciMacCommands.c
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
#include "FunctionLib.h"
#include "MemManager.h"

#if gFsciIncluded_c
#include "FsciInterface.h"
#include "FsciCommands.h"
#include "FsciMacCommands.h"
#include "FsciCommunication.h"
#endif

#include "MacInterface.h"
#include "PhyInterface.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
extern uint16_t mlmeGetSizeOfPIB(pibId_t  pib);
void fsciMlmeAsyncReqHandler(void *pData, void* param, uint32_t interfaceId);

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
uint8_t fsciToMacBinding[gMacInstancesCnt_c];
#endif

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief   This function registers the MAC SAP monitors and commands handlers
*
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciRegisterMac( instanceId_t macInterface, uint32_t fsciInterfaceId )
{
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
    
    if( macInterface >= gMacInstancesCnt_c )
        return;

    /* Bind MAC instance to interfaceId */
    fsciToMacBinding[macInterface] = fsciInterfaceId;

    /* Regidter Handler for requests coming from the serial interface */
    FSCI_RegisterOpGroup( gFSCI_NwkMlmeOpcodeGroup_c, 
                          gFsciMonitorMode_c, 
                          fsciMlmeReqHandler, 
                          NULL, 
                          fsciInterfaceId );

    FSCI_RegisterOpGroup( gFSCI_NwkMcpsOpcodeGroup_c, 
                          gFsciMonitorMode_c, 
                          fsciMcpsReqHandler, 
                          NULL, 
                          fsciInterfaceId );

    /* Register SAP Monitor Handler */
    FSCI_RegisterOpGroup( gFSCI_McpsSapId_c,
                          gFsciMonitorMode_c,
                          McpsSapMonitor,
                          NULL,
                          fsciInterfaceId );

    FSCI_RegisterOpGroup( gFSCI_MlmeSapId_c,
                          gFsciMonitorMode_c,
                          MlmeSapMonitor,
                          NULL,
                          fsciInterfaceId );
#endif /* gFsciIncluded_c && gFSCI_IncludeMacCommands_c */
}

void MAC_Monitor(uint8_t sapId, void *pMsg, void* param, uint8_t instanceId )
{
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
   if(gFsciSAPHook_c == FSCI_Monitor(sapId, pMsg, param, fsciGetMacInterfaceId(instanceId)) )
   {
       
   }
#endif
}


#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
/*! *********************************************************************************
* \brief   This is the handler function for the MCPS Requests received over the
           serial interface
*
* \param[in] pData pointer to data location
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciMcpsReqHandler(void *pData, void* param, uint32_t interfaceId)
{
#define pClientPacket ((clientPacket_t*)pData)
    nwkToMcpsMessage_t *pMcpsReq = MEM_BufferAlloc(sizeof(nwkToMcpsMessage_t) + gMaxPHYPacketSize_c);

    if( NULL == pMcpsReq )
    {
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
        MEM_BufferFree( pData );
        return;
    }

    switch( pClientPacket->structured.header.opCode ) {
    case mFsciNwkMcpsDataReq_c:
        {
            uint8_t *p = &pClientPacket->structured.payload[0];

            pMcpsReq->msgType = gMcpsDataReq_c;

            FLib_MemCpy( &pMcpsReq->msgData.dataReq.dstAddr, p,
                        sizeof(pMcpsReq->msgData.dataReq.dstAddr) );
            p += sizeof(pMcpsReq->msgData.dataReq.dstAddr);

            FLib_MemCpy( &pMcpsReq->msgData.dataReq.dstPanId, p,
                         sizeof(pMcpsReq->msgData.dataReq.dstPanId));
            p += sizeof(pMcpsReq->msgData.dataReq.dstPanId);

            pMcpsReq->msgData.dataReq.dstAddrMode = (addrModeType_t)*p;
            p += sizeof(pMcpsReq->msgData.dataReq.dstAddrMode);


            FLib_MemCpy( &pMcpsReq->msgData.dataReq.srcAddr, p,
                        sizeof(pMcpsReq->msgData.dataReq.srcAddr) );
            p += sizeof(pMcpsReq->msgData.dataReq.srcAddr);

            FLib_MemCpy( &pMcpsReq->msgData.dataReq.srcPanId, p,
                         sizeof(pMcpsReq->msgData.dataReq.srcPanId));
            p += sizeof(pMcpsReq->msgData.dataReq.srcPanId);

            pMcpsReq->msgData.dataReq.srcAddrMode   = (addrModeType_t)*p++;
            pMcpsReq->msgData.dataReq.msduLength    = *p++;
            pMcpsReq->msgData.dataReq.msduHandle    = *p++;
            pMcpsReq->msgData.dataReq.txOptions     = (macTxOptions_t)*p++;
            pMcpsReq->msgData.dataReq.securityLevel = (macSecurityLevel_t)*p++;
            pMcpsReq->msgData.dataReq.keyIdMode     = (keyIdModeType_t)*p++;

            FLib_MemCpy(&pMcpsReq->msgData.dataReq.keySource, p,
                        sizeof(pMcpsReq->msgData.dataReq.keySource));
            p += sizeof(pMcpsReq->msgData.dataReq.keySource);
            pMcpsReq->msgData.dataReq.keyIndex = *p++;
            pMcpsReq->msgData.dataReq.pMsdu = (uint8_t*)&pMcpsReq->msgData.dataReq.pMsdu +
                        sizeof(pMcpsReq->msgData.dataReq.pMsdu);
            FLib_MemCpy(pMcpsReq->msgData.dataReq.pMsdu, p,
                        pMcpsReq->msgData.dataReq.msduLength);
        }
        break;

    case mFsciNwkMcpsPurgeReq_c:
        pMcpsReq->msgType = gMcpsPurgeReq_c;
        pMcpsReq->msgData.purgeReq.msduHandle = pClientPacket->structured.payload[0];
        break;

    default:
        MEM_BufferFree( pData );
        FSCI_Error( gFsciUnknownOpcode_c, interfaceId );
        return;
    }

    /* To reduce peak memory usage, free the FSCI request before calling MAC SAPs */
    MEM_BufferFree( pData );
    NWK_MCPS_SapHandler( pMcpsReq, fsciGetMacInstanceId(interfaceId) );
#undef pClientPacket
}

/*! *********************************************************************************
* \brief   This is the handler function for All MLME Requests received over the
           serial interface
*
* \param[in] pData pointer to data location
* \param[in] param pointer to a parameter to be passed to the function
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciMlmeReqHandler(void *pData, void* param, uint32_t interfaceId)
{
#define pClientPacket ((clientPacket_t*)pData)
    mlmeMessage_t  mlmeReq;

    switch (pClientPacket->structured.header.opCode) {
    case mFsciNwkMlmeGetReq_c:
        mlmeReq.msgType = gMlmeGetReq_c;
        mlmeReq.msgData.getReq.pibAttribute      = pClientPacket->structured.payload[0];
        mlmeReq.msgData.getReq.pibAttributeIndex = pClientPacket->structured.payload[1];
        mlmeReq.msgData.getReq.pibAttributeValue = &pClientPacket->structured.payload[2];
        break;

    case mFsciNwkMlmeSetReq_c:
        mlmeReq.msgType = gMlmeSetReq_c;
        mlmeReq.msgData.setReq.pibAttribute      = pClientPacket->structured.payload[0];
        mlmeReq.msgData.setReq.pibAttributeIndex = pClientPacket->structured.payload[1];
        mlmeReq.msgData.setReq.pibAttributeValue = &pClientPacket->structured.payload[2];
        break;

    case mFsciNwkMlmeResetReq_c:
        mlmeReq.msgType = gMlmeResetReq_c;
        mlmeReq.msgData.resetReq.setDefaultPIB = pClientPacket->structured.payload[0];
        break;

    default:
        fsciMlmeAsyncReqHandler(pData, param, interfaceId);
        return;
    }

    (void)NWK_MLME_SapHandler(&mlmeReq, fsciGetMacInstanceId(interfaceId));
    MEM_BufferFree( pData );
#undef pClientPacket
}

/*! *********************************************************************************
* \brief   This is the handler function for Asynchronous MLME Requests received
           over the serial interface
*
* \param[in] pData pointer to data location
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void fsciMlmeAsyncReqHandler(void *pData, void* param, uint32_t interfaceId)
{
#define pClientPacket ((clientPacket_t*)pData)
    mlmeMessage_t  *pMlemReq = MEM_BufferAlloc(sizeof(mlmeMessage_t));
    uint8_t        *p = &pClientPacket->structured.payload[0];

    if (NULL == pMlemReq) {
        FSCI_Error(gFsciOutOfMessages_c, interfaceId);
        return;
    }

    switch (pClientPacket->structured.header.opCode) {
    case mFsciNwkMlmeAssociateReq_c:
        pMlemReq->msgType = gMlmeAssociateReq_c;
        pMlemReq->msgData.associateReq.channelPage = gDefaultChannelPageId_c;
        FLib_MemCpy(&pMlemReq->msgData.associateReq.coordAddress, p, 8);
        p += 8;
        FLib_MemCpy( &pMlemReq->msgData.associateReq.coordPanId,
                     p, sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.associateReq.coordAddrMode  = (addrModeType_t)*p++;
        pMlemReq->msgData.associateReq.logicalChannel = *p++;
        pMlemReq->msgData.associateReq.securityLevel  = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.associateReq.keyIdMode      = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.associateReq.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.associateReq.keyIndex = *p++;
        pMlemReq->msgData.associateReq.capabilityInfo = *p++;
        break;

    case mFsciNwkMlmeAssociateRes_c:
        pMlemReq->msgType = gMlmeAssociateRes_c;
        FLib_MemCpy(&pMlemReq->msgData.associateRes.deviceAddress, p, 8);
        p += 8;
        FLib_MemCpy( &pMlemReq->msgData.associateRes.assocShortAddress, p,
                    sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.associateRes.securityLevel  = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.associateRes.keyIdMode      = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.associateRes.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.associateRes.keyIndex = *p++;
        pMlemReq->msgData.associateRes.status   = (resultType_t)*p++;
        break;

    case mFsciNwkMlmeDisassociateReq_c:
        pMlemReq->msgType = gMlmeDisassociateReq_c;
        FLib_MemCpy(&pMlemReq->msgData.disassociateReq.deviceAddress, p, 8);
        p += 8;
        FLib_MemCpy( &pMlemReq->msgData.disassociateReq.devicePanId, p,
                     sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.disassociateReq.deviceAddrMode  = (addrModeType_t)*p++;
        pMlemReq->msgData.disassociateReq.disassociateReason = (macDisassociateReason_t)*p++;
        pMlemReq->msgData.disassociateReq.txIndirect = *p++;
        pMlemReq->msgData.disassociateReq.securityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.disassociateReq.keyIdMode     = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.disassociateReq.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.disassociateReq.keyIndex = *p++;
        break;

    case mFsciNwkMlmeGtsReq_c:
        pMlemReq->msgType = gMlmeGtsReq_c;
        *(uint8_t*)(&pMlemReq->msgData.gtsReq.gtsCharacteristics) = *p++;
        pMlemReq->msgData.gtsReq.securityLevel      = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.gtsReq.keyIdMode          = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.gtsReq.keySource, p,
                    sizeof(pMlemReq->msgData.gtsReq.keySource));
        p += sizeof(pMlemReq->msgData.gtsReq.keySource);
        pMlemReq->msgData.gtsReq.keyIndex = *p;
        break;

    case mFsciNwkMlmeOrphanRes_c:
        pMlemReq->msgType = gMlmeOrphanRes_c;
        FLib_MemCpy(&pMlemReq->msgData.orphanRes.orphanAddress, p, 8);
        p += 8;
        FLib_MemCpy( &pMlemReq->msgData.orphanRes.shortAddress, p,
                    sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.orphanRes.associatedMember = *p++;
        pMlemReq->msgData.orphanRes.securityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.orphanRes.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.orphanRes.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.orphanRes.keyIndex = *p++;
        break;

    case mFsciNwkMlmeRxEnableReq_c:
        pMlemReq->msgType = gMlmeRxEnableReq_c;
        pMlemReq->msgData.rxEnableReq.deferPermit = *p++;
        pMlemReq->msgData.rxEnableReq.rxOnTime = 0;
        pMlemReq->msgData.rxEnableReq.rxOnDuration = 0;
        FLib_MemCpy(&pMlemReq->msgData.rxEnableReq.rxOnTime, p, 3);
        p += 3;
        FLib_MemCpy(&pMlemReq->msgData.rxEnableReq.rxOnDuration, p, 3);
#ifdef gMAC2011_d
        p += 3;
        pMlemReq->msgData.rxEnableReq.rangingRxControl = *p;
#endif
        break;

    case mFsciNwkMlmeScanReq_c:
        pMlemReq->msgType = gMlmeScanReq_c;
        pMlemReq->msgData.scanReq.scanType = (macScanType_t)*p++;
        FLib_MemCpy( &pMlemReq->msgData.scanReq.scanChannels, p, sizeof(channelMask_t) );
        p += sizeof(channelMask_t);
        pMlemReq->msgData.scanReq.scanDuration = *p++;
        pMlemReq->msgData.scanReq.securityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.scanReq.keyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.scanReq.keySource, p, 8);
        p += 8;
        pMlemReq->msgData.scanReq.keyIndex = *p++;
        break;

    case mFsciNwkMlmeStartReq_c:
        pMlemReq->msgType = gMlmeStartReq_c;
        pMlemReq->msgData.startReq.channelPage = gDefaultChannelPageId_c;
        FLib_MemCpy( &pMlemReq->msgData.startReq.panId, p,
                    sizeof(uint16_t));
        p += sizeof(uint16_t);
        pMlemReq->msgData.startReq.logicalChannel = *p++;
        FLib_MemCpy( &pMlemReq->msgData.startReq.startTime, p, sizeof(uint32_t) );
        p += sizeof(uint32_t);
        pMlemReq->msgData.startReq.beaconOrder = *p++;
        pMlemReq->msgData.startReq.superframeOrder = *p++;
        pMlemReq->msgData.startReq.panCoordinator = *p++;
        pMlemReq->msgData.startReq.batteryLifeExtension = *p++;
        pMlemReq->msgData.startReq.coordRealignment = *p++;
        pMlemReq->msgData.startReq.coordRealignSecurityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.startReq.coordRealignKeyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.startReq.coordRealignKeySource, p, 8);
        p += 8;
        pMlemReq->msgData.startReq.coordRealignKeyIndex = *p++;
        pMlemReq->msgData.startReq.beaconSecurityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.startReq.beaconKeyIdMode = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.startReq.beaconKeySource, p, 8);
        p += 8;
        pMlemReq->msgData.startReq.beaconKeyIndex = *p++;
        break;

    case mFsciNwkMlmeSyncReq_c:
        pMlemReq->msgType = gMlmeSyncReq_c;
        pMlemReq->msgData.syncReq.channelPage = gDefaultChannelPageId_c;
        pMlemReq->msgData.syncReq.logicalChannel = *p++;
        pMlemReq->msgData.syncReq.trackBeacon    = *p;
        break;

    case mFsciNwkMlmePollReq_c:
        pMlemReq->msgType = gMlmePollReq_c;
        FLib_MemCpy(&pMlemReq->msgData.pollReq.coordAddress, p, sizeof(pMlemReq->msgData.pollReq.coordAddress));
        p += sizeof(pMlemReq->msgData.pollReq.coordAddress);
        FLib_MemCpy( &pMlemReq->msgData.pollReq.coordPanId, p,
                     sizeof(pMlemReq->msgData.pollReq.coordPanId));
        p += sizeof(pMlemReq->msgData.pollReq.coordPanId);
        pMlemReq->msgData.pollReq.coordAddrMode = (addrModeType_t)*p++;
        pMlemReq->msgData.pollReq.securityLevel = (macSecurityLevel_t)*p++;
        pMlemReq->msgData.pollReq.keyIdMode     = (keyIdModeType_t)*p++;
        FLib_MemCpy(&pMlemReq->msgData.pollReq.keySource, p,
                    sizeof(pMlemReq->msgData.pollReq.keySource));
        p += sizeof(pMlemReq->msgData.pollReq.keySource);
        pMlemReq->msgData.pollReq.keyIndex = *p++;
        break;

    default:
        FSCI_Error(gFsciUnknownOpcode_c, interfaceId);
        return;
    }
    /* To reduce peak memory usage, free the FSCI request before calling MAC SAPs */
    MEM_BufferFree( pData );
    if( gSuccess_c != NWK_MLME_SapHandler(pMlemReq, fsciGetMacInstanceId(interfaceId)) )
    {
        MEM_BufferFree( pMlemReq );
    }
#undef pClientPacket
}

/*! *********************************************************************************
* \brief   This function is called to monitor the MAC's MCPS SAP
*
* \param[in] pData pointer to data location
* \param[in] param pointer to a parameter to be passed to the function
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void McpsSapMonitor(void *pData, void* param, uint32_t interfaceId)
{
#define pMscpCnf ((mcpsToNwkMessage_t*)pData)
#define pMcpsReq ((nwkToMcpsMessage_t*)pData)
    clientPacket_t     *pFsciPacket;
    uint8_t            *p;
    uint16_t            size = sizeof(clientPacketHdr_t) + sizeof(uint8_t);

    /* Determine the size needed to be allocated */
    switch( ((macMsgHeader_t*)pData)->msgType )
    {
    case gMcpsDataReq_c:
        {
            mcpsDataReq_t *pSrc = &pMcpsReq->msgData.dataReq;
            size += sizeof(mcpsDataReq_t) + pSrc->msduLength;
        }
        break;
    case gMcpsPurgeReq_c:
        size += sizeof(mcpsPurgeReq_t);
        break;
    case gMcpsDataInd_c:
        {
            mcpsDataInd_t *pSrc = &pMscpCnf->msgData.dataInd;
            size += sizeof(mcpsDataInd_t) + pSrc->msduLength;
        }
        break;
    case gMcpsDataCnf_c:
        size += sizeof(mcpsDataCnf_t);
        break;
    case gMcpsPurgeCnf_c:
        size += sizeof(mcpsPurgeCnf_t);
        break;
    case gMcpsPromInd_c:
        {
            mcpsPromInd_t *pSrc = &pMscpCnf->msgData.promInd;
            size += sizeof(mcpsPromInd_t) + pSrc->msduLength;
        }
        break;
    }
    
    pFsciPacket = MEM_BufferAlloc( size );
    if (NULL == pFsciPacket) {
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
        return;
    }

    pFsciPacket->structured.header.opGroup = 0x86;
    p = pFsciPacket->structured.payload;

    switch( ((macMsgHeader_t*)pData)->msgType )
    {
/* MCPS requests */
    case gMcpsDataReq_c:         //87 00
        {
            mcpsDataReq_t *pSrc = &pMcpsReq->msgData.dataReq;

            pFsciPacket->structured.header.opCode = 0x00;
            pFsciPacket->structured.header.opGroup = 0x87;
            FLib_MemCpy(p, &pSrc->dstAddr, sizeof(pSrc->dstAddr));
            p += sizeof(pSrc->dstAddr);
            FLib_MemCpy( p, &pSrc->dstPanId, sizeof(pSrc->dstPanId) );
            p += sizeof(pSrc->dstPanId);
            *p++ = pSrc->dstAddrMode;

            FLib_MemCpy(p, &pSrc->srcAddr, sizeof(pSrc->srcAddr));
            p += sizeof(pSrc->srcAddr);
            FLib_MemCpy( p, &pSrc->srcPanId, sizeof(pSrc->srcPanId));
            p += sizeof(pSrc->srcPanId);
            *p++ = pSrc->srcAddrMode;

            *p++ = pSrc->msduLength;
            *p++ = pSrc->msduHandle;
            *p++ = pSrc->txOptions;
            *p++ = pSrc->securityLevel;
            *p++ = pSrc->keyIdMode;

            FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
            p += sizeof(pSrc->keySource);
            *p++ = pSrc->keyIndex;
            FLib_MemCpy(p, pSrc->pMsdu, pSrc->msduLength);
            p += pSrc->msduLength;
        }
        break;

    case gMcpsPurgeReq_c:        //87 01
        pFsciPacket->structured.header.opCode = 0x01;
        pFsciPacket->structured.header.opGroup = 0x87;
        *p++ = pMcpsReq->msgData.purgeReq.msduHandle;
        break;

/* MCPS confirms and indications */
    case gMcpsDataCnf_c:         //86 00
        pFsciPacket->structured.header.opCode = 0x00;
        *p++ = pMscpCnf->msgData.dataCnf.msduHandle;
        *p++ = pMscpCnf->msgData.dataCnf.status;
        break;

    case gMcpsDataInd_c:         //86 01
        {
            mcpsDataInd_t *pSrc = &pMscpCnf->msgData.dataInd;

            pFsciPacket->structured.header.opCode = 0x01;
            FLib_MemCpy(p, &pSrc->dstAddr, sizeof(pSrc->dstAddr));
            p += sizeof(pSrc->dstAddr);
            FLib_MemCpy( p, &pSrc->dstPanId, sizeof(pSrc->dstPanId));
            p += sizeof(pSrc->dstPanId);
            *p++ = pSrc->dstAddrMode;

            FLib_MemCpy(p, &pSrc->srcAddr, sizeof(pSrc->srcAddr));
            p += sizeof(pSrc->srcAddr);
            FLib_MemCpy( p, &pSrc->srcPanId, sizeof(pSrc->srcPanId) );
            p += sizeof(pSrc->srcPanId);
            *p++ = pSrc->srcAddrMode;

            *p++ = pSrc->msduLength;
            *p++ = pSrc->mpduLinkQuality;
            *p++ = pSrc->dsn;
            FLib_MemCpy( p, &pSrc->timestamp, sizeof(uint32_t) );
            p += sizeof(uint32_t);
            *p++ = pSrc->securityLevel;
            *p++ = pSrc->keyIdMode;
            FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
            p += sizeof(pSrc->keySource);
            *p++ = pSrc->keyIndex;
            FLib_MemCpy(p, pSrc->pMsdu, pSrc->msduLength);
            p += pSrc->msduLength;
        }
        break;

    case gMcpsPurgeCnf_c:        //86 02
        pFsciPacket->structured.header.opCode = 0x02;
        *p++ = pMscpCnf->msgData.purgeCnf.msduHandle;
        *p++ = pMscpCnf->msgData.purgeCnf.status;
        break;

    case gMcpsPromInd_c:         //86 03
        {
            mcpsPromInd_t *pSrc = &pMscpCnf->msgData.promInd;

            pFsciPacket->structured.header.opCode = 0x03;
            *p++ = pSrc->mpduLinkQuality;
            FLib_MemCpy( p, &pSrc->timeStamp, sizeof(uint32_t) );
            p += sizeof(uint32_t);
            *p++ = pSrc->msduLength;
            FLib_MemCpy(p, pSrc->pMsdu, pSrc->msduLength);
            p += pSrc->msduLength;
        }
        break;
    } /* switch( pMsg->msgType ) */

    /* Send data over the serial interface */
    pFsciPacket->structured.header.len = (fsciLen_t)(p - pFsciPacket->structured.payload);

    if ( pFsciPacket->structured.header.len )
        FSCI_transmitFormatedPacket( pFsciPacket, interfaceId );
    else
        MEM_BufferFree( pFsciPacket );
#undef pMscpCnf
#undef pMcpsReq
}

/*! *********************************************************************************
* \brief   This function is called to monitor the MAC's MLME SAP
*
* \param[in] pData pointer to data location
* \param[in] param pointer to a parameter to be passed to the function
* \param[in] interfaceId
*
* \return None.
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
void MlmeSapMonitor(void *pData, void* param, uint32_t interfaceId)
{
#define pMlmeReq ((mlmeMessage_t*)pData)
#define pMlmeCnf ((nwkMessage_t*)pData)
    uint8_t        *p;
    clientPacket_t *pFsciPacket;
    uint16_t        size = sizeof(clientPacketHdr_t) + sizeof(uint8_t);

    /* Determine the size needed to be allocated */
    switch( pMlmeReq->msgType )
    {
    case gMlmeAssociateReq_c:
        size += sizeof(mlmeAssociateReq_t);
        break;
    case gMlmeAssociateRes_c:
        size += sizeof(mlmeAssociateRes_t);
        break;
    case gMlmeDisassociateReq_c:
        size += sizeof(mlmeDisassociateReq_t);
        break;
    case gMlmeGetReq_c:
        size += sizeof(mlmeGetReq_t);
        break;
    case gMlmeGtsReq_c:
        size += sizeof(mlmeGtsReq_t);
        break;
    case gMlmeOrphanRes_c:
        size += sizeof(mlmeOrphanRes_t);
        break;
    case gMlmeResetReq_c:
        size += sizeof(mlmeResetReq_t);
        break;
    case gMlmeRxEnableReq_c:
        size += sizeof(mlmeRxEnableReq_t);
        break;
    case gMlmeScanReq_c:
        size += sizeof(mlmeScanReq_t);
        break;
    case gMlmeSetReq_c:
        size += sizeof(mlmeSetReq_t) + sizeof(uint64_t);
        break;
    case gMlmeStartReq_c:
        size += sizeof(mlmeStartReq_t);
        break;
    case gMlmeSyncReq_c:
        size += sizeof(mlmeSyncReq_t);
        break;
    case gMlmePollReq_c:
        size += sizeof(mlmePollReq_t);
        break;
        
    case gMlmeAssociateInd_c:
        size += sizeof(mlmeAssociateInd_t);
        break;
    case gMlmeAssociateCnf_c:
        size += sizeof(mlmeAssociateCnf_t);
        break;
    case gMlmeDisassociateInd_c:
        size += sizeof(mlmeDisassociateInd_t);
        break;
    case gMlmeDisassociateCnf_c:
        size += sizeof(mlmeDisassociateCnf_t);
        break;
    case gMlmeBeaconNotifyInd_c:
        size += gFsciMaxPayloadLen_c;
        break;
    case gMlmeGetCnf_c:
        size += sizeof(mlmeGetCnf_t) + mlmeGetSizeOfPIB(pMlmeReq->msgData.getReq.pibAttribute);
        break;
    case gMlmeGtsInd_c:
        size += sizeof(mlmeGtsInd_t);
        break;
    case gMlmeGtsCnf_c:
        size += sizeof(mlmeGtsCnf_t);
        break;
    case gMlmeOrphanInd_c:
        size += sizeof(mlmeOrphanInd_t);
        break;
    case gMlmeResetCnf_c:
        size += sizeof(mlmeResetCnf_t);
        break;
    case gMlmeRxEnableCnf_c:
        size += sizeof(mlmeRxEnableCnf_t);
        break;
    case gMlmeScanCnf_c:
        size += gFsciMaxPayloadLen_c;
        break;
    case gMlmeCommStatusInd_c:
        size += sizeof(mlmeCommStatusInd_t);
        break;
    case gMlmeSetCnf_c:
        size += sizeof(mlmeSetCnf_t);
        break;
    case gMlmeStartCnf_c:
        size += sizeof(mlmeStartCnf_t);
        break;
    case gMlmeSyncLossInd_c:
        size += sizeof(mlmeSyncLossInd_t);
        break;
    case gMlmePollCnf_c:
        size += sizeof(mlmePollCnf_t);
        break;
    case gMlmePollNotifyInd_c:
        size += sizeof(mlmePollNotifyInd_t);
        break;
    }

    pFsciPacket = MEM_BufferAlloc( size );
    if( NULL == pFsciPacket ) 
    {
        FSCI_Error( gFsciOutOfMessages_c, interfaceId );
        return;
    }

    pFsciPacket->structured.header.opGroup = 0x84;
    p = pFsciPacket->structured.payload;

    switch( pMlmeReq->msgType )
    {
/* MLME requests */
    case gMlmeAssociateReq_c:    //85 00
#define pSrc (&pMlmeReq->msgData.associateReq)
        pFsciPacket->structured.header.opCode = 0x00;
        pFsciPacket->structured.header.opGroup = 0x85;
        FLib_MemCpy(p, &pSrc->coordAddress, sizeof(pSrc->coordAddress));
        p += sizeof(pSrc->coordAddress);
        FLib_MemCpy( p, &pSrc->coordPanId, sizeof(pSrc->coordPanId) );
        p += sizeof(pSrc->coordPanId);
        *p++ = pSrc->coordAddrMode;
        *p++ = pSrc->logicalChannel;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
        *p++ = pSrc->capabilityInfo;
#undef pSrc
        break;

    case gMlmeAssociateRes_c:    //85 01
#define pSrc (&pMlmeReq->msgData.associateRes)
        pFsciPacket->structured.header.opCode = 0x01;
        pFsciPacket->structured.header.opGroup = 0x85;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        FLib_MemCpy( p, &pSrc->assocShortAddress, sizeof(pSrc->assocShortAddress));
        p += sizeof(pSrc->assocShortAddress);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
        *p++ = pSrc->status;
#undef pSrc
        break;

    case gMlmeDisassociateReq_c: //85 02
#define pSrc (&pMlmeReq->msgData.disassociateReq)
        pFsciPacket->structured.header.opCode = 0x02;
        pFsciPacket->structured.header.opGroup = 0x85;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        FLib_MemCpy( p, &pSrc->devicePanId, sizeof(pSrc->devicePanId) );
        p += sizeof(pSrc->devicePanId);
        *p++ = pSrc->deviceAddrMode;
        *p++ = pSrc->disassociateReason;
        *p++ = pSrc->txIndirect;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeGetReq_c:          //85 03
        pFsciPacket->structured.header.opCode = 0x03;
        pFsciPacket->structured.header.opGroup = 0x85;

        *p++ = pMlmeReq->msgData.getReq.pibAttribute;
        *p++ = pMlmeReq->msgData.getReq.pibAttributeIndex;
        break;

    case gMlmeGtsReq_c:          //85 04
#define pSrc (&pMlmeReq->msgData.gtsReq)
        pFsciPacket->structured.header.opCode = 0x04;
        pFsciPacket->structured.header.opGroup = 0x85;
        *p++ = *((uint8_t*)&pSrc->gtsCharacteristics);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeOrphanRes_c:       //85 05
#define pSrc (&pMlmeReq->msgData.orphanRes)
        pFsciPacket->structured.header.opCode = 0x05;
        pFsciPacket->structured.header.opGroup = 0x85;
        FLib_MemCpy(p, &pSrc->orphanAddress, sizeof(pSrc->orphanAddress));
        p += sizeof(pSrc->orphanAddress);
        FLib_MemCpy( p, &pSrc->shortAddress, sizeof(pSrc->shortAddress) );
        p += sizeof(pSrc->shortAddress);
        *p++ = pSrc->associatedMember;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeResetReq_c:        //85 06
        pFsciPacket->structured.header.opCode = 0x06;
        pFsciPacket->structured.header.opGroup = 0x85;
        *p++ = pMlmeReq->msgData.resetReq.setDefaultPIB;
        break;

    case gMlmeRxEnableReq_c:     //85 07
#define pSrc (&pMlmeReq->msgData.rxEnableReq)
        pFsciPacket->structured.header.opCode = 0x07;
        pFsciPacket->structured.header.opGroup = 0x85;
        *p++ = pSrc->deferPermit;
        FLib_MemCpy(p, &pSrc->rxOnTime, 3);
        p += 3;
        FLib_MemCpy(p, &pSrc->rxOnDuration, 3);
        p += 3;
#undef pSrc
        break;

    case gMlmeScanReq_c:         //85 08
#define pSrc (&pMlmeReq->msgData.scanReq)
        pFsciPacket->structured.header.opCode = 0x08;
        pFsciPacket->structured.header.opGroup = 0x85;
        *p++ = pSrc->scanType;
        FLib_MemCpy(p, &pSrc->scanChannels, sizeof(uint32_t));
        p += sizeof(uint32_t);
        *p++ = pSrc->scanDuration;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeSetReq_c:          //85 09
        pFsciPacket->structured.header.opCode = 0x09;
        pFsciPacket->structured.header.opGroup = 0x85;
        *p++ = pMlmeReq->msgData.setReq.pibAttribute;
        *p++ = pMlmeReq->msgData.setReq.pibAttributeIndex;
        size = mlmeGetSizeOfPIB(pMlmeReq->msgData.setReq.pibAttribute);
        FLib_MemCpy(p, pMlmeReq->msgData.setReq.pibAttributeValue, size);
        p += size;
        break;

    case gMlmeStartReq_c:        //85 0A
#define pSrc (&pMlmeReq->msgData.startReq)
        pFsciPacket->structured.header.opCode = 0x0A;
        pFsciPacket->structured.header.opGroup = 0x85;
        FLib_MemCpy( p, &pSrc->panId, sizeof(pSrc->panId) );
        p += sizeof(uint16_t);
        *p++ = pSrc->logicalChannel;
        FLib_MemCpy( p, &pSrc->startTime, sizeof(uint32_t) );
        p += sizeof(uint32_t);
        *p++ = pSrc->beaconOrder;
        *p++ = pSrc->superframeOrder;
        *p++ = pSrc->panCoordinator;
        *p++ = pSrc->batteryLifeExtension;
        *p++ = pSrc->coordRealignment;
        *p++ = pSrc->coordRealignSecurityLevel;
        *p++ = pSrc->coordRealignKeyIdMode;
        FLib_MemCpy(p, &pSrc->coordRealignKeySource, sizeof(pSrc->coordRealignKeySource));
        p += sizeof(pSrc->coordRealignKeySource);
        *p++ = pSrc->coordRealignKeyIndex;
        *p++ = pSrc->beaconSecurityLevel;
        *p++ = pSrc->beaconKeyIdMode;
        FLib_MemCpy(p, &pSrc->beaconKeySource, sizeof(pSrc->beaconKeySource));
        p += sizeof(pSrc->beaconKeySource);
        *p++ = pSrc->beaconKeyIndex;
#undef pSrc
        break;

    case gMlmeSyncReq_c:         //85 0B
        pFsciPacket->structured.header.opCode = 0x0B;
        pFsciPacket->structured.header.opGroup = 0x85;
        *p++ = pMlmeReq->msgData.syncReq.logicalChannel;
        *p++ = pMlmeReq->msgData.syncReq.trackBeacon;
        break;

    case gMlmePollReq_c:         //85 0C
#define pSrc (&pMlmeReq->msgData.pollReq)
        pFsciPacket->structured.header.opCode = 0x0C;
        pFsciPacket->structured.header.opGroup = 0x85;
        FLib_MemCpy(p, &pSrc->coordAddress, sizeof(pSrc->coordAddress));
        p += sizeof(pSrc->coordAddress);
        FLib_MemCpy( p, &pSrc->coordPanId, sizeof(pSrc->coordPanId) );
        p += sizeof(pSrc->coordPanId);
        *p++ = pSrc->coordAddrMode;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

/* MLME confirms and indications */
    case gMlmeAssociateInd_c:    //84 00
#define pSrc (&pMlmeCnf->msgData.associateInd)
        pFsciPacket->structured.header.opCode = 0x00;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
        *p++ = pSrc->capabilityInfo;
#undef pSrc
        break;

    case gMlmeAssociateCnf_c:    //84 01
#define pSrc (&pMlmeCnf->msgData.associateCnf)
        pFsciPacket->structured.header.opCode = 0x01;
        
        FLib_MemCpy( p, &pSrc->assocShortAddress, sizeof(pSrc->assocShortAddress) );
        p += sizeof(pSrc->assocShortAddress);
        if ((uint32_t)param != gSuccess_c)
        {
            *p++ = (uint32_t)param;
        }
        else
        {
            *p++ = pSrc->status;
        }
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeDisassociateInd_c: //84 02
#define pSrc (&pMlmeCnf->msgData.disassociateInd)
        pFsciPacket->structured.header.opCode = 0x02;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        *p++ = pSrc->disassociateReason;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeDisassociateCnf_c: //84 03
#define pSrc (&pMlmeCnf->msgData.disassociateCnf)
        pFsciPacket->structured.header.opCode = 0x03;
        FLib_MemCpy(p, &pSrc->deviceAddress, sizeof(pSrc->deviceAddress));
        p += sizeof(pSrc->deviceAddress);
        FLib_MemCpy( p, &pSrc->devicePanId, sizeof(pSrc->devicePanId) );
        p += sizeof(pSrc->devicePanId);
        *p++ = pSrc->deviceAddrMode;
        if ((uint32_t)param != gSuccess_c)
            *p++ = (uint32_t)param;
        else
            *p++ = pSrc->status;
#undef pSrc
        break;

    case gMlmeBeaconNotifyInd_c: //84 04
#define pSrc (&pMlmeCnf->msgData.beaconNotifyInd)
        pFsciPacket->structured.header.opCode = 0x04;
        *p++ = pSrc->bsn;
        *p++ = pSrc->pendAddrSpec;
        *p++ = pSrc->sduLength;
        //pending short address
        size  = sizeof(uint16_t) * (pSrc->pendAddrSpec & 0x07);
        //pending extended address
        size += sizeof(uint64_t) * ((pSrc->pendAddrSpec >> 4) & 0x07);
        FLib_MemCpy(p, pSrc->pAddrList, size);
        p += size;

        FLib_MemCpy(p, pSrc->pPanDescriptor, sizeof(panDescriptor_t));
        p += sizeof(panDescriptor_t);
        FLib_MemCpy(p, pSrc->pSdu, pSrc->sduLength);
        p += pSrc->sduLength;
#undef pSrc
        break;

    case gMlmeGetCnf_c:          //84 05
        pFsciPacket->structured.header.opCode = 0x05;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
        *p++ = pMlmeReq->msgData.getReq.pibAttribute;
        *p++ = pMlmeReq->msgData.getReq.pibAttributeIndex;
        size = mlmeGetSizeOfPIB(pMlmeReq->msgData.getReq.pibAttribute);
        FLib_MemCpy( p, &size, sizeof(size) );
        p += sizeof(size);
        FLib_MemCpy(p, pMlmeReq->msgData.getReq.pibAttributeValue, size);
        p += size;
        break;

    case gMlmeGtsInd_c:          //84 06
#define pSrc (&pMlmeCnf->msgData.gtsInd)
        pFsciPacket->structured.header.opCode = 0x06;
        FLib_MemCpy( p, &pSrc->deviceAddress, sizeof(uint16_t) );
        p += sizeof(uint16_t);
        *p++ = *((uint8_t*)&pSrc->gtsCharacteristics);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy (p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeGtsCnf_c:          //84 07
        pFsciPacket->structured.header.opCode = 0x07;
        if ((uint32_t)param != gSuccess_c)
        {
            *p++ = (uint32_t)param;
        }
        else
        {
            *p++ = pMlmeCnf->msgData.gtsCnf.status;
        }
        *p++ = *((uint8_t*)&(pMlmeCnf->msgData.gtsCnf.gtsCharacteristics));
        break;

    case gMlmeOrphanInd_c:       //84 08
#define pSrc (&pMlmeCnf->msgData.orphanInd)
        pFsciPacket->structured.header.opCode = 0x08;
        FLib_MemCpy(p, &pSrc->orphanAddress, sizeof(pSrc->orphanAddress));
        p += sizeof(pSrc->orphanAddress);
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeResetCnf_c:        //84 09
        pFsciPacket->structured.header.opCode = 0x09;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
        break;

    case gMlmeRxEnableCnf_c:     //84 0A
        pFsciPacket->structured.header.opCode = 0x0A;
        if ((uint32_t)param != gSuccess_c)
            *p++ = (uint8_t)((uint32_t)param);
        else
            *p++ = pMlmeCnf->msgData.rxEnableCnf.status;
        break;

    case gMlmeScanCnf_c:         //84 0B
#define pSrc (&pMlmeCnf->msgData.scanCnf)
        pFsciPacket->structured.header.opCode = 0x0B;
        *p++ = pSrc->status;
        *p++ = pSrc->scanType;
        *p++ = pSrc->resultListSize;
        FLib_MemCpy( p, &pSrc->unscannedChannels, sizeof(channelMask_t) );
        p += sizeof(channelMask_t);
        
        if ( pSrc->status == gSuccess_c )
        {
            if( (pSrc->scanType == gScanModeED_c) ||
               (pSrc->scanType == gScanModeFastED_c))
            {
                size = pSrc->resultListSize * sizeof(pSrc->resList.pEnergyDetectList[0]);
                FLib_MemCpy(p, pSrc->resList.pEnergyDetectList, size);
                p += size;
            }
            else if ( (pSrc->scanType == gScanModeActive_c) ||
                     (pSrc->scanType == gScanModePassive_c) )
            {
                uint16_t len = sizeof(pSrc->status) +
                               sizeof(pSrc->scanType) +
                               sizeof(pSrc->resultListSize) +
                               sizeof(pSrc->unscannedChannels);
                panDescriptorBlock_t* pCrtPDBlock = pSrc->resList.pPanDescriptorBlockList;
                uint8_t pdIdx = 0;
                uint8_t resIdx = 0;
                
                size = sizeof(panDescriptor_t);
                
                while ( (pCrtPDBlock) &&
                       (resIdx < pSrc->resultListSize) &&
                           ( (len + size) < gFsciMaxPayloadLen_c ) )
                {
                    FLib_MemCpy(p, &pCrtPDBlock->panDescriptorList[pdIdx], size);
                    p += size;
                    len += size;
                    resIdx++;
                    
                    if ( ++pdIdx >= gScanResultsPerBlock_c )
                    {
                        pCrtPDBlock = pCrtPDBlock->pNext;
                        pdIdx = 0;
                    }
                }
                
                /* store how many pan descriptors were put in buffer */
                pFsciPacket->structured.payload[2] = resIdx;
            }
        }
#undef pSrc
        break;

    case gMlmeCommStatusInd_c:   //84 0C
#define pSrc (&pMlmeCnf->msgData.commStatusInd)
        pFsciPacket->structured.header.opCode = 0x0C;
        FLib_MemCpy(p, &pSrc->srcAddress, sizeof(pSrc->srcAddress));
        p += sizeof(pSrc->srcAddress);
        FLib_MemCpy( p, &pSrc->panId, sizeof(uint16_t) );
        p += sizeof(uint16_t);
        *p++ = pSrc->srcAddrMode;
        FLib_MemCpy(p, &pSrc->destAddress, sizeof(pSrc->destAddress));
        p += sizeof(pSrc->destAddress);
        *p++ = pSrc->destAddrMode;
        
        if ((uint32_t)param != gSuccess_c)
            *p++ = (uint32_t)param;
        else
            *p++ = pSrc->status;
        
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmeSetCnf_c:          //84 0D
        pFsciPacket->structured.header.opCode = 0x0D;
        *p++ = (uint8_t)((uint32_t)param); //Status of Sync request
        *p++ = pMlmeReq->msgData.setReq.pibAttribute;
        *p++ = pMlmeReq->msgData.setReq.pibAttributeIndex;
        break;

    case gMlmeStartCnf_c:        //84 0E
        pFsciPacket->structured.header.opCode = 0x0E;
        if ((uint32_t)param != gSuccess_c)
            *p++ = (uint8_t)((uint32_t)param);
        else
            *p++ = pMlmeCnf->msgData.startCnf.status;
        break;

    case gMlmeSyncLossInd_c:     //84 0F
#define pSrc (&pMlmeCnf->msgData.syncLossInd)
        pFsciPacket->structured.header.opCode = 0x0F;
        *p++ = pSrc->lossReason;
        FLib_MemCpy( p, &pSrc->panId, sizeof(uint16_t) );
        p += sizeof(uint16_t);
        *p++ = pSrc->logicalChannel;
        *p++ = pSrc->securityLevel;
        *p++ = pSrc->keyIdMode;
        FLib_MemCpy(p, &pSrc->keySource, sizeof(pSrc->keySource));
        p += sizeof(pSrc->keySource);
        *p++ = pSrc->keyIndex;
#undef pSrc
        break;

    case gMlmePollCnf_c:         //84 10
        pFsciPacket->structured.header.opCode = 0x10;
        if ((uint32_t)param != gSuccess_c)
        {
            *p++ = (uint32_t)param;
        }
        else
        {
            *p++ = pMlmeCnf->msgData.pollCnf.status;
        }
        break;

    case gMlmePollNotifyInd_c:   //84 14
#define pSrc (&pMlmeCnf->msgData.pollNotifyInd)
        pFsciPacket->structured.header.opCode = 0x14;
        *p++ = pSrc->srcAddrMode;
        FLib_MemCpy(p, &pSrc->srcAddr, sizeof(pSrc->srcAddr));
        p += sizeof(pSrc->srcAddr);
        FLib_MemCpy( p, &pSrc->srcPanId, sizeof(uint16_t) );
        p += sizeof(uint16_t);
#undef pSrc
        break;
/*
    case BeaconStartInd:         //84 12
        pFsciPacket->structured.header.opCode = 0x12;
        *p++ = source
        break;

    case MacMaintenanceScanCnf:  //84 13
        pFsciPacket->structured.header.opCode = 0x13;
        *p++ = status
        break;
*/
    default:                     //84 11
        pFsciPacket->structured.header.opCode = 0x11;
        //*p++ = error...
    }

    /* Send data over the serial interface */
    pFsciPacket->structured.header.len = (fsciLen_t)(p - pFsciPacket->structured.payload);

    if ( pFsciPacket->structured.header.len )
        FSCI_transmitFormatedPacket( pFsciPacket, interfaceId );
    else
        MEM_BufferFree( pFsciPacket );
#undef pMlmeReq
#undef pMlmeCnf
}
#endif /* gFsciIncluded_c && gFSCI_IncludeMacCommands_c */

/*! *********************************************************************************
* \brief   This function determines the instance of the MAC registered on the 
*          specified interface
*
* \param[in] interfaceId
*
* \return The instance of the MAC
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
instanceId_t fsciGetMacInstanceId( uint32_t interfaceId )
{
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
    uint32_t i;

    for( i=0; i<gMacInstancesCnt_c; i++ )
    {
        if( interfaceId == fsciToMacBinding[i] )
        {
            return (instanceId_t)i;
        }
    }
#endif
    return gInvalidInstanceId_c;
}

/*! *********************************************************************************
* \brief   This function determines the interface Id of the specified MAC instance
*
* \param[in] macInstance
*
* \return The interface Id on which the MAC is registered
*
* \pre
*
* \post
*
* \remarks
*
********************************************************************************** */
uint32_t fsciGetMacInterfaceId( instanceId_t macInstance )
{
#if gFsciIncluded_c && gFSCI_IncludeMacCommands_c
    return fsciToMacBinding[macInstance];
#else
    return 0;
#endif
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/************************************************************************************/
