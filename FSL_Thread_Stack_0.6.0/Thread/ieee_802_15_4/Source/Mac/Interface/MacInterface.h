/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MacInterface.h
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

#ifndef _MAC_INTERFACE_H
#define _MAC_INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "MacConfig.h"
#include "MacTypes.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
#define gMacVerMajor_c   5

#ifdef gPHY_802_15_4g_d
  #define gMacVerMinor_c   1
  #define gMacVerPatch_c   0
  #define gMacBuildNo_c    30
#else
  #define gMacVerMinor_c   0
  #define gMacVerPatch_c   1
  #define gMacBuildNo_c    02
#endif

#if (gMacUsePackedStructs_c)
#define MAC_STRUCT PACKED_STRUCT
#define MAC_UNION PACKED_UNION
#else
#define MAC_STRUCT struct
#define MAC_UNION union
#endif
        
/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

#ifdef gPHY_802_15_4g_d
#define gMacFirstLogicalChannel_c    (gLogicalChannel0_c)
#define gMacLastLogicalChannel_c     (gLogicalChannel127_c)
#else
#define gMacFirstLogicalChannel_c    (gLogicalChannel11_c)
#define gMacLastLogicalChannel_c     (gLogicalChannel26_c)
#endif //gPHY_802_15_4g_d

#ifdef gPHY_802_15_4g_d
#define gDefaultChannelPageId_c     (gChannelPageId9_c)
#else                                   
#define gDefaultChannelPageId_c     (gChannelPageId0_c)
#endif

typedef enum
{
    #include "MacMessages.h"
}macMessageId_t;

typedef enum {
    gMacStateIdle_c     = 0x00,
    gMacStateBusy_c     = 0x01,
    gMacStateNotEmpty_c = 0x02
}macState_t;

/************************************************************************************/

typedef MAC_STRUCT mcpsDataReq_tag
{
    uint16_t                srcPanId; /* Not in Spec */
    uint64_t                srcAddr; /* Not in Spec */
    addrModeType_t          srcAddrMode;
    addrModeType_t          dstAddrMode;
    uint16_t                dstPanId;
    uint64_t                dstAddr;
    uint16_t                msduLength;
    uint8_t                 msduHandle;
    macTxOptions_t          txOptions;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
    uint8_t                 *pMsdu;
} mcpsDataReq_t;

typedef MAC_STRUCT mcpsDataCnf_tag
{
    uint8_t                 msduHandle;
    resultType_t            status;
    uint32_t                timestamp;
} mcpsDataCnf_t;

typedef MAC_STRUCT mcpsDataInd_tag
{
    uint64_t                dstAddr;
    uint16_t                dstPanId;
    addrModeType_t          dstAddrMode;
    uint64_t                srcAddr;
    uint16_t                srcPanId;
    addrModeType_t          srcAddrMode;
    uint16_t                msduLength;
    uint8_t                 mpduLinkQuality;
    uint8_t                 dsn;
    uint32_t                timestamp;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
    uint8_t                 *pMsdu;
} mcpsDataInd_t;

typedef MAC_STRUCT mcpsPurgeReq_tag
{
    uint8_t                 msduHandle;
} mcpsPurgeReq_t;

typedef MAC_STRUCT mcpsPurgeCnf_tag
{
    uint8_t                 msduHandle;
    resultType_t            status;
} mcpsPurgeCnf_t;

typedef MAC_STRUCT mcpsPromInd_tag
{
    uint8_t                 mpduLinkQuality;
    uint32_t                timeStamp;
    uint8_t                 msduLength;
    uint8_t                 *pMsdu;
} mcpsPromInd_t;

typedef MAC_STRUCT mlmeAssociateReq_tag
{
    uint64_t                coordAddress;
    uint16_t                coordPanId;
    addrModeType_t          coordAddrMode;
    logicalChannelId_t      logicalChannel;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
    macCapabilityInfo_t     capabilityInfo;
    channelPageId_t         channelPage;
} mlmeAssociateReq_t;

typedef MAC_STRUCT mlmeAssociateCnf_tag
{
    uint16_t                assocShortAddress;
    resultType_t            status;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeAssociateCnf_t;

typedef MAC_STRUCT mlmeAssociateRes_tag
{
    uint64_t                deviceAddress;
    uint16_t                assocShortAddress;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
    resultType_t            status;
} mlmeAssociateRes_t;

typedef MAC_STRUCT mlmeAssociateInd_tag
{
    uint64_t                deviceAddress;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
    macCapabilityInfo_t     capabilityInfo;
} mlmeAssociateInd_t;

typedef MAC_STRUCT mlmeDisassociateReq_tag
{
    uint64_t                deviceAddress;
    uint16_t                devicePanId;
    addrModeType_t          deviceAddrMode;
    macDisassociateReason_t disassociateReason;
    bool_t                  txIndirect;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeDisassociateReq_t;

typedef MAC_STRUCT mlmeDisassociateCnf_tag
{
    uint64_t                deviceAddress;
    uint16_t                devicePanId;
    addrModeType_t          deviceAddrMode;
    resultType_t            status;
} mlmeDisassociateCnf_t;

typedef MAC_STRUCT mlmeDisassociateInd_tag
{
    uint64_t                deviceAddress;
    macDisassociateReason_t disassociateReason;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeDisassociateInd_t;

typedef MAC_STRUCT mlmeGetReq_tag
{
    pibId_t                 pibAttribute;
    uint8_t                 pibAttributeIndex;
    void*                   pibAttributeValue; // Not in spec.
}mlmeGetReq_t;

typedef MAC_STRUCT mlmeGetCnf_tag
{
    resultType_t            status;
    pibId_t                 pibAttribute;
    uint8_t                 pibAttributeIndex;
    void*                   pibAttributeValue;
}mlmeGetCnf_t;

typedef MAC_STRUCT mlmeRxEnableReq_tag
{
    bool_t                  deferPermit;
    uint32_t                rxOnTime;
    uint32_t                rxOnDuration;
#ifdef gMAC2011_d
    macRangingRxControl_t   rangingRxControl;
#endif
} mlmeRxEnableReq_t;

typedef MAC_STRUCT mlmeRxEnableCnf_tag
{
    resultType_t            status;
} mlmeRxEnableCnf_t;

typedef MAC_STRUCT mlmeScanReq_tag
{
    macScanType_t           scanType;
    channelMask_t           scanChannels;
    uint8_t                 scanDuration;
    channelPageId_t         channelPage;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeScanReq_t;

typedef MAC_STRUCT panDescriptorBlock_tag
{
    panDescriptor_t               panDescriptorList[gScanResultsPerBlock_c];
    uint8_t                       panDescriptorCount;
    struct panDescriptorBlock_tag *pNext;
} panDescriptorBlock_t;

typedef MAC_STRUCT mlmeScanCnf_tag
{
    resultType_t            status;
    macScanType_t           scanType;
    channelPageId_t         channelPage;
    uint8_t                 resultListSize;
    channelMask_t           unscannedChannels;
    MAC_UNION {
        uint8_t*              pEnergyDetectList;
        panDescriptorBlock_t* pPanDescriptorBlockList; // must be freed by the upper layer
    } resList;
} mlmeScanCnf_t;

typedef MAC_STRUCT mlmeCommStatusInd_tag
{
    uint64_t                srcAddress;
    uint16_t                panId;
    addrModeType_t          srcAddrMode;
    uint64_t                destAddress;
    addrModeType_t          destAddrMode;
    resultType_t            status;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeCommStatusInd_t;

typedef MAC_STRUCT mlmeSetReq_tag
{
    pibId_t                 pibAttribute;
    uint8_t                 pibAttributeIndex;
    void*                   pibAttributeValue;
} mlmeSetReq_t;

typedef MAC_STRUCT mlmeSetCnf_tag
{
    resultType_t            status;
    pibId_t                 pibAttribute;
    uint8_t                 pibAttributeIndex;
} mlmeSetCnf_t;

typedef MAC_STRUCT mlmeResetReq_tag
{
    bool_t                  setDefaultPIB;
} mlmeResetReq_t;

typedef MAC_STRUCT mlmeResetCnf_tag
{
    resultType_t            status;
} mlmeResetCnf_t;

typedef MAC_STRUCT mlmeStartReq_tag
{
    uint16_t                panId;
    logicalChannelId_t      logicalChannel;
    channelPageId_t         channelPage;
    uint32_t                startTime;
    uint8_t                 beaconOrder;
    uint8_t                 superframeOrder;
    bool_t                  panCoordinator;
    bool_t                  batteryLifeExtension;
    bool_t                  coordRealignment;
    macSecurityLevel_t      coordRealignSecurityLevel;
    keyIdModeType_t         coordRealignKeyIdMode;
    uint64_t                coordRealignKeySource;
    uint8_t                 coordRealignKeyIndex;
    macSecurityLevel_t      beaconSecurityLevel;
    keyIdModeType_t         beaconKeyIdMode;
    uint64_t                beaconKeySource;
    uint8_t                 beaconKeyIndex;
} mlmeStartReq_t;

typedef MAC_STRUCT mlmeStartCnf_tag
{
    resultType_t             status;
} mlmeStartCnf_t;

typedef MAC_STRUCT mlmePollReq_tag
{
    addrModeType_t          coordAddrMode;
    uint16_t                coordPanId;
    uint64_t                coordAddress;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmePollReq_t;

typedef MAC_STRUCT mlmePollCnf_tag
{
    resultType_t            status;
} mlmePollCnf_t;

typedef MAC_STRUCT mlmeBeaconNotifyInd_tag
{
    uint8_t                 bsn;
    uint8_t                 pendAddrSpec;
    uint8_t                 sduLength;
    uint8_t*                pAddrList;
    panDescriptor_t*        pPanDescriptor;
    uint8_t*                pSdu;
    void*                   pBufferRoot; // Upper layer must free this buffer before freeing the indication message
} mlmeBeaconNotifyInd_t;

typedef MAC_STRUCT mlmeGtsReq_tag
{
    gtsCharacteristics_t    gtsCharacteristics;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeGtsReq_t;

typedef MAC_STRUCT mlmeGtsCnf_tag
{
    resultType_t            status;
    gtsCharacteristics_t    gtsCharacteristics;
} mlmeGtsCnf_t;

typedef MAC_STRUCT mlmeGtsInd_tag
{
    uint16_t                deviceAddress;
    gtsCharacteristics_t    gtsCharacteristics;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeGtsInd_t;

typedef MAC_STRUCT mlmeOrphanInd_tag
{
    uint64_t                orphanAddress;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeOrphanInd_t;

typedef MAC_STRUCT mlmeOrphanRes_tag
{
    uint64_t                orphanAddress;
    uint16_t                shortAddress;
    bool_t                  associatedMember;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeOrphanRes_t;

typedef MAC_STRUCT mlmeSyncReq_tag
{
    logicalChannelId_t      logicalChannel;
    channelPageId_t         channelPage;
    bool_t                  trackBeacon;
} mlmeSyncReq_t;

typedef MAC_STRUCT mlmeSyncLossInd_tag
{
    resultType_t            lossReason;
    uint16_t                panId;
    logicalChannelId_t      logicalChannel;
    channelPageId_t         channelPage;
    macSecurityLevel_t      securityLevel;
    keyIdModeType_t         keyIdMode;
    uint64_t                keySource;
    uint8_t                 keyIndex;
} mlmeSyncLossInd_t;


typedef MAC_STRUCT mlmePollNotifyInd_tag
{
    addrModeType_t          srcAddrMode;
    uint64_t                srcAddr;
    uint16_t                srcPanId;
} mlmePollNotifyInd_t;

/************************************************************************************/

typedef MAC_STRUCT macMsgHeader_tag{
    macMessageId_t      msgType;
} macMsgHeader_t;

typedef MAC_STRUCT nwkToMcpsMessage_tag
{
    macMessageId_t      msgType;
    MAC_UNION
    {
        mcpsDataReq_t       dataReq;
        mcpsPurgeReq_t      purgeReq;
    } msgData;
} nwkToMcpsMessage_t;

typedef MAC_STRUCT mcpsToNwkMessage_tag
{
    macMessageId_t      msgType;
    MAC_UNION
    {
        mcpsDataCnf_t       dataCnf;
        mcpsDataInd_t       dataInd;
        mcpsPurgeCnf_t      purgeCnf;
        mcpsPromInd_t       promInd;
    } msgData;
} mcpsToNwkMessage_t;

typedef MAC_STRUCT mlmeMessage_tag
{
    macMessageId_t      msgType;
    MAC_UNION
    {
        mlmeAssociateReq_t      associateReq;
        mlmeAssociateRes_t      associateRes;
        mlmeDisassociateReq_t   disassociateReq;
        mlmeGetReq_t            getReq;
        mlmeGtsReq_t            gtsReq;
        mlmeOrphanRes_t         orphanRes;
        mlmeResetReq_t          resetReq;
        mlmeRxEnableReq_t       rxEnableReq;
        mlmeScanReq_t           scanReq;
        mlmeSetReq_t            setReq;
        mlmeStartReq_t          startReq;
        mlmeSyncReq_t           syncReq;
        mlmePollReq_t           pollReq;
    } msgData;
} mlmeMessage_t;

typedef MAC_STRUCT nwkMessage_tag
{
    macMessageId_t      msgType;
    MAC_UNION
    {
        mlmeAssociateInd_t      associateInd;
        mlmeAssociateCnf_t      associateCnf;
        mlmeDisassociateInd_t   disassociateInd;
        mlmeDisassociateCnf_t   disassociateCnf;
        mlmeBeaconNotifyInd_t   beaconNotifyInd;
        mlmeGetCnf_t            getCnf;         // Not used
        mlmeGtsInd_t            gtsInd;
        mlmeGtsCnf_t            gtsCnf;
        mlmeOrphanInd_t         orphanInd;
        mlmeResetCnf_t          resetCnf;       // Not used
        mlmeRxEnableCnf_t       rxEnableCnf;
        mlmeScanCnf_t           scanCnf;
        mlmeCommStatusInd_t     commStatusInd;
        mlmeSetCnf_t            setCnf;         // Not used
        mlmeStartCnf_t          startCnf;
        mlmeSyncLossInd_t       syncLossInd;
        mlmePollCnf_t           pollCnf;
        //    mlmeErrorCnf_t           errorCnf;       // Test framework primitive.
        //    mlmeBeaconStartInd_t     beaconStartInd;
        //    mlmeMaintenanceScanCnf_t maintenanceScanCnf;
        mlmePollNotifyInd_t      pollNotifyInd;
    } msgData;
} nwkMessage_t;

/************************************************************************************/

/* MAC SAP handler functions types */
typedef resultType_t (*MCPS_NWK_SapHandler_t) (mcpsToNwkMessage_t* pMsg, instanceId_t upperInstanceId);
typedef resultType_t (*MLME_NWK_SapHandler_t) (nwkMessage_t* pMsg, instanceId_t upperInstanceId);

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/
void MAC_Init( void );

instanceId_t BindToMAC( instanceId_t nwkId );

void UnBindFromMAC ( instanceId_t macInstanceId );

macState_t Mac_GetState( void );

void Mac_RegisterSapHandlers( MCPS_NWK_SapHandler_t pMCPS_NWK_SapHandler,
                              MLME_NWK_SapHandler_t pMLME_NWK_SapHandler,
                              instanceId_t macInstanceId );

/* NWK to MAC SAP Handler */
resultType_t NWK_MCPS_SapHandler( nwkToMcpsMessage_t* pMsg, instanceId_t macInstanceId );
resultType_t NWK_MLME_SapHandler( mlmeMessage_t*      pMsg, instanceId_t macInstanceId );

/* Phy to MAC SAP Handler */
resultType_t PD_MAC_SapHandler  ( void* pMsg, instanceId_t macInstanceId );
resultType_t PLME_MAC_SapHandler( void* pMsg, instanceId_t macInstanceId );

/* Mac Helper Functions */
uint8_t      Mac_GetMaxMsduLength( mcpsDataReq_t* pParams );

#ifdef __cplusplus
    }
#endif

#endif  /* _MAC_INTERFACE_H */

