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

#ifndef _SIXLOWPAN_H
#define _SIXLOWPAN_H
/*!=================================================================================================
\file       sixlowpan.h
\brief      This is a header file for the 6LoWPAN module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "EmbeddedTypes.h"
#include "network_utils.h"
#include "sixlowpan_interface.h"
#include "sixlowpan_ib.h"

/*==================================================================================================
Public macros
==================================================================================================*/



/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef enum
{
    gAdpTransmissionTypeUnicast_c,                  /* Unicast trasmission */
    gAdpTransmissionTypeMulticast_c,                /* Multicast transmission */
    gAdpTransmissionTypeBroadcast_c                 /* Broadcast transmission */
} transmissionType_t;

typedef struct fragmentInfo_tag
{
    uint32_t        datagramOffsetInBytes;
    uint16_t        fragNumber;                     /* Fragment number. Starts from 1. */
    uint16_t        datagramTag;                    /* Datagram TAG */
} fragmentInfo_t;

typedef struct compressionInfo_tag
{
    uint8_t *           pUncompData;                /* Pointer to remaining Uncompressed Data
                                                       from AdpPayload */
    uint8_t *           pCompHeadPos;               /* Original AdpPayload size - Compressed Data size */
} compressionInfo_t;

typedef struct dispatchInfo_tag
{
    uint8_t type;
    uint8_t command;
} dispatchInfo_t;

typedef struct slwpPktInfo_tag
{
    instanceId_t            instanceId;             /* 6LoWPAN instance Id */

    macAbsMcpsDataReq_t *   pMcpsDataReq;           /* Pointer to McpsDataReq structure that will be passed to MAC */
    nwkBuffer_t *           pMeshBCastHeader;       /* Pointer to Mesh & Broadcast Header */
    nwkBuffer_t *           pCompressedHeader;      /* Pointer to Compressed Header */
    compressionInfo_t       compressInfo;           /* Compression information */
    fragmentInfo_t          fragInfo;               /* Fragment information */
    dispatchInfo_t          dispatchInfo;           /* Dispatch information */ ///TODO: maybe remove it?
    transmissionType_t      transmissionType;       /* Unicast / Multicast / Broadcast */
    adpPacketType_t         packetType;             /* Type of Packet: IPv6, LBP, Mesh Routing, etc */
    uint32_t                txTimestamp;            /* Timestamp of MAC transmission */
    nwkBuffer_t             adpPayload;             /* Original ADP Payload from upper layers
                                                       (IPv6, LBP, MESH) */
    uint8_t                 nsduHandle;             /* nsduHandle for the ADP packet */
    uint8_t                 retriesNb;              /* number of 6LoWPAN retries performed so far */
} slwpPktInfo_t;

typedef struct slwpStruct_tag
{
    instanceId_t        macInstanceId;
    instanceId_t        ifInstanceId;
    macAbsRequests_t *  pMacAbsReq;
    adpIb_t **          ppAdpIb;
    adpdCb_t *          pAdpdCb;
    adpmCb_t *          pAdpmCb;
    isSleepyDev_t       pfIsSleepyDev;
#if (SLWPCFG_LBP_ENABLED)
    lbpCb_t *           pLbpCb;
#endif
} slwpStruct_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

extern taskMsgQueue_t mSlwpMsgQueue;
extern slwpStruct_t * mpSlwpStruct[];
extern adpIb_t *      mapAdpIb[];

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn     void SLWP_Task(void const *pArgument)
\brief  6LoWPAN layer task

\param  [in]    pArgument    Task private data
***************************************************************************************************/
void SLWP_Task(osaTaskParam_t argument);

/*!*************************************************************************************************
\fn     void SLWP_HandleAdpdDataReq(void * pPayload)
\brief  Function to handle the ADPD Data Request message.

\param  [in]    pPayload       Pointer to the ADPD Data Request structure.
***************************************************************************************************/
void SLWP_HandleAdpdDataReq(void * pPayload);

/*!*************************************************************************************************
\fn     void SLWP_HandleMcpsDataCnf(void *pPayload)
\brief  Function used to handle the received MCPS Data Confirmation.

\param  [in]    pPayload    Pointer to the received MCPS Data Confirmation.
***************************************************************************************************/
void SLWP_HandleMcpsDataCnf(void * pPayload);

/*!*************************************************************************************************
\fn     void SLWP_HandleMcpsDataInd(void *pPayload)
\brief  Function used to handle the received MCPS Data Indication.

\param  [in]    pPayload    Pointer to the received MCPS Data Indication.
***************************************************************************************************/
void SLWP_HandleMcpsDataInd(void * pPayload);

/*!*************************************************************************************************
\fn     void SWLP_HandleMlmePollIndCB(void * pPayload)
\brief  Function used to handle the received Poll Indication.

\param  [in]    pPayload    Pointer to the received Poll Indication.    
***************************************************************************************************/
void SWLP_HandleMlmePollIndCB(void * pPayload);

/*!*************************************************************************************************
\fn     void SLWP_FreePktInfo(slwpPktInfo_t * pSlwpPktInfo)
\brief  Function used to free the 6LoWPAN packet information.

\param  [in]    pSlwpPktInfo      Pointer to the 6LoWPAN packet information structure.
***************************************************************************************************/
void SLWP_FreePktInfo(slwpPktInfo_t * pSlwpPktInfo);

/*!*************************************************************************************************
\fn     void SLWP_ForwardPacket(macAbsMcpsDataInd_t * pMcpsDataInd, llAddr_t nextHopAddr)
\brief  Function used to forward a received MCPS Data Indication.

\param  [in]    pMcpsDataInd    Pointer to the received MAC Data Indication.
\param  [in]    nextHopAddr     Link-Layer address of the next hop.
***************************************************************************************************/
void SLWP_ForwardPacket(macAbsMcpsDataInd_t * pMcpsDataInd, llAddr_t nextHopAddr);

/*!*************************************************************************************************
\fn     void SLWP_DropPacket(macAbsMcpsDataInd_t * pMcpsDataInd)
\brief  Function used to free the MCPS Data Indication.

\param  [in]    pMcpsDataInd      Pointer to the MCPS Data Indication structure.
***************************************************************************************************/
void SLWP_DropPacket(macAbsMcpsDataInd_t * pMcpsDataInd);

/*!*************************************************************************************************
\fn     void SLWP_GenerateAdpdDataInd(macAbsMcpsDataInd_t * pMcpsDataInd)
\brief  Function used to generate a ADPD Data Indication from a received MCPS Data Indication.

\param  [in]    pMcpsDataInd  Pointer to the MCPS Data Indication (MUST NOT be NULL)
***************************************************************************************************/
void SLWP_GenerateAdpdDataInd(macAbsMcpsDataInd_t * pMcpsDataInd);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/


#endif  /*_SIXLOWPAN_H */
