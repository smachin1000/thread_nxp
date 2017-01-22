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

#ifndef _SIXLOWPAN_TBL_H
#define _SIXLOWPAN_TBL_H
/*!=================================================================================================
\file       sixlowpan_tbl.h
\brief      This is a header file for the SixlowPan table management functions.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "sixlowpan.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#define RFC4944_CLEAN_UP_PERIOD_MS                  1000

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef struct dgramOffset_tag
{
    uint16_t offset;
    uint16_t fragSize;
} dgramOffset_t;

typedef struct fragInfo_tag
{
    macAbsMcpsDataInd_t * pMcpsDataInd;
    uint32_t timerId;
    uint16_t dgramSize;
    uint16_t dgramTag;
    uint16_t totalRcvSize;
    uint16_t fragOptions;
    dgramOffset_t dgramOffset[SLWPCFG_RFC4944_MAX_FRAGMENTS];
} fragInfo_t;

typedef struct ndContextEntry_tag
{
    ipAddr_t    context;            /*!< Context */
    void **     pIfHandle;          /*!< Double pointer to media interface configuration structure */

    uint32_t    lifetime;           /*!< Valid Lifetime
                                     * 32-bit unsigned integer. The length of time in
                                     * seconds. */

    uint32_t    expireTimestamp;

    uint16_t    contextLength;      /*!< Context length (in bits). The number of leading bits
                                     * in the Context that are valid. */
    uint8_t     cid;                /*!< Context Id */
    uint8_t     usedForContextComp; /*!< Flag indication whether the entry can be used */
    uint32_t    stateTimestamp;     /*!< Timestamp for the last state change. Used for context expiration */
} ndContextEntry_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn     bool_t RFC4944_IsInAdpGroupTable(uint16_t adpGroup)
\brief  Function used to check if a MESH group address is in the MESH multicast group table.

\param  [in]    adpGroup    Mesh multicast group.

\retval         FALSE       The group is not in the group table.
\retval         TRUE        The group is in the group table.
***************************************************************************************************/
bool_t RFC4944_IsInAdpGroupTable(uint16_t adpGroup);

/*!*************************************************************************************************
\fn     bool_t RFC4944_IsInBCastLogTable(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t bcastSeqNum)
\brief  Function used to check if a broadcast packet is in the broadcast log table.

\param  [in]    pMcpsDataInd    The MCPS Data Indication containing the received packet.
\param  [in]    bcastSeqNum     The broadcast sequence number of the received packet.

\retval         FALSE       The packet has not been received before.
\retval         TRUE        The packet has already been received.
***************************************************************************************************/
bool_t RFC4944_IsInBCastLogTable(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t bcastSeqNum);

/*!*************************************************************************************************
\fn     void RFC4944_BCastLogTableAdd(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t bcastSeqNum)
\brief  Function used to add a broadcast packet is in the broadcast log table.

\param  [in]    pMcpsDataInd    The MCPS Data Indication containing the received packet.
\param  [in]    bcastSeqNum     The broadcast sequence number of the received packet.
***************************************************************************************************/
void RFC4944_BCastLogTableAdd(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t bcastSeqNum);

/*!*************************************************************************************************
\fn     void RFC4944_InitQueues(void)
\brief  Function used initialize the 6LoWPAN queues.
***************************************************************************************************/
void RFC4944_InitQueues(void);

/*!*************************************************************************************************
\fn     fragInfo_t * RFC4944_RxQueueGet(macAbsMcpsDataInd_t * pMcpsDataInd, uint16_t dgramSize, 
                                             uint16_t dgramTag)
\brief  Function used to retrieve a packet stored in the Rx queue.

\param  [in]    pMcpsDataInd    The MCPS Data Indication containing the received packet.
\param  [in]    dgramSize       The datagram size.
\param  [in]    dgramTag        The datagram tag.

\return         fragInfo_t      The Rx queue element.
***************************************************************************************************/
fragInfo_t * RFC4944_RxQueueGet(macAbsMcpsDataInd_t * pMcpsDataInd, uint16_t dgramSize, uint16_t dgramTag);

/*!*************************************************************************************************
\fn     void RFC4944_CleanupRxQueue(void *pParam)
\brief  Function used to clean the 6LoWPAN Rx queue of packets that have not received all fragments 
        for a predefined period of time.

\param  [in]    pParam      Pointer to the interval timer parameters.
***************************************************************************************************/
void RFC4944_CleanupRxQueue (void *pParam);

/*!*************************************************************************************************
\fn     bool_t RFC4944_RxQueueAdd(fragInfo_t * pFragInfo)
\brief  Function used to add a packet to the 6LoWPAN Rx queue.

\param  [in]    pFragInfo       Fragment information structure.

\return         bool_t          TRUE if element was added to the list
                                FALSE if list is full
***************************************************************************************************/
bool_t RFC4944_RxQueueAdd(fragInfo_t * pFragInfo);

/*!*************************************************************************************************
\fn     void RFC4944_RxQueueRemove(fragInfo_t * pFragInfo)
\brief  Function used to remove a packet from the 6LoWPAN Rx queue.

\param  [in]    pFragInfo       Fragment information structure.
***************************************************************************************************/
void RFC4944_RxQueueRemove(fragInfo_t * pFragInfo);

/*!*************************************************************************************************
\fn     slwpPktInfo_t * RFC4944_TxQueueGet(uint8_t msduHandle, instanceId_t instanceId)
\brief  Function used to retrieve a packet stored in the Tx queue.

\param  [in]    msduHandle      The MSDU handle of the packet in the Tx queue.
\param  [in]    instanceId      The 6LoWPAN instace id.

\return         slwpPktInfo_t   The Tx queue element.
***************************************************************************************************/
slwpPktInfo_t * RFC4944_TxQueueGet(uint8_t msduHandle, instanceId_t instanceId);

/*!*************************************************************************************************
\fn     slwpPktInfo_t * RFC4944_TxQueueGetByDest(uint64_t address, uint8_t addMode, uint16_t panId,
                                                 instanceId_t instanceId)
\brief  Function used to retrieve a packet stored in the Tx queue.

\param  [in]    address         The destination address of the packet in the Tx queue.
\param  [in]    addrMode        The destination address mode of the packet in the Tx queue.
\param  [in]    panId           The destination PAN Id of the packet in the Tx queue.
\param  [in]    instanceId      The 6LoWPAN instace id.

\return         slwpPktInfo_t   The Tx queue element.
***************************************************************************************************/
slwpPktInfo_t * RFC4944_TxQueueGetByDest(uint64_t address, uint8_t addMode, uint16_t panId, 
                                                          instanceId_t instanceId);

/*!*************************************************************************************************
\fn     void RFC4944_RxQueueAdd(slwpPktInfo_t * pSlwpPktInfo)
\brief  Function used to add a packet to the 6LoWPAN Tx queue.

\param  [in]    pSlwpPktInfo       6LoWPAN transmission structure for the packet.
***************************************************************************************************/
void RFC4944_TxQueueAdd(slwpPktInfo_t * pSlwpPktInfo);

/*!*************************************************************************************************
\fn     void RFC4944_TxQueueRemove(slwpPktInfo_t * pSlwpPktInfo)
\brief  Function used to remove a packet to the 6LoWPAN Tx queue.

\param  [in]    pSlwpPktInfo       6LoWPAN transmission structure for the packet.
***************************************************************************************************/
void RFC4944_TxQueueRemove(slwpPktInfo_t * pSlwpPktInfo);

/*!*************************************************************************************************
\fn     ndContextEntry_t* RFC6282_ContextTableGet(void **pIfHandle, ipAddr_t *pContext,
                                                uint32_t contextLen,uint32_t *pContextIdx)
\brief  This function is used for finding an entry in the context table corresponding to the media
        interface pointed by ifHandle and having the context equal to context indicated by pContext

\param  [in]  pIfHandle      double pointer to media interface configuration structure
\param  [in]  pContext       pointer to the context
\param  [in]  contextLen     context length
\param  [out] pContextIdx    pointer to where to write the context index

\retval       double pointer to the context entry or NULL if the entry is not found
***************************************************************************************************/
ndContextEntry_t ** RFC6282_ContextTableGet(void **pIfHandle, ipAddr_t *pContext,
                                                uint32_t contextLen,uint32_t *pContextIdx);

/*!*************************************************************************************************
\fn     ndContextEntry_t* RFC6282_ContextTableGetByCID(instanceId_t instanceId, uint8_t cid)
\brief  This function is used for finding an entry in the context table corresponding to the media
        interface pointed by ifHandle and having the context equal to context indicated by pContext

\param  [in]  instanceId    pointer to media interface
\param  [in]  cid           context id

\retval       pointer to the context entry or NULL if the entry is not found
***************************************************************************************************/
ndContextEntry_t * RFC6282_ContextTableGetByCID(instanceId_t instanceId, uint8_t cid);

/*!*************************************************************************************************
\fn     bool_t RFC6282_ContextTableAdd(void **pIfHandle, ipAddr_t *pContext, uint32_t contextLength,
                           uint32_t cid, uint32_t bUsedForContextComp, uint16_t lifetime)
\brief  This function is used for adding an entry in the context table corresponding to the media
        interface pointed by ifHandle and having the context equal to context indicated by pContext.
        If there are no free entries in the context table then the oldest entry corrsponding to this
        media interface will be replaced. If there are no free entries in the table and there are
        no entries corresponding to this media interface then FALSE is returned and the entry is
        not added.

\param  [in]  pIfHandle            double pointer to media interface configuration structure
\param  [in]  pContext             pointer to the context
\param  [in]  contextLength        context length, must be bigger than 0
\param  [in]  cid                  context identifier
\param  [in]  bUsedForContextComp  context compression flag
\param  [in]  lifetime             context lifetime

\retval       TRUE          returned when the entry was successfully added in the context table
\retval       FALSE         otherwise
***************************************************************************************************/
bool_t RFC6282_ContextTableAdd(void **pIfHandle, ipAddr_t *pContext, uint32_t contextLength,
                           uint32_t cid, uint32_t bUsedForContextComp, uint16_t lifetime);

/*!*************************************************************************************************
\fn     void RFC6282_ContextTableRemove(uint32_t contextIdx)
\brief  This function is used for removing an entry from the table. This entry is freed and set to NULL.

\param  [in]  contextIdx      index in the Context table entry
***************************************************************************************************/
void RFC6282_ContextTableRemove(uint32_t contextIdx);

/*!*************************************************************************************************
\fn     ndContextEntry_t * RFC6282_ContextTableGetByIndex(uint32_t index)
\brief  Counts the total number of contexts stored per an interface.

\param  [in]    index                   Index in the context table.

\return         ndContextEntry_t *      Pointer to table entry.
***************************************************************************************************/
ndContextEntry_t * RFC6282_ContextTableGetByIndex(uint32_t index);

/*!*************************************************************************************************
\fn   ndContextEntry_t* RFC6282_GetLongestMatchingContext(uint8_t* pIpAddress, instanceId_t instanceId)
\brief  This function is used for finding the longest context that matches the provided IPv6 address

\param [in]   pIpAddress          IPv6 address to match
\param [in]   instanceId          Pointer to media interface configuration structure

\retval       ndContextEntry_t*  pointer to the context entry or NULL if entry is not found
***************************************************************************************************/
ndContextEntry_t * RFC6282_GetLongestMatchingContext(uint8_t* pIpAddress, instanceId_t instanceId);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/

#endif  /* _SIXLOWPAN_TBL_H */
