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

#ifndef _MLE_H
#define _MLE_H
/*!=================================================================================================
\file       mle.h
\brief      This is a header file for the Mesh Link Establishment module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"

/* Network Includes */
#include "network_utils.h"

/*==================================================================================================
Public macros
==================================================================================================*/

#define MLE_MSG_HOP_LIMIT           255

#define MLE_CMD_TYPE_SIZE           1

#define MLE_TLV_HEADER_SIZE         2

/* Macros to Register the Tlv And Command Handlers */

#if defined(__IAR_SYSTEMS_ICC__)
#pragma section="MLE_HANDLERS"
#endif

#if defined(__IAR_SYSTEMS_ICC__)

#define MLE_RegisterTlvHandlers(type, pfAddFunction, pfRcvFunction) _Pragma("location=\"MLE_HANDLERS\"") __root const mleTlvHandlers_t \
TlvHandlerEntry_##type = { pfAddFunction, pfRcvFunction, type }

#elif defined(__GNUC__)

#define MLE_RegisterTlvHandlers(type, pfAddFunction, pfRcvFunction) \
    const mleTlvHandlers_t TlvHandlerEntry_#type __attribute__ ((section ("MLE_HANDLERS"), used)) = \
    { pfAddFunction, pfRcvFunction, type }

#else

#define MLE_RegisterTlvHandlers(type, pfAddFunction, pfRcvFunction) \
    const mleTlvHandlers_t TlvHandlerEntry_#type = \
    { pfAddFunction, pfRcvFunction, type }

#endif

/*! Macro used for extracting the MLE command type from the beginning of a MLE message */
#define MLE_CMD_TYPE(p)             (*((uint8_t *)(p)))

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*! Structure which contains details about the received MLE packet */
typedef struct mleCallbackParams_tag
{
    ipAddr_t senderIp;
    uint8_t *pMleCommand;
    void *pIfHandle;
    void *pReceivedBuffer;
    uint16_t tlvDataSize;
    bool_t secured;
    uint8_t linkMargin;
} mleCallbackParams_t;

/*! Function pointer type declaration for the function which is to be registered as a callback */
typedef void (*mleCallback_t)(void *pParams);

/*! Function pointer type declaration for MLE processing functions */
typedef uint32_t (*mleTlvRcvFct_t)(uint8_t *pTlv, list_t *pTlvList);

/*! Function pointer type declaration for MLE processing functions */
typedef void (*mleTlvAddFct_t)(void *pParams, list_t *pTlvList);

/*! Structure which holds the add/receive function pointers for a TLV */
typedef struct mleTlvHandlers_tag
{
    mleTlvAddFct_t  pfMleTlvAdd;
    mleTlvRcvFct_t  pfMleTlvRcv;
    uint32_t        tlvType;
} mleTlvHandlers_t;

/*! MLE Command Type type */
typedef uint8_t mleCmdType_t;

/*! MLE TLV Type type */
typedef uint8_t mleTlvType_t;

/**************/
/* TLV Header */
/**************
     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
     |    Type       |    Length     |
     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

***************/

/*! TLV Header */
typedef struct tlvHdr_tag
{
    uint8_t type;
    uint8_t length;
} tlvHeader_t;

/*! Neighbor Data Record */
typedef struct mleNeighborRecord_tag
{
    uint8_t  incoming;
    uint8_t  outgoing;
    uint8_t  priority;
    uint8_t  incomingIDR;
    llAddr_t address;
} mleNeighborRecord_t;

/*! Network Parameter TLV Type */
typedef uint16_t nwkParamType_t;

/*! Network Parameter Channel TLV */
typedef struct nwkParamChannel_tag
{
    uint32_t channel;
} nwkParamChannel_t;

/*! Network Parameter PAN Id TLV */
typedef struct nwkParamPanID_tag
{
    uint32_t panId;
} nwkParamPanID_t;

/*! Network Parameter Permit Joining TLV */
typedef struct nwkParamPermitJoining_tag
{
    uint32_t permitJoining;
} nwkParamPermitJoining_t;

/*! Network Parameter Route TLV */
typedef struct mleTlvRoute_tag
{
    mleTlvType_t tlvType;
    uint16_t idSequence;
    uint16_t nbOfRouters;
    uint8_t  aNeighborRouteCosts[];
} mleTlvRoute_t;

typedef struct mleTlvServer_tag
{
    mleTlvType_t tlvType;
    list_t subTlvList;
} mleTlvServerData_t;

/*! MLE send parameters */
typedef struct mleSendParams_tag
{    
    list_t *pTlvList;           /*!< Pointer to TLV list */
    void *pIfHandle;            /*!< Double pointer to interface struct */
    ipAddr_t *pIpSrcAddr;       /*!< Pointer to source IP address */
    ipAddr_t *pIpDstAddr;       /*!< Pointer to destination IP address */
    bool_t secured;             /*!< TRUE if MLE frame should be sent secured or FALSE otherwise */
    uint8_t macSecurityLevel;   /*!< MAC security level */
    uint8_t hopLimit;           /*!< the number of hops to be set in the IP packet only for this send */
    mleCmdType_t cmdType;       /*!< The MLE command type */
}mleSendParams_t;

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
\fn     void MLE_Init(taskMsgQueue_t *pTaskMsgQueue)
\brief  Interface function for the Mesh Link Establishment module. Initializes the module.

\param  [in]    pTaskMsgQueue       task message queue
***************************************************************************************************/
void MLE_Init(taskMsgQueue_t *pTaskMsgQueue);

/*!*************************************************************************************************
\fn     void MLE_RegisterCb(mleCallback_t pfMleCallback)
\brief  Interface function for the Mesh Link Establishment module. It registers a callback function
        which will be called when a MLE packet is received.

\param  [in]    pfMleCallback       Pointer to the function that will be called when a MLE packet
                                    is received.
***************************************************************************************************/
void MLE_RegisterCb(mleCallback_t pfMleCallback);

/*!*************************************************************************************************
\fn     bool_t MLE_TlvAdd(void *pParams, list_t *pTlvList)
\brief  Interface function for the Mesh Link Establishment module. It adds a TLV to a TLV list.

\param  [in]    pParams         Pointer to the TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.

\retval         TRUE            If the TLV was added to the list.
\retval         FALSE           If the TLV was not added to the list.
***************************************************************************************************/
bool_t MLE_TlvAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn void MLE_Send(mleSendParams_t *pMleSendParams)
\brief  Interface function for the Mesh Link Establishment module. It sends a MLE packet.

\param  [in]    pMleSendParams      Pointer to the MLE_Send parameters
***************************************************************************************************/
void MLE_Send(mleSendParams_t *pMleSendParams);

/*!*************************************************************************************************
\fn     uint8_t *MLE_TlvGetNext(uint8_t *pStart, uint16_t *pRemainingLength)
\brief  MLE function used to get a pointer to the next TLV in a TLV buffer list.

\param  [in]     pStart              Pointer to the start of the buffer list.
\param  [in,out] pRemainingLength    Pointer to the length left to parse from the original buffer.

\return         uint8_t*            Pointer to the next TLV.
***************************************************************************************************/
uint8_t *MLE_TlvGetNext(uint8_t *pStart, uint16_t *pRemainingLength);

#ifdef __cplusplus
}
#endif

/*================================================================================================*/
#endif  /* _PMTU_DISCOVERY_H */
