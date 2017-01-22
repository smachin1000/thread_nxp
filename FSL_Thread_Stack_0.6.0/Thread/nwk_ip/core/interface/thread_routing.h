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

#ifndef _THREAD_ROUTING_H_
#define _THREAD_ROUTING_H_

/*!=================================================================================================
\file       thread_routing.h
\brief      This is a header file for the Thread routing module.
==================================================================================================*/


/*==================================================================================================
Include Files
==================================================================================================*/

#include "thread_cfg.h"
#include "ip.h"
#include "network_utils.h"

#if THR_ROUTING_ENABLE
/*==================================================================================================
Public macros
==================================================================================================*/
    
#define THR_ROUTE_COST_MASK         0x0F
#define THR_IN_QUALITY_MASK         0x30
#define THR_IN_QUALITY_SHIFT        4U
#define THR_OUT_QUALITY_MASK        0xC0
#define THR_OUT_QUALITY_SHIFT       6U
    
#define THR_R_ID_ADDR_SHIFT         (16U-THR_ROUTER_BITS_SIZE)
#define THR_R_ID_TO_SHORT_ADDR(x)   (x << THR_R_ID_ADDR_SHIFT)
#define THR_SHORT_ADDR_TO_R_ID(x)   (x >> THR_R_ID_ADDR_SHIFT)
#define THR_GET_IN_QUALITY(x)       ((x & THR_IN_QUALITY_MASK) >> 4)

#define THR_NO_NEXT_HOP_ADDR   0xFFFF

/*==================================================================================================
Public type definitions
==================================================================================================*/


typedef enum
{   
    gLeaderAddrOk_c = 0x00,
    gLeaderNoAddrAval_c = 0x01,
    gLeaderNotActive_c = 0x02
}thrLeaderAssignStatus_t;

typedef enum
{   
    gRouteNull_c = 0x00,
    gRouteValid_c = 0x01,
    gRouteInvalid_c = 0x02,
    gRouteLeaderDelete_c =0x03
}thrRouteEntryStatus_t;


/*! Thread Routing link margin to link cost table entry */
typedef struct thrLinkMargintoLinkQEntry_tag
{
    uint8_t min;
    uint8_t max;
    uint16_t quality;
}thrLinkMargintoLinkQEntry_t;

/*! Thread routing Router ID set */
typedef struct thrRouterIdSet_tag
{
    uint8_t thrIdSeqNb;
    uint8_t thrIdSet[THR_MAX_ROUTER_ID/8];    
}thrRouterIdSet_t;

/*! Thread routing Link set */
typedef struct thrLinkSet_tag
{
    uint32_t thrLinkAge;    
    uint16_t thrRouterId;
    uint8_t thrLinkMargin;
    uint8_t thrOutgoingQual;
}thrLinkSet_t;

/*! Thread routing Route set */
typedef struct thrRouteSet_tag
{
    uint16_t thrMultiHopRouterId;
    uint16_t thrNextHopRouterId;
    uint8_t thrMultihopRouteCost;
    uint8_t thrRouteStatus;
}thrRouteSet_t;

/*! Thread ID Assignment set */
typedef struct thrIdAssignSet_tag
{   
    uint32_t thrReuseTime;
    uint8_t thrOwnerEui[gLlayerAddrEui64_c];    
}thrIdAssignSet_t;

/*! Structure with all Thread routing parameters for an interface */
typedef struct thrInterfaceSet_tag
{   
    ifHandle_t*    pIfHandle;
    thrRouteSet_t* pThreadRoutingTbl;
    thrLinkSet_t*  pThreadLinkSet;
    ipAddr_t*  ulaPrefix;
    thrIdAssignSet_t* pThreadIdAssignSet;
    tmrTimerID_t singleShotTmrId;
    tmrTimerID_t periodicTmrId;
    uint16_t deviceShortAddr; 
    thrRouterIdSet_t threadRouterIdSet;
    uint8_t thrRouterCount;   
}thrInterfaceSet_t;

/* Routing TLVs Types */
typedef enum routingTlvType_tag
{
    gThrTlvRoute_c = 0x09U
} routingTlvType_e;

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
\fn void THR_RoutingInit(taskMsgQueue_t *pTaskMsgQueue, ifHandle_t* pIfHandle,ipAddr_t* ulaPrefix,
         uint16_t deviceAddr, uint8_t* dataPtr)
\brief This function initializes the Thread Routing module.

\param [in]  pTaskMsgQueue     pointer to the task message queue
\param [in]  pIfHandle         double pointer to interface handle
\param [in]  ulaPrefix       pointer ULA prefix to be used to construct Router addresses
\param [in]  deviceAddr        device short address
\param [in]  dataPtr           data pointer to ID Seq number and ID Set received with the Router ID
                               or NULL if we are starting as leader device
                               
\retval      none
***************************************************************************************************/
void THR_RoutingInit(taskMsgQueue_t *pTaskMsgQueue, ifHandle_t* pIfHandle,ipAddr_t* ulaPrefix,
                             uint16_t deviceAddr, uint8_t* dataPtr);

/*!*************************************************************************************************
\fn void THR_RoutingInputService(void *param, ifHandle_t* pIfHandle, uint16_t peerShortAddr)
\brief  This function is the callback of Thread routing input module.

\param [in]     param       pointer to user data
\param [in]     pIfHandle   double pointer to interface
\param [in]     param       advertisement sender short address
\param [in]     linkMargin  link margin received from lower layers

\retval         none
***************************************************************************************************/
void THR_RoutingInputService(void *param, ifHandle_t* pIfHandle, uint16_t peerShortAddr, 
                                       uint8_t linkMargin);

/*!*************************************************************************************************
\fn uint8_t* THR_RoutingOutputService(ifHandle_t* pIfHandle)
\brief  This function is the callback of Thread routing output module.

\param  [in]    pIfHandle   pointer to interface handle

\retval         uint8_t *   pointer to output data allocated by this function. The memory should be
                            freed by the caller function when it is no longer needed.
***************************************************************************************************/
uint8_t* THR_RoutingOutputService(ifHandle_t* pIfHandle);

/*!*************************************************************************************************
\fn void THR_LeaderStart(taskMsgQueue_t *pTaskMsgQueue, ipAddr_t* ulaPrefix,ifHandle_t* pIfHandle)
\brief  Initializes the Leader functionality inside the Routing module. Calls THR_RoutingInit()
        to initialize the Routing module first.        

\param [in]  pTaskMsgQueue   pointer to the task message queue
\param [in]  ulaPrefix       pointer ULA prefix to be used to construct Router addresses
\param [in]  pIfHandle       double pointer to interface handle

\retval      none
***************************************************************************************************/
void THR_LeaderStart(taskMsgQueue_t *pTaskMsgQueue, ipAddr_t* ulaPrefix, 
                             ifHandle_t* pIfHandle);

/*!*************************************************************************************************
\fn thrLeaderAssignStatus_t THR_LeaderAssignId(uint8_t* eui, ifHandle_t* pIfHandle, 
    ipAddr_t** pOutIpAddr)
\brief  Assigns the first free Router Id found in the Id Set and returns UL Address constructed 
        with router Id 

\param [in]  eui         EUI64 of the device requesting a Router Id
\param [in]  pIfHandle   double pointer to interface handle
\param [out] pOutIpAddr  double pointer to where to store the UL Address

\retval      gLeaderAddrOk_c - address assignemet ok
\retval      gLeaderNoAddrAval_c - all the router ID's have been allocated
\retval      gLeaderNotActive_c - the device is not the active leader
***************************************************************************************************/
thrLeaderAssignStatus_t THR_LeaderAssignId(uint8_t* eui, ifHandle_t* pIfHandle, 
                                                     ipAddr_t** pOutIpAddr);

/*!*************************************************************************************************
\fn void THR_LeaderReadIdSeqAndMask(ifHandle_t* pIfHandle, void* pOutData)
\brief  This function is used to read from the Leader module the Id Sequence number and the Id Set 
        mask

\param [in]   pIfHandle    double pointer to interface handle
\param [out]  pOutData       pointer to where to store Id Seq number and Id Set mask

\retval       none
***************************************************************************************************/
void THR_LeaderReadIdSeqAndMask(ifHandle_t* pIfHandle, void* pOutData);

/*!*************************************************************************************************
\fn void THR_LeaderRemoveRouter(ifHandle_t* pIfHandle, uint8_t routerId)
\brief  Removes a router from the ID set and resets the trickle timer to send an update

\param [in]   pIfHandle    double pointer to interface handle
\param [out]  routerId   router short address to remove

\retval       none
***************************************************************************************************/
void THR_LeaderRemoveRouter(ifHandle_t* pIfHandle, uint8_t routerId);

/*!*************************************************************************************************
\fn void THR_NewLeaderDataNotify(ifHandle_t* pIfHandle)
\brief 

\param [in]  param   pointer to user data(interface index value)
    
\retval      none
***************************************************************************************************/
void THR_NewLeaderDataNotify(ifHandle_t* pIfHandle);

#ifdef __cplusplus
}
#endif

#endif /* THR_ROUTING_ENABLE */
/*================================================================================================*/
#endif  /*_THREAD_ROUTING_H_ */
