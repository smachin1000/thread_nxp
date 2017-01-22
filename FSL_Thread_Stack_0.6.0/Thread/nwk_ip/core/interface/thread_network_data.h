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

#ifndef _THREAD_NETWORK_DATA_H
#define _THREAD_NETWORK_DATA_H
/*!=================================================================================================
\file       thread_network_data.h
\brief      This is a header file for the Thread Network Data module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "mle.h"
#include "mle_extended.h"
#include "stack_manager_if.h"
#include "thread_manager.h"
#include "ip.h"
#include "coap.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#define LEADER_EXPIRATION_TIMEOUT           (30U) /*!< Seconds to wait for a leader update */


/*! Version number set strucuture and leader information */
typedef struct versionNumberSet_tag
{
    uint8_t         version;
    uint8_t         stableVersion;
    bool_t          stableOnly;
    bool_t          isLeader;
} versionNumberSet_t;

/*! Child Version Number Set */
typedef struct childVersNbSet_tag
{
    uint16_t        childShortAddr;
    bool_t          childStableOnly;
    uint8_t         childVersion;
} childVersNbSet_t;

/*! External Route Set */
typedef struct externalRouteSet_tag
{
    uint8_t         brEui[gLlayerAddrEui64_c];
    bool_t          brStableRoute;
    uint8_t         brPrefixIndex;
    uint16_t        brShortAddr;
} externalRouteSet_t;

/*! DHCPv6 Server Set */
typedef struct dhcpServerSet_tag
{
    uint8_t         dhcpEui[gLlayerAddrEui64_c];
    bool_t          dhcpStableRoute;
    uint8_t         dhcpPrefixIndex;
    uint16_t        dhcpShortAddr;
} dhcpServerSet_t;

/*! Context Id Set */
typedef struct contextIdSet_tag
{
    uint8_t         contextPrefixIndex;
    uint8_t         contextId;
    bool_t          contextFlags; /*!< lower 4 bits Stable Flag, higher 4 bits Compress Flag */
    uint8_t         contextLength;
} contextIdSet_t;

typedef struct
{
    ipAddr_t*             pNwkDataPrefixTbl;
    uint8_t*              pNwkDataPrefixLenTbl;
    externalRouteSet_t*   pNwkDataExtRouteTbl;
    dhcpServerSet_t*      pNwkDataDhcp6Tbl;
} serverData_t;

/*! Structure with all Thread routing parameters for an interface */
typedef struct nwkDataInterfaceSet_tag
{
    ifHandle_t*           pIfHandle;
    
    ipAddr_t*             pNwkDataPrefixTbl;
    uint8_t*              pNwkDataPrefixLenTbl;
    externalRouteSet_t*   pNwkDataExtRouteTbl;
    dhcpServerSet_t*      pNwkDataDhcp6Tbl;
    contextIdSet_t*       pNwkDataContextTbl;
    childVersNbSet_t*     pNwkDataChildVersNbSet;

    mleOtaTlvLeaderData_t leaderData;
    versionNumberSet_t    nwkDataLeaderInfo;
    serverData_t          serverData;
}nwkDataInterfaceSet_t;

typedef struct nwkDataPropagateParams_tag
{
    ifHandle_t  *pIfHandle;
    ipAddr_t    *pDestIpAddr;
    bool_t      onlyStable;
    bool_t      hideStableShort;
} nwkDataPropagateParams_t;

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*!*************************************************************************************************
\fn  void Thread_NwkDataInit(taskMsgQueue_t *pTaskMsgQueue, threadInstance_t *pThreadInstance,
       bool_t stableOnly )
\brief This is a public function used to Initialize the Network Data Module.

\param [in] pTaskMsgQueue       pointer to the RX packet parameters
\param [in] pThreadInstance     pointer to Thread instance
\param [in] stableOnly          TRUE if device requires only stable data

\retval     none
***************************************************************************************************/
void Thread_NwkDataInit(taskMsgQueue_t *pTaskMsgQueue, threadInstance_t *pThreadInstance,
                                  bool_t stableOnly);

/*!*************************************************************************************************
\fn    Thread_NwkDataProcess(mleCallbackParams_t *pMleCallbackParams, uint8_t *pTlv)
\brief This is a public function used to update the Leader Information on this Node.

\param [in] pMleCallbackParams pointer to the RX packet parameters
\param [in] pTlv               pointer to the TLV array

\retval     none
***************************************************************************************************/
void Thread_NwkDataProcess(    ifHandle_t *pIfHandle, uint8_t *pTlv);

/*!*************************************************************************************************
\fn     void Thread_LeaderDataProcess(mleOtaTlvLeaderData_t *pOtaLeaderData,ifHandle_t* pIfHandle)
\brief  This is a public function used process the Leader TLV on routers and End Devices

\param  [in]    pOtaLeaderData  pointer to the Leader Data received over the air.
\param  [in]    pIfHandle       pointer to the interface handle

\return         bool_t          TRUE if Network Data has changed and needs update
                                FALSE otherwise
***************************************************************************************************/
bool_t Thread_LeaderDataProcess(ifHandle_t* pIfHandle,
                                               ipAddr_t *pSenderIp,
                                               mleOtaTlvLeaderData_t *pOtaLeaderData);
/*!*************************************************************************************************
\fn    void THR_LeaderIncrementVerNb(ifHandle_t* pIfHandle, uint16_t leaderShortAddr)
\brief Initializes the Leader Data

\param [in]   pIfHandle        double pointer to interface handle
\param [in]   leaderShortAddr  Leader short address

\retval       none
***************************************************************************************************/
void Thread_LeaderDataInit(ifHandle_t* pIfHandle, uint16_t leaderShortAddr);

/*!*************************************************************************************************
\fn     void* Thread_LeaderDataTlvAdd(ifHandle_t* pIfHandle)
\brief  Function used to add the Leader Data TLV to a TLV list.

\param [in]   pIfHandle    double pointer to interface handle

\return       void*        pointer to the allocated TLV
***************************************************************************************************/
void* Thread_LeaderDataTlvAdd(ifHandle_t* pIfHandle);
/*!*************************************************************************************************
\fn    uint16_t Thread_NwkDataGetLeaderAddr(pIfHandle)
\brief This is a public function used to Get the Leader short address.

\param [in]  pIfHandle    pointer to the interface handle

\retval      uint16_t     current Leader short address
***************************************************************************************************/
uint16_t Thread_NwkDataGetLeaderAddr(ifHandle_t* pIfHandle);

/*!*************************************************************************************************
\fn     void Thread_NwkDataMleTlvAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add a TLV to a TLV list.

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void Thread_NwkDataMleTlvAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void Thread_NwkGetUlaPrefix(ipAddr_t *pUlaPrefix, uint32_t *pUlaPrefixLen)
\brief  This is a public function used to get a pointer to the ULA prefix

\param [out]     ppUlaPrefix        Double Pointer to ULA prefix variable
\param [out]     pUlaPrefixLen      Pointer to ULA prefix length variable

\retval         None.
***************************************************************************************************/
void Thread_NwkGetUlaPrefix(ipAddr_t **ppUlaPrefix, uint32_t *pUlaPrefixLen);

uint8_t * Thread_NwkDataCreateTlv(ifHandle_t * pIfHandle, bool_t onlyStable, bool_t hideStableShort);

void Thread_NwkDataDhcpServerSet(ifHandle_t *pIfHandle,    uint8_t *pDhcpPrefix,
                                                   uint32_t prefixLength, dhcpServerSet_t* pDhcpInfo);

void Thread_NwkDataExtRouteSet(ifHandle_t *pIfHandle, uint8_t *pRouterPrefix,
                                               uint32_t prefixLength, externalRouteSet_t* pExtRouteInfo);

void Thread_DataSetDhcpServer(ifHandle_t *pIfHandle,    uint8_t *pDhcpPrefix,
                                                   uint32_t prefixLength, dhcpServerSet_t* pDhcpInfo);

void Thread_DataSetExtRoute(ifHandle_t *pIfHandle, uint8_t *pRouterPrefix,
                                               uint32_t prefixLength, externalRouteSet_t* pExtRouteInfo);


void Thread_NwkDataChildVersionSet(ifHandle_t *pIfHandle, childVersNbSet_t *pChildVersionSet);

childVersNbSet_t * Thread_NwkDataChildVersionGet(ifHandle_t *pIfHandle, uint16_t childAddress);

void Thread_NwkDataIncrementVersion(ifHandle_t *pIfHandle, bool_t stableModified);

void Thread_NwkDataPropagate(ifHandle_t *pIfHandle, ipAddr_t *pDestIpAddr,
                                        bool_t onlyStable, bool_t hideStableShort);

void Thread_NwkDataSleepyNodesPropagate(ifHandle_t *pIfHandle);

void Thread_NwkDataRequest(ifHandle_t *pIfHandle, ipAddr_t *pDestIp);

void Thread_ServerDataRegisterRcv(bool_t sessionStatus,void* pData, coapSession_t* pSession, uint32_t pDataLen);

void Thread_ServerDataRegister(void * pData);

void Thread_SolicitGlobalAddr(threadInstance_t *pThreadInstance);

void Thread_ServerDataRemoveAll(ifHandle_t *pIfHandle);


/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _THREAD_NETWORK_DATA_H */
