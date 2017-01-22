/*!=================================================================================================
\file       ip_if_6LowPan.c
\brief      This is a private source file for the 6LowPan Media interface.
            
\copyright  Freescale Confidential Proprietary. No part of this document must be reproduced in any 
\copyright  form - including copied, transcribed, printed or by any electronic means - without 
\copyright  specific written permission from Freescale.
\copyright  (c) Copyright 2013, Freescale, Inc.  All rights reserved.
==================================================================================================*/
#include "app_to_stack_config.h"
#include "stack_config.h"

#include "ip.h"
#include "ip_if_management.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "nd.h"
#include "sixlowpan.h"

#if IP_IP6_ENABLE
/*==================================================================================================
Private macros
==================================================================================================*/

#define MTU_6LOWPAN  1280               
/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

void IP_6LowPanRecv(adpdDataInd_t * pAdpdDataInd);
void IP_6LowPanDataCnf(adpdDataCnf_t* pAdpdDataCnf);

uint32_t IP_6LowPanOpen(ifHandle_t* pIfHandle);
uint32_t IP_6LowPanClose(ifHandle_t* pIfHandle);
uint32_t IP_6LowPanSend(ipPktInfo_t* pIpPktInfo);
uint32_t IP_6LowPanGetIID(ifHandle_t* pIfHandle,llAddr_t* macAddr,ipAddr_t* ipAddr);
uint32_t IP_6LowPanJoin(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr,uint16_t protocol);
uint32_t IP_6LowPanLeave(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr,uint16_t protocol);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

static const uint8_t* mIfName = "6LoWPAN";

static const mediaIfStruct_t g6LowPanMediaIf =
{
    IP_6LowPanOpen,
    IP_6LowPanClose, 
    NULL,
    NULL,
    IP_6LowPanSend,
    IP_6LowPanGetIID,
    IP_6LowPanJoin,
    IP_6LowPanLeave
};

const mediaIfStruct_t* g6LowPanMediaIfPtr = (mediaIfStruct_t*) &g6LowPanMediaIf;

static const adpdCb_t mAdpCallbacks = {IP_6LowPanDataCnf,IP_6LowPanRecv};
static uint32_t mNsduHandle = 0;
/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\fn  void IP_6LowPanRecv(ipPktInfo_t* pIpPktInfo)

\brief  Services an 6LowPan packet

\param [in]  pParam    the received data indication 
    
\retval      none
***************************************************************************************************/
void IP_6LowPanRecv
(
    adpdDataInd_t * pAdpdDataInd
)
{ 
    ipPktInfo_t* pIpPktInfo;
   
    IPIF_STATS_ENABLED(((ifHandle_t)pAdpdDataInd->ifInstanceId)->stats.commonStats.rxTotal++);
    IPIF_STATS_ENABLED(((ifHandle_t)pAdpdDataInd->ifInstanceId)->stats.rxOctets += pAdpdDataInd->nsduLength);

    pIpPktInfo = NWKU_CreateIpPktInfo();
    pIpPktInfo->pNwkBuff = NWKU_CreateNwkBuffer(0);

    pIpPktInfo->pNwkBuff->size = pAdpdDataInd->nsduLength;
    pIpPktInfo->pNwkBuff->pData = pAdpdDataInd->pNsdu;
    pIpPktInfo->pNextProt = pAdpdDataInd->pNsdu;
    pIpPktInfo->pIpPktOptions->lqi = pAdpdDataInd->linkQualityIndicator;
    pIpPktInfo->pIpPktOptions->ifHandle = (void*)pAdpdDataInd->ifInstanceId;
    pIpPktInfo->pIpPktOptions->security = pAdpdDataInd->securityLevel;

    IP_Receive(pIpPktInfo,gIpProtv6_c);
}

/*!*************************************************************************************************
\fn  IP_6LowPanDataCnf(adpdDataCnf_t* pParam)

\brief  Handles ADP Data Confirm callback from 6LowPan

\param [in]  pParam    the received data confirm
    
\retval      none
***************************************************************************************************/
void IP_6LowPanDataCnf(adpdDataCnf_t* pAdpdDataCnf)
{   
    if(gAdpStatusSuccess_c != pAdpdDataCnf->status )
    {
        IPIF_STATS_ENABLED(((ifHandle_t)pAdpdDataCnf->ifInstanceId)->stats.commonStats.txErrors++);        
    }
}
/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn uint32_t IP_6LowPanOpen(ifHandle_t* pIfHandle)
\brief  Registers IP with the 6LowPan layer

\param [in]  pIfHandle    interface handle 
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_6LowPanOpen
(
    ifHandle_t* pIfHandle
)
{
    uint32_t  error = gIpGeneralError_c;
    slwpStruct_t* slpHandle = *(slwpStruct_t **)(*pIfHandle)->ifDriverHandle;

    /* add interface name */
    (*pIfHandle)->ifNamePtr = (uint8_t*)mIfName;

    /* open interface if ipVersion6 is 1. Ignore ipVersion4 value */
    if(1 == (*pIfHandle)->ipVersion6)
    {
        /* assign scope id to interface */
        IP_IF_AssignScopeId6(pIfHandle);

        (*pIfHandle)->ifMtu = MTU_6LOWPAN; 
        (*pIfHandle)->ifDevAddrTbl[0].addrSize = gLlayerAddrEui64_c;

        /* add extended address */
        uint64_t tempAddr = slpHandle->pMacAbsReq->GetExtendedAddress(slpHandle->macInstanceId);
        htonall((*pIfHandle)->ifDevAddrTbl[0].eui,tempAddr);

        /* register Media interface callbacks */
        SWLP_RegisterAdpdCallbacks((instanceId_t)(*pIfHandle)->ifDriverHandle,
                                   (adpdCb_t*)&mAdpCallbacks);

        /* register private data from data indication and confirm callback */
        SWLP_RegisterIfInstance((instanceId_t)(*pIfHandle)->ifDriverHandle,(instanceId_t)pIfHandle);

        /* RFC4861 6.3.3: The host joins the all-nodes multicast address on all multicast capable interfaces. */
        //IP_IF_Join(iHandle,(ipAddr_t*)&in6addr_linklocal_allnodes);
         
        /* Link-Local Address Generation/Auto configuration.
         * It comprises of '1111111010' as the first ten bits followed by 54 zeroes 
         * and a 64 bit interface identifier.
         * For all autoconfiguration types, a link-local address is always configured.*/
        error = IP_IF_BindAddr6(pIfHandle, NULL, ip6AddrTypeAutoconfigurable_c,
                                IP6_ADDRESS_LIFETIME_INFINITE, 64U);

        if(error != gIpOk_c)
        {
            IP_6LowPanClose(pIfHandle);
        }
    }
    return error;
}
/*!*************************************************************************************************
\fn uint32_t IP_6LowPanClose(ifHandle_t* pIfHandle)
\brief  Unregisters IP with the 6LowPan layer

\param [in]  pIfHandle  interface handle 
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_6LowPanClose
(
    ifHandle_t* pIfHandle
)
{
#if IP_DISABLE_INTERFACE_FUNCTIONALITY
    uint32_t  error, i;

#if ND_ENABLED    
    /* Relaese ND for this interface. */
    ND_Close(pIfHandle);    
#endif /* ND_ENABLED */
    
    /* Unbind all addresses.  */
    for(i = 0; i < IP_IF_IP6_ADDR_NB; i++)
    {   
        ip6IfAddrData_t* addr = IP_IF_GetAddrByNr6(i);
        if(NULL != addr)
        {
            IP_IF_UnbindAddr6(pIfHandle, &addr->ip6Addr); 
        }
    }
    
    /* Leaves the all-nodes multicast address. */
    IP_IF_Leave(pIfHandle, &in6addr_linklocal_allnodes);
        
    return error;
#else
    return 0;
#endif
}
/*!*************************************************************************************************
\fn uint32_t IP_6LowPanSend(ipPktInfo_t* pIpPktInfo)
\brief  Sends an IPv6 packet.

\param [in]  pIpPktInfo    the packet to send
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_6LowPanSend
(
    ipPktInfo_t* pIpPktInfo
)
{   
#if ND_ENABLED
    ndNeighborEntry_t **ppNd6NeighborEntry = NULL;
#endif
    adpdDataReq_t* pDataReq = MEM_BufferAlloc(sizeof(adpdDataReq_t));
    ifHandle_t ifPtr = *((ifHandle_t*)pIpPktInfo->pIpPktOptions->ifHandle);
    
    IPIF_STATS_ENABLED(ifPtr->stats.commonStats.txTotal++);
    IPIF_STATS_ENABLED(ifPtr->stats.txOctets +=NWKU_NwkBufferTotalSize(pIpPktInfo->pNwkBuff));

    pDataReq->nsduHandle = mNsduHandle;
    mNsduHandle++;
    
    pDataReq->nsduLength = NWKU_NwkBufferTotalSize(pIpPktInfo->pNwkBuff);

    /* If we have a fragmented nwk buffer use NWKU_NwkBufferToRegularBuffer to unify the fragments
     * in a new buffer */
    if(pIpPktInfo->pNwkBuff->next)
    {
        pDataReq->pNsdu = NWKU_NwkBufferToRegularBuffer(pIpPktInfo->pNwkBuff);
    }
    /* else, use the same buffer received as parameter */
    else
    {
        pDataReq->pNsdu = (uint8_t*)pIpPktInfo->pNwkBuff->pData;
    }

    /* initialize with pIpPktInfo->pIpDstAddr */
    pDataReq->pIpDstAddr = pIpPktInfo->pIpDstAddr;

    if(!IP6_IsMulticastAddr(pIpPktInfo->pIpDstAddr))
    {
        if (ifPtr->ip6If.ip6IsAddrOnLink)
        {
            if(FALSE == ifPtr->ip6If.ip6IsAddrOnLink(pIpPktInfo->pIpDstAddr,(uint32_t)pIpPktInfo->pIpPktOptions->ifHandle))
            {
#if ND_ENABLED
                /* Find the Default Router Address */
                ppNd6NeighborEntry = ND_GetDefaultRouter((uint32_t)pIpPktInfo->pIpPktOptions->ifHandle);
                if(NULL != ppNd6NeighborEntry)
                {
                    IP_AddrCopy(pDataReq->pIpDstAddr,&(*ppNd6NeighborEntry)->ipAddr);
                }
                else
#endif /* ND_ENABLED */                    
                {
                    pDataReq->pIpDstAddr = NULL;
                }            
            }
        }
        else
        {
            /* No way to determine if address is on link or not */
            pDataReq->pIpDstAddr = NULL;
        }
    }

    if(NULL != pDataReq->pIpDstAddr)
    {   
        if(ifPtr->ip6If.ip6ResolveUnicastAddr)
        {
            if(FALSE == ifPtr->ip6If.ip6ResolveUnicastAddr(pIpPktInfo))
            {   
                /* Free the ADPD Data Request buffer only if it is allocated in this function */
                if(pIpPktInfo->pNwkBuff->next)
                {
                    MEM_BufferFree(pDataReq->pNsdu);
                }
                MEM_BufferFree(pDataReq);

                ///TODO: Change to have only one exit point
                return 0;
            }
        }
        
        /* prevent the destination address from being freed */
        pIpPktInfo->pIpDstAddr = NULL;

        /* Prevent the network buffer data to be freed if we have only one buffer. This buffer is
                * used as pDataReq->pNsdu */
        if(!pIpPktInfo->pNwkBuff->next)
        {
            pIpPktInfo->pNwkBuff->pData = NULL;
        }
        
        pDataReq->securityLevel = pIpPktInfo->pIpPktOptions->security;
        
        /* free the pIpPktInfo */
        NWKU_FreeIpPktInfo(&pIpPktInfo);

        pDataReq->discoverRoute = FALSE;
        pDataReq->qualityOfService = gAdpQoSNormalPriority_c;
        pDataReq->slwpInstanceId = (instanceId_t)ifPtr->ifDriverHandle;

        SLWP_DataRequest(pDataReq);
    }
    else
    {
        NWKU_FreeIpPktInfo(&pIpPktInfo);
        MEM_BufferFree(pDataReq->pNsdu);
        MEM_BufferFree(pDataReq);
    }

    return 0;
}

/*!*************************************************************************************************
\fn uint32_t IP_6LowPanGetIID(ifHandle_t* pIfHandle)
\brief  Gets the Interface identifier 

\param [in]  pIfHandle  interface handle
\param [in]  macAddr    link layer address
\param [out] ipAddr     IP address to store the IID in
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_6LowPanGetIID
(
    ifHandle_t* pIfHandle,
    llAddr_t* macAddr,
    ipAddr_t* ipAddr
)
{
    slwpStruct_t* slpHandle = *(slwpStruct_t **)((*pIfHandle)->ifDriverHandle);
    adpIb_t * pAdpIb = *(slpHandle->ppAdpIb);
    uint16_t panId = 0;
    
    if(gAdpIIDWithPanId_c == pAdpIb->adpIIDType)
    {
        panId = slpHandle->pMacAbsReq->GetPANId(slpHandle->macInstanceId);
    }
    NWKU_GetIIDFromLLADDR(macAddr,panId,&ipAddr->addr8[8]);
    //SLWP_GetIIDFromLLADDR(macAddr,panId,&ipAddr->addr8[8],(instanceId_t)slpHandle);
    
    return 0;

}
/*!*************************************************************************************************
\fn uint32_t IP_6LowPanJoin(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr,uint16_t protocol)
\brief  Joins an IPv6 multicast group.

\param [in]  pIfHandle  the packet to send
\param [in]  pIpAddr    the multicast group
\param [in]  protocol   the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_6LowPanJoin
(
    ifHandle_t* pIfHandle, 
    ipAddr_t* pIpAddr,
    uint16_t protocol
)
{
  return 0;
}   
/*!*************************************************************************************************
\fn uint32_t IP_6LowPanLeave(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr,uint16_t protocol)
\brief  Leaves an IPv6 multicast group.

\param [in]  pIfHandle  the packet to send
\param [in]  pIpAddr    the multicast group
\param [in]  protocol   the protocol for the multicast group(IPv4 or IPv6)
     
\retval      uint32_t   error
***************************************************************************************************/ 
uint32_t IP_6LowPanLeave
(    
    ifHandle_t* pIfHandle, 
    ipAddr_t* pIpAddr,
    uint16_t protocol
)
{

#if IP_DISABLE_INTERFACE_FUNCTIONALITY
 
#else
    return 0;
#endif
}

#endif
