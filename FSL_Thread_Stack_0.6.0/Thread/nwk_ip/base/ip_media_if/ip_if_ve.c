/*!=================================================================================================
\file       ip_if_ve.c
\brief      This is a private source file for the Virtual enet interface.
            
\copyright  Freescale Confidential Proprietary. No part of this document must be reproduced in any 
\copyright  form - including copied, transcribed, printed or by any electronic means - without 
\copyright  specific written permission from Freescale.
\copyright  (c) Copyright 2013, Freescale, Inc.  All rights reserved.
==================================================================================================*/

#include "ip.h"
#include "arp.h"
#include "ip_if_management.h"
#include "virtual_enet_driver.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "nd.h"
#include "dhcp_client.h"

#if IP_IP4_ENABLE || (IP_IP6_ENABLE && ND_ENABLED)

/*==================================================================================================
Private macros
==================================================================================================*/

#define IP4_BROADCAST_TO_ENET_ADDR(eth_addr) \
        (eth_addr)[0] = 0xFF, \
        (eth_addr)[1] = 0xFF, \
        (eth_addr)[2] = 0xFF, \
        (eth_addr)[3] = 0xFF, \
        (eth_addr)[4] = 0xFF, \
        (eth_addr)[5] = 0xFF

#define IP4_MULTICAST_TO_ENET_ADDR(ip,eth_addr)  \
        (eth_addr)[0] = 0x01, \
        (eth_addr)[1] = 0x00, \
        (eth_addr)[2] = 0x5E, \
        (eth_addr)[3] = ((ip) >> 16) & 0x7F, \
        (eth_addr)[4] = ((ip) >>  8) & 0xFF, \
        (eth_addr)[5] =  (ip)        & 0xFF

#define IP6_MULTICAST_TO_ENET_ADDR(ip6,eth_addr)  \
        (eth_addr)[0] = 0x33,             \
        (eth_addr)[1] = 0x33,             \
        (eth_addr)[2] = ip6->addr8[12],   \
        (eth_addr)[3] = ip6->addr8[13],   \
        (eth_addr)[4] = ip6->addr8[14],   \
        (eth_addr)[5] = ip6->addr8[15]

#define ENET_HEADER_SIZE  14U

#define ENET_TYPE_UNICAST                    0x01
#define ENET_TYPE_BROADCAST                  0x02
#define ENET_TYPE_MULTICAST                  0x04

/*==================================================================================================
Private type definitions
==================================================================================================*/

typedef struct enetHdr_tag
{
   uint8_t    dest[6];
   uint8_t    source[6];
   uint8_t    type[2];
}enetHdr_t; 

/*==================================================================================================
Private prototypes
==================================================================================================*/


uint32_t IP_veOpen(ifHandle_t* pIfHandle);
uint32_t IP_veClose(ifHandle_t* pIfHandle);
uint32_t IP_veSendCommon(ipPktInfo_t* pIpPktInfo, llAddr_t* pLlAddr,uint16_t protocol);
void IP_veRecv (uint8_t* pInData, uint32_t size);
uint32_t IP_veJoin(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr,uint16_t protocol);
uint32_t IP_veLeave(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr,uint16_t protocol);

#if IP_IP4_ENABLE
uint32_t IP_veOpen4(ifHandle_t* pIfHandle);
uint32_t IP_veSend4(ipPktInfo_t* pIpPktInfo);
uint32_t IP_veSendArp(ipPktInfo_t* pIpPktInfo, llAddr_t* pLlAddr);
#endif

#if IP_IP6_ENABLE
uint32_t IP_veSend6(ipPktInfo_t* pIpPktInfo);
uint32_t IP_veOpen6(ifHandle_t* pIfHandle);
uint32_t IP_veGetIID(ifHandle_t* pIfHandle,llAddr_t* macAddr,ipAddr_t* ipAddr);
#endif

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

static const uint8_t* mIfName = "eth";

static const mediaIfStruct_t gVirtualEnetMediaIf =
{
    IP_veOpen,
    IP_veClose,
#if IP_IP4_ENABLE   
    IP_veSend4, 
    IP_veSendArp,
#else
    NULL,
    NULL,
#endif
#if IP_IP6_ENABLE   
    IP_veSend6,
    IP_veGetIID,
#else
    NULL,
    NULL,
#endif
    IP_veJoin,
    IP_veLeave
};

const mediaIfStruct_t* gVirtualEnetMediaIfPtr = (mediaIfStruct_t*) &gVirtualEnetMediaIf;

static ifHandle_t* mIpVeInterfaceHandle;
/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\fn  void IP_veRecv(ipPktInfo_t* pIpPktInfo)

\brief  Services an Virtual Ethernet packet

\param [in]  pIpPktInfo    the received packet 
\param [in]  protocol      the received protocol 
     
\retval      none
***************************************************************************************************/
void IP_veRecv
(
      uint8_t* pInData,
      uint32_t size
)
{ 

    ipPktInfo_t* pIpPktInfo;
    enetHdr_t* enetpkt = (enetHdr_t*)(pInData + sizeof(i2cHdr_t));
    uint16_t protocol = ntohas(enetpkt->type);

    IPIF_STATS_ENABLED((*mIpVeInterfaceHandle)->stats.commonStats.rxTotal++);
    IPIF_STATS_ENABLED((*mIpVeInterfaceHandle)->stats.rxOctets += size);

    pIpPktInfo = NWKU_CreateIpPktInfo();
    pIpPktInfo->pIpPktOptions->ifHandle = mIpVeInterfaceHandle;

    /* create NWK Buffer but do not allocate memory for it */
    pIpPktInfo->pNwkBuff = NWKU_CreateNwkBuffer(0);
    
    pIpPktInfo->pNwkBuff->pData = pInData;
    pIpPktInfo->pNwkBuff->size = size;
    pIpPktInfo->pNextProt = pInData + ENET_HEADER_SIZE + sizeof(i2cHdr_t);
    
#if IP_DISCARD_SELF_BCASTS
    
    uint8_t enetType;

    if (enetpkt->dest[0] & 1) 
        {
        if ((enetpkt->dest[0] == 0xFF)
        && (enetpkt->dest[1] == 0xFF)
        && (enetpkt->dest[2] == 0xFF)
        && (enetpkt->dest[3] == 0xFF)
        && (enetpkt->dest[4] == 0xFF)
        && (enetpkt->dest[5] == 0xFF)) 
        {
            IPIF_STATS_ENABLED((*mIpVeInterfaceHandle)->stats.rxBroadcast++);
            enetType = ENET_TYPE_BROADCAST;
        } 
        else
        {
            IPIF_STATS_ENABLED((*mIpVeInterfaceHandle)->stats.rxMulticast++);
            enetType = ENET_TYPE_MULTICAST;
        } 
    } 
    else
    {
        IPIF_STATS_ENABLED((*mIpVeInterfaceHandle)->stats.rxUnicast++);
        enetType = ENET_TYPE_UNICAST;
    }

    /* discard any broadcast pkts that we sent, as they are likely echoes */ 
    if((enetType == ENET_TYPE_BROADCAST) && 
       (FLib_MemCmp(enetpkt->source,(*mIpVeInterfaceHandle)->ifDevAddrTbl[0].eui, 
        mIpVeInterfaceHandle->ifDevAddrTbl[0].addrSize) == 0)) 
    {
        IPIF_STATS_ENABLED((*mIpVeInterfaceHandle)->stats.commonStats.rxDiscarded++);
        NWKU_FreeIpPktInfo(&pIpPktInfo);
        return;
    }    
#endif

    IP_Receive(pIpPktInfo,protocol);
}
#if IP_IP6_ENABLE
/*!*************************************************************************************************
\fn uint32_t IP_veOpen6(ifHandle_t* pIfHandle)
\brief  Enable Ipv6 on the provided interface

\param [in]  pIfHandle    interface handle 
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_veOpen6
(
    ifHandle_t* pIfHandle
)
{
    uint32_t  error = gIpGeneralError_c;
    
    /* assign scope id to interface */
    IP_IF_AssignScopeId6(pIfHandle);   
    
    /* Register IP6 */
    error = VIRTUAL_ENET_open(gIpProtv6_c,IP_veRecv);

    if (error == gIpOk_c) 
    {
        /* RFC4861 6.3.3: The host joins the all-nodes multicast address on all 
           multicastcapable interfaces. */
        IP_IF_Join(pIfHandle,(ipAddr_t*)&in6addr_linklocal_allnodes);

        /* Link-Local Address Generation/Auto configuration.
         * It comprises of '1111111010' as the first ten bits followed by 54 zeroes 
         * and a 64 bit interface identifier.
         * For all autoconfiguration types, a link-local address is always configured.*/
        error = IP_IF_BindAddr6(pIfHandle,NULL,ip6AddrTypeAutoconfigurable_c,
                                IP6_ADDRESS_LIFETIME_INFINITE,64U);
    }    
    return error;
}
#endif
#if IP_IP4_ENABLE
/*!*************************************************************************************************
\fn uint32_t IP_veOpen4(ifHandle_t* pIfHandle)
\brief  Enable Ipv4 on the provided interface

\param [in]  pIfHandle    interface handle 
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_veOpen4
(
    ifHandle_t* pIfHandle
)
{
    uint32_t  error = gIpGeneralError_c;
    
    /* Register IP4 */
    error = VIRTUAL_ENET_open(gIpProtv4_c,IP_veRecv);

    if(error == gIpOk_c)
    {
        error = VIRTUAL_ENET_open(gIpProtArp_c,IP_veRecv);    

        if(error == gIpOk_c)
        {
            error = ARP_Open(pIfHandle);                
        }        
    }
    return error;
}
#endif
/*!*************************************************************************************************
\fn uint32_t IP_veSend4(ipPktInfo_t* pIpPktInfo,llAddr_t* pLlAddr,uin16_t protocol)
\brief  Sends an IP packet.

\param [in]  pIpPktInfo    the packet to send
\param [in]  pLlAddr       the link layer address to send the packet
\param [in]  protocol      the protocol to send
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_veSendCommon
(
    ipPktInfo_t* pIpPktInfo,
    llAddr_t* pLlAddr,
    uint16_t protocol
)
{   
    uint32_t error;
    IPIF_STATS_ENABLED(ifHandle_t ifHandle = *((ifHandle_t*)pIpPktInfo->pIpPktOptions->ifHandle));

    IPIF_STATS_ENABLED(ifHandle->stats.commonStats.txTotal++);
    IPIF_STATS_ENABLED(ifHandle->stats.txOctets +=NWKU_NwkBufferTotalSize(pIpPktInfo->pNwkBuff));

    if(pLlAddr)
    {
        error = VIRTUAL_ENET_send(pIpPktInfo,protocol, pLlAddr->eui, 0);
    }
    else
    {   
        IPIF_STATS_ENABLED(ifHandle->stats.commonStats.txDiscarded++);
        error = gIpGeneralError_c;
    }
    
    if(error)
    {
        IPIF_STATS_ENABLED(ifHandle->stats.commonStats.txErrors++);
    }

    return error;

}
/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn uint32_t IP_veOpen(ifHandle_t* pIfHandle)
\brief  Registers IP and ARP with an Virtual Ethernet packet driver

\param [in]  pIfHandle    interface handle 
     
\retval      uint32_t     error
***************************************************************************************************/
uint32_t IP_veOpen
(
    ifHandle_t* pIfHandle
)
{
    uint32_t  error1 = gIpOk_c;
    uint32_t  error2 = gIpOk_c;
    
    (*pIfHandle)->ifNamePtr = (uint8_t*)mIfName;
    (*pIfHandle)->ifMtu= VIRTUAL_ENET_get_MTU();
    (*pIfHandle)->ifDevAddrTbl[0].addrSize = gLlayerAddrEui48_c;
    
    VIRTUAL_ENET_get_address((*pIfHandle)->ifDevAddrTbl[0].eui);

    /* Keep ethernet handle for later use(i.e. IP_veRecv function) */
    mIpVeInterfaceHandle = pIfHandle;

#if IP_IP4_ENABLE 
    if((*pIfHandle)->ipVersion4)
    {
        error1 = IP_veOpen4(pIfHandle);
    }
#endif

#if IP_IP6_ENABLE    
    if((*pIfHandle)->ipVersion6)
    {
        error2 = IP_veOpen6(pIfHandle);
    }        
#endif
    
    if((error1 != gIpOk_c) || (error2 != gIpOk_c))
    {
        IP_veClose(pIfHandle);
    }
    return error1 + error2;
}
/*!*************************************************************************************************
\fn uint32_t IP_veClose(ifHandle_t* pIfHandle)
\brief  Unregisters IP and ARP with an Virtual Ethernet packet driver

\param [in]  pIfHandle    interface handle 
     
\retval      uint32_t     error
***************************************************************************************************/
uint32_t IP_veClose
(
    ifHandle_t* pIfHandle
)
{
#if IP_DISABLE_INTERFACE_FUNCTIONALITY
    uint32_t  error, i;

    /* Relaese ND for this interface. */
    ND_Close(pIfHandle);    
    
    /* Unbind all addresses bound to this interface */
    for(i = 0; i < IP_IF_IP6_ADDR_NB; i++)
    {   
        ip6IfAddrData_t* addr = IP_IF_GetAddrByNr6(i);
        if(NULL != addr)
        {   
            if(addr->ifPtr == pIfHandle)
            {
                IP_IF_UnbindAddr6(pIfHandle, &addr->ip6Addr); 
            }
        }
    }
    
    /* Leaves the all-nodes multicast address. */
    IP_IF_Leave(pIfHandle, &in6addr_linklocal_allnodes);
      
    /* Deregister IP6. */
    error = VIRTUAL_ENET_close();
     
    return error;
#else
    return 0;
#endif
}
#if IP_IP4_ENABLE
/*!*************************************************************************************************
\fn uint32_t IP_veSendArp(ipPktInfo_t* pIpPktInfo, llAddr_t* pLlAddr)
\brief  Sends an ARP packet

\param [in]  pInData    the APR packet to send
\param [in]  pLlAddr    pointer to link layer address to send data
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_veSendArp
(
    ipPktInfo_t* pIpPktInfo,
    llAddr_t* pLlAddr
)
{

    return IP_veSendCommon(pIpPktInfo,pLlAddr,gIpProtArp_c);

}
/*!*************************************************************************************************
\fn uint32_t IP_veSend4(ipPktInfo_t* pIpPktInfo)
\brief  Sends an IPv4 packet.

\param [in]  pIpPktInfo    the packet to send
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_veSend4
(
    ipPktInfo_t* pIpPktInfo
)
{
    uint32_t error = gIpOk_c;
    llAddr_t enetDst;
    enetDst.addrSize = gLlayerAddrEui48_c;
    
    llAddr_t* pEnetDstInf = &enetDst;
    IPIF_STATS_ENABLED(ifHandle_t ifHandle = *((ifHandle_t*)pIpPktInfo->pIpPktOptions->ifHandle));

    uint32_t ipDst = IP4_AddrToUint32(pIpPktInfo->pIpDstAddr);
  
    if (ipDst == IP4_ADDR_BROADCAST) 
    {
        IPIF_STATS_ENABLED(ifHandle->stats.txBroadcast++);
        IP4_BROADCAST_TO_ENET_ADDR(enetDst.eui);
    }
    else 
    {
        if (IP4_MULTICAST(ipDst)) 
        {
            IPIF_STATS_ENABLED(ifHandle->stats.txMulticast++);
            IP4_MULTICAST_TO_ENET_ADDR(ipDst,enetDst.eui);
        } 
        else 
        {
            /* resolve unicast address */
            IPIF_STATS_ENABLED(ifHandle->stats.txUnicast++);
            pEnetDstInf = ARP_Resolve(pIpPktInfo);
        }
    } 
    error = IP_veSendCommon(pIpPktInfo,pEnetDstInf,gIpProtv4_c);
    
    return error;
}
#endif
#if IP_IP6_ENABLE
/*!*************************************************************************************************
\fn uint32_t IP_veSend6(ipPktInfo_t* pIpPktInfo)
\brief  Sends an IPv6 packet.

\param [in]  pIpPktInfo    the packet to send
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_veSend6
(
    ipPktInfo_t* pIpPktInfo
)
{   
    uint32_t error = gIpGeneralError_c;
    llAddr_t enetDst;
    llAddr_t* pEnetDstInf = NULL;
    ifHandle_t ifHandle = *((ifHandle_t*)pIpPktInfo->pIpPktOptions->ifHandle);
    ndCfg_t** ppNd6Cfg = ND_GetCfg(ifHandle);

    if(IP6_IsMulticastAddr(pIpPktInfo->pIpDstAddr))
    {
        IP6_MULTICAST_TO_ENET_ADDR(pIpPktInfo->pIpDstAddr, enetDst.eui);
        pEnetDstInf = &enetDst;
        //pEnetDstInf->addrSize = gLlayerAddrEui48_c;
        
        IPIF_STATS_ENABLED(ifHandle->stats.txMulticast++);        
    }
    else
    {
        if((*ppNd6Cfg)->ndIf->ndResolveDstLinkLayerAddr)
        {
            pEnetDstInf = (*ppNd6Cfg)->ndIf->ndResolveDstLinkLayerAddr(pIpPktInfo);
            IPIF_STATS_ENABLED(ifHandle->stats.txUnicast++);
        }
        else
        {
            IPIF_STATS_ENABLED(ifHandle->stats.commonStats.txDiscarded++);
            NWKU_FreeIpPktInfo(&pIpPktInfo);
            return error;
        }
    }
    error = IP_veSendCommon(pIpPktInfo,pEnetDstInf,gIpProtv6_c);
    
    return error;
}

/*!*************************************************************************************************
\fn uint32_t IP_veGetIID(ifHandle_t* pIfHandle)
\brief  Gets the Interface identifier 

\param [in]  pIfHandle  interface handle
\param [in]  macAddr    link layer address
\param [out] ipAddr     IP address to store the IID in
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_veGetIID
(
    ifHandle_t* pIfHandle,
    llAddr_t* macAddr,
    ipAddr_t* ipAddr
)
{
    if(6U == macAddr->addrSize) /* IEEE 802.11 48-bit addresses. MAC address for Ethernet */ 
    {
        /* [RFC2464] The OUI of the Ethernet address (the first three octets) becomes the
           company_id of the EUI-64 (the first three octets).  The fourth and
           fifth octets of the EUI are set to the fixed value FFFE hexadecimal.
           The last three octets of the Ethernet address become the last three
           octets of the EUI-64. */
        FLib_MemCpy(&(ipAddr->addr8[8]), macAddr->eui, 3U);
        ipAddr->addr8[11] = 0xff;
        ipAddr->addr8[12] = 0xfe;
        FLib_MemCpy(&(ipAddr->addr8[13]),&(macAddr->eui[3]),  3U);

    }
    else /* macAddr->devAddrlen == 8U */
    {
        /* IEEE EUI-64 identifier. IEEE 802.15.4 */    

        /* In this case, the Interface Identifier is formed
         * from the EUI-64 according to the "IPv6 over Ethernet" specification
         * [RFC2464]. */
        FLib_MemCpy(&(ipAddr->addr8[8]),macAddr->eui, 8U);
    } 
    /* The Interface Identifier is then formed from the EUI-64 by
       complementing the "Universal/Local" (U/L) bit, which is the next-to-
       lowest order bit of the first octet of the EUI-64. */            
    ipAddr->addr8[8] ^= 0x02;

    return 0;

}
#endif
/*!*************************************************************************************************
\fn uint32_t IP_veJoin6(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr,uint16_t protocol)
\brief  Joins an IP multicast group(IPv4 or IPv6 depending on the protocol parameter).

\param [in]  pIfHandle  the packet to send
\param [in]  pIpAddr    the multicast group
\param [in]  protocol   the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_veJoin
(
    ifHandle_t* pIfHandle, 
    ipAddr_t* pIpAddr,
    uint16_t protocol
)
{
    uint8_t enetGroupAddr[6];

    if(gIpProtv4_c == protocol)
    {
        uint32_t ipAddr = IP4_AddrToUint32(pIpAddr);
        IP4_MULTICAST_TO_ENET_ADDR(ipAddr,enetGroupAddr);
    }
    else
    {
        IP6_MULTICAST_TO_ENET_ADDR(pIpAddr, enetGroupAddr);
    }
    return VIRTUAL_ENET_join(enetGroupAddr,protocol);
}   
/*!*************************************************************************************************
\fn uint32_t IP_veLeave6(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr,uint16_t protocol)
\brief  Leaves an IP multicast group(IPv4 or IPv6 depending on the protocol parameter).

\param [in]  pIfHandle  the packet to send
\param [in]  pIpAddr    the multicast group
\param [in]  protocol   the protocol for the multicast group(IPv4 or IPv6)
     
\retval      uint32_t   error
***************************************************************************************************/ 
uint32_t IP_veLeave
(    
    ifHandle_t* pIfHandle, 
    ipAddr_t* pIpAddr,
    uint16_t protocol
)
{

#if IP_DISABLE_INTERFACE_FUNCTIONALITY
    uint8_t enetGroupAddr[6];

    if(gIpProtv4_c == protocol)
    {   
        uint32_t ipAddr = IP4_AddrToUint32(pIpAddr);
        IP4_MULTICAST_TO_ENET_ADDR(ipAddr,enetGroupAddr);
    }
    else
    {
        IP6_MULTICAST_TO_ENET_ADDR(pIpAddr, enetGroupAddr);
    }
    return VIRTUAL_ENET_leave(enetGroupAddr,protocol);
#else
    return 0;
#endif
}

#endif
