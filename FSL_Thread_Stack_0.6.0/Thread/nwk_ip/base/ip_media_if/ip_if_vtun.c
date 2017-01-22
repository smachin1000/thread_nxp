/*!=================================================================================================
\file       ip_if_vtun.c
\brief      This is a private source file for the Virtual TUN interface.
            
\copyright  Freescale Confidential Proprietary. No part of this document must be reproduced in any 
\copyright  form - including copied, transcribed, printed or by any electronic means - without 
\copyright  specific written permission from Freescale.
\copyright  (c) Copyright 2013, Freescale, Inc.  All rights reserved.
==================================================================================================*/

#include "ip.h"
#include "ip_if_management.h"
#include "virtual_enet_driver.h"
#include "virtual_tun_driver.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "nd.h"
#include "dhcp_client.h"

#include "FsciInterface.h"
#include "app_fsci_commands.h"


/*==================================================================================================
Private macros
==================================================================================================*/
#define VTUN_MTU (1500U)


/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/
uint32_t IP_vtunOpen(ifHandle_t* pIfHandle);
uint32_t IP_vtunClose(ifHandle_t* pIfHandle);
uint32_t IP_vtunSend6(ipPktInfo_t* pIpPktInfo);
uint32_t IP_vtunOpen6(ifHandle_t* pIfHandle);
uint32_t IP_vtunGetIID(ifHandle_t* pIfHandle,llAddr_t* macAddr,ipAddr_t* ipAddr);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static ifHandle_t* mIpVtunInterfaceHandle;

static const uint8_t* mIfName = "vtun";
static const mediaIfStruct_t gVirtualTunMediaIf =
{
    IP_vtunOpen,
    IP_vtunClose,
    NULL,
    NULL,
#if IP_IP6_ENABLE
    IP_vtunSend6,
    IP_vtunGetIID,
#else
    NULL,
    NULL,
#endif
    NULL,
    NULL
};
const mediaIfStruct_t* gVirtualTunMediaIfPtr = (mediaIfStruct_t*) &gVirtualTunMediaIf;


/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\fn  void IP_vtunRecv(ipPktInfo_t* pIpPktInfo)

\brief  Sends a packet from VTUN interface to IP.

\param [in]  pIpPktInfo    the received packet 
\param [in]  size          the size of the received packet
     
\retval      none
***************************************************************************************************/
void IP_vtunRecv
(
      uint8_t* pInData,
      uint32_t size
)
{
    ipPktInfo_t* pIpPktInfo;

    IPIF_STATS_ENABLED((*mIpVtunInterfaceHandle)->stats.commonStats.rxTotal++);
    IPIF_STATS_ENABLED((*mIpVtunInterfaceHandle)->stats.rxOctets += size);

    pIpPktInfo = NWKU_CreateIpPktInfo();
    pIpPktInfo->pIpPktOptions->ifHandle = mIpVtunInterfaceHandle;

    /* create NWK Buffer and keep a pointer to the allocated buffer */
    pIpPktInfo->pNwkBuff = NWKU_CreateNwkBuffer(0);
    pIpPktInfo->pNwkBuff->pData = pInData;
    pIpPktInfo->pNwkBuff->size = size;
    pIpPktInfo->pNextProt = pInData;
    pIpPktInfo->nextProtLen = size;

    IP_Receive(pIpPktInfo, gIpProtv6_c);
}

/*!*************************************************************************************************
\fn uint32_t IP_vtunOpen6(ifHandle_t* pIfHandle)
\brief  Enable Ipv6 on the provided interface

\param [in]  pIfHandle    interface handle 
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_vtunOpen6
(
    ifHandle_t* pIfHandle
)
{
    uint32_t  error = gIpGeneralError_c;
    
    mIpVtunInterfaceHandle = pIfHandle;

    /* assign scope id to interface */
    IP_IF_AssignScopeId6(pIfHandle);
    
    /* Register IP6 */
    //error = VIRTUAL_ENET_open(gIpProtv6_c,IP_vtunRecv);

    //if (error == gIpOk_c)
    {
        /* RFC4861 6.3.3: The host joins the all-nodes multicast address on all 
           multicast capable interfaces. */
        //IP_IF_Join(pIfHandle,(ipAddr_t*)&in6addr_linklocal_allnodes);

        /* Link-Local Address Generation/Auto configuration.
         * It comprises of '1111111010' as the first ten bits followed by 54 zeroes 
         * and a 64 bit interface identifier.
         * For all autoconfiguration types, a link-local address is always configured.*/
        error = IP_IF_BindAddr6(pIfHandle,NULL,ip6AddrTypeAutoconfigurable_c,
                                IP6_ADDRESS_LIFETIME_INFINITE,64U);
    }    
    return error;
}

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn uint32_t IP_vtunOpen(ifHandle_t* pIfHandle)
\brief  Registers IP and ARP with an Virtual Ethernet packet driver

\param [in]  pIfHandle    interface handle 
     
\retval      uint32_t     error
***************************************************************************************************/
uint32_t IP_vtunOpen
(
    ifHandle_t* pIfHandle
)
{
    uint32_t  error1 = gIpOk_c;
    uint32_t  error2 = gIpOk_c;

    (*pIfHandle)->ifNamePtr = (uint8_t*)mIfName;
    (*pIfHandle)->ifMtu= VTUN_MTU;
    (*pIfHandle)->ifDevAddrTbl[0].addrSize = gLlayerAddrEui48_c;
    
    VIRTUAL_TUN_get_address((*pIfHandle)->ifDevAddrTbl[0].eui);

    /* Keep ethernet handle for later use(i.e. IP_vtunRecv function) */
    mIpVtunInterfaceHandle = pIfHandle;

   
    if((*pIfHandle)->ipVersion6)
    {
        error2 = IP_vtunOpen6(pIfHandle);
    }

    
    if((error1 != gIpOk_c) || (error2 != gIpOk_c))
    {
        IP_vtunClose(pIfHandle);
    }
    return error1 + error2;
}
/*!*************************************************************************************************
\fn uint32_t IP_vtunClose(ifHandle_t* pIfHandle)
\brief  Unregisters IP and ARP with an Virtual Ethernet packet driver

\param [in]  pIfHandle    interface handle 
     
\retval      uint32_t     error
***************************************************************************************************/
uint32_t IP_vtunClose
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
    //IP_IF_Leave(pIfHandle, &in6addr_linklocal_allnodes);
      
    /* Deregister IP6. */
    //error = VIRTUAL_ENET_close();
     
    return error;
#else
    return 0;
#endif
}

/*!*************************************************************************************************
\fn uint32_t IP_vtunSend6(ipPktInfo_t* pIpPktInfo)
\brief  Sends an IPv6 packet via FSCI interface.

\param [in]  pIpPktInfo    the packet to send
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_vtunSend6
(
    ipPktInfo_t* pIpPktInfo
)
{
    ifHandle_t ifHandle = *((ifHandle_t*)pIpPktInfo->pIpPktOptions->ifHandle);
    uint32_t bufferSize;
    uint8_t *pBuff;
    nwkBuffer_t *pNwkBuff;
    uint32_t currentPos;

    (void)ifHandle;
    
    IPIF_STATS_ENABLED(ifHandle->stats.commonStats.txTotal++);
    IPIF_STATS_ENABLED(ifHandle->stats.txOctets += NWKU_NwkBufferTotalSize(pIpPktInfo->pNwkBuff));

    bufferSize = NWKU_NwkBufferTotalSize(pIpPktInfo->pNwkBuff);
    pBuff = MEM_BufferAlloc(bufferSize);
    pNwkBuff = pIpPktInfo->pNwkBuff;
    currentPos = 0;
    while(pNwkBuff)
    {
        FLib_MemCpy(pBuff + currentPos, pNwkBuff->pData, pNwkBuff->size);

        currentPos += pNwkBuff->size;
        pNwkBuff = pNwkBuff->next;
    }

    FSCI_transmitPayload(gFSCI_FlipOpGCnf_c, gFSCI_FlipVtunReceive_c, pBuff, bufferSize,
        FSCI_FLIP_INTERFACE);

    MEM_BufferFree(pBuff);
    NWKU_FreeIpPktInfo(&pIpPktInfo);

    return 0;
}

/*!*************************************************************************************************
\fn uint32_t IP_vtunGetIID(ifHandle_t* pIfHandle)
\brief  Gets the Interface identifier 

\param [in]  pIfHandle  interface handle
\param [in]  macAddr    link layer address
\param [out] ipAddr     IP address to store the IID in
     
\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_vtunGetIID
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

