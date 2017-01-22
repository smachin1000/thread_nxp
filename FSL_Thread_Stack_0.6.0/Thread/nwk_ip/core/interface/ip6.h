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

#ifndef _IP6_H_
#define _IP6_H_

/*!=================================================================================================
\file       ip6.h
\brief      This is a header file for the IPv6 layer.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "network_utils.h"
#include "ip_cfg.h"

#if IP_IP6_ENABLE
/*==================================================================================================
Public macros
==================================================================================================*/

/*! IPv6 Header size. */
#define IP6_HDR_LEN (40U)

#define IP6_MINIMUM_MTU         1280U

/*! Default Hop Limit. */
#define IP6_DEFAULT_HOPLIMIT    64

#define IP6_HEADER_VERSION      0x60
#define IP6_HEADER_VERSION_MASK 0xf0

#define IP6_PAD1_OPTION         (0x00)
#define IP6_PADN_OPTION         (0x01)
#define IP6_MPL_OPTION          (0x6D)

#define IP6_OPTION_TYPE_UNRECOGNIZED_MASK           (0xC0)
#define IP6_SKIP_UNRECOGNIZED_OPTION_TYPE           (0x00)
#define IP6_DISCARD_UNRECOGNIZED_OPTION_TYPE        (0x40)
#define IP6_DISCARD_ICMP_UNRECOGNIZED_OPTION_TYPE   (0x80)
#define IP6_DISCARD_UICMP_UNRECOGNIZED_OPTION_TYPE  (0xC0)

#define IP6_FRAGMENT_MF             0x1     /* If 1, this is not last fragment */
#define IP6_FRAGMENT_OFFSET_SHIFT   3U

#define ADDR_TYPE_MASK      0xF0U
#define ADDR_TYPE_OFFSET    4U
#define ADDR_STATE_MASK     0x0FU
#define ADDR_STATE_OFFSET   0U

#define ADDR_TYPE_SET(a, b)  ((a) = (((a) & (~ADDR_TYPE_MASK)) | ((b) << ADDR_TYPE_OFFSET)))
#define ADDR_TYPE_GET(a)     (((a) & ADDR_TYPE_MASK) >> ADDR_TYPE_OFFSET)

#define ADDR_STATE_SET(a, b)  ((a) = (((a) & (~ADDR_STATE_MASK)) | ((b) << ADDR_STATE_OFFSET )))
#define ADDR_STATE_GET(a)     (((a) & ADDR_STATE_MASK) >> ADDR_STATE_OFFSET)



/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef enum
{
    /* Not used.*/
    ip6AddrStateNotUsed_c = 0x00,
    ip6AddrStateTentative_c,
    ip6AddrStatePreferred_c
} ip6AddrState_t;

typedef enum
{
    /*! Manual Address */
    ip6AddrTypeManual_c = 0xFF,

    /*! Autoconfigurable address  -Default MAC */
    ip6AddrTypeAutoconfigurable_c = 0x00,

    /*! Autoconfigurable address  -second MAC address */
    ip6AddrTypeAutoconfigurableMac2_c = 0x01,
} ip6AddrType_t;

typedef enum
{
    ip6AddrScopeInterfaceLocal_c = 0x01,
    ip6AddrScopeLinkLocal_c = 0x02,
    ip6AddrScopeSiteLocal_c = 0x05,
    ip6AddrScopeOrgLocal_c = 0x08,
    ip6AddrScopeGlobal_c = 0x0e,
} ip6AddrScope_t;


typedef enum
{
    ipExtHdrHopByHopOpt_c = 0,
    ip6Header_c = 41,
    ipExtHdrRoutingHeader_c = 43,
    ipExtHdrFragmentHeader_c = 44,
    ipExtHdrAuthenticationHeader_c = 51,
    ipExtHdrNoNextHeader_c = 59,
    ipExtHdrDestOptions_c = 60,
    ipExtHdrMobilityHeader_c = 135,
} ip6ExtHdrType_t;

typedef struct ip6IfAddrData_tag
{
    /*! IPv6 address.*/
    ipAddr_t      ip6Addr;
    /*! pointer to interface this IP6 address is binded to*/
    void*         ifPtr;
    /*! Time of entry creation (in seconds).*/
    uint32_t      creationTime;
    /* Address lifetime expire timestamp (in seconds). 0xFFFFFFFF= Infinite Lifetime */
    uint32_t      lifetime;
    /* Address type (4 bits) and current state (4 bits) .*/
    uint8_t        ip6AddrTypeAndState;
    /* Counter used by DAD. Equals to the number of NS transmits till DAD is finished .*/
    uint8_t        dadTransmitCounter;
    /*! The number of leading bits in the Prefix that are valid. -Not used, maybe used for routing */
    uint8_t        prefixLength;
    /* index in the interface mac address table of the mac address this IP6 address is assigned to*/
    uint8_t        macAddrIndex;
}ip6IfAddrData_t;


typedef struct ip6MulticastAddrData_tag
{
    /*! IPv6 address.*/
    ipAddr_t      mCastAddrAddr;
    /*! pointer to interface this IP6 address is binded to*/
    void*         ifPtr;
}ip6MulticastAddrData_t;


typedef struct ip6IfStruct_tag
{
    uint32_t   scope_id;
    void**     ppNdCfg;
    bool_t     (*ip6IsAddrOnLink)(ipAddr_t *pIpDestAddr, uint32_t instanceId);
    bool_t     (*ip6ResolveUnicastAddr)(ipPktInfo_t* pIpDestAddr);
    uint32_t   (*ip6McastForward)(ipPktInfo_t *,uint8_t);
    ipAddr_t*  (*ip6UnicastForward)(ipPktInfo_t *,uint8_t);
}ip6IfStruct_t;


/*  The IPv6 Hop-by-Hop Options header is used to carry MPL information and has the
    following format:

    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |  Next Header  |  Hdr Ext Len  |                               |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+                               +
    |                                                               |
    .                                                               .
    .                            Options                            .
    .                                                               .
    |                                                               |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
*/
typedef struct ip6HopByHopOptHdr_tag
{
   /*! 8-bit selector. Identifies the type of header immediately following the Options header. */
   uint8_t  nextHeader;
   /*! 8-bit unsigned integer. Length of the Hop-by-Hop Options header in 8-octet units, not
      including the first 8 octets. */
   uint8_t  hdrExtLenght;
}ip6HopByHopOptHdr_t;

typedef struct ip6OptionHdr_tag
{
   /*! 8-bit identifier of the type of option. */
   uint8_t type;
   /*! 8-bit unsigned integer. Length of the Option Data field of this option, in octets. */
   uint8_t dataLenght;
}ip6OptionHdr_t;


typedef struct ip6RoutingHdr_tag
{
    /*! Identifies the type of header immediately following the Options header. */
    uint8_t    nextHeader;
    /*! Length in 8-octet units, not including the first 8 octets. */
    uint8_t    hdrExtLenght;
    /*! 8-bit identifier of a particular Routing header variant.*/
    uint8_t    routingType;
    /*! Number of route segments remaining. */
    uint8_t    segmentsLeft;
    /*! The complete Routing header is an integer multiple of 8 octets long. */
    uint8_t    data[4];
}ip6RoutingHdr_t;

typedef struct ip6FragmentHdr_tag
{
   /*! Identifies the initial header type of the Fragmentable Part of the original packet. */
   uint8_t    nextHeader;
   /*! Initialized to zero for transmission; ignored on reception. */
   uint8_t    reserved;
   /*! @ 13-bit The offset, in 8-octet units, of the data following this header, relative to the
        start of the Fragmentable Part of the original packet.
       @ 2-bit reserved field.  Initialized to zero for transmission; ignored on reception.
       @ 1 = more fragments; 0 = last fragment.*/
   uint8_t    fragOffset[2];
   /*! Identification. */
   uint8_t    id[4];
} ip6FragmentHdr_t;


typedef struct ip6RoutingTblEntry_tag
{
    /*! destination IPv6 address or prefix of network */
    ipAddr_t dstPrefix;
    /*! pointer in the neigbour cache of the next hop router to destination */
    ipAddr_t nextHopRouter;
    /*! pointer in the neigbour cache of the next hop router to destination */
    void*  nextHopIf;
    /*! length of IPv6 prefix */
    uint16_t prefixLen;
    /*! routing protocol type */
    uint8_t routeProtType;
    /*! flag that signales if a route is valid or is in a deletion phase */
    uint8_t validRoute;

}ip6RoutingTblEntry_t;


/*==================================================================================================
Public global variables declarations
==================================================================================================*/



/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

#if IP_IP6_ENABLE_REASSEMBLY
/*!*************************************************************************************************
\fn void IP6_ReassemExpireEventHandle(ipPktInfo_t* dgram)
\brief  Called by the Timer. Expire an IP datagram and frees al the occupied memory.

\param [in]  dgram    ipPktInfo_t type datagram that holds the fragments received so far

\retval      none
***************************************************************************************************/
void IP6_ReassemExpireEventHandle(ipPktInfo_t* dgram);
#endif

#if IP_IP6_STATS_ENABLE
/*!*************************************************************************************************
\fn ip6Stats_t* IP6_GetIPStats(void)
\brief Returns structure of IPv6 statistics

\retval      ip6Stats_t*      pointer to IPv6 statistics structure
***************************************************************************************************/
ip6Stats_t* IP6_GetIPStats(void);
{
    return &mIp6Statistics;
}
#endif
/*!*************************************************************************************************
\fn uint32_t IP6_Send(ipPktInfo_t* pIpPktInfo)
\brief  Sends an IP6 packet generated on the local host

\param [in]  pIpPktInfo    the packet to send

\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP6_Send(ipPktInfo_t* pIpPktInfo);

/*!*************************************************************************************************
\fn void IP6_Service(void* pInData)
\brief  Parses a received IP6 packet

\param [in]  pInData    the packet received

\retval      none
***************************************************************************************************/
void IP6_Service(void* pInData);

#if IP_IP6_ROUTING_ENABLE
/*!*************************************************************************************************
\fn ipAddr_t* IP6_Forward(ipPktInfo_t* pIpPktInfo,uint8_t direction)
\brief  Forwards an IP6 packet to the destination interface

\param [in]  pIpPktInfo    the packet to send
\param [in]  direction     TX or RX

\retval      ipAddr_t*     pointer to next hop address or NULL
***************************************************************************************************/
ipAddr_t* IP6_Forward(ipPktInfo_t* pIpPktInfo, uint8_t direction);

/*!*************************************************************************************************
\fn ip6RoutingTblEntry_t** IP6_GetRoutingTblPtr(void)
\brief  Get pointer to IP6 Routing Table(Routing table only holds pointer to each routing table
        entry which is allocated for dynamic memory or kept in NVM)

\retval      ip6RoutingTblEntry_t*      pointer to start of routing table
***************************************************************************************************/
ip6RoutingTblEntry_t** IP6_GetRoutingTblPtr(void);

/*!*************************************************************************************************
\fn void IP6_SetStaticRoute(ipAddr_t* setAddr, uint8_t prefixLen, ipAddr_t* nextHopAddr,
         void* ifPtr)
\brief Sets a static route in the IP6 routing table.

\param [in]  setAddr       the address to set in the routing table
\param [in]  prefixLen     prefix length of the address to set in the routing table
\param [in]  nextHopAddr   the address of the next hop router to destination
\param [in]  ifPtr         the interface of the next hop router to destination

\retval      none
***************************************************************************************************/
void IP6_SetStaticRoute(ipAddr_t* setAddr,uint8_t prefixLen,ipAddr_t* nextHopAddr, void* ifPtr);

/*!*************************************************************************************************
\fn void IP6_RemStaticRoute(ipAddr_t* remAddr)
\brief Removes a static route in the IP6 routing table. 
        
\param [in]  remAddr       the address to remove from the routing table
      
\retval      none
***************************************************************************************************/
void IP6_RemStaticRoute(ipAddr_t* remAddr);

/*!*************************************************************************************************
\fn ipAddr_t* IP6_GetDefaultRoute(ipPktInfo_t* pIpPktInfo,uint8_t direction)
\brief  Searches the IP6 Routing Table for the default route

\param [in]  pIpPktInfo    the packet to send
\param [in]  direction     TX or RX

\retval      ipAddr_t*     pointer to next hop address or NULL
***************************************************************************************************/
ipAddr_t* IP6_GetDefaultRoute(ipPktInfo_t* pIpPktInfo, uint8_t direction);

#endif
/*!*************************************************************************************************
\fn uint32_t IP_GetAddrScope6(ipAddr_t *ipAddr)
\brief  Returns scope of the IPv6 address (Node-local,link-local, site-local or global)

\param [in]  ipAddr     the ip addres

\retval      uint32_t   addres scope
***************************************************************************************************/
uint32_t IP_GetAddrScope6(ipAddr_t *pIpAddr);

/*!*************************************************************************************************
\fn void IP6_GetSolicitedMcastAddr(ipAddr_t* ipAddr, ipAddr_t *solicitedMastAddr)
\brief  Get IPv6 solicited-node multicast address. It has the prefix FF02:0:0:0:0:1:FF00:0000/104
        concatenated with the 24 low-order bits of a corresponding IPv6 unicast or anycast address.

\param [in]  ipAddr                 the ip address from witch to construct multicast address
\param [out] solicitedMastAddr      the solictied multicast address

\retval      none
***************************************************************************************************/
void IP6_GetSolicitedMcastAddr(ipAddr_t* ipAddr, ipAddr_t* solicitedMastAddr);

/*!*************************************************************************************************
\fn    void IP6_AddPadding(uint8_t* pHeader, uint8_t headerLength, uint8_t dataLength)
\brief Function used to add Pad1 or PadN option when an IP extension
       header is not multiple of 8 bytes

\param [in]  pHeader         pointer to the start of the header
\param [in]  headerLength    total header length (multiple of 8 bytes)
\param [in]  dataLength      header util data length

\return      none
 ***************************************************************************************************/
void IP6_AddPadding(uint8_t* pHeader, uint8_t headerLength, uint8_t dataLength);

/*!*************************************************************************************************
\fn void IP6_Init(void);
\brief  Init function for IPv6

\retval      none
***************************************************************************************************/
void IP6_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* IP_IP6_ENABLE */
/*================================================================================================*/
#endif  /*_IP6_H_ */
