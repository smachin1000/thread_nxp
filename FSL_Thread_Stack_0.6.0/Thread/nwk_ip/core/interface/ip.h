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

#ifndef _IP_H_
#define _IP_H_

/*!=================================================================================================
\file       ip.h\
\brief      This is a header file for the IP layer.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "network_utils.h"
#include "TimersManager.h"
#include "ip_cfg.h"
#include "ip6.h"
#include "ip4.h"
/*==================================================================================================
Public macros
==================================================================================================*/

#define IP_VERSION_4       0x04        /* Version 4 */
#define IP_VERSION_6       0x06        /* Version 6 */

/* IP protocol types */
#define IPPROTO_HOPOPTS    0        /* IPv6 hop-by-hop options            */
#define IPPROTO_ICMP       1        /* Internet Control Message Protocol  */
#define IPPROTO_IGMP       2        /* Internet Group Management Protocol */
#define IPPROTO_IP         4        /* IP-in-IP encapsulation             */
#define IPPROTO_IPIP       4        /* IP-in-IP encapsulation             */
#define IPPROTO_TCP        6        /* Transmission Control Protocol      */
#define IPPROTO_UDP        17       /* User Datagram Protocol             */
#define IPPROTO_IPV6       41       /* IPv6-in-IP encapsulation           */
#define IPPROTO_ROUTING    43       /* IPv6 routing header                */
#define IPPROTO_FRAGMENT   44       /* IPv6 fragmentation header          */
#define IPPROTO_ESP        50       /* Encapsulating Security Payload     */
#define IPPROTO_AH         51       /* Authentication Header              */
#define IPPROTO_ICMPV6     58       /* ICMPv6                             */
#define IPPROTO_NONE       59       /* IPv6 no next header                */
#define IPPROTO_DSTOPTS    60       /* IPv6 destination options           */
#define IPPROTO_OSPF       89       /* Open Shortest Path Protocol        */
#define IPPROTO_COMP       108      /* IP compression                     */

#define IP6_ADDRESS_LIFETIME_INFINITE (0xFFFFFFFF)

/* ip statistics macros */
#if IP_IP6_STATS_ENABLE
    #define IP6_STATS_ENABLED(x) x
#else
    #define IP6_STATS_ENABLED(x)
#endif

/* ip statistics macros */
#if IP_IP4_STATS_ENABLE
    #define IP4_STATS_ENABLED(x) x
#else
    #define IP4_STATS_ENABLED(x)
#endif

#if IP_IF_STATS_ENABLE
    #define IPIF_STATS_ENABLED(x) x
#else
    #define IPIF_STATS_ENABLED(x)
#endif

#define IP_REASSEM_EXPIRE_TIME 60000U

#define IP_TIMER_PERIOD_MS      1000U                      /*!< Timer interval in miliseconds */
#define IP_TIMER_PERIOD_SEC     1U

/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef enum
{
    gIpOk_c = 0x00000000U,
    gIpGeneralError_c,
    gIpBadAddrError_c,
    gIpUnreachableError_c,
    gIpInvalidParameterError_c,
    gIpNoAddressSpaceError_c,
    gIpReasemFrag_c,
    gIpReasemDatagram_c,
    gIpUnusedError_c = 0xFFFFFFFF,
}ipErrorTypes_t;

typedef enum
{
    gIpProtv4_c = 0x0800,
    gIpProtv6_c = 0x86DD,
    gIpProtArp_c = 0x0806
}ipProtocolType_t;

typedef enum
{
    gIpRouteRIPng_c = 0x01U,
    gIpRouteStatic = 0x02U
}ipRoutingTypes_t;

typedef enum
{
    gIpForwardTx_c,
    gIpForwardRx_c
}ipForwardDirection_t;

#if IP_IF_STATS_ENABLE
typedef struct ipCommonStats_tag
{
   uint32_t  rxTotal;
   uint32_t  rxMissed;
   uint32_t  rxDiscarded;
   uint32_t  rxErrors;

   uint32_t  txTotal;
   uint32_t  txMissed;
   uint32_t  txDiscarded;
   uint32_t  txErrors;
}ipCommonStats_t;


typedef struct ip6Stats_tag
{
   ipCommonStats_t commonStats;

   uint32_t  rxHdrErrors;       /* Discarded -- error in IP header    */
   uint32_t  rxAddrErrors;      /* Discarded -- illegal destination   */
   uint32_t  rxNoProto;         /* Discarded -- unrecognized protocol */
   uint32_t  rxDelivered;       /* Datagrams delivered to upper layer */
   uint32_t  rxForwarded;       /* Datagrams forwarded                */

   /* These are included in rxDiscarded and rxHdrErrors */
   uint32_t  rxBadVersion;      /* Datagrams with version != 6        */
   uint32_t  rxBadSource;       /* Datagrams with invalid src address */
   uint32_t  rxSmallPkt;        /* Datagrams larger than frame        */
   uint32_t  rxTtlExceeded;     /* Datagrams to route with TTL = 0    */

   uint32_t  rxFragRecvd;       /* Number of received IP fragments    */
   uint32_t  rxFragReasmd;      /* Number of reassembled datagrams    */
   uint32_t  rxFragDiscarded;   /* Number of discarded fragments      */

   uint32_t  txFragSent;        /* Number of sent fragments           */
   uint32_t  txFragFragd;       /* Number of fragmented datagrams     */
   uint32_t  txFragDiscarded;   /* Number of fragmentation failures   */

}ip6Stats_t;


typedef struct ip4Stats_tag
{
   ipCommonStats_t commonStats;

   uint32_t  rxHdrErrors;       /* Discarded -- error in IP header    */
   uint32_t  rxAddrErrors;      /* Discarded -- illegal destination   */
   uint32_t  rxNoProto;         /* Discarded -- unrecognized protocol */
   uint32_t  rxDelivered;       /* Datagrams delivered to upper layer */
   uint32_t  rxForwarded;       /* Datagrams forwarded                */

   /* These are included in rxDiscarded and rxHdrErrors */
   uint32_t  rxBadVersion;      /* Datagrams with version != 4        */
   uint32_t  rxBadChecksum;     /* Datagrams with invalid checksum    */
   uint32_t  rxBadSource;       /* Datagrams with invalid src address */
   uint32_t  rxSmallHdr;        /* Datagrams with header too small    */
   uint32_t  rxSmallPkt;        /* Datagrams larger than frame        */
   uint32_t  rxTtlExceeded;     /* Datagrams to route with TTL = 0    */

   uint32_t  rxFragRecvd;       /* Number of received IP fragments    */
   uint32_t  rxFragReasmd;      /* Number of reassembled datagrams    */
   uint32_t  rxFragDiscarded;   /* Number of discarded fragments      */

   uint32_t  txFragSent;        /* Number of sent fragments           */
   uint32_t  txFragFragd;       /* Number of fragmented datagrams     */
   uint32_t  txFragDiscarded;   /* Number of fragmentation failures   */

}ip4Stats_t;

typedef struct ipIfStats_tag
{
   ipCommonStats_t commonStats;

   uint32_t  rxOctets;           /* total bytes received       */
   uint32_t  rxUnicast;          /* unicast packets received   */
   uint32_t  rxMulticast;        /* multicast packets received */
   uint32_t  rxBroadcast;        /* broadcast packets received */

   uint32_t  txOctets;           /* total bytes sent           */
   uint32_t  txUnicast;          /* unicast packets sent       */
   uint32_t  txMulticast;        /* multicast packets sent     */
   uint32_t  txBroadcast;        /* broadcast packets sent     */

}ipIfStats_t;
#endif

struct ipIfStruct_tag;

typedef struct mediaIfStruct_tag
{
   uint32_t (* ifOpen) (struct ipIfStruct_tag**);
   uint32_t (* ifClose)(struct ipIfStruct_tag**);
   uint32_t (* ifSend4) (ipPktInfo_t*);
   uint32_t (* ifSendArp) (ipPktInfo_t*, llAddr_t*);
   uint32_t (* ifSend6) (ipPktInfo_t*);
   uint32_t (* ifGetIID)(struct ipIfStruct_tag**,llAddr_t*, ipAddr_t*);
   uint32_t (* ifJoin) (struct ipIfStruct_tag**, ipAddr_t*,uint16_t);
   uint32_t (* ifLeave)(struct ipIfStruct_tag**, ipAddr_t*,uint16_t);

}mediaIfStruct_t;

typedef struct ipReasemStruct_tag
{
    /*! datagram ID used for ressembly */
    uint32_t id;
    /*! total length of the fragmented part of the IP datagram */
    uint32_t totalLen;
    /*! unfragmentable part size */
    uint16_t unfragtLen;
    /*! next header value of the first fragment header */
    uint8_t nextHdr;
    /*! dummy byte used for alignment */
    uint8_t dummy;
    /*! handle for timer event used for IP ressembly timeout */
    tmrTimerID_t timerID;
}ipReasemStruct_t;

typedef struct ipIfStruct_tag
{
    /*! handle for media link layer module */
    void *               ifDriverHandle;
     /*! pointer to media interface functions */
    mediaIfStruct_t*     ifFunctions;
    /*! interface maximum transmission unit */
    uint16_t             ifMtu;
     /*! if ipVersion4 == 1->IPv4 is enabled on this interface */
    uint8_t              ipVersion4;
    /*! if ipVersion6 == 1->IPv6 is enabled on this interface */
    uint8_t              ipVersion6;
    /*! pointer to string with interface name */
    uint8_t*             ifNamePtr;
#if IP_IP6_ENABLE
    ip6IfStruct_t        ip6If;
#endif
#if IP_IP4_ENABLE
    ip4IfStruct_t        ip4If;
#endif

#if IP_IF_STATS_ENABLE
    ipIfStats_t         stats;
#endif

   /*! media link layer addr */
   llAddr_t             ifDevAddrTbl[IP_IF_MAC_ADDR_NB];
} ipIfStruct_t;

typedef ipIfStruct_t* ifHandle_t;

/*! Function for servicing transport packets */
typedef void (* ipTransportService)(ipPktInfo_t* );

/*! Function for packet forwarding */
typedef void (* ipForwardService)(ipPktInfo_t *);

typedef struct ipTransportServiceStruct_tag
{
   /*! Owner's service function */
   ipTransportService service;
   /*! Next header identifier */
   uint32_t protocol;
} ipTransportServiceStruct_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
extern taskMsgQueue_t mIpMsgQueue;

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn void IP_Task(void const *argument)
\brief  IP layer task

\param [in]  argument    task private data

\retval      none
***************************************************************************************************/
void IP_Task(osaTaskParam_t argument);

/*!*************************************************************************************************
\fn void IP_Receive(ipPktInfo_t* pIpPktInfo, uint32_t  protocol)
\brief  Receives a data packet from a media interface and, based on the received protocol(IPv4,
        IPv6) sends a message to the IP task appropriate function

\param [in]  pIpPktInfo    the packet to send
\param [in]  protocol      the upper layer protocol

\retval      uint32_t   error
***************************************************************************************************/
void IP_Receive(ipPktInfo_t* pIpPktInfo, uint32_t  protocol);

/*!*************************************************************************************************
\fn uint32_t IP_Send(ipPktInfo_t* pIpPktInfo, uint32_t  protocol)
\brief  Sends an IP packet generated on the local host. Determines based on source and destionation
        address the IP protocol(v4, v6)

\param [in]  pIpPktInfo    the packet to send
\param [in]  protocol      the upper layer protocol

\retval      uint32_t   error
***************************************************************************************************/
uint32_t IP_Send(ipPktInfo_t* pIpPktInfo, uint32_t  protocol);

/*!*************************************************************************************************
\fn ipTransportServiceStruct_t* IP_GetTransportServiceStruct(uint32_t transportProtocol)
\brief  Returns the pointer to the ipTransportServiceStruct_t that matches the provided protocol

\param [in]  transportProtocol           transport protocol next header value

\retval      ipTransportServiceStruct_t*  pointer to transport service descriptor struct
***************************************************************************************************/
ipTransportServiceStruct_t* IP_GetTransportServiceStruct(uint32_t transportProtocol);

/*!*************************************************************************************************
\fn uint32_t IP_SetTransportServiceStruct(ipTransportServiceStruct_t* pServiceStructData)
\brief  Registers a transport layer protocol callback with the IP layer.

\param [in]  pServiceStructData

\retval      gIpOk_c if ok
\retval      gIpGeneralError_c if error
***************************************************************************************************/
uint32_t IP_SetTransportServiceStruct(ipTransportServiceStruct_t* pServiceStructData);

/*!*************************************************************************************************
\fn void IP_Init(void)
\brief  Initializes the IP layer

\retval   taskMsgQueue_t    IP Task Queue Id
***************************************************************************************************/
taskMsgQueue_t * IP_Init(void);

/*!*************************************************************************************************
\fn uint32_t IP_GetUpperLayerProt(ipPktInfo_t* pIpDatagram,uint32_t* pUpperLayerOffset)
\brief  Processes an IP datagram and returns the upper layer transport protocol present in the IP
        datagram

\param [in]  pIpDatagram        pointer to the IP datagram
\param [out] pUpperLayerOffset  can be NULL. When not NULL it shall indicate the offset to upper
                                transport layer data inside the IP packet

\retval      uint32_t           the upper layer protocol present in the IP datagram
***************************************************************************************************/
uint32_t IP_GetUpperLayerProt(ipPktInfo_t* pIpDatagram, uint32_t* pUpperLayerOffset);


#if IP_IP6_ENABLE_REASSEMBLY  | IP_IP4_ENABLE_REASSEMBLY
/*!*************************************************************************************************
\fn ipPktInfo_t* IP_AddReasemDatagram(ipPktInfo_t* pInData,uint32_t id,
    list_t* pIpReasemblyList)
\brief  Find the ipPktInfo_t type datagram descriptor matching the received ip fragment based on
        ipsrc, ipdst, proto and id or creates a new one.

\param [in]  pInData          the packet to process
\param [in]  id               datagram ID
\param [in]  pIpReasemblyList descriptor of the list that keeps the all the datagrams waiting
                               reassembly

\retval      ipPktInfo_t*   the IP datagram descriptor
***************************************************************************************************/
ipPktInfo_t* IP_AddReasemDatagram(ipPktInfo_t* pInData, uint32_t id,
                                             list_t* pIpReasemblyList);

/*!*************************************************************************************************
\fn void IP_AddDatagramFrag(ipPktInfo_t*  dgram, ipPktInfo_t** pInData, uint8_t* pFragData,
      uint32_t fragLen, uint32_t fragOffset)
\brief  Adds a received IP fragment to an ipPktInfo_t datagram.

\param [in]  dgram          ipPktInfo_t type datagram that holds the fragments received so far
\param [in]  pInData        the Ip fragment to add
\param [in]  pFragData      pointer to the fragment data
\param [in]  fragLen        length of the fragment data
\param [in]  fragOffset     offset in the original datagram of the fragmen data

\retval      none
***************************************************************************************************/
void IP_AddDatagramFrag(ipPktInfo_t*  dgram, ipPktInfo_t** pInData, uint8_t* pFragData,
                                  uint32_t fragLen, uint32_t fragOffset);
/*!*************************************************************************************************
\fn bool_t IP_CanReasemDgram(ipPktInfo_t* dgram)
\brief  Checks if a datagram can be reassabled by looking if all the received fragments combined
        fill the entire original datagram data

\param [in]  dgram    ipPktInfo_t type datagram that holds the fragments received so far

\retval      none
***************************************************************************************************/
bool_t IP_CanReasemDgram(ipPktInfo_t* dgram);

/*!*************************************************************************************************
\fn void IP_ReasemDgram(ipPktInfo_t* dgram)
\brief  Reasembles an ipPktInfo_t datagram from all the Ip fragments.

\param [in]  dgram          ipPktInfo_t type datagram that holds the fragments received so far

\retval      none
***************************************************************************************************/
void IP_ReasemDgram(ipPktInfo_t* dgram);

#endif

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /*_IP_H_ */
