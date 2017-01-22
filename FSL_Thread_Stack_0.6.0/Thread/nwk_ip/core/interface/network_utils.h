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

#ifndef _NETWORK_UTILS_H
#define _NETWORK_UTILS_H
/*!=================================================================================================
\file       network_utils.h
\brief      This is a header file for the Network Utils module.
==================================================================================================*/


/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "messaging.h"
#include "fsl_osa_ext.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#if __ICCARM__
#define gLittleEndian_c __LITTLE_ENDIAN__
#else
#error "No Compiler was set"
#endif

/* Max unsigned integers values */
#define UINT32_T_MAX_VALUE 0xFFFFFFFFU
#define UINT16_T_MAX_VALUE 0xFFFFU

/* Length for IP address string size (used to compute size used in ntop) */
#define INET_ADDRSTRLEN         16
#define INET6_ADDRSTRLEN        46

/* Special IPv4 addresses */
#define IP4_ADDR_ANY               0UL
#define IP4_ADDR_LOOPBACK          0x7F000001L
#define IP4_ADDR_ALLHOSTS_GROUP    0xE0000001L
#define IP4_ADDR_ALLROUTERS_GROUP  0xE0000002L
#define IP4_ADDR_RIP_GROUP         0xE0000009L
#define IP4_ADDR_NTP_GROUP         0xE0000101L
#define IP4_ADDR_IGMP_GROUP        0xE0000016L
#define IP4_ADDR_BROADCAST         0xFFFFFFFFL
/* IPv4 addresses mapped to IPv6 */
#define INADDR_ANY_INIT                                    \
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00 }

/* Macros to classify IPv4 addresses */
#define IP4_ZERONET(a)         (((a) & 0xFF000000L) == 0x00000000L)
#define IP4_LOOPBACK(a)        (((a) & 0xFF000000L) == 0x7F000000L)
#define IP4_MULTICAST(a)       (((a) & 0xF0000000L) == 0xE0000000L)
#define IP4_LOCAL_MULTICAST(a) (((a) & 0xFFFFFF00L) == 0xE0000000L)
#define IP4_EXPERIMENTAL(a)    (((a) & 0xF0000000L) == 0xF0000000L)

#define IP4_CLASS_A(a)          (((a) & 0x80000000L) == 0x00000000L)
#define IP4_CLASS_A_MASK         0xFF000000L
#define IP4_CLASS_B(a)          (((a) & 0xC0000000L) == 0x80000000L)
#define IP4_CLASS_B_MASK         0xFFFF0000L
#define IP4_CLASS_C(a)          (((a) & 0xE0000000L) == 0xC0000000L)
#define IP4_CLASS_C_MASK         0xFFFFFF00L

/*! IPV6 addresses */
#define IN6ADDR_ANY_INIT                                    \
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define IN6ADDR_LOOPBACK_INIT                               \
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }
#define IN6ADDR_NODELOCAL_ALLNODES_INIT                     \
        { 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }
#define IN6ADDR_INTFACELOCAL_ALLNODES_INIT                  \
        { 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }
#define IN6ADDR_LINKLOCAL_ALLNODES_INIT                     \
        { 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }
#define IN6ADDR_LINKLOCAL_ALLROUTERS_INIT                   \
        { 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 }
#define IN6ADDR_LINKLOCAL_ALLV2ROUTERS_INIT                 \
        { 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16 }

#define IN6ADDR_LINKLOCAL_ALL_DHCP_ROUTERS_AND_RELAY_AGENTS  \
        {0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02}

#define IN6ADDR_REALMLOCAL_ALL_DHCP_LEASEQUERY_SERVERS       \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03}


#define IN6ADDR_SITELOCAL_ALLDHCPSERVERS                                  \
        {0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03}

#define IN6ADDR_REALMLOCAL_ALLNODES_INIT \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}

#define IN6ADDR_REALMLOCAL_ALLROUTERS_INIT \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}

#define IN6ADDR_SITELOCAL_ALLNODES_INIT \
        {0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}

#define IN6ADDR_SITELOCAL_ALLROUTERS_INIT \
        {0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}

#define IN6ADDR_LINK_LOCAL_PREFIX_INIT \
        {0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

/*! Macro for IP address copy */
#define IP_AddrCopy(dst, src) \
        (dst)->addr64[0] = (src)->addr64[0]; \
        (dst)->addr64[1] = (src)->addr64[1];

/*! Macro for IP address conversion to uint32_t */
#define IP4_AddrToUint32(addr) (ntohal(&(addr)->addr8[12]))


/*! Macro for IPV6 address comparison */
#define IP_IsAddrEqual(addr1, addr2) \
        (((addr1)->addr64[0] == (addr2)->addr64[0]) && \
         ((addr1)->addr64[1] == (addr2)->addr64[1]))

#define IP6_IsUnspecifiedAddr(addr) \
        (((addr)->addr64[0] == 0U) && \
         ((addr)->addr64[1] == 0U))

#define IP6_IsLinkLocalAddr(addr) \
        (((addr)->addr8[0] == 0xFE) && (((addr)->addr8[1] & 0xC0) == 0x80))

#define IP6_IsSiteLocalAddr(addr) \
        (((addr)->addr8[0] == 0xFE) && (((addr)->addr8[1] & 0xC0) == 0xC0))

#define IP6_IsUniqueLocalAddr(addr) \
        (((addr)->addr8[0] == 0xFD)||((addr)->addr8[0] == 0xFC) )

#define IP6_IsGlobalAddr(addr) \
        ((((addr)->addr8[0] & 0xF0) == 0x20)||(((addr)->addr8[0] & 0xF0) == 0x30))

#define IP6_IsMulticastAddr(addr) \
        ((addr)->addr8[0] == 0xFF)

#define IP6_IsLoopbackAddr(addr) \
        (((addr)->addr64[0] == 0U) && \
         ((addr)->addr64[1] == 0x0100000000000000U))

#define IP_ADDR(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16) \
        { (a1), (a2), (a3), (a4), (a5), (a6), (a7), (a8),       \
    (a9), (a10), (a11), (a12), (a13), (a14), (a15), (a16) }

/*! Mask for IPV4 address identification(RFC4291: 2.5.5.2) */
#define IPV4_Mask32_g   (0xFFFFU)

#define IP_IsAddrIPv4(addr) \
        ((addr)->addr64[0] == 0U &&         \
         (addr)->addr16[4] == 0U &&         \
         (addr)->addr16[5] == IPV4_Mask32_g)

#define IP4_IsUnspecifiedAddr(addr) \
        (((addr)->addr32[3] == 0U))

#define IP_IsAddrIPv6(addr) (!(IP_IsAddrIPv4(addr)))

#define NWKU_AppendNwkBuffer(dst, src) \
        (dst)->next = (src);

#define NWKU_IsLlAddrValid(llAddr) (gLlayerAddrNoAddr_c != llAddr.addrSize)

#define NWKU_GetLastArrayIndex(arraySize) ((arraySize) - 1U)

#if gLittleEndian_c

/*! Network byte order is OTA order */
#ifndef ntohs
#define ntohs(val)                      NWKU_Revert16((uint16_t)(val))
#endif
#ifndef htons
#define htons(val)                      NWKU_Revert16((uint16_t)(val))
#endif

#ifndef ntohl
#define ntohl(val)                      NWKU_Revert32((uint32_t)(val))
#endif
#ifndef htonl
#define htonl(val)                      NWKU_Revert32((uint32_t)(val))
#endif

#ifndef ntohll
#define ntohll(val)                     NWKU_Revert64((uint64_t)(val))
#endif
#ifndef htonll
#define htonll(val)                     NWKU_Revert64((uint64_t)(val))
#endif

#ifndef ntohas
#define ntohas(p)                       NWKU_TransformArrayToUint16(p)
#endif
#ifndef htonas
#define htonas(p, x)                    NWKU_TransformUint16ToArray(p, (uint16_t)(x))
#endif

#ifndef ntohal
#define ntohal(p)                       NWKU_TransformArrayToUint32(p)
#endif
#ifndef htonal
#define htonal(p, x)                    NWKU_TransformUint32ToArray(p, (uint32_t)(x))
#endif

#ifndef ntohall
#define ntohall(p)                      NWKU_TransformArrayToUint64(p)
#endif
#ifndef htonall
#define htonall(p, x)                   NWKU_TransformUint64ToArray(p, (uint64_t)(x))
#endif

#else /*gBigEndian_c */

#ifndef ntohs
#define ntohs(val)                      (uint16_t)(val)
#endif
#ifndef htons
#define htons(val)                      (uint16_t)(val)
#endif

#ifndef ntohl
#define ntohl(val)                      (uint32_t)(val)
#endif
#ifndef htonl
#define htonl(val)                      (uint32_t)(val)
#endif

#ifndef ntohll
#define ntohll(val)                     (uint64_t)(val)
#endif
#ifndef htonll
#define htonll(val)                     (uint64_t)(val)
#endif

#ifndef ntohas
#define ntohas(p)                       (*((uint16_t*)(p)))
#endif
#ifndef htonas
#define htonas(p, x)                    *((uint16_t*)(p)) = x
#endif

#ifndef ntohal
#define ntohal(p)                       (*((uint32_t*)(p)))
#endif
#ifndef htonal
#define htonal(p, x)                    *((uint32_t*)(p)) = x
#endif

#ifndef ntohall
#define ntohall(p)                      (*((uint64_t*)(p)))
#endif
#ifndef htonall
#define htonall(p, x)                   *((uint64_t*)(p)) = x
#endif

#endif

/* Supported address families. */
#define AF_UNSPEC   0
#define AF_UNIX     1   /* Unix domain sockets      */
#define AF_LOCAL    1   /* POSIX name for AF_UNIX   */
#define AF_INET     2   /* Internet IP Protocol     */
#define AF_AX25     3   /* Amateur Radio AX.25      */
#define AF_IPX      4   /* Novell IPX           */
#define AF_APPLETALK    5   /* AppleTalk DDP        */
#define AF_NETROM   6   /* Amateur Radio NET/ROM    */
#define AF_BRIDGE   7   /* Multiprotocol bridge     */
#define AF_ATMPVC   8   /* ATM PVCs         */
#define AF_X25      9   /* Reserved for X.25 project    */
#define AF_INET6    10  /* IP version 6         */
#define AF_ROSE     11  /* Amateur Radio X.25 PLP   */
#define AF_DECnet   12  /* Reserved for DECnet project  */
#define AF_NETBEUI  13  /* Reserved for 802.2LLC project*/
#define AF_SECURITY 14  /* Security callback pseudo AF */
#define AF_KEY      15      /* PF_KEY key management API */
#define AF_NETLINK  16
#define AF_ROUTE    AF_NETLINK /* Alias to emulate 4.4BSD */
#define AF_PACKET   17  /* Packet family        */
#define AF_ASH      18  /* Ash              */
#define AF_ECONET   19  /* Acorn Econet         */
#define AF_ATMSVC   20  /* ATM SVCs         */
#define AF_RDS      21  /* RDS sockets          */
#define AF_SNA      22  /* Linux SNA Project (nutters!) */
#define AF_IRDA     23  /* IRDA sockets         */
#define AF_PPPOX    24  /* PPPoX sockets        */
#define AF_WANPIPE  25  /* Wanpipe API Sockets */
#define AF_LLC      26  /* Linux LLC            */
#define AF_CAN      29  /* Controller Area Network      */
#define AF_TIPC     30  /* TIPC sockets         */
#define AF_BLUETOOTH    31  /* Bluetooth sockets        */
#define AF_IUCV     32  /* IUCV sockets         */
#define AF_RXRPC    33  /* RxRPC sockets        */
#define AF_ISDN     34  /* mISDN sockets        */
#define AF_PHONET   35  /* Phonet sockets       */
#define AF_IEEE802154   36  /* IEEE802154 sockets       */
#define AF_CAIF     37  /* CAIF sockets         */
#define AF_ALG      38  /* Algorithm sockets        */
#define AF_NFC      39  /* NFC sockets          */
#define AF_VSOCK    40  /* vSockets         */
#define AF_MAX      41  /* For now.. */

#define DEFAULT_LLADDR_IDX    0
/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef union uuint16_tag
{
    uint16_t    u16;
    uint8_t     u8[2];
} uuint16_t;

typedef union uuint32_tag
{
    uint32_t    u32;
    uint16_t    u16[2];
    uint8_t     u8[4];
} uuint32_t;

typedef union uuint64_tag
{
    uint64_t    u64;
    uint32_t    u32[2];
    uint16_t    u16[4];
    uint8_t     u8[8];
} uuint64_t;

typedef union ipAddr_tag
{
    uint8_t     addr8[16];
    uint16_t    addr16[8];
    uint32_t    addr32[4];
    uint64_t    addr64[2];
} ipAddr_t;                             /*!< Generic structure for holding IP address information */

typedef struct nwkBuffer_tag
{
    struct nwkBuffer_tag *next;
    uint8_t *pData;
    uint32_t size;
    uint8_t freeBuffer;
} nwkBuffer_t;                          /*!< Generic structure for holding buffer information */

typedef enum
{
    gLlayerAddrNoAddr_c     = 0x00, /* No address (addressing fields omitted) */
    gLlayerAddrReserved_c   = 0x01, /* Reserved value*/
    gLlayerAddrEui16_c      = 0x02, /* 16-bit short Link Layer address (size 2 bytes) */
    gLlayerAddrEui48_c      = 0x06, /* 48-bit ethernet MAC Address (size 6 bytes) */
    gLlayerAddrEui64_c      = 0x08, /* 64-bit extended Link Layer address (size 8 bytes) */
} llAddrSize_t;

typedef struct llAddr_tag
{
    uint8_t         eui[8];          /*!< Destination address: short/extended */
    llAddrSize_t    addrSize;        /*!< Destination address type: short/extended */
} llAddr_t;

typedef struct ip6Header_tag
{
    uint8_t versionTraficClass;
    uint8_t trafficClassFlowLabel;
    uint8_t flowLabel[2];
    uint8_t payloadLength[2];
    uint8_t nextHeader;
    uint8_t hopLimit;
    uint8_t srcAddr[16];
    uint8_t dstAddr[16];
}ip6Header_t;

typedef struct ipPktOptions_tag
{
    void*   ifHandle;
    uint8_t hopLimit;
    uint8_t security;
    uint8_t lqi;
    uint8_t qos;
    void*   extendedOptions;
} ipPktOptions_t;

typedef struct ipPktInfo_tag
{
    nwkBuffer_t*    pNwkBuff;       /* free */
    ipAddr_t*       pIpSrcAddr;     /* free */
    ipAddr_t*       pIpDstAddr;     /* free */
    /* Pointer to the next protocol in pNwkBuff->pData. Do not free this one!!!  */
    uint8_t*        pNextProt;
    union {
        /* size of the data of next protocol in pNwkBuff->pData */
        uint32_t        nextProtLen;
        /* protocol type */
        uint32_t        protocolType;
    };
    ipPktOptions_t* pIpPktOptions;  /* free */
} ipPktInfo_t;

/*! Function for servicing transport packets */
typedef void (* nwkMsgHandler)(void*);

typedef struct nwkMsg_tag
{
    nwkMsgHandler pFunc;
    void* pPload;
}nwkMsg_t;

typedef struct taskMsgQueue_tag
{
    msgQueue_t msgQueue;
    osaTaskId_t taskId;
    osaEventId_t taskEventId;
}taskMsgQueue_t;

typedef void (* tspDataIndCb_t)(osaEventId_t eventId);

/*! Lookup tables with 8 bits elements */
typedef struct lut8_tag
{
    uint8_t type;
    uint8_t idx;
}lut8_t;

typedef struct nwkuStats_tag
{
    uint8_t ipktUsed;
    uint8_t ipktMax;
    uint8_t nwkBuffUsed;
    uint8_t nwkBuffMax;
} nwkStats_t;
/*==================================================================================================
Public global variables declarations
==================================================================================================*/

extern const ipAddr_t inaddr_any;
extern const ipAddr_t in6addr_any;
extern const ipAddr_t in6addr_loopback;
extern const ipAddr_t in6addr_nodelocal_allnodes;
extern const ipAddr_t in6addr_linklocal_allnodes;
extern const ipAddr_t in6addr_linklocal_allrouters;
extern const ipAddr_t in6addr_linklocal_allv2routers;
extern const ipAddr_t in6addr_sitelocal_alldhcpservers;
extern const ipAddr_t in6addr_realmlocal_allleasequeryservers;
extern const ipAddr_t in6addr_realmlocal_alldhcpservers;
extern const ipAddr_t in6addr_sitelocal_allnodes;
extern const ipAddr_t in6addr_sitelocal_allrouters;
extern const ipAddr_t in6addr_link_local_prefix;

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn    void NWKU_SendMsg(nwkMsgHandler pFunc, void* pPload, taskMsgQueue_t* msgQueue);
\brief Network Utils module function used to send a message between two tasks

\param [in]   pFunc     pointer to message handler function
\param [in]   pPload    pointer to message data
\param [in]   msgQueue  pointer to structure holding message queue and task id to send message

\return       none
 ***************************************************************************************************/
void NWKU_SendMsg(nwkMsgHandler pFunc, void* pPload, taskMsgQueue_t* msgQueue);

/*!*************************************************************************************************
\fn    void NWKU_RecvMsg(taskMsgQueue_t* msgQueue)
\brief Network Utils module function used to receive a message in a task

\param [in]   msgQueue  pointer to structure holding message queue and task id to receive message

\return       none
 ***************************************************************************************************/
void NWKU_RecvMsg(taskMsgQueue_t* msgQueue);
/*!*************************************************************************************************
\fn    ipAddr_t *NWKU_CreateIpAddr(void)
\brief Network Utils module function used to create an ipAddr_t structure

\return       ipPktInfo_t     pointer to the allocated ipPktInfo_t
 ***************************************************************************************************/
ipAddr_t *NWKU_CreateIpAddr(void);

/*!*************************************************************************************************
\fn void NWKU_ConvertIp4Addr(uint32_t  ip4Addr, ipAddr_t* pOutIpAddr)
\brief Network Utils module function used to convert an IPv4 address in uint32_t format to an
       ipAddr_t type address

\param [in]   ip4Addr       IPv4 address
\param [out]  pOutIpAddr    pointer to ipAddr_t to store the converterd address

\return       none
 ***************************************************************************************************/
void NWKU_ConvertIp4Addr(uint32_t  ip4Addr, ipAddr_t* pOutIpAddr);

/*!*************************************************************************************************
\fn    ipPktInfo_t *NWKU_CreateIpPktInfo(void)
\brief Network Utils module function used to create an ipPktInfo_t structure

\return       ipPktInfo_t     pointer to the allocated ipPktInfo_t
 ***************************************************************************************************/
ipPktInfo_t *NWKU_CreateIpPktInfo(void);

/*!*************************************************************************************************
\fn    void NWKU_FreeIpPktInfo(ipPktInfo_t **pIpPktInfo)
\brief Network Utils module function used to free one ipPktInfo_t structure

\param [in]   pIpPktInfo      double pointer to the ipPktInfo_t structure

\return       none
 ***************************************************************************************************/
void NWKU_FreeIpPktInfo(ipPktInfo_t **pIpPktInfo);

/*!*************************************************************************************************
\fn    nwkBuffer_t *NWKU_CreateNwkBuffer(uint32_t dataSize)
\brief Network Utils module function used to create a nwkBuffer_t structure and allocate memory for
       data

\param [in]   dataSize         size of the data available in the buffer

\return       nwkBuffer_t*     pointer to the allocated nwkBuffer_t
 ***************************************************************************************************/
nwkBuffer_t *NWKU_CreateNwkBuffer(uint32_t dataSize);

/*!*************************************************************************************************
\fn    void NWKU_FreeAllNwkBuffers(nwkBuffer_t **pNwkBufferStart)
\brief Network Utils module function used to free all nwkBuffer_t structs(starting with
       pNwkBufferStart) and change the start of the list to NULL

\param [in]   pNwkBufferStart double pointer to the start of data buffer

\return       void
 ***************************************************************************************************/
void NWKU_FreeAllNwkBuffers(nwkBuffer_t **pNwkBufferStart);

/*!*************************************************************************************************
\fn    void NWKU_FreeNwkBufferElem(nwkBuffer_t **pNwkBufferStart, nwkBuffer_t *pElem)
\brief Network Utils module function used to free one nwkBuffer_t element

\param [in]   pNwkBufferStart double pointer to the start of data buffer
\param [in]   pElem           pointer to the element to be freed

\return       void
 ***************************************************************************************************/
void NWKU_FreeNwkBufferElem(nwkBuffer_t **pNwkBufferStart, nwkBuffer_t *pElem);

/*!*************************************************************************************************
\fn    uint32_t NWKU_NwkBufferTotalSize(nwkBuffer_t *pNwkBufferStart)
\brief Network Utils module function used to calculate the total size of a nwkBuffer_t list,
       starting with pNwkBufferStart

\param [in]   pNwkBufferStart pointer to the start of nwkBuffer

\return       uint32_t        size of the whole list
 ***************************************************************************************************/
uint32_t NWKU_NwkBufferTotalSize(nwkBuffer_t *pNwkBufferStart);
/*!*************************************************************************************************
\fn void NWKU_MemCopyFromNwkBuffer(nwkBuffer_t** pNwkBuffer, uint8_t** pSrcPtr, uint8_t* pDstPtr,
                                   uint32_t size)
\brief Network Utils module function used to copy from a network fragmented buffer into a regular
       linear buffer

\param [in,out]  pNwkBuffer  pointer to the start network buffer - pointer to end network buffer
\param [in,out]  pSrcPtr     pointer to the source date in the strt network buffer - returns last
                             position in the end network buffer
\param [in]      pDstPtr     destination pointer
\param [in]      size        size to copy

\return          none
 ***************************************************************************************************/
void  NWKU_MemCopyFromNwkBuffer(nwkBuffer_t** pNwkBuffer,uint8_t** pSrcPtr, uint8_t* pDstPtr,
                                uint32_t size);
/*!*************************************************************************************************
\fn    uint32_t NWKU_NwkBufferNumber(nwkBuffer_t *pNwkBufferStart)
\brief Network Utils module function used to return the number of nwkBuffer_t fragments in the list

\param [in]   pNwkBufferStart pointer to the start of data buffer

\return       uint32_t        number of nwkBuffer_t fragments in the list
 ***************************************************************************************************/

uint32_t NWKU_NwkBufferNumber(nwkBuffer_t *pNwkBufferStart);

/*!*************************************************************************************************
\fn    uint8_t * NWKU_NwkBufferToRegularBuffer(nwkBuffer_t *pNwkBufferStart)
\brief Network Utils module function used to transform a network fragmented buffer into a regular
       linear buffer.

\param [in]   pNwkBufferStart   pointer to the start of network buffer

\return       uint8_t *         pointer to an allocated regular buffer that gets created
***************************************************************************************************/

uint8_t * NWKU_NwkBufferToRegularBuffer(nwkBuffer_t *pNwkBufferStart);

/*!*************************************************************************************************
\fn    nwkBuffer_t *NWKU_CreatePseudoHeader4(ipAddr_t *pSrcIp, ipAddr_t *pDstIp, uint32_t length, uint8_t nextHeader)
\brief Network Utils module function used to create the pseudoheader for IPv4 protocols

\param [in]  pSrcIp           pointer to the source IP address
\param [in]  pDstIp           pointer to the destination IP address
\param [in]  length           length of the protocol(header + data)
\param [in]  nextHeader       value of the next header

\return      nwkBuffer_t      pointer to a new allocated nwkBuffer_t element
 ***************************************************************************************************/

nwkBuffer_t *NWKU_CreatePseudoHeader4(ipAddr_t *pSrcIp, ipAddr_t *pDstIp, uint32_t length, uint8_t nextHeader);

/*!*************************************************************************************************
\fn    nwkBuffer_t *NWKU_CreatePseudoHeader6(ipAddr_t *pSrcIp, ipAddr_t *pDstIp, uint32_t length, uint8_t nextHeader)
\brief Network Utils module function used to create the pseudoheader for IPv6 protocols

\param [in]  pSrcIp           pointer to the source IP address
\param [in]  pDstIp           pointer to the destination IP address
\param [in]  length           length of the protocol(header + data)
\param [in]  nextHeader       value of the next header

\return      nwkBuffer_t      pointer to a new allocated nwkBuffer_t element
 ***************************************************************************************************/

nwkBuffer_t *NWKU_CreatePseudoHeader6(ipAddr_t *pSrcIp, ipAddr_t *pDstIp, uint32_t length, uint8_t nextHeader);

/*!*************************************************************************************************
\fn    uint16_t NWKU_CalculateChecksum(nwkBuffer_t *pStart)
\brief Network Utils module function used to calculate the checksum for a nwkBuffer_t list starting
with pStart element

\param [in]  pStart          pointer to the start of the list

\return      uint16_t        checksum for the whole list
 ***************************************************************************************************/

uint16_t NWKU_CalculateChecksum(nwkBuffer_t *pStart);

/*!*************************************************************************************************
\fn bool_t NWKU_CmpAddrPrefix6(uint8_t * addr1, uint8_t *addr2, uint32_t prefixLen)
\brief Compares first "prefixLen" bits of the ipv6 addresses.

\param [in]  addr1      first prefix to compare
\param [in]  addr2      second prefix to compare
\param [in]  prefixLen  lenght in bits to compare

\return      bool_t     TRUE if match
                        FALSE otherwise
 ***************************************************************************************************/
bool_t NWKU_CmpAddrPrefix6(uint8_t * addr1, uint8_t *addr2, uint32_t prefixLen);

/*!*************************************************************************************************
\fn  bool_t NWKU_BitCmp(uint8_t *pStr1, uint8_t *pStr2, uint8_t startBit, uint8_t stopBit)
\brief  Compare two strings bit by bit

\param [in]   pStr1            the start address of the first string to be compared
\param [in]   pStr2            the start address of the second string to be compared
\param [in]   startBit         the start bit number in the the 2 strings
\param [in]   stopBit          the stop bit number in the the 2 strings

\retval       bool_t           TRUE - if the strings match
                               FALSE - if the strings don't match
***************************************************************************************************/
bool_t NWKU_BitCmp(uint8_t *pStr1, uint8_t *pStr2, uint8_t startBit, uint8_t stopBit);

/*!*************************************************************************************************
\fn  bool_t NWKU_IsLLAddrEqual(uint8_t *pFirstLlAddr, uint32_t firstLlAddrSize,
                               uint8_t *pSecondLlAddr,uint32_t secondLlAddrSize)
\brief  Compare two Link Layer addresses

\param [in]   pFirstLlAddr      the start address of the first address to be compared
\param [in]   firstLlAddrSize   the size of the first address to be compared
\param [in]   pSecondLlAddr     the start address of the second address to be compared
\param [in]   secondLlAddrSize  the size of the second address to be compared

\retval       bool_t           TRUE - if the Link Layer addresses are the same
                               FALSE - if the Link Layer addresses are different
***************************************************************************************************/
bool_t NWKU_IsLLAddrEqual(uint8_t *pFirstLlAddr, uint32_t firstLlAddrSize, uint8_t *pSecondLlAddr,
                                    uint32_t secondLlAddrSize);

/*!*************************************************************************************************
\fn uint32_t NWKU_GetCommonPrefixLen6(ipAddr_t *addr1, ipAddr_t*addr2)
\brief The common prefix length CommonPrefixLen(A, B) of two addresses A and B is the length of
       the longest prefix (looking at the most significant, or leftmost, bits) that the two
       addresses have in common.

\param [in]  addr1      first prefix to compare
\param [in]  addr2      second prefix to compare

\return      uin32_t    longhet prefix lenght in bits (0 - 128)
 ***************************************************************************************************/
uint32_t NWKU_GetCommonPrefixLen6(ipAddr_t *addr1, ipAddr_t*addr2);

/*!*************************************************************************************************
\fn    uint64_t NWKU_TransformArrayToValue(uint8_t* pArray, uint32_t nbOfBytes)
\brief  Converts an array to a numeric value.

\param [in]   pArray          the start address of the array
\param [in]   nbOfBytes       the length of the data to be converted

\retval       uint32_t        the value converted from the array
 ***************************************************************************************************/
uint64_t NWKU_TransformArrayToValue(uint8_t* pArray, uint32_t nbOfBytes);

/*!*************************************************************************************************
\fn    void NWKU_TransformValueToArray(uint64_t value, uint8_t* pArray, uint32_t nbOfBytes)
\brief  Converts a numeric value to array.

\param [in]    value            the value to be converted
\param [out]   pArray           the start address of the array
\param [in]    nbOfBytes        the length of the data to be converted

\retval        none
 ***************************************************************************************************/
void NWKU_TransformValueToArray(uint64_t value, uint8_t* pArray, uint32_t nbOfBytes);

/*!*************************************************************************************************
\fn    uint16_t NWKU_Revert16(uint16_t value)
\brief  Reverts a 16 bit numeric value.

\param [in]    value            the value to be converted

\retval        uint16_t         the converted value
 ***************************************************************************************************/
uint16_t NWKU_Revert16(uint16_t value);

/*!*************************************************************************************************
\fn    uint32_t NWKU_Revert32(uint32_t value)
\brief  Reverts a 32 bit numeric value.

\param [in]    value            the value to be converted

\retval        uint32_t         the converted value
 ***************************************************************************************************/
uint32_t NWKU_Revert32(uint32_t value);

/*!*************************************************************************************************
\fn    uint64_t NWKU_Revert64(uint64_t value)
\brief  Reverts a 64 bit numeric value.

\param [in]    value            the value to be converted

\retval        uint64_t         the converted value
 ***************************************************************************************************/
uint64_t NWKU_Revert64(uint64_t value);

/*!*************************************************************************************************
\fn    uint16_t NWKU_TransformArrayToUint16(uint8_t* pArray)
\brief  Converts an big endian array to a 16 bit numeric value.

\param [in]    pArray           the start address of the array

\retval        uint16_t         the converted value
 ***************************************************************************************************/
uint16_t NWKU_TransformArrayToUint16(uint8_t* pArray);

/*!*************************************************************************************************
\fn    uint32_t NWKU_TransformArrayToUint32(uint8_t* pArray)
\brief  Converts an big endian array to a 32 bit numeric value.

\param [in]    pArray           the start address of the array

\retval        uint32_t         the converted value
 ***************************************************************************************************/
uint32_t NWKU_TransformArrayToUint32(uint8_t* pArray);

/*!*************************************************************************************************
\fn    uint64_t NWKU_TransformArrayToUint64(uint8_t* pArray)
\brief  Converts an big endian array to a 64 bit numeric value.

\param [in]    pArray           the start address of the array

\retval        uint64_t         the converted value
 ***************************************************************************************************/
uint64_t NWKU_TransformArrayToUint64(uint8_t* pArray);

/*!*************************************************************************************************
\fn    void NWKU_TransformUint16ToArray(uint8_t* pArray, uint16_t value)
\brief  Converts a 16 bit numeric value to array.

\param [in]    value            the value to be converted
\param [out]   pArray           the start address of the array

\retval        none
 ***************************************************************************************************/
void NWKU_TransformUint16ToArray(uint8_t* pArray, uint16_t value);

/*!*************************************************************************************************
\fn    void NWKU_TransformUint32ToArray(uint8_t* pArray, uint32_t value)
\brief  Converts a 32 bit numeric value to array.

\param [in]    value            the value to be converted
\param [out]   pArray           the start address of the array

\retval        none
 ***************************************************************************************************/
void NWKU_TransformUint32ToArray(uint8_t* pArray, uint32_t value);

/*!*************************************************************************************************
\fn    void NWKU_TransformUint64ToArray(uint8_t* pArray, uint64_t value)
\brief  Converts a 64 bit numeric value to array.

\param [in]    value            the value to be converted
\param [out]   pArray           the start address of the array

\retval        none
 ***************************************************************************************************/
void NWKU_TransformUint64ToArray(uint8_t* pArray, uint64_t value);

/*!*************************************************************************************************
\fn    bool_t NWKU_GetLut8(lut8_t* pLutTable, uint8_t lutTableSize, uint8_t type,
                           uint8_t* pEntryIndex)
\brief  Searches an entry in the lookup table indicated by pLutTable.

\param [in]    pLutTable     pointer to the lookup table
\param [in]    lutTableSize  lookup table size
\param [in]    type          type to find

\param [out]   pEntryIndex   index of the entry in case the entry is found

\retval        TRUE          returned when the entry is found
\retval        FALSE         otherwise
 ***************************************************************************************************/
bool_t NWKU_GetLut8(lut8_t* pLutTable, uint8_t lutTableSize, uint8_t type, uint8_t* pEntryIndex);

/*!*************************************************************************************************
\fn    int32_t pton(uint8_t af, char *pTxt, ipAddr_t *pIpAddr)
\brief  Converts a string into an ipAddr_t. Presentation to network function.

\param [in]    af       address family(AF_INET, AF_INET6)
\param [in]    pTxt     pointer to the start of the string to be parsed
\param [in]    pIpAddr  pointer to the start of the allocated ipAddr_t structure

\retval     1 on success
            0 string address is not valid
            -1 on error
***************************************************************************************************/
int32_t pton(uint8_t af, char *pTxt, ipAddr_t *pIpAddr);

/*!*************************************************************************************************
\fn    char *ntop(uint8_t af, ipAddr_t *pIpAddr, char *pStr, uint32_t strLen)
\brief  Converts an ipAddr_t into a string. Network to presentation function.

\param [in]    af       address family(AF_INET, AF_INET6)
\param [in]    pIpAddr  pointer to the start of the allocated ipAddr_t structure
\param [out]   pStr     pointer to the allocated string where to put the result
\param [in]    strLen   size of the input buffer

\retval        char*    pointer to the resulted buffer
***************************************************************************************************/
char *ntop(uint8_t af, ipAddr_t *pIpAddr, char *pStr, uint32_t strLen);

/*!*************************************************************************************************
\fn    uint32_t NWKU_TmrRtcGetElapsedTimeInSeconds(uint32_t timestamp)
\brief Calculates the time passed in seconds from the provided timestamp.

\param [in]    timestamp     timestamp in seconds

\return        uint32_t      number of seconds that have passed since the provided timestamp
 ***************************************************************************************************/
uint32_t NWKU_TmrRtcGetElapsedTimeInSeconds(uint32_t timestamp);

/*!*************************************************************************************************
\fn    bool_t NWKU_IsNUmber(char *pString)
\brief Check if a string is a number.

\param [in]    pString      pointer to the string

\return        bool_t       TRUE if the string represents a number
                            FALSE if the string does not represent a number
***************************************************************************************************/
bool_t NWKU_IsNUmber(char *pString);

/*!*************************************************************************************************
\fn    uint32_t NWKU_GetRandomNoFromInterval(uint32_t startInterval, uint32_t endInterval)
\brief This function returns a random number from a given interval.

\param    [in]  startInterval   Start value of the interval
\param    [in]  endInterval     End value of the interval

\retval   uint32_t - random value
***************************************************************************************************/
uint32_t NWKU_GetRandomNoFromInterval(uint32_t startInterval, uint32_t endInterval);
/*!*************************************************************************************************
\fn     void NWKU_IncrementIp6Addr(ipAddr_t* pIpAddr)
\brief  This function increments a IPv6 type address

\param    [in]  pIpAddr   pointer to IPv6 address

\retval   none
***************************************************************************************************/
void NWKU_IncrementIp6Addr(ipAddr_t* pIpAddr);

/*!*************************************************************************************************
\fn     uint32_t NWKU_RightRotate(uint32_t val, uint8_t amount)
\brief  This function a 32bit number to the right with an amount of bits.

\param    [in]  val     number
\param    [in]  amount  number of bits to rotate

\retval   none
***************************************************************************************************/
uint32_t NWKU_RightRotate(uint32_t val, uint8_t amount);

/*!*************************************************************************************************
\fn     void NWKU_GetIIDFromLLADDR(llAddr_t * pLlAddr, uint16_t panId, uint8_t * pIID)
\brief  The function returns the IID from a Link-Layer address.

\param  [in]    pLlAddr         Pointer to the Link-Layer address
\param  [in]    panId           PAN ID
\param  [out]   pIID            Pointer to the variable which will hold the IID
***************************************************************************************************/
void NWKU_GetIIDFromLLADDR(llAddr_t* llAddr, uint16_t panId, uint8_t * pIID);

/*!*************************************************************************************************
\fn     void NWKU_GetLLADDRFromIID(uint8_t * pIID, llAddr_t * pLlAddr)
\brief  This function returns the Link-Layer address from the IID.

\param  [in]    pIID            Pointer to the IID
\param  [out]   pLlAddr         Pointer to the variable which will hold the Link-Layer address
***************************************************************************************************/
void NWKU_GetLLADDRFromIID(uint8_t * pIID, llAddr_t * pLlAddr);

/*!*************************************************************************************************
\fn     bool_t NWKU_GetBit(uint32_t bitNr, uint8_t* pArray)
\brief  This function returns the value of a bit in an array.

\param  [in]    bitNr           bit number in the whole array
\param  [in]    pArray          pointer to the start of the array

\retval         TRUE            if the bit is set
\retval         FALSE           if the bit is not set
***************************************************************************************************/
bool_t NWKU_GetBit(uint32_t bitNr, uint8_t* pArray);

/*!*************************************************************************************************
\fn     void NWKU_SetBit(uint32_t bitNr, uint8_t* pArray)
\brief  This function sets a bit in an array.

\param  [in]    bitNr           bit number in the whole array
\param  [in]    pArray          pointer to the start of the array
***************************************************************************************************/
void NWKU_SetBit(uint32_t bitNr, uint8_t* pArray);

/*!*************************************************************************************************
\fn     void NWKU_ClearBit(uint32_t bitNr, uint8_t* pArray)
\brief  This function clears a bit in an array.

\param  [in]    bitNr           bit number in the whole array
\param  [in]    pArray          pointer to the start of the array
***************************************************************************************************/
void NWKU_ClearBit(uint32_t bitNr, uint8_t* pArray);

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetFirstBitValueInRange(uint8_t* pArray, uint32_t lowBitNr, uint32_t
        highBitNr, bool_t bitValue)
\brief  This function returns the first bit with value=bitValue in a range in the array.

\param  [in]    pArray          pointer to the start of the array
\param  [in]    lowBitNr        starting bit number
\param  [in]    highBitNr       ending bit number
\param  [in]    bitValue        bit value

\retval         uint32_t        bit number
***************************************************************************************************/
uint32_t NWKU_GetFirstBitValueInRange(uint8_t* pArray, uint32_t lowBitNr, uint32_t highBitNr, bool_t bitValue);

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetFirstBitValue(uint8_t* pArray, uint32_t arrayBytes, bool_t bitValue)
\brief  This function returns the index of the first bit with value=bitValue.

\param  [in]    pArray          pointer to the start of the array
\param  [in]    arrayBytes      number of bytes in the array
\param  [in]    bitValue        bit value

\retval         uint32_t        bit value
***************************************************************************************************/
uint32_t NWKU_GetFirstBitValue(uint8_t* pArray, uint32_t arrayBytes, bool_t bitValue);

/*!*************************************************************************************************
\fn     uint32_t FSCI_AddTblEntry(uint32_t entry, uint32_t *pTable, uint32_t tableSize)
\brief  This function adds a new entry in a table. The table needs to have uint32_t elements.

\param  [in]    entry       entry value
\param  [in]    pTable      pointer to the start of the table
\param  [in]    tableSize   the size of the table

\return         entry index or -1(0xFFFFFFFF) in case of error
***************************************************************************************************/
uint32_t NWKU_AddTblEntry(uint32_t entry, uint32_t *pTable, uint32_t tableSize);

/*!*************************************************************************************************
\fn     uint32_t FSCI_GetTblEntry(uint32_t entry, uint32_t *pTable, uint32_t tableSize)
\brief  This function search for an element in a table.

\param  [in]    entry       entry value
\param  [in]    pTable      pointer to the start of the table
\param  [in]    tableSize   the size of the table

\return         entry index or NULL in case of error
***************************************************************************************************/
uint32_t NWKU_GetTblEntry(uint32_t index, uint32_t *pTable, uint32_t tableSize);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _NETWORK_UTILS_H */
