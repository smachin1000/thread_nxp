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

#ifndef _ND_TABLES_H
#define _ND_TABLES_H
/*!=================================================================================================
\file       nd_tables.h
\brief      This is a header file for the Neighbor Discovery tables management module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"

#include "FunctionLib.h"

/* Network Includes */
#include "ip.h"
#include "network_utils.h"

/* Module Includes */
#include "nd_cfg.h"

#if IP_IP6_ENABLE

/*==================================================================================================
Public macros
==================================================================================================*/

#define NC_ROUTER_FLAG_OFFSET               7U
#define NC_ROUTER_FLAG_MASK                 0x80U

#define NC_STATE_OFFSET                     4U
#define NC_STATE_MASK                       0x70U

#define NC_NB_OF_SOLICIT_MASK               0x0FU

#define PL_ON_LINK_FLAG_OFFSET              31U
#define PL_ON_LINK_FLAG_MASK                0x80000000U

#define PL_AUTONOMOUS_FLAG_OFFSET           30U
#define PL_AUTONOMOUS_FLAG_MASK             0x40000000U

#define PL_PREFIX_LEN_FLAG_MASK             0x3FFFFFFFU

#define NC_ROUTER_FLAG_SET(a, b)            ((a) = (((a) & (~NC_ROUTER_FLAG_MASK))   | ((b) << NC_ROUTER_FLAG_OFFSET)))
#define NC_ROUTER_FLAG_GET(a)               (((a) & NC_ROUTER_FLAG_MASK) >> NC_ROUTER_FLAG_OFFSET)

#define NC_STATE_SET(a, b)                  ((a) = (((a) & (~NC_STATE_MASK))   | ((b) << NC_STATE_OFFSET)))
#define NC_STATE_GET(a)                     (((a) & NC_STATE_MASK) >> NC_STATE_OFFSET)

#define NC_NB_OF_SOLICIT_INCREMENT(a)       ((a) = ((a) & (~NC_NB_OF_SOLICIT_MASK)) | ((a + 1) & NC_NB_OF_SOLICIT_MASK))
#define NC_NB_OF_SOLICIT_GET(a)             ((a) & NC_NB_OF_SOLICIT_MASK)
#define NC_NB_OF_SOLICIT_RESET(a)           ((a) &= (~NC_NB_OF_SOLICIT_MASK))

#define PL_ON_LINK_FLAG_SET(a, b)           ((a) = (((a) & (~PL_ON_LINK_FLAG_MASK))   | ((b) << PL_ON_LINK_FLAG_OFFSET)))
#define PL_ON_LINK_FLAG_GET(a)              (((a) & PL_ON_LINK_FLAG_MASK) >> PL_ON_LINK_FLAG_OFFSET)

#define PL_AUTONOMOUS_FLAG_SET(a, b)        ((a) = (((a) & (~PL_AUTONOMOUS_FLAG_MASK))   | ((b) << PL_AUTONOMOUS_FLAG_OFFSET)))
#define PL_AUTONOMOUS_FLAG_GET(a)           (((a) & PL_AUTONOMOUS_FLAG_MASK) >> PL_AUTONOMOUS_FLAG_OFFSET)

#define PL_PREFIX_LENGTH_SET(a, b)          ((a) = (((a) & (~PL_PREFIX_LEN_FLAG_MASK  )) | ((b) & PL_PREFIX_LEN_FLAG_MASK)))
#define PL_PREFIX_LENGTH_GET(a)             ((a) & PL_PREFIX_LEN_FLAG_MASK)

#define ND_LIFETIME_INFINITE_2_BYTES        0xFFFFU

#define ND_LIFETIME_INFINITE_4_BYTES        0xFFFFFFFFU

#define ND_ROUTER_LIFETIME_INFINITE         ND_LIFETIME_INFINITE_2_BYTES

#define ND_CONTEXT_LIFETIME_INFINITE        ND_LIFETIME_INFINITE_4_BYTES

#define ND_LIFETIME_INFINITE                ND_LIFETIME_INFINITE_4_BYTES

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*! Neighbor’s reachability state., based on RFC4861 */
typedef enum nd6NeighborState_tag
{
    gNd6NeighborStateNotUsed_c     = 0U,   /*!< The entry is not used - free.*/
    gNd6NeighborStateIncomplete_c  = 1U,   /*!< Address resolution is in progress and the link-layer
                                            * address of the neighbor has not yet been determined.*/
    gNd6NeighborStateReachable_c   = 2U,   /*!< Roughly speaking, the neighbor is known to have been
                                            * reachable recently (within tens of seconds ago).*/
    gNd6NeighborStateStale_c       = 3U,   /*!< The neighbor is no longer known to be reachable but
                                            * until traffic is sent to the neighbor, no attempt
                                            * should be made to verify its reachability.*/
    gNd6NeighborStateDelay_c       = 4U,   /*!< The neighbor is no longer known to be reachable, and
                                            * traffic has recently been sent to the neighbor.
                                            * Rather than probe the neighbor immediately, however,
                                            * delay sending probes for a short while in order to
                                            * give upper-layer protocols a chance to provide
                                            * reachability confirmation.*/
    gNd6NeighborStateProbe_c       = 5U,   /*!< The neighbor is no longer known to be reachable, and
                                            * unicast Neighbor Solicitation probes are being sent to
                                            * verify reachability.*/
} nd6NeighborState_e;

typedef uint8_t nd6NeighborState_t;

typedef enum ndArState_tag
{
    gNdArStateGarbageCollect_c = 0x0000U,
    gNdArStateTentative_c = 0x0001U,
    gNdArStateRegistered_c = 0x0002U
} ndArState_e;

typedef uint16_t ndArState_t;

/*! Neighbor Cache entry, based on RFC4861 */
typedef struct ndNeighborEntry_tag
{
    ipAddr_t ipAddr;                   /*!< Neighbor’s on-link unicast IP address. */

    ifHandle_t* pIfHandle;             /*!< Pointer to media interface configuration structure */
    ipPktInfo_t *pWaitingPacket;       /*!< Pointer to any queued packet waiting for address resolution to complete.*/
                                       /*!< RFC 4861 7.2.2: While waiting for address resolution to complete, the sender MUST,
                                        * for each neighbor, retain a small queue of packets waiting for
                                        * address resolution to complete. The queue MUST hold at least one
                                        * packet, and MAY contain more.
                                        * When a queue  overflows, the new arrival SHOULD replace the oldest entry.*/

    llAddr_t llAddrInf;                /*!< Its link-layer address and size */

    uint8_t extraParam;                /*!< Bit  [7]    - isRouter          (1 - router, 0 - node)
                                        *   Bits [6-4]  - state             (nd6NeighborState_t)
                                        *   Bits [3-0]  - sentSolicitations (nb. of sent probes) */

    uint16_t dummy;

    /* Default Router list entry info: routerLifetime */
    uint32_t routerExpireTimestampSec; /*!< The lifetime associated
                                        * with the default router in units of seconds. The
                                        * field can contain values up to 65535 and receivers
                                        * should handle any value, while the sending rules in
                                        * Section 6 limit the lifetime to 9000 seconds. A
                                        * Lifetime of 0 indicates that the router is not a
                                        * default router and SHOULD NOT appear on the default router list.
                                        * It is used only if "isRouter" is 1.*/

    uint32_t stateTimestamp;           /*!< Timestamp of the last state event.*/

#if (ND_OPT_AR_RCV_ENABLED | ND_MSG_DAR_RCV_ENABLED)

    ndArState_t aRegState;

    uint16_t regLifetime;

    uint8_t eui64[8];

#endif
} ndNeighborEntry_t, * ndNeighborEntryPtr;

/*! Redirect entry, based on RFC4861 */
typedef struct ndDstCacheEntry_tag
{
    ipAddr_t dstAddr;                    /* Destination Address. The IP address of the destination
                                          * that is redirected to the target. */
    //ifHandle_t pIfHandle;                /*!< Pointer to media interface configuration structure */

    ndNeighborEntry_t* pTargetNeighbor; /* Pointer to the ndNeighborEntry_t entry with the Target
                                          * Address. An IP address that is a better first hop to use
                                          * for the ICMP Destination Address. When the target is the
                                          * actual endpoint of communication, i.e., the
                                          * destinatison is a neighbor, the Target Address field
                                          * MUST contain the same value as the ICMP Destination
                                          * Address field. Otherwise, the target is a better
                                          * first-hop router and the Target Address MUST be the
                                          * router’s link-local address so that hosts can
                                          * uniquely identify routers. */
    uint16_t age;                        /* Age of the entry in seconds */
    uint16_t pmtu;                       /* Path MTU */
} ndDstCacheEntry_t;

/*! Prefix List entry, based on RFC4861 */
typedef struct ndPrefixEntry_tag
{
    ipAddr_t    prefix;             /*!< Prefix of an IP address. */

    ifHandle_t*  pIfHandle;         /*!< Pointer to media interface configuration structure */

    uint32_t    validLifetime;      /*!< Valid Lifetime
                                     * 32-bit unsigned integer. The length of time in
                                     * seconds (relative to the time the packet is sent)
                                     * that the prefix is valid for the purpose of on-link
                                     * determination. A value of all one bits
                                     * (0xffffffff) represents infinity. The Valid
                                     * Lifetime is also used by [ADDRCONF].*/

    uint32_t    prfLifetime;        /*!< Preferred Lifetime
                                     * 32-bit unsigned integer.  The length of time in
                                     * seconds (relative to the time the packet is sent)
                                     * that addresses generated from the prefix via
                                     * stateless address autoconfiguration remain
                                     * preferred [ADDRCONF].  A value of all one bits
                                     * (0xffffffff) represents infinity. */

    uint32_t    extraParam;         /*!< Bit  [31] - onLinkFlag     [1 - Prefix is on link]
                                     *   Bit  [30] - autonomousFlag [1 - Can be used for stateless
                                     *                                   address autoconfiguration]
                                     *   Bits [29-0] - prefixLen    [Prefix length in bits] */
} ndPrefixEntry_t, * nd6PrefixEntryPtr;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/



/*==================================================================================================
Public function prototypes
==================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#if IP_IP6_ENABLE

#if ND_ENABLED

/*!*************************************************************************************************
\fn     void ND_TablesInit(taskMsgQueue_t *pTaskMsgQueue)
\brief  Interface function for the Neighbor Discovery module. It initializes the module.

\param  [in]    pTaskMsgQueue     Pointer to task message queue
***************************************************************************************************/
void ND_TablesInit(taskMsgQueue_t *pTaskMsgQueue);

/*!*************************************************************************************************
\fn     ndNeighborEntry_t ** ND_NeighborCacheGet(ifHandle_t* pIfHandle, ipAddr_t *pIpAddr)
\brief  Searches for and returns a Neighbor Cache entry.

\param  [in]    pIfHandle     Interface handle.
\param  [in]    pIpAddr       Pointer to the neighbor s Ip address.

\return         ndNeighborEntry_t      Pointer to the Neighbor Cache entry if found or NULL.
***************************************************************************************************/
ndNeighborEntry_t ** ND_NeighborCacheGet(ifHandle_t* pIfHandle, ipAddr_t *pIpAddr);
/*!*************************************************************************************************
\fn     ndNeighborEntry_t * ND_NeighborCacheGetByIdx(ifHandle_t* pIfHandle, uint32_t index)
\brief  Searches for and returns a Neighbor Cache entry.

\param  [in]    pIfHandle     Double pointer to Interface handle.
\param  [in]    index         Index in the Neighbor Cache of the entry

\return         ndNeighborEntry_t*      Pointer to the Neighbor Cache entry if found or NULL.
***************************************************************************************************/
ndNeighborEntry_t * ND_NeighborCacheGetByIdx(ifHandle_t* pIfHandle, uint32_t index);

/*!*************************************************************************************************
\fn     ndNeighborEntry_t ** ND_NeighborCacheAdd(ifHandle_t* pIfHandle, ipAddr_t *pIpAddr,
                                                 uint8_t* pLlAddr, uint32_t llAddrSize,
                                                 nd6NeighborState_t state)
\brief  Adds an entry to the Neighbor Cache.

\param  [in]    pIfHandle       Pointer to Interface handle.
\param  [in]    pIpAddr         Pointer to neighbor s Ip address.
\param  [in]    pLlAddr         Pointer to neighbor s Link Layer address.
\param  [in]    llAddrSize      The neighbor s Link Layer address size.
\param  [in]    state           The state of the Neighbor cache entry.

\return ndNeighborEntry_t **    Double pointer to the added Neighbor Entry.
***************************************************************************************************/
ndNeighborEntry_t ** ND_NeighborCacheAdd(ifHandle_t* pIfHandle, ipAddr_t *pIpAddr, uint8_t* pLlAddr,
                                                     uint32_t llAddrSize, nd6NeighborState_t state);

/*!*************************************************************************************************
\fn     ndDstCacheEntry_t * ND_DestCacheGet(ipAddr_t *pDestAddr)
\brief  Searches the Destination Cache for an entry

\param  [in]    pDestAddr       pointer to Ip address to look for

\return ndNeighborEntry_t **    double pointer to the Neighbor Entry if found
                                pointer to NULL entry otherwise
***************************************************************************************************/
ndDstCacheEntry_t * ND_DestCacheGet(ipAddr_t *pDestAddr);
/*!*************************************************************************************************
\fn     ndDstCacheEntry_t * ND_DestCacheGetByIdx(uint32_t destCacheIndex)
\brief  Returns the Destination Cache with the index provided.

\param  [in]    destCacheIndex  index in the Destination Cache table

\return ndNeighborEntry_t **    Double pointer to the Destination Cache Entry if found
                                NULL otherwise
***************************************************************************************************/
ndDstCacheEntry_t * ND_DestCacheGetByIdx(uint32_t destCacheIndex);

/*!*************************************************************************************************
\fn    ndDstCacheEntry_t * ND_DestCacheAdd(ifHandle_t* pIfHandle, ipAddr_t *pDestAddr,
                                           ipAddr_t *pTargetAddr)
\brief  Adds an entry to the Destination Cache.

\param  [in]    pIfHandle       double pointer to Interface handle.
\param  [in]    pDestAddr       pointer to destination Ip address.
\param  [in]    pTargetAddr     pointer to target IP address.

\return ndDstCacheEntry_t**    double pointer to the added Destination Cache Entry.
***************************************************************************************************/
ndDstCacheEntry_t * ND_DestCacheAdd(ifHandle_t* pIfHandle, ipAddr_t *pDestAddr,
                                            ipAddr_t *pTargetAddr);
/*!*************************************************************************************************
\fn     void ND_DestCacheClean(ndNeighborEntry_t *routerNeighborEntry)
\brief  Removes a specified entry from the Destination cache.

\param  [in]    routerNeighborEntry      Pointer to the Destination cache entry.
***************************************************************************************************/
void ND_DestCacheClean(ndNeighborEntry_t *routerNeighborEntry);

/*!*************************************************************************************************
\fn     void ND_NeighborCacheRemove(ndNeighborEntry_t ** ppNdNeighborEntry)
\brief  Removes a specified entry from the Neighbor cache.

\param  [in]    ppNdNeighborEntry      Pointer to the Neighbor cache entry.
***************************************************************************************************/
void ND_NeighborCacheRemove(ndNeighborEntry_t ** ppNdNeighborEntry);

/*!*************************************************************************************************
\fn     ndPrefixEntry_t* ND_PrefixListGet(ifHandle_t* pIfHandle, ipAddr_t *pPrefix,
                                         uint32_t prefixLength, uint32_t * pPrefixIdx)
\brief  This function is used for finding an entry in the prefix list corresponding to the media
        interface pointed by ifHandle and having the prefix equal to prefix indicated by pPrefix

\param  [in]  pIfHandle     double pointer to media interface configuration structure
\param  [in]  pPrefix       pointer to the prefix
\param  [in]  prefixLength  prefix length
\param  [out] pPrefixIdx    prefix index in the prefix table

\retval       double pointer to the prefix entry or NULL if the entry is not found
***************************************************************************************************/
ndPrefixEntry_t ** ND_PrefixListGet(ifHandle_t* pIfHandle, ipAddr_t *pPrefix, uint32_t prefixLength,
                                          uint32_t * pPrefixIdx);

/*!*************************************************************************************************
\fn     bool_t ND_PrefixListAdd(ifHandle_t* pIfHandle, ipAddr_t *pPrefix, uint32_t prefixLen,
                               uint32_t lifetime, uint32_t prfLifetime, uint32_t onLinkFlag,
                               uint32_t autonomousFlag)
\brief  This function is used for adding an entry in the prefix list corresponding to the media
        interface pointed by ifHandle and having the prefix equal to prefix indicated by pPrefix.
        If there are no free entries in the prefix table then the oldest entry corrsponding to this
        media interface will be replaced. If there are no free entries in the table and there are
        no entries corresponding to this media interface then FALSE is returned and the entry is
        not added.

\param  [in]  pIfHandle      double pointer to media interface configuration structure
\param  [in]  pPrefix        pointer to the prefix
\param  [in]  prefixLen      prefix length
\param  [in]  lifetime       prefix valid lifetime
\param  [in]  prfLifetime    prefered lifetime
\param  [in]  onLinkFlag     flag indicating if the prefix is on-link or off-link
\param  [in]  autonomousFlag autonomous adrress autoconfiguration flag

\retval       TRUE          returned when the entry was successfully added in the prefix list
\retval       FALSE         otherwise
***************************************************************************************************/
bool_t ND_PrefixListAdd(ifHandle_t* pIfHandle, ipAddr_t *pPrefix, uint32_t prefixLen,
                               uint32_t lifetime, uint32_t prfLifetime, uint32_t onLinkFlag,
                               uint32_t autonomousFlag);

/*!*************************************************************************************************
\fn     void ND_PrefixListRemove(uint32_t prefixIdx)
\brief  This function is used for removing an entry from the prefix list corresponding to the
        prefix entry pointed by pNd6PrefixEntry. This entry is freed and set to NULL.

\param  [in]  prefixIdx      index of the prefix entry

\retval       none
***************************************************************************************************/
void ND_PrefixListRemove(uint32_t prefixIdx);

/*!*************************************************************************************************
\fn     ndPrefixEntry_t * ND_PrefixListGetByIndex(uint32_t index)
\brief  Counts the total number of prefixes stored per an interface.

\param  [in]    index                   Index in the prefix table.

\return         ndPrefixEntry_t *       Pointer to table entry.
***************************************************************************************************/
ndPrefixEntry_t * ND_PrefixListGetByIndex(uint32_t index);

/*!*************************************************************************************************
\fn   ndPrefixEntry_t* ND_GetLongestMatchingPrefix(uint8_t* pIpAddress, instanceId_t instanceId)
\brief  This function is used for finding the longest prefix that matches the provided IPv6 address

\param [in]   pIpAddress          IPv6 address to match
\param [in]   instanceId          Pointer to media interface configuration structure

\retval       ndPrefixEntry_t*   pointer to the prefix entry or NULL if entry is not found
***************************************************************************************************/
ndPrefixEntry_t * ND_GetLongestMatchingPrefix(uint8_t* pIpAddress, instanceId_t instanceId);

/*!*************************************************************************************************
\fn     bool_t ND_IsFirstHopRouter(ifHandle_t* pIfHandle, ipAddr_t* pRouterIpAddr)
\brief  This function is used for determining if a node having pRouterIpAddr is a router.

\param  [in]  pIfHandle         double pointer to media interface configuration structure
\param  [in]  pRouterIpAddr     pointer to the IPv6 address of the node

\retval       TRUE          returned if the node is router
\retval       FALSE         otherwise
***************************************************************************************************/
bool_t ND_IsFirstHopRouter(ifHandle_t* pIfHandle, ipAddr_t* pRouterIpAddr);

/*!*************************************************************************************************
\fn     void ND_RouterListAdd(ndNeighborEntry_t **ppNeighborEntry, uint32_t lifetime)
\brief  Sets a neighbor in the Neighbor cache as a router.

\param  [in]    ppNeighborEntry  Pointer to the Neighbor cache entry for the Router.
\param  [in]    lifetime         The lifetime of the router.
***************************************************************************************************/
void ND_RouterListAdd(ndNeighborEntry_t **ppNeighborEntry, uint32_t lifetime);

/*!*************************************************************************************************
\fn     void ND_RouterListRemove(ndNeighborEntry_t** pNeighborEntry)
\brief  Removes a specified router from the Router list.

\param  [in]    pNeighborEntry      Entry to be removed.
***************************************************************************************************/
void ND_RouterListRemove(ndNeighborEntry_t **ppNeighborEntry);

/*!*************************************************************************************************
\fn     void ND_IsAddrOnlink(ipAddr_t *pIpAddr, uint32_t pIfHandle)
\brief  Interface function for the Neighbor Discovery module. It checks whether the address is
        on link or not.

\param  [in]    pIpAddr         Pointer to the IP address.
\param  [in]    pIfHandle       double pointer to the interface handle.
***************************************************************************************************/
bool_t ND_IsAddrOnlink(ipAddr_t *pIpAddr, uint32_t pIfHandle);

/*!*************************************************************************************************
\fn     ndNeighborEntry_t** ND_GetDefaultRouter(uint32_t pIfHandle)
\brief  Interface function for the Neighbor Discovery module. It returns the default router for
        an interface.

\param  [in]    pIfHandle     double pointer to the interface handle.
***************************************************************************************************/
ndNeighborEntry_t** ND_GetDefaultRouter(uint32_t pIfHandle);

/*!*************************************************************************************************
\fn     void ND_RedirectAddr(ifHandle_t* pIfHandle, ipAddr_t * pIpDestAddr)
\brief  Interface function for the Neighbor Discovery module. It verifies if the destination
        address must be redirected.

\param  [in]    pIfHandle     double pointer to the interface handle.
\param  [in]    pIpDestAddr   pointer to the IPv6 destination address

\param  [out]   pIpDestAddr   pointer to the IPv6 redirected destination address

\retval         none
***************************************************************************************************/
void ND_RedirectAddr(ifHandle_t* pIfHandle, ipAddr_t * pIpDestAddr);

/*!*************************************************************************************************
\fn     void ND_ManageNeighborCache(uint32_t timerIntervalInSec)
\brief  Manages the Neighbor cache entries.

\param  [in]    timerIntervalInSec      The time interval that passed since the last call of the func.
***************************************************************************************************/
void ND_ManageNeighborCache(uint32_t timerIntervalInSec);

/*!*************************************************************************************************
\fn     void ND_ManagePrefixList(uint32_t timerIntervalInSec)
\brief  Manages the Prefix list entries.

\param  [in]    timerIntervalInSec      The time interval that passed since the last call of the func.
***************************************************************************************************/
void ND_ManagePrefixList(uint32_t timerIntervalInSec);

/*!*************************************************************************************************
\fn     void ND_ManageDestinationCache(uint32_t timerIntervalInSec)
\brief  Manages the Destination Cache entries.

\param  [in]    timerIntervalInSec      The time interval that passed since the last call of the func.
***************************************************************************************************/
void ND_ManageDestinationCache(uint32_t timerIntervalInSec);

/*!*************************************************************************************************
\fn     void ND_ManageContextTable(uint32_t timerIntervalInSec)
\brief  Manages the Context list entries.

\param  [in]    timerIntervalInSec      The time interval that passed since the last call of the func.
***************************************************************************************************/
void ND_ManageContextTable(uint32_t timerIntervalInSec);

/*!*************************************************************************************************
\fn     void ND_ManageDad(ip6IfAddrData_t **ppIp6IfAddrData)
\brief  Manages the Duplicate Address Detection algorithm.

\param  [in]    ppIp6IfAddrData      The Ip address data.
***************************************************************************************************/
void ND_ManageDad(ip6IfAddrData_t **ppIp6IfAddrData);

/*!*************************************************************************************************
\fn     void ND_ManageAddrRegistration(ip6IfAddrData_t *pIp6IfAddrData)
\brief  Manages the periodic Address reregistration.

\param  [in]    pIp6IfAddrData      The Ip address data.
***************************************************************************************************/
void ND_ManageAddrRegistration(ip6IfAddrData_t *pIp6IfAddrData);

/*!*************************************************************************************************
\fn     void ND_ManageIpAddr(uint32_t timerIntervalInSec, ip6IfAddrData_t **ppIp6IfAddrData)
\brief  Manages the Duplicate Address Detection algorithm.

\param  [in]    timerIntervalInSec      The time interval that passed since the last call of the func.
\param  [in]    ppIp6IfAddrData         The Ip address data.
***************************************************************************************************/
void ND_ManageIpAddr(uint32_t timerIntervalInSec, ip6IfAddrData_t **ppIp6IfAddrData);

/*!*************************************************************************************************
\fn     void ND_ManageRouterDisc(void)
\brief  Manages the Router Discovery procedure for all interfaces.
***************************************************************************************************/
void ND_ManageRouterDisc(void);

#endif /* ND_ENABLED */
    
#endif /* IP_IP6_ENABLE */

#ifdef __cplusplus
}
#endif

#endif
/*================================================================================================*/
#endif  /* _ND_TABLES_H */
