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


#ifndef _ICMP_H
#define _ICMP_H
/*!=================================================================================================
\file       icmp.h
\brief      This is a header file for the ICMP (ICMPv6 and ICMPv4) module. It contains the declarations of
            the interface functions.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "icmp_cfg.h"
#include "ip_cfg.h"
#include "icmp6.h"
#include "icmp4.h"

#include "network_utils.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#define ICMP_CODE_DEFAULT 0U

#define ICMP_CODE_ANY 0xFFU

#if defined(__IAR_SYSTEMS_ICC__)
#pragma section="ICMP_PROT_HANDLERS"
#endif

#define SET_HANDLER_STRUCT_NAME_CONCAT(structName, moduleName, line) g##structName##_##moduleName##_##line

#define SET_HANDLER_STRUCT_NAME(structName, moduleName, line) SET_HANDLER_STRUCT_NAME_CONCAT(structName, moduleName, line)

#if defined(__IAR_SYSTEMS_ICC__)

#define IMCP_RegisterHandler(moduleName, protocol, pIfHandle, icmpProtMsgTypeHandlerSize, pIcmpProtMsgTypeHandler) \
    _Pragma("location=\"ICMP_PROT_HANDLERS\"") __root \
    const icmpMsgTypeHandler_t \
    SET_HANDLER_STRUCT_NAME(IcmpHandlerEntry, moduleName, __LINE__) \
    = { pIfHandle, pIcmpProtMsgTypeHandler, protocol, icmpProtMsgTypeHandlerSize }

#elif defined(__GNUC__)

#define IMCP_RegisterHandler(moduleName, protocol, pIfHandle, icmpProtMsgTypeHandlerSize, pIcmpProtMsgTypeHandler) \
    const icmpMsgTypeHandler_t \
    SET_HANDLER_STRUCT_NAME(IcmpHandlerEntry, moduleName, __LINE__) \
    __attribute__ ((section ("ICMP_PROT_HANDLERS"), used)) \
    = { pIfHandle, pIcmpProtMsgTypeHandler, protocol, icmpProtMsgTypeHandlerSize }

#else

#define IMCP_RegisterHandler(moduleName, protocol, pIfHandle, icmpProtMsgTypeHandlerSize, pIcmpProtMsgTypeHandler) \
    const icmpMsgTypeHandler_t \
    SET_HANDLER_STRUCT_NAME(IcmpHandlerEntry, moduleName, __LINE__) \
    = { pIfHandle, pIcmpProtMsgTypeHandler, protocol, icmpProtMsgTypeHandlerSize }

#endif

/*==================================================================================================
Public type definitions
==================================================================================================*/
/*! Description of the receive callback  */
typedef void (*icmpMessageTypeHandlerCb_t)(ipPktInfo_t* pIpPktInfo);

/*! Description of the statistics structure  */
typedef struct icmpStats_tag
{
    uint32_t count;
    uint8_t  type;
}icmpStats_t;

/*! Description of the structure used for registering a receive callback(s) */
typedef struct icmpProtMsgTypeHandler_tag
{
    icmpMessageTypeHandlerCb_t icmpMessageTypeHandlerCb;    /*!< Pointer to callback */
    uint8_t type;                                           /*!< Any value accepted */
    uint8_t code;                                           /*!< Use ICMP_CODE_ANY to accept all codes */
}icmpProtMsgTypeHandler_t;

typedef struct icmpMsgTypeHandler_tag
{
    void*   pIfHandle;
    icmpProtMsgTypeHandler_t* pIcmpProtMsgTypeHandler;
    uint8_t protocol;
    uint8_t icmpProtMsgTypeHandlerSize;
    uint8_t padding[2];
}icmpMsgTypeHandler_t;

typedef enum
{
    gIcmpStatsTypeBadChecksum_c = 250U,
    gIcmpStatsTypeTotalSuccess_c = 251U,
    gIcmpStatsTypeTotalFailed_c = 252U,
#if IP_IP6_ENABLE
    gIcmpStatsTypeDestinationUnreachable6_c = gIcmp6TypeDestinationUnreachable_c,
    gIcmpStatsTypePacketTooBig6_c = gIcmp6TypePacketTooBig_c,
    gIcmpStatsTypeTimeExceeded6_c = gIcmp6TypeTimeExceeded_c,
    gIcmpStatsTypeParameterProblem6_c = gIcmp6TypeParameterProblem_c,
    gIcmpStatsTypeEchoRequest6_c = gIcmp6TypeEchoRequest_c,
    gIcmpStatsTypeEchoReply6_c = gIcmp6TypeEchoReply_c,
#endif /* IP_IP6_ENABLE */

#if IP_IP4_ENABLE
    gIcmpStatsTypeDestinationUnreachable4_c = gIcmp4TypeDestinationUnreachable_c,
    gIcmpStatsTypeTimeExceeded4_c = gIcmp4TypeTimeExceeded_c,
    gIcmpStatsTypeParameterProblem4_c = gIcmp4TypeParameterProblem_c,
    gIcmpStatsTypeEchoRequest4_c = gIcmp4TypeEchoRequest_c,
    gIcmpStatsTypeEchoReply4_c = gIcmp4TypeEchoReply_c,
    gIcmpStatsTypeRedirect4_c = gIcmp4TypeRedirect_c,
    gIcmpStatsTypeTimestampReply4_c = gIcmp4TypeTimestampReply_c,
#endif /* IP_IP4_ENABLE */
}icmpStatsType_t;

typedef enum
{
    gIcmpStatsDirReceive_c = 0U,
    gIcmpStatsDirTransmit_c,
}icmpStatsDirection_t;

typedef enum
{
    gIcmpStatsProtIcmp6_c = 58U,
    gIcmpStatsProtIcmp4_c = 1U,
}icmpStatsProtocol_t;

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
\fn     void ICMP_Init(taskMsgQueue_t *pTaskMsgQueue)
\brief  This function initializes the ICMP module (e.g. resets the statistics)

\param [in]   pTaskMsgQueue     Pointer to the task message queue.

\retval       none
***************************************************************************************************/
void ICMP_Init(taskMsgQueue_t *pTaskMsgQueue);


#if ICMP_UNREGISTER_MSG_TYPE_HANDLER_ENABLED

/*!*************************************************************************************************
\fn     bool_t ICMP_UnregisterMsgTypeHandler(uint8_t protocol, void* pIfHandle,
                                             icmpProtMsgTypeHandler_t* pIcmpProtMsgTypeHandler)
\brief  This function unregisters all callback handlers corresponding to the "protocol" parameter and to
        "pIcmpProtMsgTypeHandler" parameter

\param [in]   protocol                  protocol number as assigned by IANA (e.g. IPPROTO_ICMPV6,
                                        IPPROTO_ICMP, IPPROTO_UDP, IPPROTO_TCP)
\param [in]   pIfHandle                 pointer to the media interface handler (if known) or NULL.
                                        When set to NULL this callback is unregistered on all interfaces
\param [in]   pIcmpProtMsgTypeHandler   pointer to a table containing one or more ICMP message type,
                                        ICMP message code and icmpMessageTypeHandlerCb_t entries

\retval       TRUE   if the callback is unregistered
\retval       FALSE  otherwise
***************************************************************************************************/
bool_t ICMP_UnregisterMsgTypeHandler(uint8_t protocol, void* pIfHandle,
                                     icmpProtMsgTypeHandler_t* pIcmpProtMsgTypeHandler);

#endif /* ICMP_UNREGISTER_MSG_TYPE_HANDLER_ENABLED */

/*!*************************************************************************************************
\fn     void ICMP_Receive(ipPktInfo_t* pIpPktInfo)
\brief  This function is called by IP layer whenever an ICMP packet is received

\param [in]   pIpPktInfo  pointer to received IP packet and additional IP and link layer information.
                          pNextProt represents the start address of the ICMP header.
                          If the pIpPktInfo pointer or its pIpDstAddr, pIpSrcAddr, pIpPktOptions
                          and pNwkBuff fields are not valid then the message is not processed.
                          This parameter is freed before returning from this function.

\retval       none
***************************************************************************************************/
void ICMP_Receive(ipPktInfo_t* pIpPktInfo);

/*!*************************************************************************************************
\fn     void ICMP_SendError(ipPktInfo_t** ppErrorIpPktInfo, uint8_t type, uint8_t code, uint32_t param)
\brief  This function is called for sending an ICMP error packet to IP transmission module

\param [in]   ppErrorIpPktInfo  double pointer to the received incorrect IP packet and additional IP and link
                          layer information. If the incorrect IP packet must not generate an error
                          then the error is not be sent.
                          pErrorIpPktInfo->pNwkBuff->pData must point to the IP header
                          This parameter will be freed in this function.
\param [in]   type        ICMP error message type
\param [in]   code        ICMP error message code
\param [in]   param       ICMP error parameter (the first 4 bytes after the ICMP header)

\retval       none
***************************************************************************************************/
void ICMP_SendError(ipPktInfo_t** ppErrorIpPktInfo, uint8_t type, uint8_t code, uint32_t param);

/*!*************************************************************************************************
\fn     void ICMP_Send(ipPktInfo_t* pIpPktInfo, uint8_t type, uint8_t code)
\brief  This function is called for sending a generic ICMP packet to IP transmission module.

\param [in]   pIpPktInfo  pointer to the IP packet and additional IP and link layer information
                          that will be sent to IP layer
                          If the pIpPktInfo pointer or its pIpDstAddr, pIpPktOptions and pNwkBuff
                          fields are not valid then the message is not sent and the pIpPktInfo is freed.
                          The pointer to pIpSrcAddrIP source address may be missing from pIpPktInfo.
                          In this case the source address will be determined based on the
                          destination address.
\param [in]   type        ICMP message type
\param [in]   code        ICMP message code

\retval       none
***************************************************************************************************/
void ICMP_Send(ipPktInfo_t* pIpPktInfo, uint8_t type, uint8_t code);

/*!*************************************************************************************************
\fn     uint32_t ICMP_GetStatistics(icmpStatsProtocol_t icmpStatsProtocol,
                              icmpStatsDirection_t icmpStatsDirection, icmpStatsType_t icmpStatsType)
\brief  This function is used for the reading of the ICMP statistics

\param [in]   icmpStatsProtocol  ICMP statistics protocol (ICMPv6 or ICMPv4)
\param [in]   icmpStatsDirection ICMP statistics direction (reception or transmission)
\param [in]   icmpStatsType      ICMP statistics

\retval       number of statistics
***************************************************************************************************/
uint32_t ICMP_GetStatistics(icmpStatsProtocol_t icmpStatsProtocol, icmpStatsDirection_t icmpStatsDirection,
                            icmpStatsType_t icmpStatsType);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _ICMP_H */
