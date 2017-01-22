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

#ifndef _UDP_H
#define _UDP_H
/*!=================================================================================================
\file       udp.h
\brief      This is a header file for the UDP implementation.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include <stdlib.h>
#include "ip.h"
#include "udp_cfg.h"

/* UDP Debug purposes */
#if UDP_DEBUG
//#include "IO_Map.h "
#endif /* TCP_DEBUG */

/*==================================================================================================
Public macros
==================================================================================================*/

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef enum
{
    gUdpStatusCallOK_c          = 0,    /*!< UDP OK */
    gUdpStatusCallError_c       = 1,    /*!< UDP connection error */
    gUdpStatusNoFreePorts_c     = 2,    /*!< No more free ports */
    gUdpStatusPortInUse_c       = 3,    /*!< Port is used by another connection */

} udpStatus_t;

typedef enum udpConnFlags_tag
{
    gUdpConnFlagReusePort_c = 0x01      /*!< Reuse port */
}udpConnFlags_t;


typedef struct udpHeader_tag
{
    uint16_t    srcPort;                /*!< Source port number */
    uint16_t    dstPort;                /*!< Destination port number */
    uint16_t    length;                 /*!< UDP header + UDP data */
    uint16_t    checksum;               /*!< Checksum */
} udpHeader_t; /*!< UDP header */

typedef struct udpPacket_tag
{
    ipAddr_t        srcIpAddr;          /*!< Source IP address */
    ipAddr_t        remIpAddr;          /*!< Remote IP address */
    uint8_t         *udpPayload;        /*!< The UDP data buffer */
    uint8_t         *pStartPayload;     /*!< Pointer in the UDP data buffer which is used if
                                             multiple reads are required for getting this packet */
    uint16_t        dataSize;           /*!< The length of the UDP data buffer(w/o the UDP header) */
    uint16_t        remPort;            /*!< Remote port number */
    ipPktOptions_t  *pIpPacketOptions;  /*!< Pointer to IP packet options */
} udpPacket_t; /*!< UDP data including payload */

typedef struct udpConn_tag
{
    ipAddr_t        localIPAddr;
    ipAddr_t        remIPAddr;
    list_t          rxList;             /*!< receive queue */
    uint16_t        localPort;          /*!< Local port number */
    uint16_t        remPort;            /*!< Remote port number */
    tspDataIndCb_t  pUdpDataIndCb;      /*!< function pointer to be called when we have RX data */
    osaEventId_t    pUdpEventId;        /*!< Pointer to the event ID to signal */
    uint8_t         flags;              /*!< Flags (udpConnFlags_t) */
    uint8_t         hopLimit;           /*!< Hop limit for IPv6 or TTL for IPv4 */
} udpConn_t; /*!< Structure containing one UDP connection data */
/* 52 + 4 bytes */

typedef struct udpMsgData_tag
{
    ipAddr_t localIPAddr;               /*!< local IP address */
    ipAddr_t remIPAddr;                 /*!< remote IP address */
    void *pInterface;                   /*!< Pointer to be freed at the end of each function */
    uint16_t localPort;                 /*!< local port */
    uint16_t remPort;                   /*!< remote port */

    nwkBuffer_t *pNwkBuff;              /*!< pointer to network buffer */
    udpPacket_t *pUdpPacket;            /*!< pointer to udp packet in the RX queue */

    udpConn_t *pUdpConn;                /*!< the index of the udp connection */
    uint8_t flags;                      /*!< various flags */
    uint8_t status;                     /*!< status for the function call */
    uint8_t security;                   /*!< MAC security level */
    uint8_t extraParam;                 /*!< parameter for future use */
}udpMsgData_t; /*!< Structure used for message exchange between Upper Layers and Transport Layer */

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
\fn     void UDP_Init(taskMsgQueue_t *pTranspMsgQueue)
\brief  This function initialize the UDP module.

\param  [in]    pTranspMsgQueue pointer to the transport task message queue

\return void
***************************************************************************************************/
void UDP_Init(taskMsgQueue_t *pTranspMsgQueue);

/*!*************************************************************************************************
\fn     udpConn_t *UDP_Open(void)
\brief  This is function is used to open a new UDP connection.

\retval udpConn_t   pointer to the new UDP connection
\retval NULL        if there is no more memory for a new connection
***************************************************************************************************/
udpConn_t *UDP_Open(void);

/*!*************************************************************************************************
\fn     void UDP_Close(udpConn_t *pUdpConn)
\brief  This function is used to close an UDP connection.

\param  [in]    pUdpConn    pointer to the UDP connection

\return void
***************************************************************************************************/
void UDP_Close(udpConn_t *pUdpConn);

/*!*************************************************************************************************
\fn     udpStatus_t UDP_Bind(udpConn_t *pUdpConn, ipAddr_t *pIPAddr, uint16_t localPort)
\brief  This function is used to bind a local IP and port combination to an opened UDP connection.

\param  [in]    pUdpConn    pointer to the UDP connection
\param  [in]    pIPAddr     pointer to the local ip address
\param  [in]    localPort   local port number

\retval         mUdpCallError   if the open call failed
\retval         mUdpCallOk      if the open call succeeded
***************************************************************************************************/
udpStatus_t UDP_Bind(udpConn_t *pUdpConn, ipAddr_t *pIPAddr, uint16_t localPort);

/*!*************************************************************************************************
\fn     uint32_t UDP_Send(udpMsgData_t *pUdpParams)
\brief  This function is used to Send data over an opened UDP connection.

\param  [in]    pUdpParams          pointer to the UDP parameters structure

\return         uint32_t            the size of the data sent
***************************************************************************************************/
uint32_t UDP_Send(udpMsgData_t *pUdpParams);

/*!*************************************************************************************************
\fn     udpPacket_t *UDP_Receive(udpConn_t *pUdpConn)
\brief  This function is used to get data from the RX queue for a specific UDP connection.

\param [in]   pUdpConn  pointer to the UDP connection

\return       udpPacket pointer to an udpPacket_t structure containing zero length data if there is
                        nothing in the RX queue or the last data in the RX queue
***************************************************************************************************/
udpPacket_t *UDP_Receive(udpConn_t *pUdpConn);

/*!*************************************************************************************************
\fn    bool_t UDP_HasActivity(udpConn_t *pUdpConn)
\brief This function is used to find if a connection has any data in the RX queue.

\param [in] pUdpConn    pointer to the UDP connection

\retval     TRUE        there is data in this connection's RX queue
\retval     FALSE       there is no data in this connection's RX queue
***************************************************************************************************/
bool_t UDP_HasActivity
(
    udpConn_t *pUdpConn
);

/*!*************************************************************************************************
\fn     void UDP_RegisterDataIndCb(udpConn_t *pUdpConn, tspDataIndCb_t pDataIndCb, osaEventId_t
        eventId)
\brief  This function is used to register a callback to be called by each layer after any data was
        received.

\param  [in]    pUdpConn    pointer to the UDP connection
\param  [in]    pDataIndCb  pointer to the callback to be registered
\param  [in]    eventId     id of the event which will be waiting

\return         void
***************************************************************************************************/
void UDP_RegisterDataIndCb(udpConn_t *pUdpConn, tspDataIndCb_t pDataIndCb, osaEventId_t threadId);


/*!*************************************************************************************************
\fn     uint32_t UDP_Connect(udpConn_t *pUdpConn, ipAddr_t *pDstIPAddr, uint16_t dstPort)
\brief  This function is used to set the remote peer.

\param  [in]    pUdpConn    pointer to the UDP connection
\param  [in]    pDstIPAddr  pointer to the destination IP address
\param  [in]    dstPort     destination port number

\retval         mUdpCallOK  if the call succeeded
***************************************************************************************************/
udpStatus_t UDP_Connect(udpConn_t *pUdpConn, ipAddr_t *pDstIPAddr, uint16_t dstPort);

/*!*************************************************************************************************
\fn     void UDP_HandlePacket(ipPktInfo_t *pIpPktInfo, uint16_t udpSrcPort, uint16_t udpDstPort,
        uint8_t *pData, uint16_t udpLength)
\brief  This function handles UDP packets. This can be called from IP->UDP_DataIndHandler or from
        other protocol if there is an encapsulated UDP payload.

\param  [in]    pIpPktInfo  pointer to a Packet Info structure
\param  [in]    udpSrcPort  source port for this packet(in host byte order)
\param  [in]    udpDstPort  destination port for this packet(in host byte order)
\param  [in]    pData       pointer to the UDP application data
\param  [in]    udpLength   size of the UDP application data

\return         void
***************************************************************************************************/
void UDP_HandlePacket(ipPktInfo_t *pIpPktInfo, uint16_t udpSrcPort, uint16_t udpDstPort,
    uint8_t *pData, uint16_t udpLength);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _UDP_H */
