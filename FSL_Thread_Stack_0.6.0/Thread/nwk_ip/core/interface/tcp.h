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

#ifndef _TCP_H
#define _TCP_H
/*!=================================================================================================
\file       tcp.h
\brief      This is a header file for the TCP module. It contains the TCP implementation from
            RFC 793.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include <stdlib.h>
#include "ip.h"
#include "tcp_cfg.h"
#include "GenericList.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#define TCP_MAX_SYN_RETRANSM        (6)     /*!< Maximum number of SYN retransmissions */
#define TCP_MAX_DATA_RETRANSM       (12)    /*!< Maximum number of data retransmissions */

#define TCP_DEFAULT_TX_BUFF_SIZE    (536)
#define TCP_DEFAULT_RX_BUFF_SIZE    (536)
#define TCP_DEFAULT_TTL             (255)   /*!< Time to live  */
//#define TCP_MSS_DEFAULT             (536U)   /*!< IPv4 (RFC1122, RFC2581) 576 - 20 - 20 */
#define TCP_MSS_DEFAULT             (1200U) /*!< IPv6 (1280 - 40 - 20) */
#define TCP_WND                     (3000)  /*!< SHould be TCP_MSS_DEFAULT * TCP_MAX_RX_PACKETS */
#define TCP_SND_BUFF                (12 * TCP_MSS_DEFAULT)
#define TCP_SND_Q_SIZE              (40)    /*!< Maximum number of segments to send */

#define TCP_MAX_SEG_LIFETIME        (120000)/*!< Maximum segment lifetime(ms): 2 minutes */

/* TCP timer settings */
#define TCP_FAST_TIMER_PERIOD       (250)   /*!< [ms] */
#define TCP_SLOW_TIMER_PERIOD       (500)   /*!< [ms] */
#define TCP_CONN_EXPIRE_TIMEOUT     (75)    /*!< connection expire timeout[s] */
#define TCP_DELAYED_ACK_IMEOUT      (200)   /*!< [ms]  */
#define TCP_DEFAULT_RTO             (3)     /*!< [s]  */

/* Events for TCP */
#define TCP_TX_DONE                 (0x10U) /* Event to be used for TCP TX blocking */
#define TCP_QUEUE_NOT_FULL          (0x20U)

#define TCP_HEADER_SIZE             (20)    /*!< TCP header size in bytes */
#define TCP_HEADER_SIZE_W           (5)     /*!< TCP header size in 32bit words */

/* Macros for comparing sequence numbers(in modulo 32 math) */
#define TCP_SEQ_LT(a,b)             ((int32_t)((uint32_t)(a) - (uint32_t)(b)) < 0)
#define TCP_SEQ_LEQ(a,b)            ((int32_t)((uint32_t)(a) - (uint32_t)(b)) <= 0)
#define TCP_SEQ_GT(a,b)             ((int32_t)((uint32_t)(a) - (uint32_t)(b)) > 0)
#define TCP_SEQ_GEQ(a,b)            ((int32_t)((uint32_t)(a) - (uint32_t)(b)) >= 0)

#define MIN(a,b)                    (((a) < (b))?(a):(b))

#define TCP_NODELAY                 (1)     /*!< Turn off Nagle's algorithm. */
#define TCP_DONTWAIT                (0x01U) /*!< Blocking TX flag */

#define TCP_DEFAULT_KEEP_IDLE       (7200000U)  /*!< RFC1122 keep Idle time[ms] */

#if TCP_STATS_ENABLE
#   define TCP_STATS_INC(x)         (++x)
#else
#   define TCP_STATS_INC(x)         ((void)x)
#endif

#define TF_ACK_DELAY        ((uint8_t)0x01U)    /*!< Delayed ACK. */
#define TF_ACK_NOW          ((uint8_t)0x02U)    /*!< Immediate ACK. */
#define TF_INFR             ((uint8_t)0x04U)    /*!< In fast recovery. */
#define TF_TIMESTAMP        ((uint8_t)0x08U)    /*!< Timestamp option enabled */
#define TF_RXCLOSED         ((uint8_t)0x10U)    /*!< rx closed by tcp_shutdown */
#define TF_FIN              ((uint8_t)0x20U)    /*!< Connection was closed locally (FIN segment
                                                     enqueued). */
#define TF_NODELAY          ((uint8_t)0x40U)    /*!< Disable Nagle algorithm */
#define TF_NAGLEMEMERR      ((uint8_t)0x80U)    /*!< nagle enabled, memerr, try to output to prevent
                                                     delayed ACK to happen */

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef enum
{
    mTcpOK_g,                   /*!<  */
    mTcpError_g,                /*!<  */

    mTcpErrInsufRes_g,          /*!<  */
    mTcpErrForSockUnspec_g,     /*!<  */
    mTcpErrConnClosing_g,       /*!<  */
    mTcpErrConnClosed_g,        /*!<  */
    mTcpErrConnDoesNotExist_g,  /*!<  */
    mTcpErrConnInUse_g,         /*!<  */
    mTcpErrPortInUse_g,         /*!<  */
    mTcpErrNoFreePorts_g,       /*!<  */
    mTcpErrWrongState_g,        /*!<  */

    mTcpErrIpSend_g,            /*!< Error in calling IP_Send() */
} tcpErrorCode_t;

typedef enum
{
    mTcpStateClosed_g = 0U, /*!< Connection is closed */
    mTcpStateListen_g,      /*!< waiting for a connection request from any remote TCP and port */
    mTcpStateSynSent_g,     /*!< have sent SYN(active open) */
    mTcpStateSynRecvd_g,    /*!< have sent and received SYN, waiting for ACK */
    mTcpStateEstab_g,       /*!< connection is open, data received can be delivered to the user */
    mTcpStateFinWait1_g,    /*!< have closed, FIN was sent, waiting for ACK and FIN */
    mTcpStateFinWait2_g,    /*!< close, waiting for FIN */
    mTcpStateCloseWait_g,   /*!< received FIN, waiting for application close */
    mTcpStateClosing_g,     /*!< close on both peers, waiting for ACK */
    mTcpStateLastAck_g,     /*!< received FIN was closed, waiting for ACK */
    mTcpStateTimeWait_g     /*!< 2*MSL wait state after active close */
}tcpStates_t;

typedef enum
{
    mTcpEmptySegAck,        /*!< Send segment with ACK flag set and no payload */
    mTcpNonEmptySeg         /*!< Send segment containing payload */
}tcpSegType_t;

typedef enum
{
    mTcpOpenPassive_g,      /*!< Open connection in passive mode */
    mTcpOpenActive_g        /*!< Open connection in active mode */
}tcpOpenMode_t;

typedef enum
{
    mTcpFlagFIN_g = 0x01U,
    mTcpFlagSYN_g = 0x02U,
    mTcpFlagRST_g = 0x04U,
    mTcpFlagPSH_g = 0x08U,
    mTcpFlagACK_g = 0x10U,
    mTcpFlagURG_g = 0x20U
}tcpFlags_t; /*!< Flags used for the TCP header */

typedef enum
{
    mTcpConnFlagACKNow_g        = 0x0001U,      /*!< Immediate ACK */
    mTcpConnFlagACKDelayed_g    = 0x0002U,      /*!< Delayed ACK */
    mTcpConnFlagTS              = 0x0004U,      /*!< Time stamp enabled */
    mTcpConnFlagFin_g           = 0x0008U,      /*!< Connection was closed locally(FIN segment
                                                     enqueued) */
    mTcpConnFlagNoDelay_g       = 0x0010U,      /*!< Disable Nagle algorithm */

    mTcpConnFlagTxBlock_g       = 0x0020U,      /*!< TX is blocking */
    mTcpConnFlagSoReuseaddr_g   = 0x0040U,      /*!< SO_REUSEADDR socket option */
    mTcpConnFlagIFR_g           = 0x0080U,      /*!< In fast recovery */

    mTcpConnFlagActive_g        = 0x0100U,      /*!< Active connection(has initiated a connect) */
    mTcpConnFlagRxClosed_g      = 0x0200U,      /*!< Notification that this connection should be
                                                     closed and cleared from the socket's
                                                     structure */

    mTcpConnFlagReset_g         = 0x0400U,      /*!< This connection needs to send a reset segment */
    mTcpConnFlagGetData_g       = 0x0800U,      /*!< Move the data into upper layer after processing the packet */
}tcpConnFlags_t; /*!< Flags used for the connection */

typedef enum
{
    mTcpConnOptMSS_g = 0x01U,           /*!< MSS */
    mTcpConnOptTimestamp_g = 0x02U     /*!< timestamp */
}tcpConnOpt_t; /*!< TCP connection options */

typedef enum tcpOpt_tag
{
    mTcpOptEnd_g = 0x00U,       /*!< End of options */
    mTcpOptNoOp_g = 0x01U,      /*!< No operation/padding */
    mTcpOptMss_g = 0x02U,       /*!< Maximum segment size */
    mTcpOptTimestamp_g = 0x08U, /*!< Timestamp */

}tcpOpt_t; /*!< TCP header options */

/*
    0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |          Source Port          |       Destination Port        |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                        Sequence Number                        |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                    Acknowledgment Number                      |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |  Data |           |U|A|P|R|S|F|                               |
   | Offset| Reserved  |R|C|S|S|Y|I|            Window             |
   |       |           |G|K|H|T|N|N|                               |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |           Checksum            |         Urgent Pointer        |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                    Options                    |    Padding    |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                             data                              |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
typedef struct tcpHeader_tag
{
    uint8_t srcPort[2];        /*!< Source port number */
    uint8_t dstPort[2];        /*!< Destination port number */
    uint8_t seqNumber[4];      /*!< Sequence number */
    uint8_t ackNumber[4];      /*!< Acknowledgment number */
    uint8_t dataAndFlags[2];   /*!< Data offset(4 bits) + Reserved(6 bits) + Control bits(6 bits) */
    uint8_t windowSize[2];     /*!< Window */
    uint8_t checksum[2];       /*!< Checksum */
    uint8_t urgPointer[2];
} tcpHeader_t; /*!< TCP header */

typedef struct tcpPacket_tag
{
    uint8_t         *pPayload;          /*!< The TCP data buffer */
    uint32_t        dataSize;           /*!< The length of the TCP data buffer(w/o the TCP header) */
    ipPktOptions_t  *pIpPacketOptions;  /*!< Pointer to IP packet options */
    struct tcpPacket_tag *next;
} tcpPacket_t; /*!< TCP data including payload */

typedef struct tcpSeg_tag
{
    uint8_t     *payload;           /*!< TCP header + data buffer(SHOULD BE FREED!) */
    uint32_t    dataSize;           /*!< The length of the TCP segment including header and
                                         payload */
    tcpHeader_t *pTcpHdr;           /*!< Pointer to the header(DO NOT FREE THIS!) */
    uint8_t     flags;              /*!< Flags for this segment */
    uint8_t     reserved;           /*!< Used for alignment */
    uint16_t    rTime;              /*!< Retransmission time */
    struct tcpSeg_tag *next;
} tcpSeg_t; /*!< TCP segment representation */


typedef struct tcpConn_tag
{
    ipAddr_t        localIPAddr;    /*!< Local IP address */
    ipAddr_t        remIPAddr;      /*!< Remote IP address */

    list_t          rxList;

    uint16_t        localPort;      /*!< Local port number */
    uint16_t        remPort;        /*!< Remote port number */

    uint32_t        mss;            /*!< Maximum segment size  */
    uint16_t        sndBuffSize;    /*!< Maximum space available for sending[B] */

    /* Fast retransmit/recovery */
    uint8_t         dupAcks;        /*!< count of how many ACKs has been received for the sequence
                                        number in lastack */

    uint8_t         lastTimer;
    uint32_t        tcpTmr;

    void            *pInterface;    /*!< Pointer to the IP interface structure */

    /* Send Variables */
    uint32_t        lastAck;        /*!< SND.UNA The sequence number acknowledged by the last ACK
                                         received */
    uint32_t        sndNxt;         /*!< The next sequence number to be sent */
    uint32_t        sndWnd;         /*!< The receiver's advertised window */
    uint32_t        sndWndMax;      /*!< Send window maximum size */
    uint32_t        sndUp;          /*!< Send urgent pointer */
    uint32_t        sndWl1;         /*!< Segment sequence number used for last window update */
    uint32_t        sndWl2;         /*!< Segment acknowledgment number used for last window update */
    uint32_t        iss;            /*!< Initial send sequence number */
    uint32_t        sndLbb;         /*!< Sequence number of next byte to be buffered(or the sequence
                                            number of the last byte queued for transmission ). */

    /* Receive Variables */
    uint32_t        rcvNxt;         /*!< Next sequence number expected(used when sending ACKs)
                                        It is the left or lower edge of the receive window */
    uint32_t        rcvWnd;         /*!< Receive window */
    //uint32_t        rcvUp;          /*!< Receive urgent pointer */
    uint32_t        rcvAnnRight;    /*!< Announced right edge of the receive window
                                        RCV.NXT + RCV.WND - 1 */
    uint16_t        rcvAnnWnd;      /*!< Receive window to announce */
    /* Congestion control */
#if TCP_CONGESTION
    uint16_t        cWnd;           /*!< The current congestion window[packets] */
    uint16_t        ssThreshold;    /*!< The slow start threshold*/
#endif /* TCP_CONGESTION */

    uint16_t        acked;          /*!< Number of acknowledged bytes */
    uint32_t        lastTimeStamp;  /*!< Recent timestamp */

    /* Retransmission timeout variables for Karn and Jacobson algorithm(max value is 2*MSL=240s) */
    uint32_t        rtSeqNb;        /*!< Sequence number for RTT estimation */
    int32_t         rttAvg;         /*!< RTT average */
    int32_t         rttVar;         /*!< RTT variance */
    uint32_t        rto;            /*!< Retransmission timeout[s] */
    uint16_t        rtt;            /*!< Round trip time estimation[s] */
    uint8_t         nbRetransm;     /*!< Number of retransmissions */
    int32_t         retransmTime;   /*!< Retransmission timer value for this connection. If it is >
                                        0 it is running */

    /* Segment queues */
    tcpSeg_t        *lastUnsent;    /*!< Pointer to the last unsent segment in the unsent Q */
    tcpSeg_t        *unsent;        /*!< Pointer to the start of the unsent Q(data from the APP
                                        which was not yet sent) */
    tcpSeg_t        *unackd;        /*!< Pointer to the start of the unack'd Q(data that was sent
                                        but not yet acknowledged) */
    uint16_t        unsentWaste;    /*!< Size of the data free at the end of the last unsent
                                            segment */

    uint16_t        flags;          /*!< Flags used by this connection(tcpConnFlags_t) */

    /* Events processing */
    tspDataIndCb_t  pTcpEventCb;    /*!< Pointer to the callback to be executed after some data
                                        was received on this connection:
                                        - return from blocking recv()
                                        - return from blocking select() */
    osaEventId_t    pTcpEventId;    /*!< Rx event: Pointer to the message queue of the thread to be woke up */

    uint32_t        keepIdle;       /*!< Time to wait until the next keep alive */

    struct tcpConn_tag* pActiveConn;/*!< Pointer to the active connection for this connection. This
                                            is only available for listening connections */


    tcpStates_t     state;          /*!< Connection's state */
    uint8_t         persTmrBackoff; /*!< Persistent timer back off  */
    uint8_t         persTmrCounter; /*!< Persistent timer counter  */

#if TCP_DEBUG
    uint32_t        firstSeqNb;
    uint32_t        firstAckNb;
#endif
    uint8_t         hopLimit;       /*!< Hop limit for IPv6 or TTL for IPv4 */

    uint8_t         tos;            /*!< Type of service */
    uint8_t         cAck;           /*!< Number of ACKs(used for delayed ACK mechanism) */
    uint16_t        sndSegQSize;    /*!< Number of segments in the send queue[segments] */

    uint32_t        reserved;       /* alignment */
} tcpConn_t; /*!< Structure containing one TCP connection data */

/* structure type used for message exchange between Upper Layers and Transport Layer */
typedef struct tcpMsgData_tag
{
    nwkBuffer_t     *pNwkBuff;      /*!< pointer to network buffer */
    tcpConn_t       *pTcpConn;      /*!< Pointer to the TCP connection */
    uint32_t        flags;          /*!< various flags */
    uint16_t        sndSegQSize;    /*!< Number of segments in the send queue[segments] */
}tcpMsgData_t;

typedef struct tcpStats_tag
{
    uint16_t xmit;                  /*!< Transmitted packets. */
    uint16_t recv;                  /*!< Received packets. */
    uint16_t fw;                    /*!< Forwarded packets. */
    uint16_t drop;                  /*!< Dropped packets. */
    uint16_t chkErr;                /*!< Checksum error. */
    uint16_t lenErr;                /*!< Invalid length error. */
    uint16_t memErr;                /*!< Out of memory error. */
    uint16_t rtErr;                 /*!< Routing error. */
    uint16_t protErr;               /*!< Protocol error/invalid packet */
    uint16_t optErr;                /*!< Error in options. */
    uint16_t err;                   /*!< Misc error. */
    uint16_t cacheHit;

    uint16_t resetRX;               /*!< Number of resets sent back from RX processing */
    uint16_t badAck;                /*!< Bad ACK count */
}tcpStats_t; /*!< Structure used to hold TCP statistics */

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
\fn    tcpConn_t* TCP_Create(void)
\brief Private function for the TCP module. This function creates a new TCP connection.

\return       void
***************************************************************************************************/
tcpConn_t* TCP_Create(void);

/*!*************************************************************************************************
\fn    void TCP_Init(taskMsgQueue_t *pTranspMsgQueue)
\brief Public interface function for a module. This function gets the destination address.

\param [in]  pTranspMsgQueue pointer transport message queue

\return void
***************************************************************************************************/
void TCP_Init(taskMsgQueue_t *pTranspMsgQueue);

/*!*************************************************************************************************
\fn    tcpConn_t* TCP_Open(tcpConn_t *pTcpConn, tcpOpenMode_t mode)
\brief Public interface function for the TCP module. This is used to open a new TCP connection.

\param [in]  pTcpConn  pointer to the TCP connection
\param [out] mode      whether to open the connection in passive mode or active mode

\retval NULL           if a new connection could not be created
        tcpConn_t*     pointer to a new TCP connection
***************************************************************************************************/
tcpConn_t* TCP_Open(tcpConn_t *pTcpConn, tcpOpenMode_t mode);

/*!*************************************************************************************************
\fn    void TCP_Close(tcpConn_t *pTcpConn)
\brief Public interface function for the TCP module. This is used to close TCP connection.

\param [in]  pTcpConn  pointer to the TCP connection

\return     void
***************************************************************************************************/
void TCP_Close(tcpConn_t *pTcpConn);

/*!*************************************************************************************************
\fn    uint8_t TCP_Bind(tcpConn_t *pTcpConn, ipAddr_t *pIPAddr, uint16_t localPort)
\brief Public interface function for the TCP module. This is used to bind a local IP and port
combination to an opened tcp connection.

\param [in] pTcpConn        pointer to the TCP connection
\param [in] pIPAddr         pointer to the local ip address
\param [in] localPort       local port number

\retval mUdpCallError       if the open call failed
\retval mUdpCallOk          if the open call succeeded
\retval mUdpPortInUse_g     if the required port is in use
\retval mUdpNoFreePorts_g   if all the ports were used
***************************************************************************************************/
uint8_t TCP_Bind(tcpConn_t *pTcpConn, ipAddr_t *pIPAddr, uint16_t localPort);

/*!*************************************************************************************************
\fn    uint32_t TCP_Send(tcpConn_t *pTcpConn, nwkBuffer_t *pAppNwkBufferData, ipAddr_t *pDstIPAddr,
    uint16_t dstPort, uint32_t flags);
\brief Public interface function for the TCP module. This is used to Send data on an opened TCP
connection.

\param [in]   pTcpConn          pointer to the TCP connection
\param [in]   pAppNwkBufferData pointer to the app and network data nwkBUffer
\param [in]   pDstIPAddr        pointer to the destination IP address
\param [in]   dstPort           destination port number
\param [in]   flags             flags used for send operation

\return       uint32_t          the size of the data sent
***************************************************************************************************/
uint32_t TCP_Send(tcpConn_t *pTcpConn, nwkBuffer_t *pAppNwkBufferData, ipAddr_t *pDstIPAddr,
    uint16_t dstPort, uint32_t flags);


/*!*************************************************************************************************
\fn    bool_t TCP_HasActivity(tcpConn_t *pTcpConn)
\brief Public interface function for the TCP module. This is used to find if a connection has
something in the RX queue.

\param [in]   pTcpConn    pointer to the connection

\return       bool_t      status of the interrogation
***************************************************************************************************/
bool_t TCP_HasActivity(tcpConn_t *pTcpConn);

/*!*************************************************************************************************
\fn    int32_t TCP_Receive(tcpConn_t *pTcpConn, uint8_t *buffer, uint16_t size, uint32_t flags)
\brief Public interface function for the TCP module.

\param [in]   pTcpConn  pointer to the TCP connection
\param [in]   buffer    pointer to the allocated buffer where to put the received data
\param [in]   size      size of the buffer where to put the received data
\param [in]   flags     flags

\return       uint32_t  the size of the data received
***************************************************************************************************/
int32_t TCP_Receive(tcpConn_t *pTcpConn, uint8_t *buffer, uint16_t size, uint32_t flags);

/*!*************************************************************************************************
\fn    tcpConn_t* TCP_DuplicateConn(tcpConn_t *pTcpConn)
\brief Public interface function for the TCP module. This function is used to set up a new
        connection based on an existing connection(this is use in accept calls).

\param [in]   pTcpConn    pointer to the connection which needs to be duplicated

\return       tcpConn_t*  pointer to a new connection
 ***************************************************************************************************/
tcpConn_t* TCP_DuplicateConn(tcpConn_t *pTcpConn);

/*!*************************************************************************************************
\fn     void TCP_RegisterDataIndCb(tcpConn_t *pTcpConn, tspDataIndCb_t pDataIndCb, osaEventId_t
        eventId)
\brief  Public interface function for the TCP module. This is used to register a callback to be
        called by each layer after any data was received.

\param  [in]    pTcpConn      pointer to the TCP connection
\param  [in]    pDataIndCb    pointer to the callback
\param  [in]    eventId       id of the event which will be waiting

\return         void
***************************************************************************************************/
void TCP_RegisterDataIndCb(tcpConn_t *pTcpConn, tspDataIndCb_t pDataIndCb, osaEventId_t eventId);

/*!*************************************************************************************************
\private
\fn    uint8_t TCP_ClearConnection(tcpConn_t *pTcpConn)
\brief Private function for the TCP module. This function is used to clear any data allocated in
       this connection, including the connection itself.

\param [in]   pTcpConn  pointer to the TCP connection

\return       void
***************************************************************************************************/
void TCP_ClearConnection(tcpConn_t *pTcpConn);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _TCP_H */


