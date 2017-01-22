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

#ifndef _SOCKETS_H
#define _SOCKETS_H
/*!=================================================================================================
\file       sockets.h
\brief      This is a header file for the Sockets module. It contains the sockets implementation
            using the BSD interface.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "network_utils.h"
#include "sockets_cfg.h"
#include "udp.h"
#if TCP_ENABLED
#include "tcp.h"
#endif

/*==================================================================================================
Public macros
==================================================================================================*/
#define BSDS_RECV_EVENT             (0x04U)   /*!< Event to be used for Socket receive blocking */
#define BSDS_CANCEL_SELECT_EVENT    (0x08U)   /*!< Event to be used for Socket select blocking */
#define BSDS_CONN_DONE_EVENT        (0x10U)   /*!< Event to be used for receiving a connection */

#define SOCK_DGRAM                  (0)     /*!< Datagram socket type */
#define SOCK_STREAM                 (1)     /*!< Stream socket type */

/* TODO: !!!Clear following defines!!! */

/* Protocol families, same as address families. */
#define PF_UNSPEC   AF_UNSPEC
#define PF_UNIX     AF_UNIX
#define PF_LOCAL    AF_LOCAL
#define PF_INET     AF_INET
#define PF_AX25     AF_AX25
#define PF_IPX      AF_IPX
#define PF_APPLETALK    AF_APPLETALK
#define PF_NETROM   AF_NETROM
#define PF_BRIDGE   AF_BRIDGE
#define PF_ATMPVC   AF_ATMPVC
#define PF_X25      AF_X25
#define PF_INET6    AF_INET6
#define PF_ROSE     AF_ROSE
#define PF_DECnet   AF_DECnet
#define PF_NETBEUI  AF_NETBEUI
#define PF_SECURITY AF_SECURITY
#define PF_KEY      AF_KEY
#define PF_NETLINK  AF_NETLINK
#define PF_ROUTE    AF_ROUTE
#define PF_PACKET   AF_PACKET
#define PF_ASH      AF_ASH
#define PF_ECONET   AF_ECONET
#define PF_ATMSVC   AF_ATMSVC
#define PF_RDS      AF_RDS
#define PF_SNA      AF_SNA
#define PF_IRDA     AF_IRDA
#define PF_PPPOX    AF_PPPOX
#define PF_WANPIPE  AF_WANPIPE
#define PF_LLC      AF_LLC
#define PF_CAN      AF_CAN
#define PF_TIPC     AF_TIPC
#define PF_BLUETOOTH    AF_BLUETOOTH
#define PF_IUCV     AF_IUCV
#define PF_RXRPC    AF_RXRPC
#define PF_ISDN     AF_ISDN
#define PF_PHONET   AF_PHONET
#define PF_IEEE802154   AF_IEEE802154
#define PF_CAIF     AF_CAIF
#define PF_ALG      AF_ALG
#define PF_NFC      AF_NFC
#define PF_VSOCK    AF_VSOCK
#define PF_MAX      AF_MAX

/* Flags we can use with send/ and recv.
   Added those for 1003.1g not all are supported yet
 */

#define MSG_OOB             1
#define MSG_PEEK            2
#define MSG_DONTROUTE       4
#define MSG_TRYHARD         4           /*!< Synonym for MSG_DONTROUTE for DECnet */
#define MSG_CTRUNC          8
#define MSG_PROBE           0x10        /*!< Do not send. Only probe path f.e. for MTU */
#define MSG_TRUNC           0x20
#define MSG_DONTWAIT        0x40        /*!< Nonblocking io        */
#define MSG_EOR             0x80        /*!< End of record */
#define MSG_WAITALL         0x100       /*!< Wait for a full request */
#define MSG_FIN             0x200
#define MSG_SYN             0x400
#define MSG_CONFIRM         0x800       /*!< Confirm path validity */
#define MSG_RST             0x1000
#define MSG_ERRQUEUE        0x2000      /*!< Fetch message from error queue */
#define MSG_NOSIGNAL        0x4000      /*!< Do not generate SIGPIPE */
#define MSG_MORE            0x8000      /*!< Sender will send more */
#define MSG_WAITFORONE      0x10000     /*!< recvmmsg(): block until 1+ packets avail */
#define MSG_SENDPAGE_NOTLAST 0x20000    /*!< sendpage() internal : not the last page */
#define MSG_EOF             MSG_FIN

#define MSG_FASTOPEN        0x20000000  /*!< Send data in TCP SYN */
#define MSG_CMSG_CLOEXEC    0x40000000  /*!< Set close_on_exit for file descriptor received through
                                        SCM_RIGHTS */

#define IPV6_UNICAST_HOPS   (16)
#define IPV6_MULTICAST_HOPS (18)    /*!< Set the multicast hop limit for the socket */
#define IPV6_ADD_MEMBERSHIP (20)
#define IPV6_DROP_MEMBERSHIP (21)
#define IPV6_MTU            (24)    /*!< Set/Get MTU only for connected socket */

#define IPV6_JOIN_ANYCAST   (27)

#define IP_ADD_MEMBERSHIP   (35)    /*!< Joins the multicast group specified */
#define IP_DROP_MEMBERSHIP  (36)    /*!< Leaves the multicast group specified */
#define IP_MULTICAST_IF     (32)    /*!< Sets the interface over which outgoing multicast datagrams
                                    are sent */
#define IP_MULTICAST_TTL    (33)    /*!< Sets the Time To Live (TTL) in the IP header for outgoing
                                    multicast datagrams */
#define IP_MULTICAST_LOOP   (34)    /*!< Specifies whether or not a copy of an outgoing multicast
                                    datagram is delivered to the sending host as long as it is a
                                    member of the multicast group */

#ifndef IPV6_JOIN_GROUP     /* APIv0 compatibility */
#define IPV6_JOIN_GROUP     IPV6_ADD_MEMBERSHIP
#endif

#ifndef IPV6_LEAVE_GROUP
#define IPV6_LEAVE_GROUP    IPV6_DROP_MEMBERSHIP
#endif

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef int32_t(*pfSendFunc_t)(int32_t sockFd, void *pAppData, uint32_t pAppDataSize, uint32_t flags);
typedef int32_t(*pfRecvFunc_t)(int32_t sockFd, void *pAppData, uint32_t pAppDataSize, uint32_t flags);

typedef enum
{
    gBsdsSockSuccess_c      = 0U,   /*!< Success */
    gBsdsSockUnavailable_c  = 1U,   /*!< Socket to be removed is not available */
    gBsdsSockAdded_c        = 2U,   /*!< Socket was successfully added to list */
    gBsdsSockRemoved_c      = 3U,   /*!< Socket was successfully removed from list */
    gBsdsSockListFull_c     = 4U,   /*!< The sockets list is full */
    gBsdsSockError_g        = 5U,   /*!< Error */
    gBsdsSockPortInUse_c    = 6U,   /*!< Port was already used */
    gBsdsNoMoreFreePorts_c  = 7U,   /*!< There are no  more free ports */
    gBsdsSockFound_c        = 8U,   /*!< Socket was found in list */
    gBsdsSockInvalid_c      = 0xFFFFFFFFU
} sockFuncErr_t; /*!< Socket functions error codes */

typedef enum
{
    gBsdsSockUnbound_c,             /*!< Socket is not in use */
    gBsdsSockBound_c,               /*!< Socket is bound to an address/port combination */
    gBsdsSockListening_c,           /*!< Socket is in listening state */
    gBsdsSockUnConnected_c,         /*!< Socket is not connected */
    gBsdsSockConnected_c            /*!< Socket is connected */
} sockStateErr_t; /*!< Socket states */

#if BSDS_OPTIONS_SUPPORT
typedef enum
{
    SOL_SOCKET,                     /*!< Sockets layer */
    SOL_IP,                         /*!< IP layer */
    SOL_UDP,                        /*!< UDP layer */
    SOL_TCP,                        /*!< TCP layer */
    SOL_MAC                         /*!< MAC layer */
} sockOptLayer_t; /*!< Socket options layers */

typedef enum
{
    SO_TYPE,                        /*!< Compatible name for SO_STYLE */
    SO_REUSEADDR    = 0x0004,       /*!< Reuse IP address before previous connection to the same
                                         address is closed */
    SO_BINDTODEVICE = 0x0019,       /*!< Bind this socket to an interface */
    SO_PEERNAME     = 0x001C,       /*!< sockAddrStorage of the peer */
    SO_PEERSEC      = 0x001F,       /*!< Peer security */
    SO_REUSEPORT    = 0x0200,       /*!< Allow local address and port reuse */
    SO_SNDBUF       = 0x1001,       /*!< Send buffer size */
    SO_RCVBUF       = 0x1002        /*!< Last received buffer size */
} sockOptGeneric_t; /*!< Generic socket options */

typedef enum
{
    UDP_PKTOPT = 0
}udpSockOpt_t;

typedef enum
{
    TCP_PKTOPT = 0
}tcpSockOpt_t;

typedef enum
{
    MAC_SECURITY_LEVEL = 0          /*!< Mac security level */
}macSockOpt_t;

typedef enum
{
    mSockFlagsBlock_c       = 0x01  /*!< The socket is blocking */
} sockFlags_t;
#endif /* BSDS_OPTIONS_SUPPORT */

/* Socket structures */
typedef struct sockaddrIn
{
    ipAddr_t      sin_addr;     /*!< Internet address     */
    uint16_t      sin_family;   /*!< Address family       */
    uint16_t      sin_port;     /*!< Port number          */
} sockaddrIn_t; /*!< Structure containing socket address information(IPV4) */

typedef struct sockaddrIn6
{
    ipAddr_t    sin6_addr;      /*!< IPV6 address */
    uint16_t    sin6_family;    /*!< The address family we used when we set up the socket (AF_INET6)*/
    uint16_t    sin6_port;      /*!< The port number (the transport address) */
    uint32_t    sin6_flowinfo;  /*!< IPV6 flow information(traffic class and flow label) */
    uint32_t    sin6_scope_id;  /*!< set of interfaces for a scope (RFC2553) */
} sockaddrIn6_t; /*!< Structure containing socket address information(IPV6) */

typedef struct sockaddrStorage
{
    ipAddr_t    ss_addr;        /*!< Internet address     */
    uint16_t    ss_family;      /*!< address family */
    uint8_t     _data[sizeof(sockaddrIn_t) - sizeof(uint16_t) - sizeof(ipAddr_t)];
} sockaddrStorage_t; /*!< Structure containing socket address information */

typedef uint32_t socklen_t;

#if BSDS_SELECT_SUPPORT
typedef struct fdSet_tag{
    uint32_t count;
    int32_t fds[BSDS_SELECT_MAX_FDS];
} fdSet_t; /*!< Structure containing file descriptors set information */
#endif /* BSDS_SELECT_SUPPORT */

struct timeval
{
  uint32_t tv_sec;
  uint32_t tv_usec;
}; /*!< Structure containing time information */

typedef struct ipMreq
{
    ipAddr_t imrMultiaddr;   /*!< IP multicast group address */
    ipAddr_t imrInterface;   /*!< IP address of local interface*/
} ipMreq_t; /*!< Request structure for multicast socket options */

typedef struct socketCallback_tag
{
    int32_t(*SocketBind)(int32_t sockfd, sockaddrStorage_t *pLocalAddr, uint32_t addrlen);
    int32_t(*SocketConnect)(int32_t sockfd, sockaddrStorage_t *serv_addr, int addrLen);
    int32_t(*SocketListen)(int32_t sockfd, uint32_t backlog);
    int32_t(*SocketAccept)(int32_t sockfd, sockaddrStorage_t *addr, int addrLen);
    int32_t(*SocketRecv)(int32_t sockfd, void *msg, uint32_t msgLen, uint32_t flags);
    int32_t(*SocketRecvFrom)(int32_t sockfd, void *msg, uint32_t msgLen, uint32_t flags,
        sockaddrStorage_t *from, socklen_t fromLen);
    int32_t(*SocketSend)(int32_t sockfd, void *msg, uint32_t msgLen, uint32_t flags);
    int32_t(*SocketSendto)(int32_t sockfd, void *msg, uint32_t msgLen, uint32_t flags,
        sockaddrStorage_t *to, socklen_t toLen);
}socketCallback_t; /*!< Structure for holding pointer to functions for each transport module */

typedef struct socket_tag
{
    uint8_t addrFam;                /*!< Address family */
    uint8_t prot;                   /*!< Protocol */
    uint8_t type;                   /*!< Socket type(datagram or stream) */
    uint8_t state;                  /*!< Socket status of the connection */

    socketCallback_t *pCallback;    /*!< Pointer to socket callbacks */

#if BSDS_OPTIONS_SUPPORT
    /* Socket options */
    void *pInterface;               /*!< Pointer to the interface used by this socket */
    uint16_t optMtu;                /*!< MTU value for this socket(only for connected sockets) */
    uint8_t optHopLimit;            /*!< Multicast hop limit for the socket(0-255) */
    uint8_t security;               /*!< Mac security level */
#endif /* BSDS_OPTIONS_SUPPORT */

    uint8_t flags;                  /*!< Socket flags */
    uint8_t sockPad[3];             /*!< padding */

    udpConn_t *pUdpConn;            /*!< Pointer to the UDP connection */
#if TCP_ENABLED
    tcpConn_t *pTcpConn;            /*!< Pointer to the TCP connection */
#endif /* TCP_ENABLED */

}sock_t; /*!< BSD socket structure(information about local and remote is found in the transport
connection) */
/* 20 bytes without UDP pointer */

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
extern const socketCallback_t sockDgramCallback;
extern const socketCallback_t sockStreamCallback;

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn     int32_t socket(uint8_t domain, uint8_t type, uint8_t protocol)
\brief  This function creates a socket structure(and initialize its values with default) using a
        specific domain, type and protocol.

\param  [in]    domain      domain which can be PF_INET or PF_INET6
\param  [in]    type        type of socket(SOCK_DGRAM or SOCK_STREAM)
\param  [in]    protocol    transport protocol to be used(IPPROTO_UDP or IPPROTO_TCP)

\return         int32_t     socket file descriptor or -1 in case of error
***************************************************************************************************/
int32_t socket
(
    uint8_t domain,
    uint8_t type,
    uint8_t protocol
);

/*!*************************************************************************************************
\fn     int32_t shutdown(int32_t sockfd, int how)
\brief  This function close a socket connection.

\param  [in]    sockfd  socket descriptor
\param  [in]    how     (UNUSED)parameter which specifies how the socket will be closed

\retval         0       on success
                -1      on failure
***************************************************************************************************/
int32_t shutdown
(
    int32_t sockfd,
    int how
);

/*!*************************************************************************************************
\fn     int32_t bind(int32_t sockfd, sockaddrStorage_t *pAddr, uint32_t addrLen)
\brief  Public interface function for Sockets module. This function is used to bind a local IP
        address and a local port to an existing socket.

\param  [in]    sockfd  socket descriptor
\param  [in]    pAddr   pointer to the socket address structure
\param  [in]    addrLen size of the pAddr structure

\retval         0       on success
                -1      on failure
***************************************************************************************************/
int32_t bind
(
    int32_t sockfd,
    sockaddrStorage_t *pLocalAddr,
    uint32_t addrlen
);

/*!*************************************************************************************************
\fn     int32_t send(int32_t sockfd, void *msg, uint32_t msgLen, uint32_t flags)
\brief  This function is used to send data to a connected socket.

\param  [in]    sockfd  socket descriptor
\param  [in]    msg     pointer to the data which needs to be send
\param  [in]    msgLen  length of the data which needs to be send
\param  [in]    flags   flags used for sending

\return         int32_t length of the data sent, -1 on failure
***************************************************************************************************/
int32_t send
(
    int32_t sockfd,
    void *msg,
    uint32_t msgLen,
    uint32_t flags
);

/*!*************************************************************************************************
\fn     int32_t sendto(int32_t sockfd, void *msg, uint32_t msgLen, uint32_t flags,
        sockaddrStorage_t *pTo, uint32_t toLen)
\brief  This function is used to send data to a specific socket.

\param  [in]    sockfd  socket descriptor
\param  [in]    msg     pointer to the data which needs to be send
\param  [in]    msgLen  length of the data which needs to be send
\param  [in]    flags   flags used for sending
\param  [in]    pTo     pointer to the remote socket address structure
\param  [in]    toLen   size of the remote address structure

\return         int32_t length of the data sent, -1 on failure
***************************************************************************************************/
int32_t sendto
(
    int32_t sockfd,
    void *msg,
    uint32_t msgLen,
    uint32_t flags,
    sockaddrStorage_t *pTo,
    uint32_t toLen
);

/*!*************************************************************************************************
\fn     int32_t recv(int32_t sockfd, void *msg, uint32_t msgLen, uint32_t flags)
\brief  This function is used to get data from a socket RX queue.

\param  [in]    sockfd  socket descriptor
\param  [out]   msg     pointer to the buffer responsible for holding received data
\param  [in]    msgLen  length of the buffer allocated for receiving data
\param  [in]    flags   flags used for receiving

\return         int32_t length of the data received, -1 on failure
***************************************************************************************************/
int32_t recv
(
    int32_t sockfd,
    void *msg,
    uint32_t msgLen,
    uint32_t flags
);

/*!*************************************************************************************************
\fn int32_t recvfrom(int32_t sockfd, void *msg, uint32_t msgLen, uint32_t flags,
                    sockaddrStorage_t *from, socklen_t fromLen)
\brief  This function is used to get data from a specific socket from the RX queue. The remote
        information will be placed in the from structure.

\param  [in]    sockfd  socket descriptor
\param  [out]   msg     pointer to the buffer responsible for holding received data
\param  [in]    msgLen  length of the buffer allocated for receiving data
\param  [in]    flags   flags used for receiving
\param  [out]   from    pointer to the remote socket address structure
\param  [in]    fromLen size of the remote address structure

\return         int32_t length of the data received, -1 on failure
***************************************************************************************************/
int32_t recvfrom
(
    int32_t sockfd,
    void *msg,
    uint32_t msgLen,
    uint32_t flags,
    sockaddrStorage_t *from,
    socklen_t fromLen
);

/*!*************************************************************************************************
\fn     int32_t connect(int32_t sockfd, sockaddrStorage_t *serv_addr, uint32_t addrLen)
\brief  This function is used to connect to a remote server.

\param  [in]    sockfd      socket descriptor
\param  [in]    serv_addr   address structure for the server to connect to
\param  [in]    addrLen     address structure length

\retval         0           on success
\retval         -1          on error
***************************************************************************************************/
int32_t connect
(
    int32_t sockfd,
    sockaddrStorage_t *serv_addr,
    uint32_t addrLen
);

#if BSDS_STREAM_SUPPORT
/*!*************************************************************************************************
\fn     int32_t listen(int32_t sockfd, uint32_t backlog)
\brief  This function is used to listen to a stream socket.

\param  [in]    sockfd  socket descriptor
\param  [in]    backlog (UNUSED)number of connections allowed on the incoming queue

\retval         0       on success
\retval         -1      on failure
***************************************************************************************************/
int32_t listen
(
    int32_t sockfd,
    uint32_t backlog
);

/*!*************************************************************************************************
\fn     int32_t accept(int32_t sockfd, sockaddrStorage_t *pAddr, uint32_t addrLen)
\brief  This function is used to accept a connection from a client.

\param  [in]    sockfd  socket descriptor
\param  [out]   pAddr   address structure for remote client
\param  [in]    addrLen address structure length

\return         int32_t pointer to the newly accepted socket or -1 on failure
***************************************************************************************************/
int32_t accept
(
    int32_t sockfd,
    sockaddrStorage_t *pAddr,
    uint32_t addrLen
);
#endif /* BSDS_STREAM_SUPPORT */


#if BSDS_SELECT_SUPPORT
/*!*************************************************************************************************
\fn     bool_t FD_ZERO(fdSet_t *pSet)
\brief  This function is used to clear all file descriptors in a file descriptor set.

\param  [in]    pSet    pointer to the set of file descriptors

\retval         TRUE    the call was successful
\retval         FALSE   the call failed
***************************************************************************************************/
bool_t FD_ZERO(fdSet_t *pSet);

/*!*************************************************************************************************
\fn     bool_t FD_SET(int32_t fd, fdSet_t *pSet)
\brief  This function is used to set a file descriptor in a set of file descriptors.

\param  [in]    fd      file descriptor
\param  [in]    pSet    pointer to the set of file descriptors

\retval         TRUE    the file descriptor could be set
\retval         FALSE   the file descriptor could not be set
***************************************************************************************************/
bool_t FD_SET(int32_t fd, fdSet_t *pSet);

/*!*************************************************************************************************
\fn     bool_t FD_ISSET(int32_t fd, fdSet_t *pSet)
\brief  This function is used to check if a file descriptor is set in a set of file descriptors.

\param  [in]    fd      file descriptor
\param  [in]    pSet    pointer to the set of file descriptors

\retval         TRUE    the file descriptor is set
\retval         FALSE   the file descriptor is not set
***************************************************************************************************/
bool_t FD_ISSET(int32_t fd, fdSet_t *pSet);

/*!*************************************************************************************************
\fn     bool_t FD_CLR(int32_t fd, fdSet_t *pSet)
\brief  This function is used to clear a file descriptor from a set of file descriptors.

\param  [in]    fd      file descriptor
\param  [in]    pSet    pointer to the set of file descriptors

\retval         TRUE    the file descriptor could be cleared
\retval         FALSE   the file descriptor could not be cleared
***************************************************************************************************/
bool_t FD_CLR(int32_t fd, fdSet_t *pSet);

/*!*************************************************************************************************
\fn     int32_t select(int numfds, fdSet_t *readfds, fdSet_t *writefds, fdSet_t *exceptfds,
        struct timeval *timeout)
\brief  This function is used to check multiple socket file descriptors for incoming/outgoing data.
        This function will block the caller task based on the timeout values

\param  [in]        numfds      number of file descriptors(unused)
\param  [in,out]    readfds     pointer to the list of the file descriptors monitored for read
                                operation
\param  [in,out]    writefds    pointer to the list of the file descriptors monitored for write
                                operation
\param  [in,out]    exceptfds   pointer to the list of the file descriptors monitored for exceptions
                                thrown(unused)
\param  [in]        timeout     pointer to a structure which contains timeout data(seconds and
                                microseconds) if timeout values are 0 then select will timeout
                                immediately(polling once the file descriptors) if timeout is NULL,
                                the function will never timeout and wait for the first file
                                descriptor socket to be ready

\retval             -1          if error,
\retval             0           if no sockets had activity(data received/sent)
\retval             1           if at least one socket was active
***************************************************************************************************/
int32_t select
(
    int numfds,
    fdSet_t *readfds,
    fdSet_t *writefds,
    fdSet_t *exceptfds,
    struct timeval *timeout
);
#endif /* BSDS_SELECT_SUPPORT */

/*!*************************************************************************************************
\fn     int32_t getsockopt(int32_t sockfd, int32_t level, int32_t optName, void *optVal,
                           int32_t *optLen)
\brief  This function retrieves information about a specified socket.

\param  [in]    sockfd              socket file descriptor
\param  [in]    level               layer for operation
\param  [in]    optName             option
\param  [out]   optVal              pointer to the value for the option
\param  [out]   optLen              pointer to the length of the option

\retval         gBsdsSockSuccess_c  if the option was set
\retval         gBsdsSockError_g    if the option cannot be set
***************************************************************************************************/
int32_t getsockopt
(
    int32_t sockfd,
    int32_t level,
    int32_t optName,
    void *optVal,
    int32_t *optLen
);

/*!*************************************************************************************************
\fn     int32_t setsockopt(int32_t sockfd, int32_t level, int32_t optName, void *optVal,
                           int32_t optLen)
\brief  This function sets information for a specified socket.

\param  [in]    sockfd              socket file descriptor
\param  [in]    level               layer for operation
\param  [in]    optName             option
\param  [in]    optVal              pointer to the value for the option
\param  [in]    optLen              the length of the option

\retval         gBsdsSockSuccess_c  if the option was set
\retval         gBsdsSockError_g    if the option cannot be set
***************************************************************************************************/
int32_t setsockopt
(
    int32_t sockfd,
    int32_t level,
    int32_t optName,
    void *optVal,
    int32_t optLen
);

/*!*************************************************************************************************
\fn     int32_t getsockname(int32_t sockfd, sockaddrStorage_t *pAddr, socklen_t addrlen)
\brief  This function retrieves information about the local address and port of a socket.

\param  [in]    sockfd              socket file descriptor
\param  [out]   pAddr               a pointer to a structure containing the local information
\param  [out]   addrlen             the size of the pAddr structure

\retval         gBsdsSockSuccess_c  if the name can be retrieved
\retval         gBsdsSockError_g    if the name cannot be retrieved
***************************************************************************************************/
int32_t getsockname
(
    int32_t sockfd,
    sockaddrStorage_t *pAddr,
    socklen_t addrlen
);

/*!*************************************************************************************************
\fn     osaEventId_t BSDS_GetEventId()
\brief  This function retrieves the sockets eventId. This should be used in sockets event handling
        (select function)

\retval         event Id
***************************************************************************************************/
osaEventId_t BSDS_GetEventId();

/*!*************************************************************************************************
\fn     void BSDS_InitEventId()
\brief  This function initialize the sockets eventId.

\retval         event Id
***************************************************************************************************/
void BSDS_InitEventId();

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _SOCKETS_H */
