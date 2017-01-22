/*!=================================================================================================
\file       fsci_commands.c
\brief      This is a public source file for the FSCI module. It contains the implementation of the
            FSCI commands.

\copyright  (c) Copyright 2013, Freescale, Inc.  All rights reserved.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "app_to_stack_config.h"
#include "stack_config.h"

#include "EmbeddedTypes.h"
#include <string.h>
#include <stdio.h>

#include "ip_if_vtun.h"
#include "virtual_tun_driver.h"

#include "Panic.h"
#include "FunctionLib.h"
#include "SerialManager.h"
#include "FsciInterface.h"

#include "network_utils.h"
#include "sockets.h"
#include "ip_if_management.h"
#include "icmp.h"
#include "sixlowpan.h"
#include "fsci_commands.h"

#include "dtls.h"
#include "app_init.h"

#include "nwk_params.h"
#include "mac_filtering.h"


#include "thread_network_data.h"

#if THREAD_USE_FSCI

extern void APP_StartDevice(void *param);

#if (gFsciIncluded_c == FALSE)
        #error "*** ERROR: FSCI module is not enabled - check app_to_fwk_config.h"
#endif
/*==================================================================================================
Private macros
==================================================================================================*/
#define FSCI_MAX_SOCK_NB            (0x5U)
#define FSCI_MAX_DATA_BYTES         (1000U)
#define PING_ID                     (1U)
#define PING_SEQ_NB                 (1U)
#define PING_PAYLOAD_DEFAULT_SIZE   (32U)
#define PING_PAYLOAD_START          'a'
#define PING_PAYLOAD_END            'w'
#define SHELL_CMD_MAX_ARGS          (10U)
#define PING_HEADER_SIZE            (4U)
#define DEFAULT_TIMEOUT             (2000U)
#define SHELL_PING_MIN_TIMEOUT      (2000U)

/*==================================================================================================
Private type definitions
==================================================================================================*/
typedef enum ztFSCIrorCode_tag
{
    mFSCI_Ok                     = 0x00U,
    mFSCI_NoSpace                = 0x01U,
    mFSCI_PingTimeout            = 0x02U,
    mFSCI_PingWrongDestAddress   = 0x03U,
    mFSCI_Err                    = 0xFFU
}fsciErrorCode_t;

typedef struct fsciGenericMsg_tag
{
    void *pData;
    uint32_t interfaceId;
}fsciGenericMsg_t;

/* Structures for creating reply for each request */
typedef struct fsciStatusConfirm_tag
{
    uint8_t status;
}dtlsStatusConfirm_t;

typedef struct sockSocket_tag
{
    uint8_t sockId;
}sockSocket_t;

typedef struct sockRecv_tag
{
    uint8_t size[2];                    /*<! Length of the received buffer */
    uint8_t data[FSCI_MAX_DATA_BYTES];  /*<! Received buffer */
}sockRecv_t;

typedef struct sockAccept_tag
{
    uint8_t status;
    uint8_t sock;
}sockAccept_t;

typedef struct sockGetOpt_tag
{
    uint8_t status;
    uint8_t optValue[4];
}sockGetOpt_t;

typedef struct pingReply_tag
{
    uint8_t status;
    uint8_t reply[2];
}pingReply_t;

typedef struct fsciDtlsConnectConfirm_tag
{
    uint8_t status;
    uint8_t peerIdx;
}fsciDtlsConnectConfirm_t;

typedef struct fsciDataConfirm_tag
{
    uint8_t peerIndex;
    uint8_t size[2];
    uint8_t data[];
}dtlsDataConfirm_t;

/* Structures for casting incoming data */
typedef struct sockParams_tag
{
    uint8_t domain;
    uint8_t type;
    uint8_t protocol;
}sockParams_t;

typedef struct shutdownParams_tag
{
    uint8_t sock;
}shutdownParams_t;

typedef struct bindParams_tag
{
    uint8_t sock;
    uint8_t ipAddr[16];
    uint8_t port[2];
    uint8_t family;
}bindParams_t;

typedef struct sendParams_tag
{
    uint8_t sock;
    uint8_t flags;
    uint8_t size[2];
    uint8_t data[FSCI_MAX_DATA_BYTES];
}sendParams_t;

typedef struct sendtoParams_tag
{
    uint8_t sock;
    uint8_t flags;
    uint8_t size[2];
    uint8_t port[2];
    uint8_t ipAddress[16];
    uint8_t data[];
}sendtoParams_t;

typedef struct recvParams_tag
{
    uint8_t sock;
    uint8_t size[2];
    uint8_t flags;
}recvParams_t;

typedef struct connectParams_tag
{
    uint8_t sock;
    uint8_t ipAddr[16];
    uint8_t port[2];
    uint8_t family;
}connectParams_t;

typedef struct listenParams_tag
{
    uint8_t sock;
    uint8_t backlog;
}listenParams_t;

typedef struct acceptParams_tag
{
    uint8_t sock;
}acceptParams_t;

typedef struct setOptParams_tag
{
    uint8_t sock;
    uint8_t level;
    uint8_t optName;
    uint8_t optVal[4];
}setOptParams_t;

typedef struct getOptParams_tag
{
    uint8_t sock;
    uint8_t level;
    uint16_t optName;
}getOptParams_t;

typedef struct ifconfigBindParams_tag
{
    ipAddr_t ipAddr;    /*!< IP address */
    uint8_t ifId;       /*!< Interface ID */
}ifconfigBindParams_t;

typedef struct pingParams_tag
{
    ipAddr_t destIpAddr;    /*!< Destination IP address */
    ipAddr_t sourceIpAddr;  /*!< Source IP address */
    uint8_t length[2];        /*!< Size of the ping packet payload */
    uint8_t timeout[2];       /*!< Timeout */
    //uint8_t count;        /*!< Number of iterations */
}pingParams_t;

typedef struct setNwkParams_tag
{
    uint8_t channel;
    uint8_t panID[2];
    uint8_t shortAddr[2];
    uint8_t extendedAddr[8];
    bool_t randomExtAddr;
    bool_t rxOnIdle;
}setNwkParams_t;

typedef struct addMacFilter_tag
{
    uint8_t extendedAddr[8];
    uint8_t shortAddr[2];
    uint8_t linkIndicator;

}addMacFilter_t;

typedef struct setNwkData_tag
{
    uint8_t interfaceID;
    uint8_t prefix[16];
    uint8_t prefixLength[4];
    uint8_t isStable;
}setNwkData_t;

typedef struct incrementVersion_tag
{
    uint8_t interfaceID;
    uint8_t isStable;
}incrementVersion_t;

typedef struct registerNwkData_tag
{
    uint8_t interfaceID;
}registerNwkData_t;

typedef struct statusConfirm_tag
{
    uint8_t status;
}statusConfirm_t;

/* DTLS structures */
typedef struct dtlsOpenConfirm_tag
{
    uint8_t status;
}dtlsOpenConfirm_t;

/* DTLS structures */
typedef struct dtlsOpenParams_tag
{
    uint8_t maxRetrCount;
    uint8_t timeout[2];
    uint8_t port[2];
}dtlsOpenParams_t;

typedef struct dtlsCloseContextParams_tag
{
    uint8_t contextNumber;
}dtlsCloseContextParams_t;

typedef struct dtlsClosePeerParams_tag
{
    uint8_t peerNumber;
}dtlsClosePeerParams_t;

typedef struct dtlsConnectParams_tag
{
    uint8_t contextNumber;
    uint8_t ipAddr[16];
    uint8_t port[2];
}dtlsConnectParams_t;

typedef struct dtlsSendParams_tag
{
    uint8_t peerNumber;
    uint8_t size[2];
    uint8_t data[];
}dtlsSendParams_t;


/*==================================================================================================
Private prototypes
==================================================================================================*/
static void FSCI_DataIndCb(void *pData, void* param, uint32_t interfaceId);
static void FSCI_DataIndHandler(void *param);

static void FSCI_BSDSockReqSocket(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqShutdown(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqBind(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqSend(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqSendTo(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqRecv(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqRecvFrom(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqConnect(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
#if BSDS_STREAM_SUPPORT && TCP_ENABLED
static void FSCI_BSDSockReqListen(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqAccept(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
#endif /* BSDS_STREAM_SUPPORT && TCP_ENABLED */
static void FSCI_BSDSockReqSetOpt(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_BSDSockReqGetOpt(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);

static void FSCI_IfconfigAll(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_IfconfigBind(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static bool_t FSCI_Ping(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);


#if FLIP_VTUN_ROUTER || THREAD_VTUN_ROUTER
/*
 * VTUN Interface functions
 */
static void FSCI_VtunOpen(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_VtunClose(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
static bool_t FSCI_VtunSend(clientPacket_t *pClientPacket, uint32_t interfaceId,
    uint8_t **pReplyData, uint16_t *pDataSize);
#endif /* FLIP_VTUN_ROUTER || THREAD_VTUN_ROUTER */

static void FSCI_SetNwkParams(clientPacket_t *pClientPacket, uint32_t interfaceId,
                              uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_StartNwk(clientPacket_t *pClientPacket, uint32_t interfaceId,
                          uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_MacFilter(clientPacket_t *pClientPacket, uint32_t interfaceId,
                           uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_SetDHCPServer(clientPacket_t *pClientPacket, uint32_t interfaceId,
                           uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_SetExtRoute(clientPacket_t *pClientPacket, uint32_t interfaceId,
                           uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_IncrementVersion(clientPacket_t *pClientPacket, uint32_t interfaceId,
                           uint8_t **pReplyData, uint16_t *pDataSize);
static void FSCI_Register(clientPacket_t *pClientPacket, uint32_t interfaceId,
                           uint8_t **pReplyData, uint16_t *pDataSize);

#if 0
static void FSCI_Register(clientPacket_t *pClientPacket, uint32_t interfaceId,
                           uint8_t **pReplyData, uint16_t *pDataSize);
#endif

#if DTLS_ENABLED
static void FSCI_DtlsOpen(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm);
static void FSCI_DtlsCloseContext(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm);
static void FSCI_DtlsClosePeer(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm);
static void FSCI_DtlsConnect(clientPacket_t *pClientPacket, fsciDtlsConnectConfirm_t *pConfirm);
static void FSCI_DtlsSend(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm);
#endif /* DTLS_ENABLED */

static void PING_EchoReplyReceiveAsync(ipPktInfo_t *pIpPktInfo);
static void PING_EchoReplyReceive(void *pParam);
static void PING_TimerCallback(void *param);
static void PING_HandleTimerCallback(void *param);
static ipPktInfo_t *PingCreatePktInfo(ipAddr_t *pDstAddr, uint32_t payloadLen);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static uint8_t fsciAppInterfaceId = FSCI_FLIP_INTERFACE; /* Index in the mFsciSerials array */
static int32_t maUserSock[FSCI_MAX_SOCK_NB];
#if DTLS_ENABLED
static int32_t maDtlsContexts[DTLS_MAX_CONTEXTS];
static int32_t maDtlsPeers[DTLS_MAX_PEERS];
dtlsCallbacks_t mDtlsCallbacks;
#endif
static taskMsgQueue_t *pmMainThreadMsgQueue;

static const icmpProtMsgTypeHandler_t mShellProtMsgTypeHandlerTbl6[] =
{
    {PING_EchoReplyReceiveAsync, 129U, ICMP_CODE_DEFAULT}
};

static const icmpProtMsgTypeHandler_t mShellProtMsgTypeHandlerTbl4[] =
{
    {PING_EchoReplyReceiveAsync, 0U, ICMP_CODE_DEFAULT}
};

/* Register ICMP receive callback */
IMCP_RegisterHandler(SHELL_APP, IPPROTO_ICMPV6, NULL, NumberOfElements(mShellProtMsgTypeHandlerTbl6),
                             (icmpProtMsgTypeHandler_t*)&mShellProtMsgTypeHandlerTbl6);

IMCP_RegisterHandler(SHELL_APP, IPPROTO_ICMP, NULL, NumberOfElements(mShellProtMsgTypeHandlerTbl4),
                             (icmpProtMsgTypeHandler_t*)&mShellProtMsgTypeHandlerTbl4);

static ipAddr_t mDstIpAddr;
static uint64_t pingTimeStamp = 0;
static tmrTimerID_t pingTimerID = gTmrInvalidTimerID_c;
static uint8_t *pmQueuedReplyData;
static opCode_t mQueuedOpCode;
static uint16_t mQueuedDataSize;
static uint32_t mQueuedInterfaceId;
static uint16_t defaultSeqNb = PING_SEQ_NB;
static uint32_t mPingTimeoutMs;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn     void APP_FsciInterface(void)
\brief  This function is used to initialize the FSCI communication.

\param  [in]    pointer to the message queue

\return         void
***************************************************************************************************/
void APP_FsciInterface
(
    taskMsgQueue_t *pMainThreadMsgQueue
)
{
    pmMainThreadMsgQueue = pMainThreadMsgQueue;

    /* Register Handler for requests coming from the serial interface */
    if(FSCI_RegisterOpGroup(gFSCI_FlipOpGReq_c, gFsciMonitorMode_c, FSCI_DataIndCb, NULL,
        fsciAppInterfaceId) != gFsciSuccess_c)
    {
        panic(0, 0, 0, 0);
    }
}

/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn     void FSCI_DataIndCb(void *pData, void* param, uint32_t interfaceId)
\brief  This is the SAP used to call different socket functions.

\param  [in]    pMsg        pointer to the MCPS message received from the MAC layer instanceId of
                            the MAC
\param  [in]    param       pointer to the parameter to be passed to this function
\param  [in]    interfaceId id of the FSCI interface

\return         void
***************************************************************************************************/
static void FSCI_DataIndCb
(
    void *pData,
    void* param,
    uint32_t interfaceId
)
{
    fsciGenericMsg_t *pFsciMessage;


    pFsciMessage = MEM_BufferAlloc(sizeof(fsciGenericMsg_t));
    pFsciMessage->interfaceId = interfaceId;
    pFsciMessage->pData = pData;

    NWKU_SendMsg(FSCI_DataIndHandler, (void*)pFsciMessage, pmMainThreadMsgQueue);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_DataIndHandler(void* param)
\brief  This is the SAP used to call different socket functions.

\param  [in]    param   pointer to the parameter to be passed to this function

\return         void
***************************************************************************************************/
static void FSCI_DataIndHandler
(
    void *param
)
{
    void *pData = ((fsciGenericMsg_t*)param)->pData;
    uint32_t interfaceId = ((fsciGenericMsg_t*)param)->interfaceId;
    clientPacket_t *pClientPacket = (clientPacket_t*)pData;
    uint16_t replyDataSize = 0;
    uint8_t *pReplyData = NULL;
    bool_t sendReply = TRUE;
    bool_t freePacket = TRUE;
#if DTLS_ENABLED    
    dtlsStatusConfirm_t basicConfirm;
#endif    

    switch(pClientPacket->structured.header.opCode)
    {
        case gFSCI_FlipBsdSocket_c:
            FSCI_BSDSockReqSocket(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdShutdown_c:
            FSCI_BSDSockReqShutdown(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdBind_c:
            FSCI_BSDSockReqBind(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdSend_c:
            FSCI_BSDSockReqSend(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdSendto_c:
            FSCI_BSDSockReqSendTo(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdRecv_c:
            FSCI_BSDSockReqRecv(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdRecvfrom_c:
            FSCI_BSDSockReqRecvFrom(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdConnect_c:
            FSCI_BSDSockReqConnect(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
#if BSDS_STREAM_SUPPORT && TCP_ENABLED
        case gFSCI_FlipBsdListen_c:
            FSCI_BSDSockReqListen(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdAccept_c:
            FSCI_BSDSockReqAccept(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
#endif /* BSDS_STREAM_SUPPORT */
        case gFSCI_FlipBsdSetsockopt_c:
            FSCI_BSDSockReqSetOpt(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipBsdGetsockopt_c:
            FSCI_BSDSockReqGetOpt(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipIfconfigAll_c:
            FSCI_IfconfigAll(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipIfconfigBind_c:
            FSCI_IfconfigBind(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipPing_c:
            sendReply = FSCI_Ping(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
#if FLIP_VTUN_ROUTER || THREAD_VTUN_ROUTER
        case gFSCI_FlipVtunOpen_c:
            FSCI_VtunOpen(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipVtunClose_c:
            FSCI_VtunClose(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_FlipVtunSend_c:
            sendReply = FSCI_VtunSend(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
#endif /* FLIP_VTUN_ROUTER || THREAD_VTUN_ROUTER*/
        case gFSCI_SetNwkParams_c:
            FSCI_SetNwkParams(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_StartNetwork_c:
            FSCI_StartNwk(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_MacFilter_c:
            FSCI_MacFilter(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_SetDHCPServer_c:
            FSCI_SetDHCPServer(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_SetExtRoute_c:
            FSCI_SetExtRoute(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_IncrementVersion_c:
            FSCI_IncrementVersion(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
        case gFSCI_Register_c:
            FSCI_Register(pClientPacket, interfaceId, &pReplyData, &replyDataSize);
            break;
#if DTLS_ENABLED
        case gFSCI_DtlsOpen_c:
        {
            FSCI_DtlsOpen(pClientPacket, &basicConfirm);
            pReplyData = (uint8_t*)&basicConfirm;
            replyDataSize = sizeof(dtlsStatusConfirm_t);
            freePacket = FALSE;
            mQueuedInterfaceId = interfaceId; /* TODO: This works only with one FSCI interface */
            break;
        }
        case gFSCI_DtlsCloseContext_c:
        {
            FSCI_DtlsCloseContext(pClientPacket, &basicConfirm);
            pReplyData = (uint8_t*)&basicConfirm;
            replyDataSize = sizeof(dtlsStatusConfirm_t);
            freePacket = FALSE;
            break;
        }
        case gFSCI_DtlsClosePeer_c:
        {
            FSCI_DtlsClosePeer(pClientPacket, &basicConfirm);
            pReplyData = (uint8_t*)&basicConfirm;
            replyDataSize = sizeof(dtlsStatusConfirm_t);
            freePacket = FALSE;
            break;
        }
        case gFSCI_DtlsConnect_c:
        {
            fsciDtlsConnectConfirm_t connectConfirm;
            FSCI_DtlsConnect(pClientPacket, &connectConfirm);
            pReplyData = (uint8_t*)&connectConfirm;
            replyDataSize = sizeof(fsciDtlsConnectConfirm_t);
            freePacket = FALSE;
            break;
        }
        case gFSCI_DtlsSend_c:
        {
            FSCI_DtlsSend(pClientPacket, &basicConfirm);
            pReplyData = (uint8_t*)&basicConfirm;
            replyDataSize = sizeof(dtlsStatusConfirm_t);
            freePacket = FALSE;
            break;
        }
#endif /* DTLS_ENABLED */
        default:
            break;
    }

    /* Do we want to send the reply now? */
    if(sendReply)
    {
        /* Send reply */
        FSCI_transmitPayload(gFSCI_FlipOpGCnf_c, pClientPacket->structured.header.opCode,
            pReplyData, replyDataSize, interfaceId);

        if(freePacket)
        {
            MEM_BufferFree(pReplyData);
        }
    }
    /* Do not send the reply and keep some pointers */
    else
    {
        /* Keep variables for later reply */
        mQueuedOpCode = pClientPacket->structured.header.opCode;
        pmQueuedReplyData = pReplyData;
        mQueuedDataSize = replyDataSize;
        mQueuedInterfaceId = interfaceId;
    }

    /* Clear received packet */
    MEM_BufferFree(pData);
    MEM_BufferFree(param);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqSocket(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the socket() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqSocket
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    sockSocket_t *pSockSocket = MEM_BufferAlloc(sizeof(sockSocket_t));
    int32_t sockFd;
    uint32_t iSock;
    fsciErrorCode_t error = mFSCI_Ok;
    sockParams_t *pSockParams;

    /* Cast received data to a known structure */
    pSockParams = (sockParams_t*)pClientPacket->structured.payload;

    /* Try to create a new socket */
    sockFd = socket(pSockParams->domain, pSockParams->type, pSockParams->protocol);

    /* Add socket to the list */
    iSock = NWKU_AddTblEntry(sockFd, (uint32_t*)maUserSock, FSCI_MAX_SOCK_NB);

    if(iSock == FSCI_MAX_SOCK_NB)
    {
        error = mFSCI_Err;
    }

    /* Set FSCI command payload */
    if(error == mFSCI_Ok)
    {
        pSockSocket->sockId = iSock;
    }
    else
    {
        pSockSocket->sockId = mFSCI_Err;
    }

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockSocket;
    *pDataSize = sizeof(sockSocket_t);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqShutdown(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the shutdown() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqShutdown
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    statusConfirm_t *pSockShutdown = MEM_BufferAlloc(sizeof(statusConfirm_t));
    int32_t result = 0;
    shutdownParams_t *pShutdownParams;

    /* Cast received data to a known structure */
    pShutdownParams = (shutdownParams_t*)pClientPacket->structured.payload;
    pSockShutdown->status = mFSCI_Err;

    if(pShutdownParams->sock < FSCI_MAX_SOCK_NB && maUserSock[pShutdownParams->sock])
    {
        /* Close a socket */
        result = shutdown(maUserSock[pShutdownParams->sock], 0);
        if(result == 0)
        {
            /* Remove the socket from the local list */
            maUserSock[pShutdownParams->sock] = 0;

            pSockShutdown->status = mFSCI_Ok;
        }
    }

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockShutdown;
    *pDataSize = sizeof(statusConfirm_t);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqBind(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the bind() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqBind
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    statusConfirm_t *pSockBind = MEM_BufferAlloc(sizeof(statusConfirm_t));
    int32_t result = 0;
    sockaddrStorage_t *pSockAddr = MEM_BufferAlloc(sizeof(sockaddrStorage_t));
    bindParams_t *pBindParams;

    /* Cast received data to a known structure */
    pBindParams = (bindParams_t*)pClientPacket->structured.payload;
    pSockBind->status = mFSCI_Err;

    if(pBindParams->sock < FSCI_MAX_SOCK_NB && maUserSock[pBindParams->sock])
    {
        uint8_t idx = 0;
        uint8_t idx2 = 15;

        /* Create sockAddr structure depending on family */
        if(pBindParams->family == AF_INET6)
        {
            ((sockaddrIn6_t*)pSockAddr)->sin6_family = AF_INET6;
            ((sockaddrIn6_t*)pSockAddr)->sin6_port = *((uint16_t*)pBindParams->port);

            /* Reverse address bytes */
            for(idx=0;idx<16;idx++,idx2--)
            {
                uint8_t *p = ((uint8_t*)pBindParams->ipAddr) + idx2;
                uint8_t *p2 = ((sockaddrIn6_t*)pSockAddr)->sin6_addr.addr8;
                p2[idx] = *(p);
            }

        }
        else if(pBindParams->family == AF_INET)
        {
            ((sockaddrIn_t*)pSockAddr)->sin_family = AF_INET;
            ((sockaddrIn_t*)pSockAddr)->sin_port = *((uint16_t*)pBindParams->port);
            FLib_MemCpy(&((sockaddrIn_t*)pSockAddr)->sin_addr, pBindParams->ipAddr,
                sizeof(ipAddr_t));
        }

        /* Bind socket */
        result = bind(maUserSock[pBindParams->sock], pSockAddr, sizeof(sockaddrStorage_t));

        /* Set FSCI command payload */
        if(result == 0)
        {
            pSockBind->status = mFSCI_Ok;
        }
    }

    /* Clear unneeded data */
    MEM_BufferFree(pSockAddr);

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockBind;
    *pDataSize = sizeof(statusConfirm_t);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqSend(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the send() function.

\param  [in]    pClientPacket pointer to the packet received from FSCI
\param  [in]    interfaceId   id of the FSCI interface
\param  [out]   pReplyData    double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize     size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqSend
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    statusConfirm_t *pSockSend = MEM_BufferAlloc(sizeof(statusConfirm_t));
    int32_t result = 0;
    sendParams_t *pSendParam;

    /* Cast received data to a known structure */
    pSendParam = (sendParams_t*)pClientPacket->structured.payload;
    pSockSend->status = mFSCI_Err;

    if(pSendParam->sock < FSCI_MAX_SOCK_NB && maUserSock[pSendParam->sock])
    {
        /* Send data */
        result = send(maUserSock[pSendParam->sock], pSendParam->data, *((uint16_t*)pSendParam->size),
            pSendParam->flags);

        /* Set FSCI command payload */
        if(result > 0)
        {
            pSockSend->status = mFSCI_Ok;
        }
    }

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockSend;
    *pDataSize = sizeof(statusConfirm_t);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqSendTo(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the send() function.

\param  [in]    pClientPacket pointer to the packet received from FSCI
\param  [in]    interfaceId   id of the FSCI interface
\param  [out]   pReplyData    double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize     size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqSendTo
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    statusConfirm_t     *pSockSend = MEM_BufferAlloc(sizeof(statusConfirm_t));
    sendtoParams_t      *pSendParam;
    sockaddrStorage_t   *pSockAddr;
    int32_t             result = 0;

    /* Cast received data to a known structure */
    pSendParam = (sendtoParams_t*)pClientPacket->structured.payload;
    pSockSend->status = mFSCI_Err;

    if((pSendParam->sock < FSCI_MAX_SOCK_NB) && (maUserSock[pSendParam->sock]))
    {
        pSockAddr = MEM_BufferAlloc(sizeof(sockaddrStorage_t));

        /* Copy IP address */
        FLib_MemCpy(&pSockAddr->ss_addr, pSendParam->ipAddress, sizeof(ipAddr_t));

        /* Revert IP address */
        {
            /* Revert bytes in place */
            uint8_t *pPos;
            pPos = (uint8_t*)&pSockAddr->ss_addr;
            uint8_t aux, idx;
            for(idx=0;idx<16/2;idx++)
            {
                aux = *(pPos + idx);
                *(pPos + idx) = *(pPos + 16 - 1 - idx);
                *(pPos + 16 - 1 - idx) = aux;
            }
        }

        /* Set family */
        pSockAddr->ss_family = (IP_IsAddrIPv6(&pSockAddr->ss_addr))?AF_INET6:AF_INET;

        /* Set remote port */
        if(pSockAddr->ss_family == AF_INET6)
        {
            ((sockaddrIn6_t*)pSockAddr)->sin6_port = ntohas(pSendParam->port);
        }
        else
        {
            ((sockaddrIn_t*)pSockAddr)->sin_port = ntohas(pSendParam->port);
        }

        /* Send data */
        result = sendto(maUserSock[pSendParam->sock], pSendParam->data,
            *((uint16_t*)pSendParam->size), pSendParam->flags, pSockAddr,
            sizeof(sockaddrStorage_t));

        MEM_BufferFree(pSockAddr);

        /* Set FSCI command payload */
        if(result > 0)
        {
            pSockSend->status = mFSCI_Ok;
        }
    }

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockSend;
    *pDataSize = sizeof(statusConfirm_t);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqRecv(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the recv() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
 ***************************************************************************************************/
static void FSCI_BSDSockReqRecv
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    recvParams_t *pRecvParam;
    uint16_t rxLength = 0;
    uint8_t *pRecvData = NULL;
    uint8_t *pSockRecv;

    /* Cast received data to a known structure */
    pRecvParam = (recvParams_t*)pClientPacket->structured.payload;

    pRecvData = MEM_BufferAlloc(FSCI_MAX_DATA_BYTES);
    if((pRecvParam->sock < FSCI_MAX_SOCK_NB) && (maUserSock[pRecvParam->sock]))
    {
        /* Get some data from the socket */
        rxLength = recv(maUserSock[pRecvParam->sock], pRecvData, *((uint16_t*)pRecvParam->size),
            pRecvParam->flags);
    }

    /* Create output packet */
    pSockRecv = MEM_BufferAlloc(sizeof(uint16_t) + rxLength);
    //htonas(pSockRecv, rxLength); /* Set size */
    *((uint16_t*)(pSockRecv)) = rxLength;
    FLib_MemCpy(pSockRecv + sizeof(uint16_t), pRecvData, rxLength); /* Set data */

    /* Free unneeded buffer */
    MEM_BufferFree(pRecvData);

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockRecv;
    *pDataSize = sizeof(uint16_t) + rxLength;
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqRecvFrom(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the recv() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
 ***************************************************************************************************/
static void FSCI_BSDSockReqRecvFrom
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    recvParams_t        *pRecvParam;
    uint8_t             *pRecvData = NULL;
    uint8_t             *pSockRecv;
    sockaddrStorage_t   *pSockAddr;
    uint16_t            rxLength = 0;
    uint8_t             outputLength = 0;

    /* Cast received data to a known structure */
    pRecvParam = (recvParams_t*)pClientPacket->structured.payload;

    if((pRecvParam->sock < FSCI_MAX_SOCK_NB) && (maUserSock[pRecvParam->sock]))
    {
        pRecvData = MEM_BufferAlloc(FSCI_MAX_DATA_BYTES);
        pSockAddr = MEM_BufferAlloc(sizeof(sockaddrStorage_t));

        /* Get some data from the socket */
        rxLength = recvfrom(maUserSock[pRecvParam->sock], pRecvData, *((uint16_t*)pRecvParam->size),
            pRecvParam->flags, pSockAddr, sizeof(sockaddrStorage_t));

        /* Create the output packet */
        outputLength =
            1 +                 /* status code */
            sizeof(ipAddr_t) +  /* remote IP address */
            sizeof(uint16_t) +  /* remote port */
            sizeof(uint16_t) +  /* length of the received packet */
            rxLength;           /* received data */
        pSockRecv = MEM_BufferAlloc(outputLength);
        FLib_MemSet(pSockRecv, 0, outputLength);

        *pSockRecv = mFSCI_Err;

        if(rxLength == 0xFFFF)
        {
            /* Error */
        }
        else
        {
            /* Set IP address */
            FLib_MemCpy(pSockRecv + 1, &pSockAddr->ss_addr, sizeof(ipAddr_t));

            /* Revert IP address */
            {
                /* Revert bytes in place */
                uint8_t *pPos;
                pPos = pSockRecv + 1;
                uint8_t aux, idx;
                for(idx=0;idx<16/2;idx++)
                {
                    aux = *(pPos + idx);
                    *(pPos + idx) = *(pPos + 16 - 1 - idx);
                    *(pPos + 16 - 1 - idx) = aux;
                }
            }

            /* Set port depending on RX packet */
            if(pSockAddr->ss_family == AF_INET6)
            {
                *((uint16_t*)(pSockRecv + 1 + sizeof(ipAddr_t))) =
                    ((sockaddrIn6_t*)pSockAddr)->sin6_port;
            }
            else
            {
                *((uint16_t*)(pSockRecv + 1 + sizeof(ipAddr_t))) =
                    ((sockaddrIn_t*)pSockAddr)->sin_port;
            }

            /* Set the length of the received packet */
            *((uint16_t*)(pSockRecv +
                           1 +                  /* status code */
                           sizeof(ipAddr_t) +   /* remote IP address */
                           sizeof(uint16_t)     /* remote port */
                           )) = rxLength;

            /* Copy received bytes */
            FLib_MemCpy(pSockRecv + outputLength - rxLength, pRecvData, rxLength);

            *pSockRecv = mFSCI_Ok;
        }

        /* Free unneeded buffer */
        MEM_BufferFree(pSockAddr);
        MEM_BufferFree(pRecvData);
    }

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockRecv;
    *pDataSize = outputLength;
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqConnect(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the connect() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqConnect
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    statusConfirm_t *pSockConnect = MEM_BufferAlloc(sizeof(statusConfirm_t));
    connectParams_t *pConnectParam;
    sockaddrStorage_t *pSockAddr = MEM_BufferAlloc(sizeof(sockaddrStorage_t));
    int32_t result;

    /* Cast received data to a known structure */
    pConnectParam = (connectParams_t*)pClientPacket->structured.payload;
    pSockConnect->status = mFSCI_Err;

    if(pConnectParam->sock < FSCI_MAX_SOCK_NB && maUserSock[pConnectParam->sock])
    {
        uint8_t idx = 0;
        uint8_t idx2 = 15;

        /* Create sockAddr structure depending on family */
        if(pConnectParam->family == AF_INET6)
        {
            ((sockaddrIn6_t*)pSockAddr)->sin6_family = AF_INET6;
            ((sockaddrIn6_t*)pSockAddr)->sin6_port = *((uint16_t*)pConnectParam->port);

            /* Reverse address bytes */
            for(idx=0;idx<16;idx++,idx2--)
            {
                uint8_t *p = ((uint8_t*)pConnectParam->ipAddr) + idx2;
                uint8_t *p2 = ((sockaddrIn6_t*)pSockAddr)->sin6_addr.addr8;
                p2[idx] = *(p);
            }

        }
        else if(pConnectParam->family == AF_INET)
        {
            ((sockaddrIn_t*)pSockAddr)->sin_family = AF_INET;
            ((sockaddrIn_t*)pSockAddr)->sin_port = *((uint16_t*)pConnectParam->port);
            FLib_MemCpy(&((sockaddrIn_t*)pSockAddr)->sin_addr, pConnectParam->ipAddr,
                sizeof(ipAddr_t));
        }

        /* Connect to the remote socket */
        result = connect(maUserSock[pConnectParam->sock], pSockAddr, sizeof(sockaddrStorage_t));

        /* Set FSCI command payload */
        if(result == 0)
        {
            pSockConnect->status = mFSCI_Ok;
        }
    }

    /* Clear unneeded data */
    MEM_BufferFree(pSockAddr);

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockConnect;
    *pDataSize = sizeof(statusConfirm_t);
}

#if BSDS_STREAM_SUPPORT && TCP_ENABLED
/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqListen(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the listen() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqListen
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    statusConfirm_t *pSockListen = MEM_BufferAlloc(sizeof(statusConfirm_t));
    listenParams_t *pListenParam;
    int32_t result;

    /* Cast received data to a known structure */
    pListenParam = (listenParams_t*)pClientPacket->structured.payload;
    pSockListen->status = mFSCI_Err;

    if(pListenParam->sock < FSCI_MAX_SOCK_NB && maUserSock[pListenParam->sock])
    {
        /* Connect to the remote socket */
        result = listen(maUserSock[pListenParam->sock], pListenParam->backlog);

        /* Set FSCI command payload */
        if(result == 0)
        {
            pSockListen->status = mFSCI_Ok;
        }
    }

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockListen;
    *pDataSize = sizeof(statusConfirm_t);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqAccept(clientPacket_t *pClientPacket, uint32_t interfaceId)
\brief  This function is used to expose the listen() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqAccept
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    sockAccept_t *pSockAccept = MEM_BufferAlloc(sizeof(sockAccept_t));
    acceptParams_t *pAcceptParam;
    int32_t result;
    uint32_t iSock;
    fsciErrorCode_t error = mFSCI_Ok;
    sockaddrStorage_t *pSockAddr = MEM_BufferAlloc(sizeof(sockaddrStorage_t));

    /* Cast received data to a known structure */
    pAcceptParam = (acceptParams_t*)pClientPacket->structured.payload;
    pSockAccept->status = mFSCI_Err;

    if(pAcceptParam->sock < FSCI_MAX_SOCK_NB && maUserSock[pAcceptParam->sock])
    {
        /* Connect to the remote socket */
        result = accept(maUserSock[pAcceptParam->sock], pSockAddr, sizeof(sockaddrStorage_t));

        /* Add socket to the list */
        for(iSock=0;iSock<FSCI_MAX_SOCK_NB;iSock++)
        {
            if(!maUserSock[iSock])
            {
                maUserSock[iSock] = result;
                error = mFSCI_Ok;
                break;
            }
        }

        /* Set FSCI command payload */
        if(error == mFSCI_Ok)
        {
            pSockAccept->status = mFSCI_Ok;
            pSockAccept->sock = iSock;
        }
    }

    /* Clear unneeded data */
    MEM_BufferFree(pSockAddr);

    /* Set up reply */
    *pReplyData = (uint8_t*)pSockAccept;
    *pDataSize = sizeof(sockAccept_t);
}
#endif /* BSDS_STREAM_SUPPORT && TCP_ENABLED */

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqSetOpt(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to expose the setsockopt() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqSetOpt
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    statusConfirm_t *pSockSetOpt = MEM_BufferAlloc(sizeof(statusConfirm_t));

    /* Cast received data to a known structure */
    setOptParams_t *pSetOptParams = (setOptParams_t*)pClientPacket->structured.payload;

    uint32_t optVal;

    pSockSetOpt->status = mFSCI_Err;

    if(pSetOptParams->sock < FSCI_MAX_SOCK_NB && maUserSock[pSetOptParams->sock])
    {

        /* Process setsockopt */

        optVal = ntohal(pSetOptParams->optVal);

        if(0U == setsockopt(maUserSock[pSetOptParams->sock], pSetOptParams->level,
            pSetOptParams->optName, &optVal, 4U))
        {
            pSockSetOpt->status = mFSCI_Ok;
        }
    }

    /* Set up reply data */
    *pReplyData = (uint8_t*)pSockSetOpt;
    *pDataSize = sizeof(statusConfirm_t);

}

/*!*************************************************************************************************
\private
\fn     void FSCI_BSDSockReqGetOpt(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to expose the getsockopt() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_BSDSockReqGetOpt
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    sockGetOpt_t *pSockGetOpt = MEM_BufferAlloc(sizeof(sockGetOpt_t));
    int32_t optLen;
    uint32_t optValue;

    /* Cast received data to a known structure */
    getOptParams_t *pGetOptParams = (getOptParams_t*)pClientPacket->structured.payload;

    pSockGetOpt->status = mFSCI_Err;

    if(pGetOptParams->sock < FSCI_MAX_SOCK_NB && maUserSock[pGetOptParams->sock])
    {
        /* Process getsockopt */
        if(gBsdsSockSuccess_c == getsockopt(maUserSock[pGetOptParams->sock], pGetOptParams->level,
            pGetOptParams->optName, &optValue, &optLen))
        {
            pSockGetOpt->status = mFSCI_Ok;
            htonal(pSockGetOpt->optValue, optValue);
        }
    }

    /* Set up reply data */
    *pReplyData = (uint8_t*)pSockGetOpt;
    *pDataSize = sizeof(sockGetOpt_t);

}


/*!*************************************************************************************************
\private
\fn     void FSCI_IfconfigAll(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to show all interfaces with their IP addresses..

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_IfconfigAll
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    uint8_t *pIfconfigAllRes;
    uint8_t *pPos;
    uint32_t iIf;
    uint32_t iAddr;
    ip6IfAddrData_t *pIp6AddrData;
#if IP_IP4_ENABLE
    ip4IfAddrData_t *pIp4AddrData;
#endif
    ifHandle_t* pIfHandle;

    uint8_t ifconfigInfo[4];
    FLib_MemSet(ifconfigInfo, 0, 4);

    /* Count addresses */
    uint16_t cBytes = 1;
    iIf = 0;
    pIfHandle = IP_IF_GetIfByNr(iIf);
    while(*pIfHandle)
    {
        cBytes += 2;
#if IP_IP4_ENABLE
        /* Get IPv4 addresse for an interface */
        pIp4AddrData = IP_IF_GetAddrByIf4(pIfHandle);
        if(pIp4AddrData)
        {
            ifconfigInfo[iIf]++;
            cBytes += 16;
        }
#endif
        /* Get IPv6 addresses for an interface */
        iAddr = 0;
        pIp6AddrData = IP_IF_GetAddrByIf6(pIfHandle, iAddr);
        while(pIp6AddrData)
        {
            ifconfigInfo[iIf]++;
            cBytes += 16;

            /* Go to the next IP address */
            iAddr++;
            pIp6AddrData = IP_IF_GetAddrByIf6(pIfHandle, iAddr);
        }

        /* Go to the next interface */
        iIf++;
        pIfHandle = IP_IF_GetIfByNr(iIf);
    }

    /* Allocate output buffer */
    pIfconfigAllRes = MEM_BufferAlloc(cBytes);
    pPos = pIfconfigAllRes;

    /* Loop through addresses */
    *pPos = iIf;
    pPos++;
    iIf = 0;
    pIfHandle = IP_IF_GetIfByNr(iIf);
    while(*pIfHandle)
    {
        *pPos = iIf; /* if ID */
        pPos++;
        *pPos = ifconfigInfo[iIf]; /* number of addresses */
        pPos++;
#if IP_IP4_ENABLE
        /* Get IPv4 addresse for an interface */
        pIp4AddrData = IP_IF_GetAddrByIf4(pIfHandle);
        if(pIp4AddrData)
        {
            ipAddr_t inIpAddr;

            NWKU_ConvertIp4Addr(pIp4AddrData->ip4Addr,&inIpAddr);
            FLib_MemCpy(pPos, (uint8_t*)&inIpAddr, 16);

            uint8_t aux, idx;
            for(idx=0;idx<8;idx++)
            {
                aux = *(pPos + idx);
                *(pPos + idx) = *(pPos + 15 - idx);
                *(pPos + 15 - idx) = aux;
            }
            pPos += 16;
        }
#endif
        /* Get IPv6 addresses for an interface */
        iAddr = 0;
        pIp6AddrData = IP_IF_GetAddrByIf6(pIfHandle, iAddr);
        while(pIp6AddrData)
        {
            FLib_MemCpy(pPos, (uint8_t*)&pIp6AddrData->ip6Addr, 16);
            uint8_t aux, idx;
            for(idx=0;idx<8;idx++)
            {
                aux = *(pPos + idx);
                *(pPos + idx) = *(pPos + 15 - idx);
                *(pPos + 15 - idx) = aux;
            }
            pPos += 16;

            /* Go to the next IP address */
            iAddr++;
            pIp6AddrData = IP_IF_GetAddrByIf6(pIfHandle, iAddr);
        }

        /* Go to the next interface */
        iIf++;
        pIfHandle = IP_IF_GetIfByNr(iIf);
    }


    /* Set up reply data */
    *pReplyData = (uint8_t*)pIfconfigAllRes;
    *pDataSize = cBytes;
}


/*!*************************************************************************************************
\private
\fn     void FSCI_IfconfigBind(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to bind an IP address to a configured interface.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_IfconfigBind
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    statusConfirm_t *pIfconfigBindRes = MEM_BufferAlloc(sizeof(statusConfirm_t));
    uint8_t idx = 0;
    uint8_t idx2 = 15;
    ipAddr_t ipAddress;
    ifHandle_t* pIfHandle;
    uint32_t bindStatus;

    /* Cast received data to a known structure */
    ifconfigBindParams_t *pIfconfigParams = (ifconfigBindParams_t*)pClientPacket->structured.payload;

    pIfconfigBindRes->status = mFSCI_Err;

    /* Reverse address bytes */
    for(idx=0;idx<16;idx++,idx2--)
    {
        uint8_t *p = ((uint8_t*)&pIfconfigParams->ipAddr) + idx2;
        uint8_t *p2 = ipAddress.addr8;
        p2[idx] = *(p);
    }

    /* Check IP address */
    if(!IP_IsAddrIPv6(&ipAddress))
    {
        //shell_write("Malformed IP address");
    }
    else
    {
        llAddr_t tempSrcLlAddr;

        pIfHandle = IP_IF_GetIfByNr(pIfconfigParams->ifId);
        bindStatus = IP_IF_BindAddr6(pIfHandle, &ipAddress,ip6AddrTypeManual_c,
            IP6_ADDRESS_LIFETIME_INFINITE, 64U);

        /* Get Source Address Type from IPv6 Header */
        NWKU_GetLLADDRFromIID(&ipAddress.addr8[8], &tempSrcLlAddr);

        if(!strstr((char*)(*pIfHandle)->ifNamePtr, "eth"))
        {
            slwpStruct_t * pSlwpStruct = *(slwpStruct_t **)(*pIfHandle)->ifDriverHandle;

            if(tempSrcLlAddr.addrSize == gLlayerAddrEui64_c)
            {
                pSlwpStruct->pMacAbsReq->SetExtendedAddress(
                    *((uint64_t*)tempSrcLlAddr.eui), pSlwpStruct->macInstanceId);
            }
            else if(tempSrcLlAddr.addrSize == gLlayerAddrEui16_c)
            {
                pSlwpStruct->pMacAbsReq->SetShortAddress(
                    ntohas((uint8_t*)tempSrcLlAddr.eui), pSlwpStruct->macInstanceId);
            }
        }

        switch (bindStatus)
        {
            case gIpOk_c:
                pIfconfigBindRes->status = mFSCI_Ok;
                break;

            case gIpNoAddressSpaceError_c:
                pIfconfigBindRes->status = mFSCI_NoSpace;
                break;
        }
    }

    /* Set up reply data */
    *pReplyData = (uint8_t*)pIfconfigBindRes;
    *pDataSize = sizeof(statusConfirm_t);
}

/*!*************************************************************************************************
\private
\fn     bool_t FSCI_Ping(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to send a ping request.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static bool_t FSCI_Ping
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    pingReply_t *pPingRes = MEM_BufferAlloc(sizeof(pingReply_t));
    uint8_t idx = 0;
    uint8_t idx2 = 15;
    ipAddr_t sourceIpAddress;
    ipPktInfo_t *pIpPktInfo;
    bool_t res = TRUE;

    /* Cast received data to a known structure */
    pingParams_t *pPingParams = (pingParams_t*)pClientPacket->structured.payload;

    pPingRes->status = mFSCI_Err;

    /* Reverse address bytes */
    for(idx=0;idx<16;idx++,idx2--)
    {
        uint8_t *p = ((uint8_t*)&pPingParams->destIpAddr) + idx2;
        uint8_t *p2 = mDstIpAddr.addr8;
        p2[idx] = *(p);
    }

    idx = 0;
    idx2 = 15;
    /* Reverse address bytes */
    for(idx=0;idx<16;idx++,idx2--)
    {
        uint8_t *p = ((uint8_t*)&pPingParams->sourceIpAddr) + idx2;
        uint8_t *p2 = sourceIpAddress.addr8;
        p2[idx] = *(p);
    }

    mPingTimeoutMs = *((uint16_t*)pPingParams->timeout);
    if (mPingTimeoutMs < SHELL_PING_MIN_TIMEOUT)
    {
        mPingTimeoutMs = SHELL_PING_MIN_TIMEOUT;
    }

    /* Destination address was set */
    if(!IP_IsAddrEqual(&mDstIpAddr, &in6addr_any))
    {
        /* Create Ping packet */
        pIpPktInfo = PingCreatePktInfo(&mDstIpAddr, *((uint16_t*)pPingParams->length));

        /* If we have a specified source address: set it */
        if(!(IP_IsAddrEqual(&sourceIpAddress, &in6addr_any)))
        {
            IP_AddrCopy(pIpPktInfo->pIpSrcAddr, &sourceIpAddress);
        }

        /* Send packet to ICMP for transmission */
        ICMP_Send(pIpPktInfo, gIcmp6TypeEchoRequest_c, ICMP_CODE_DEFAULT);

        /* Get timestamp */
        pingTimeStamp = TMR_GetTimestamp();

        if (gTmrInvalidTimerID_c == pingTimerID)
        {
            /* Allocate ping timer */
            pingTimerID = TMR_AllocateTimer();

            if (pingTimerID != gTmrInvalidTimerID_c)
            {
                /* Start timer */
                TMR_StartSingleShotTimer(pingTimerID, mPingTimeoutMs, PING_TimerCallback, NULL);
                //shell_printf("Pinging %s with %d bytes of data:\n\r", pIpAddr, pingSize);
            }
            else
            {
                //shell_write("Timer cannot be allocated!");
                //ret = CMD_RET_SUCCESS;
            }
        }
        else
        {
            //shell_write("Timer already allocated!");
            //ret = CMD_RET_SUCCESS;
        }

        /* Set up reply data */
        *pReplyData = (uint8_t*)pPingRes;

        res = FALSE;
    }
    else
    {
        /* Set up reply data */
        pPingRes->status = mFSCI_PingWrongDestAddress;
        *pReplyData = (uint8_t*)pPingRes;
    }

    *pDataSize = sizeof(pingReply_t);

    return res;
}



/*!*************************************************************************************************
\private
\fn     void PING_EchoReplyReceiveAsync(ipPktInfo_t ipPktInfo)
\brief  Interface function for the user app. It handles a received Ping Echo Reply message.

\param  [in]    pIpPktInfo      Pointer to the packet information structure.
***************************************************************************************************/
static void PING_EchoReplyReceiveAsync
(
    ipPktInfo_t *pIpPktInfo
)
{
    NWKU_SendMsg(PING_EchoReplyReceive, (void *)pIpPktInfo, pmMainThreadMsgQueue);
}

/*!*************************************************************************************************
\private
\fn     void PING_EchoReplyReceive(ipPktInfo_t ipPktInfo)
\brief  Interface function for the user app. It handles a received Ping Echo Reply message.

\param  [in]    pIpPktInfo      Pointer to the packet information structure.
***************************************************************************************************/
static void PING_EchoReplyReceive
(
    void *pParam
)
{
    uint16_t echoId;
    uint16_t seqNb;
    //uint32_t payloadLen;
    uint64_t tempTimestamp;
    ipPktInfo_t *pIpPktInfo = (ipPktInfo_t *)pParam;

    if(pmQueuedReplyData)
    {
        /* Reply from desired IP address */
        if(IP_IsAddrEqual(&mDstIpAddr, pIpPktInfo->pIpSrcAddr) || IP6_IsMulticastAddr(&mDstIpAddr))
        {
            /* Get first the echo request identifier */
            //htonas(pIpPktInfo->pNwkBuff->pData, echoId);
            echoId = ntohas(pIpPktInfo->pNextProt);
            if(echoId == PING_ID)
            {
                /* Get the echo request sequence number */
                //htonas(pIpPktInfo->pNwkBuff->pData + sizeof(echoId), defaultSeqNb);
                seqNb = ntohas(pIpPktInfo->pNextProt + sizeof(echoId));
                if(seqNb == defaultSeqNb - 1)
                {
                    /* Get payload length from the ICMP packet.
                     * The ping payload is with an offset of 4 */
                    //payloadLen = pIpPktInfo->nextProtLen - PING_HEADER_SIZE;

                    /* Compare payload */
                    /*if(FLib_MemCmp(pIpPktInfo->pNextProt + sizeof(echoId) + sizeof(seqNb),
                            (void*)PING_PAYLOAD, payloadLen - (sizeof(echoId) + sizeof(seqNb))))
                    {*/

                    tempTimestamp = TMR_GetTimestamp();
                    tempTimestamp -= pingTimeStamp;
                    tempTimestamp /= 1000;
                    pingReply_t *pPingReplyData;

                    pPingReplyData = (pingReply_t*)pmQueuedReplyData;
                    uint16_t tmstp = (uint16_t)tempTimestamp;

                    //htonas((uint8_t*)pPingReplyData->reply, (uint16_t)tempTimestamp);
                    FLib_MemCpy(pPingReplyData->reply, &tmstp, sizeof(pPingReplyData->reply));

                    /* Stop timer */
                    if(pingTimerID != gTmrInvalidTimerID_c)
                    {
                        TMR_FreeTimer(pingTimerID);
                        pingTimerID = gTmrInvalidTimerID_c;
                    }

                    /* Send reply */
                    ((pingReply_t*)pmQueuedReplyData)->status = mFSCI_Ok;
                    /*else
                    {
                        shell_write("Reply payload not matching\n\r");
                    }*/
                }
                else
                {
                    // shell_write("Reply sequence number not matching\n\r");
                    ((pingReply_t*)pmQueuedReplyData)->status = mFSCI_Err;
                }
            }
            else
            {
                // shell_write("Reply PING ID not matching\n\r");
                ((pingReply_t*)pmQueuedReplyData)->status = mFSCI_Err;
            }
        }
        else
        {
            // shell_write("Reply IP source address not matching\n\r");
            ((pingReply_t*)pmQueuedReplyData)->status = mFSCI_Err;
        }

        FSCI_transmitPayload(gFSCI_FlipOpGCnf_c, mQueuedOpCode, pmQueuedReplyData,
            mQueuedDataSize, mQueuedInterfaceId);

        MEM_BufferFree(pmQueuedReplyData);
        pmQueuedReplyData = NULL;
    }

    /* Free the input pIpPktInfo  */
    NWKU_FreeIpPktInfo(&pIpPktInfo);
}

/*!*************************************************************************************************
\private
\fn     void PING_TimerCallback(void *param)
\brief  This function sets the timeout expire value.

\param  [in]    param   unused

\return         void
***************************************************************************************************/
static void PING_TimerCallback(void *param)
{
    NWKU_SendMsg(PING_HandleTimerCallback, param, pmMainThreadMsgQueue);
}

/*!*************************************************************************************************
\private
\fn     void PING_HandleTimerCallback(void *param)
\brief  This function sets the timeout expire value.

\param  [in]    param   unused

\return         void
***************************************************************************************************/
static void PING_HandleTimerCallback(void *param)
{
    /* Ping reply was not received */
    //shell_write("Request timed out");
    /* Stop Ping timer */
    if(pingTimerID != gTmrInvalidTimerID_c)
    {
        TMR_FreeTimer(pingTimerID);
        pingTimerID = gTmrInvalidTimerID_c;
    }

    if(pmQueuedReplyData)
    {
      /* Send reply */
      ((pingReply_t*)pmQueuedReplyData)->status = mFSCI_PingTimeout;
      FSCI_transmitPayload(gFSCI_FlipOpGCnf_c, mQueuedOpCode, pmQueuedReplyData,
          mQueuedDataSize, mQueuedInterfaceId);

      MEM_BufferFree(pmQueuedReplyData);
      pmQueuedReplyData = NULL;
    }
}

/*!*************************************************************************************************
\private
\fn     ipPktInfo_t *PingCreatePktInfo(uint32_t payloadLen)
\brief  This function is used to create the packet info structure needed by ICMP ping.

\param  [in]    pDstAddr    pointer to the destination IP address
\param  [in]    payloadLen  the size of the payload

\return         ipPktInfo_t pointer to a packet info structure
***************************************************************************************************/
static ipPktInfo_t *PingCreatePktInfo(ipAddr_t *pDstAddr, uint32_t payloadLen)
{
    uint16_t echoId = PING_ID;
    ipPktInfo_t *pIpPktInfo = NWKU_CreateIpPktInfo();
    uint16_t idx;
    uint16_t iPayload;
    uint8_t *pPayload;

    /* Allocate and populate pIpPktInfo->pIpDstAddr */
    pIpPktInfo->pIpDstAddr = NWKU_CreateIpAddr();
    IP_AddrCopy(pIpPktInfo->pIpDstAddr, pDstAddr);

    /* Allocate and populate pIpPktInfo->pIpSrcAddr */
    pIpPktInfo->pIpSrcAddr = NWKU_CreateIpAddr();
    /* Determine IP source address based on IP destination address */
    if(IP_IsAddrIPv6(pIpPktInfo->pIpDstAddr))
    {
#if IP_IP6_ENABLE

      ipAddr_t*  pSrcAddr = IP_IF_SelSrcAddr6(NULL, pIpPktInfo->pIpDstAddr);
      IP_AddrCopy(pIpPktInfo->pIpSrcAddr, pSrcAddr);
#endif /* IP_IP6_ENABLE */
    }
    else
    {
#if IP_IP4_ENABLE
      NWKU_ConvertIp4Addr(IP_IF_SelSrcAddr4(pIpPktInfo->pIpDstAddr, pIpPktInfo->pIpPktOptions->ifHandle),
                          pIpPktInfo->pIpSrcAddr);
#endif /* IP_IP4_ENABLE */
    }

    /* Populate pIpPktInfo->pIpPktOptions */
    // TODO: call function to determine interface hop limit
    pIpPktInfo->pIpPktOptions->hopLimit = 0x80U;
    // TODO: pIpPktInfo->pIpPktOptions->lqi
    // TODO: pIpPktInfo->pIpPktOptions->qos
    // TODO: pIpPktInfo->pIpPktOptions->security

    /* Allocate and populate pIpPktInfo->pNwkBuff using Echo request payload and
           echo request identifier and sequence number */
    pIpPktInfo->pNwkBuff = NWKU_CreateNwkBuffer(payloadLen + sizeof(echoId) + sizeof(defaultSeqNb));

    /* Populate first the echo request identifier */
    htonas(pIpPktInfo->pNwkBuff->pData, echoId);

    /* Populate the echo request sequence number with a default value */
    htonas(pIpPktInfo->pNwkBuff->pData + sizeof(echoId), defaultSeqNb++);

    /* Set ping payload: 0x61..0x77(a..w) */
    pPayload = pIpPktInfo->pNwkBuff->pData + sizeof(echoId) + sizeof(defaultSeqNb);
    iPayload = 0;
    for(idx=0;idx<payloadLen;idx++,iPayload++)
    {
        if(iPayload > (PING_PAYLOAD_END - PING_PAYLOAD_START))
        {
            iPayload = 0;
        }
        pPayload[idx] = iPayload + PING_PAYLOAD_START;
    }


    return pIpPktInfo;
}


#if FLIP_VTUN_ROUTER || THREAD_VTUN_ROUTER
/*
 * VTUN Interface functions
 */
/*!*************************************************************************************************
\private
\fn     void FSCI_VtunOpen(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to initialize the Virtual TUN interface.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_VtunOpen
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
}

/*!*************************************************************************************************
\private
\fn     void FSCI_VtunClose(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to expose the getsockopt() function.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_VtunClose
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{

}

/*!*************************************************************************************************
\private
\fn     bool_t FSCI_VtunSend(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used transfer the data from FSCI to VTUN interface.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static bool_t FSCI_VtunSend
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    uint16_t dataSize = *((uint16_t*)pClientPacket->structured.payload);
    uint8_t *pData = MEM_BufferAlloc(dataSize);

    FLib_MemCpy(pData, pClientPacket->structured.payload + 2, dataSize);
    IP_vtunRecv(pData, dataSize);

    /* Set up reply data */
    *pReplyData = NULL;
    *pDataSize = 0;

    return FALSE;
}

#endif /* FLIP_VTUN_ROUTER || THREAD_VTUN_ROUTER*/
/*!*************************************************************************************************
\private
\fn     void FSCI_StartNwk(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to start the network.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_StartNwk
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    stackStartConfig_t *pStackConfig;
    stackConfig_t* pStack = NULL;
    statusConfirm_t* pStatus = MEM_BufferAlloc(sizeof(statusConfirm_t));

    pStatus->status = mFSCI_Err;

    NWK_GetStack(NULL, &pStackConfig, &pStack);

    if (NULL != pStack)
    {
        NWKU_SendMsg(APP_StartDevice, NULL, pmMainThreadMsgQueue);
        pStatus->status = mFSCI_Ok;
    }


    /* Set up reply data */
    *pReplyData = (uint8_t*)pStatus;
    *pDataSize = sizeof(statusConfirm_t);

}
/*!*************************************************************************************************
\private
\fn     void FSCI_SetNwkParams(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to set the network parameters.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_SetNwkParams
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    setNwkParams_t* setParams;
    stackStartConfig_t *pStackConfig;
    stackConfig_t* pStack = NULL;

    statusConfirm_t* pSetParamsConfirm = MEM_BufferAlloc(sizeof(statusConfirm_t));

    setParams = (setNwkParams_t*)pClientPacket->structured.payload;

    /* Get the first stack available (assuming only one stack is available) */
    NWK_GetStack(NULL,&pStackConfig, &pStack);

    pSetParamsConfirm->status = mFSCI_Err;

    if (NULL != pStack)
    {
        NWK_Params_Set(gNwkParamSetChannel_c,(uint8_t*)&setParams->channel,pStack);

        uint16_t panID = ntohas(setParams->panID);
        NWK_Params_Set(gNwkParamSetPanId_c, (uint8_t*)&panID,pStack);

        uint16_t shortAddr = ntohas(setParams->shortAddr);
        NWK_Params_Set(gNwkParamSetShortAddr_c,(uint8_t*)&shortAddr,pStack);

        uint64_t extendedAddr = ntohall(setParams->extendedAddr);
        NWK_Params_Set(gNwkParamSetExtAddr_c,(uint8_t*)&extendedAddr,pStack);

        NWK_Params_Set(gNwkParamsSetRndExtAddr_c,(uint8_t*)&setParams->randomExtAddr,pStack);

        NWK_Params_Set(gNwkParamsSetRxOnIdle_c,(uint8_t*)&setParams->rxOnIdle,pStack);

        pSetParamsConfirm->status = mFSCI_Ok;
    }

    /* Set up reply data */
    *pReplyData = (uint8_t*)pSetParamsConfirm;
    *pDataSize = sizeof(statusConfirm_t);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_MacFilter(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to set the network parameters.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_MacFilter
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
    addMacFilter_t* macFilter;
    statusConfirm_t* pMacFilterConfirm = MEM_BufferAlloc(sizeof(statusConfirm_t));

    macFilter = (addMacFilter_t*)pClientPacket->structured.payload;

    pMacFilterConfirm->status = mFSCI_Err;
    
    uint64_t extendedAddr;
    FLib_MemCpy(&extendedAddr,macFilter->extendedAddr,sizeof(uint64_t));

    uint16_t shortAddr;
    FLib_MemCpy(&shortAddr, macFilter->shortAddr, sizeof(uint16_t));

    uint8_t linkInd = macFilter->linkIndicator;

    MacFiltering_AddNeighbor(extendedAddr, shortAddr, linkInd);

    pMacFilterConfirm->status = mFSCI_Ok;
    

    /* Set up reply data */
    *pReplyData = (uint8_t*)pMacFilterConfirm;
    *pDataSize = sizeof(statusConfirm_t);

}

/*!*************************************************************************************************
\private
\fn     void FSCI_MacFilter(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to set the network parameters.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_SetDHCPServer
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
#if STACK_THREAD
    threadInstance_t *pThreadInstance;
    setNwkData_t* pSetDHCPServer;
    ifHandle_t * pIfHandle;
    uint8_t idx = 0;
    uint8_t idx2 = 15;
    ipAddr_t ipAddress;
    uint64_t extendedAddress;
    uint16_t shortAddr; 
    
    pSetDHCPServer = (setNwkData_t*)pClientPacket->structured.payload;
    dhcpServerSet_t * pDhcpInfo = MEM_BufferAlloc(sizeof(dhcpServerSet_t));
    statusConfirm_t* pStatus = MEM_BufferAlloc(sizeof(statusConfirm_t));

    pStatus->status = mFSCI_Err;

    /* Reverse address bytes */
    for(idx = 0; idx < 16; idx++,idx2--)
    {
        uint8_t *p = ((uint8_t*)&pSetDHCPServer->prefix) + idx2;
        uint8_t *p2 = ipAddress.addr8;
        p2[idx] = *(p);
    }

    pIfHandle = IP_IF_GetIfByNr(pSetDHCPServer->interfaceID);
    /* Get Thread Instance */
    pThreadInstance = Thread_GetInstanceByIf(pIfHandle);
    slwpStruct_t *pSlwpStruct = *((slwpStruct_t**)(*pIfHandle)->ifDriverHandle);
    /* Get extended address */
    extendedAddress = pThreadInstance->pStackCfg->pMacCfg->extendedAddr;
    /* Get short address */
    shortAddr = pSlwpStruct->pMacAbsReq->GetShortAddress(pSlwpStruct->macInstanceId);

    htonall(pDhcpInfo->dhcpEui,extendedAddress);
    pDhcpInfo->dhcpShortAddr = shortAddr;

    uint32_t prefixLength = *((uint16_t*)pSetDHCPServer->prefixLength);

    pDhcpInfo->dhcpStableRoute = pSetDHCPServer->isStable;

    Thread_DataSetDhcpServer(pIfHandle,(uint8_t*)&ipAddress,(uint32_t)prefixLength,pDhcpInfo);
    pStatus->status = mFSCI_Ok;
    
    MEM_BufferFree(pDhcpInfo);
    /* Set up reply data */
    *pReplyData = (uint8_t*)pStatus;
    *pDataSize = sizeof(statusConfirm_t);
#endif /* STACK_THREAD */
}

/*!*************************************************************************************************
\private
\fn     void FSCI_MacFilter(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to set the network parameters.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_SetExtRoute
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
#if STACK_THREAD
    threadInstance_t *pThreadInstance;
    setNwkData_t* pSetDHCPServer;
    ifHandle_t * pIfHandle;
    uint8_t idx = 0;
    uint8_t idx2 = 15;
    ipAddr_t ipAddress;
    uint64_t extendedAddress;
    uint16_t shortAddr;
    pSetDHCPServer = (setNwkData_t*)pClientPacket->structured.payload;
    externalRouteSet_t* pExtRouteSet = MEM_BufferAlloc(sizeof(externalRouteSet_t));
    statusConfirm_t* pStatus = MEM_BufferAlloc(sizeof(statusConfirm_t));

    /* Reverse address bytes */
    for(idx=0;idx<16;idx++,idx2--)
    {
        uint8_t *p = ((uint8_t*)&pSetDHCPServer->prefix) + idx2;
        uint8_t *p2 = ipAddress.addr8;
        p2[idx] = *(p);
    }

    pIfHandle = IP_IF_GetIfByNr(pSetDHCPServer->interfaceID);
    /* Get Thread Instance */
    pThreadInstance = Thread_GetInstanceByIf(pIfHandle);
    slwpStruct_t *pSlwpStruct = *((slwpStruct_t**)(*pIfHandle)->ifDriverHandle);

    /* Get extended address */
    extendedAddress = pThreadInstance->pStackCfg->pMacCfg->extendedAddr;
    /* Get short address */
    shortAddr = pSlwpStruct->pMacAbsReq->GetShortAddress(pSlwpStruct->macInstanceId);

    htonall(pExtRouteSet->brEui,extendedAddress);
                pExtRouteSet->brShortAddr = shortAddr;
    uint32_t prefixLength = *((uint16_t*)pSetDHCPServer->prefixLength);

    pExtRouteSet->brStableRoute= pSetDHCPServer->isStable;

    Thread_DataSetExtRoute(pIfHandle,(uint8_t*)&ipAddress,prefixLength,pExtRouteSet);
    pStatus->status = mFSCI_Ok;
    MEM_BufferFree(pExtRouteSet);
    /* Set up reply data */
    *pReplyData = (uint8_t*)pStatus;
    *pDataSize = sizeof(statusConfirm_t);
#endif /* STACK_THREAD */
}

/*!*************************************************************************************************
\private
\fn     void FSCI_MacFilter(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to set the network parameters.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_IncrementVersion
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
#if STACK_THREAD
    incrementVersion_t* incrementVersion;
    ifHandle_t * pIfHandle;
    uint8_t isStable;

    incrementVersion = (incrementVersion_t*)pClientPacket->structured.payload;

    statusConfirm_t* pIncrementVConfirm = MEM_BufferAlloc(sizeof(statusConfirm_t));


    pIfHandle = IP_IF_GetIfByNr(incrementVersion->interfaceID);
    isStable = incrementVersion->isStable;

    Thread_NwkDataIncrementVersion(pIfHandle, isStable);
    Thread_NwkDataPropagate(pIfHandle, NULL, FALSE, FALSE);
    Thread_NwkDataSleepyNodesPropagate(pIfHandle);
    pIncrementVConfirm->status = mFSCI_Ok;
    /* Set up reply data */
    *pReplyData = (uint8_t*)pIncrementVConfirm;
    *pDataSize = sizeof(statusConfirm_t);
#endif /* STACK_THREAD */
}

/*!*************************************************************************************************
\private
\fn     void FSCI_Register(clientPacket_t *pClientPacket, uint32_t interfaceId,
                                  uint8_t **pReplyData, uint16_t *pDataSize)
\brief  This function is used to set the network parameters.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [in]    interfaceId     id of the FSCI interface
\param  [out]   pReplyData      double pointer to the buffer of data which needs to be sent back
\param  [out]   pDataSize       size of the buffer of data which needs to be sent back

\return         void
***************************************************************************************************/
static void FSCI_Register
(
    clientPacket_t *pClientPacket,
    uint32_t interfaceId,
    uint8_t **pReplyData,
    uint16_t *pDataSize
)
{
#if STACK_THREAD

    ifHandle_t * pIfHandle;
    registerNwkData_t* registerNwkData;

    registerNwkData = (registerNwkData_t*)pClientPacket->structured.payload;

    statusConfirm_t* pStatus = MEM_BufferAlloc(sizeof(statusConfirm_t));


    pIfHandle = IP_IF_GetIfByNr(registerNwkData->interfaceID);

    Thread_ServerDataRegister(pIfHandle);

    pStatus->status = mFSCI_Ok;
    /* Set up reply data */
    *pReplyData = (uint8_t*)pStatus;
    *pDataSize = sizeof(statusConfirm_t);
#endif /* STACK_THREAD */
}

#if DTLS_ENABLED
static dtlsJpakePasswd_t password =
{
    .pSecret = "threadjpaketest",
    .secretLen = sizeof("threadjpaketest") - 1
};

static void DTLS_GetJpakePasswd(dtlsPeerPtr_t pPeer, dtlsJpakePasswd_t **pPasswd)
{
    *pPasswd = &password;
}

static void DTLS_Received
(
    dtlsPeerPtr_t pPeer,
    uint8_t *pData,
    uint32_t len
)
{
    dtlsDataConfirm_t *pDataRes;

    pDataRes = MEM_BufferAlloc(sizeof(dtlsDataConfirm_t) + len);
    if(pDataRes)
    {
        *((uint16_t*)(pDataRes->size)) = (uint16_t)len;
        FLib_MemCpy(pDataRes + sizeof(dtlsDataConfirm_t), pData, len);

        pDataRes->peerIndex = NWKU_GetTblEntry((uint32_t)pPeer, (uint32_t*)maDtlsPeers,
            NumberOfElements(maDtlsPeers));
        FSCI_transmitPayload(gFSCI_FlipOpGCnf_c, gFSCI_DtlsReceive_c, (uint8_t*)pDataRes,
            sizeof(dtlsDataConfirm_t) + len, mQueuedInterfaceId);

        MEM_BufferFree(pDataRes);
    }
}

static void DTLS_Event
(
    dtlsPeerPtr_t pPeer,
    dtlsAlertLevel_t level,
    dtlsAlertCode_t code
)
{
    uint8_t error = 1;
    fsciDtlsConnectConfirm_t connectConfirm;

    /* Check alerts */
    if((level == DTLS_ALERT_LEVEL_OK) && (code == DTLS_ALERT_CONNECTED))
    {
        uint32_t idx;

        idx = NWKU_AddTblEntry((uint32_t)pPeer, (uint32_t*)maDtlsPeers,
            NumberOfElements(maDtlsPeers));

        /* If the peer could be added */
        if(idx != (uint32_t)(-1))
        {
            connectConfirm.status = mFSCI_Ok;
            FSCI_transmitPayload(gFSCI_FlipOpGCnf_c, gFSCI_DtlsClientConnected_c,
                (uint8_t*)&connectConfirm, sizeof(fsciDtlsConnectConfirm_t), mQueuedInterfaceId);

            error = 0;
        }
    }
    else
    {
        /* error */
    }

    if(error)
    {
        connectConfirm.status = mFSCI_Err;
        FSCI_transmitPayload(gFSCI_FlipOpGCnf_c, gFSCI_DtlsClientConnected_c,
            (uint8_t*)&connectConfirm, sizeof(fsciDtlsConnectConfirm_t), mQueuedInterfaceId);
    }
}

/*!*************************************************************************************************
\private
\fn     void FSCI_DtlsOpen(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm)
\brief  This function is used to initialize a DTLS context.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [out]   pConfirm        pointer to the confirm structure

\return         void
***************************************************************************************************/
static void FSCI_DtlsOpen
(
    clientPacket_t *pClientPacket,
    dtlsStatusConfirm_t *pConfirm
)
{
    fsciErrorCode_t error = mFSCI_Err;
    dtlsOpenParams_t *pOpenParams;
    dtlsInitParams_t initParams;
    dtlsContext_t *pCtx;
    uint8_t iCtx;
    sockaddrIn6_t addrStorage;

    /* Cast received data to a known structure */
    pOpenParams = (dtlsOpenParams_t*)pClientPacket->structured.payload;

    initParams.maxRetransmitCnt = pOpenParams->maxRetrCount;
    initParams.retransmitTimeUnits = *((uint16_t*)pOpenParams->timeout) * 10;
    mDtlsCallbacks.event = DTLS_Event;
    mDtlsCallbacks.rcvd = DTLS_Received;
    mDtlsCallbacks.getJpakePasswd = DTLS_GetJpakePasswd;
    
    addrStorage.sin6_family = AF_INET6;
    addrStorage.sin6_port = *((uint16_t*)pOpenParams->port);
    IP_AddrCopy(&addrStorage.sin6_addr, &in6addr_any); /* TODO FSCI: Server addr can be set here */
    pCtx = DTLS_NewContext(&initParams, &mDtlsCallbacks, &addrStorage);
    
    if(pCtx)
    {
        iCtx = NWKU_AddTblEntry((uint32_t)pCtx, (uint32_t*)maDtlsContexts,
            NumberOfElements(maDtlsContexts));
        error = mFSCI_Ok;
    }

    /* Set FSCI command payload */
    if(error == mFSCI_Ok)
    {
        pConfirm->status = iCtx;
    }
    else
    {
        pConfirm->status = mFSCI_Err;
    }
}

/*!*************************************************************************************************
\private
\fn     void FSCI_DtlsOpen(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm)
\brief  This function is used to initialize a DTLS context.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [out]   pConfirm        pointer to the confirm structure

\return         void
***************************************************************************************************/
static void FSCI_DtlsCloseContext
(
    clientPacket_t *pClientPacket,
    dtlsStatusConfirm_t *pConfirm
)
{
    dtlsCloseContextParams_t *pCloseContextParams;
    dtlsContext_t *pCtx;

    /* Cast received data to a known structure */
    pCloseContextParams = (dtlsCloseContextParams_t*)pClientPacket->structured.payload;

    pCtx = (dtlsContext_t*)NWKU_GetTblEntry(pCloseContextParams->contextNumber, (uint32_t*)maDtlsContexts,
        NumberOfElements(maDtlsContexts));

    /* Set FSCI command payload */
    if(pCtx)
    {
        maDtlsContexts[pCloseContextParams->contextNumber] = 0;
        DTLS_FreeContext(pCtx);
        pConfirm->status = mFSCI_Ok;
    }
    else
    {
        pConfirm->status = mFSCI_Err;
    }
}

/*!*************************************************************************************************
\private
\fn     void FSCI_DtlsClosePeer(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm)
\brief  This function is used to initialize a DTLS context.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [out]   pConfirm        pointer to the confirm structure

\return         void
***************************************************************************************************/
static void FSCI_DtlsClosePeer
(
    clientPacket_t *pClientPacket,
    dtlsStatusConfirm_t *pConfirm
)
{
    dtlsClosePeerParams_t *pClosePeerParams;
    dtlsPeerPtr_t pPeer;

    /* Cast received data to a known structure */
    pClosePeerParams = (dtlsClosePeerParams_t*)pClientPacket->structured.payload;

    pPeer = (dtlsPeerPtr_t)NWKU_GetTblEntry(pClosePeerParams->peerNumber, (uint32_t*)maDtlsPeers,
        NumberOfElements(maDtlsPeers));

    /* Set FSCI command payload */
    if(pPeer)
    {
        maDtlsPeers[pClosePeerParams->peerNumber] = 0;
        DTLS_Close(pPeer);
        pConfirm->status = mFSCI_Ok;
    }
    else
    {
        pConfirm->status = mFSCI_Err;
    }
}

/*!*************************************************************************************************
\private
\fn     void FSCI_DtlsConnect(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm)
\brief  This function is used to initialize a DTLS context.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [out]   pConfirm        pointer to the confirm structure

\return         void
***************************************************************************************************/
static void FSCI_DtlsConnect
(
    clientPacket_t *pClientPacket,
    fsciDtlsConnectConfirm_t *pConfirm
)
{
    dtlsConnectParams_t *pConnectParams;
    dtlsContext_t *pCtx;
    sockaddrStorage_t *pSockAddr = MEM_BufferAlloc(sizeof(sockaddrStorage_t));

    /* Cast received data to a known structure */
    pConnectParams = (dtlsConnectParams_t*)pClientPacket->structured.payload;

    pCtx = (dtlsContext_t*)NWKU_GetTblEntry(pConnectParams->contextNumber, (uint32_t*)maDtlsContexts,
        NumberOfElements(maDtlsContexts));

    if(pCtx)
    {
        dtlsPeerPtr_t pPeer;

        /* Create sockAddr structure depending on family */
        ((sockaddrIn6_t*)pSockAddr)->sin6_family = AF_INET6;
        ((sockaddrIn6_t*)pSockAddr)->sin6_port = *((uint16_t*)pConnectParams->port);

        uint8_t idx = 0;
        uint8_t idx2 = 15;

        /* Reverse address bytes */
        for(idx=0;idx<16;idx++,idx2--)
        {
            uint8_t *p = ((uint8_t*)pConnectParams->ipAddr) + idx2;
            uint8_t *p2 = ((sockaddrIn6_t*)pSockAddr)->sin6_addr.addr8;
            p2[idx] = *(p);
        }

        pPeer = DTLS_Connect(pCtx, (sockaddrIn6_t*)pSockAddr);
        pConfirm->peerIdx = (uint8_t)NWKU_AddTblEntry((uint32_t)pPeer, (uint32_t*)maDtlsPeers,
            NumberOfElements(maDtlsPeers));

        if(pConfirm->peerIdx != (uint8_t)(-1))
        {
            pConfirm->status = mFSCI_Ok;
        }
    }
    else
    {
        pConfirm->status = mFSCI_Err;
    }

    MEM_BufferFree(pSockAddr);
}

/*!*************************************************************************************************
\private
\fn     void FSCI_DtlsSend(clientPacket_t *pClientPacket, dtlsStatusConfirm_t *pConfirm)
\brief  This function is used to initialize a DTLS context.

\param  [in]    pClientPacket   pointer to the packet received from FSCI
\param  [out]   pConfirm        pointer to the confirm structure

\return         void
***************************************************************************************************/
static void FSCI_DtlsSend
(
    clientPacket_t *pClientPacket,
    dtlsStatusConfirm_t *pConfirm
)
{
    dtlsSendParams_t *pSendParams;
    dtlsPeerPtr_t pPeer;

    /* Cast received data to a known structure */
    pSendParams = (dtlsSendParams_t*)pClientPacket->structured.payload;

    pPeer = (dtlsPeerPtr_t)NWKU_GetTblEntry(pSendParams->peerNumber, (uint32_t*)maDtlsPeers,
        NumberOfElements(maDtlsPeers));

    /* Set FSCI command payload */
    if(pPeer)
    {
        DTLS_Send(pPeer, pSendParams->data, *((uint16_t*)pSendParams->size));
        pConfirm->status = mFSCI_Ok;
    }
    else
    {
        pConfirm->status = mFSCI_Err;
    }
}

#endif /* DTLS_ENABLED */


#endif /* THREAD_USE_FSCI */

/*==================================================================================================
Private debug functions
==================================================================================================*/
