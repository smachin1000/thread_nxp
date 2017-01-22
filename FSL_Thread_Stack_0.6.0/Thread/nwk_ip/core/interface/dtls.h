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


#ifndef _DTLS_H
#define _DTLS_H


/*!=================================================================================================
\file       DTLS.h
\brief      This is a header file for the DTLS module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "dtls_config.h"

#if DTLS_ENABLED
#include "sockets.h"

#include "EmbeddedTypes.h"
#include "TimersManager.h"
#include "GenericList.h"
#include "network_utils.h"


/*==================================================================================================
Public macros
==================================================================================================*/


/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef void* dtlsPeerPtr_t;                        /*!< DTLS Peer Identifier */
/* TBD
typedef struct dtlsInitFuncs_tag{
    void (*pfInitNode)(void);
    void (*pfInitJpake)(void);
    //void (*pfInitPSK)(void);
} dtlsInitFuncs_t;
*/

typedef struct dtls_InitParams_t{
    uint8_t maxRetransmitCnt;                       /*!< Number of message retransmissions. */
    uint32_t retransmitTimeUnits;                   /*!< Number of time units to retrasmit a packet */
} dtlsInitParams_t;

typedef struct dtls_jpakePasswd_tag
{
    uint32_t  secretLen;                            /*!< length of secret */
    uint8_t*  pSecret;                              /*!< secret data */
} dtlsJpakePasswd_t;

/*! Known cipher suites.*/
typedef enum
{
    DTLS_ECJPAKE_AES_128_CCM_8 = 0xC0FF,            /*!< ECJPAKE cipher */
} dtlsCipher_t;

typedef enum
{
    TLS_COMPRESSION_NULL = 0x00                     /*!< NULL compression */
} dtlsCompression_t;

typedef enum
{
    DTLS_ALERT_LEVEL_OK         = 0,
    DTLS_ALERT_LEVEL_WARNING    = 1,
    DTLS_ALERT_LEVEL_FATAL      = 2
} dtlsAlertLevel_t;

typedef enum
{
    DTLS_ALERT_CLOSE_NOTIFY = 0,			        /*!< close_notify */
    DTLS_ALERT_UNEXPECTED_MESSAGE = 10,		        /*!< unexpected_message */
    DTLS_ALERT_BAD_RECORD_MAC = 20,		            /*!< bad_record_mac */
    DTLS_ALERT_RECORD_OVERFLOW = 22,		        /*!< record_overflow */
    DTLS_ALERT_HANDSHAKE_FAILURE = 40,		        /*!< handshake_failure */
    DTLS_ALERT_ILLEGAL_PARAMETER = 47,		        /*!< illegal_parameter */
    DTLS_ALERT_UNKNOWN_CA = 48,			            /*!< unknown_ca */
    DTLS_ALERT_ACCESS_DENIED = 49,		            /*!< access_denied */
    DTLS_ALERT_DECODE_ERROR = 50,			        /*!< decode_error */
    DTLS_ALERT_DECRYPT_ERROR = 51,		            /*!< decrypt_error */
    DTLS_ALERT_PROTOCOL_VERSION = 70,		        /*!< protocol_version */
    DTLS_ALERT_INSUFFICIENT_SECURITY = 71,	        /*!< insufficient_security */
    DTLS_ALERT_INTERNAL_ERROR = 80,		            /*!< internal_error */
    DTLS_ALERT_USER_CANCELED = 90,		            /*!< user_canceled */
    DTLS_ALERT_NO_RENEGOTIATION = 100,		        /*!< no_renegotiation */
    DTLS_ALERT_UNSUPPORTED_EXTENSION = 110,	        /*!< unsupported_extension */

    /*! Internal events sent to App */
    DTLS_ALERT_CONNECTED  = 0xB0U,                  /*!< Signal when peers were successfully
                                                    connected */

    /*TODO: Add internal alert code */
} dtlsAlertCode_t;

typedef enum dtlsErr_tag
{
    gDtlsErrOK_c       = 0x00U,
    gDtlsErrInvalidDestination_c,
    gDtlsErrGenericError_c    = 0xFFU
} dtlsErr_t;

struct dtlsContext_t;

typedef struct dtlsCallbacks_tag
{
    void (*rcvd)(dtlsPeerPtr_t pPeer, uint8_t * pData, uint32_t len);
    void (*event)(dtlsPeerPtr_t pPeer, dtlsAlertLevel_t level, dtlsAlertCode_t code);
    void (*getJpakePasswd)(dtlsPeerPtr_t pPeer, dtlsJpakePasswd_t **pPasswd);
} dtlsCallbacks_t;

/*<! Keep track of all information for DTLS module. */
typedef struct dtlsContext_tag
{
    uint32_t fd;
    dtlsInitParams_t initParams;
    tmrTimerID_t timerId;
    list_t peersList;
    list_t retransQueue;
    dtlsCallbacks_t* cb;
    uint8_t aCookieSecret[16];
} dtlsContext_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/
/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
void DTLS_Init(taskMsgQueue_t *pDtlsTaskMsgQueue);
/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
dtlsContext_t *DTLS_NewContext(dtlsInitParams_t* pDtlsInitParams, dtlsCallbacks_t *pCb,
    sockaddrIn6_t *pAddrStorage);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
void DTLS_FreeContext(dtlsContext_t *pCtx);



/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
dtlsPeerPtr_t DTLS_Connect(dtlsContext_t *pCtx, sockaddrIn6_t * pAddr);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
dtlsErr_t DTLS_Send(dtlsPeerPtr_t peer, uint8_t *pData, uint32_t len);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
dtlsErr_t DTLS_Close(dtlsPeerPtr_t pPeer);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
 ***************************************************************************************************/
void DTLS_CloseAllPeers(dtlsContext_t  *pCtx);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
sockaddrStorage_t *DTLS_GetPeerName(dtlsPeerPtr_t pPeer);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
dtlsPeerPtr_t* DTLS_GetPeer(dtlsContext_t *pCtx, ipAddr_t *pIpAddr);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
uint8_t DTLS_GetPeerType(dtlsPeerPtr_t pPeer);

#endif /*DTLS_ENABLED*/
/*================================================================================================*/
#endif
