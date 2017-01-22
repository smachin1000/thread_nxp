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


#ifndef _COAP_H
#define _COAP_H
/*!=================================================================================================
\file       C_coap.h
\brief      This is a header file for the CoAP module. 
==================================================================================================*/


/*==================================================================================================
Include Files
==================================================================================================*/
#include "ip.h"                          
#include "sockets.h"
#include "session.h"
#include "coap_cfg.h"
#include "dtls_config.h"
#include "dtls.h"

#if COAP_ENABLED
/*==================================================================================================
Public macros
==================================================================================================*/
#define COAP_MAX_URI_PATH_OPT_SIZE      6   
#define COAP_MAX_OPTIONS                3
#define COAP_MAX_OPTION_VALUE_SIZE      6

/* CoAP Option Names */
#define COAP_URI_PATH_OPTION            11
#define COAP_CONTENT_FORMAT_OPTION      12
#define COAP_URI_QUERY_OPTION           15


/* Retransmission parameters*/
#define COAP_ACK_TIMEOUT                200   /* milliseconds/10 */ 
#define COAP_MAX_RETRANSMIT             4
#define COAP_ACK_RANDOM_FACTOR          15    /* ACK_RANDOM_FACTOR*10 */ 

#define COAP_TIMER_INTERVAL             100   /* milliseconds */
/*==================================================================================================
Public type definitions
==================================================================================================*/
struct coapSession_tag;

typedef void (* coapCallback_t)(bool_t sessionStatus, void* pData, struct coapSession_tag* pSession, 
                                    uint32_t dataLen);

typedef struct coapSession_tag
{     
    uint8_t msgType;
    uint8_t coapClass;
    uint8_t detail;
    uint8_t tokenLen;        
    uint16_t messageID;
    uint16_t token; 
    uint8_t* pUriQuery;
    coapCallback_t pCallback;
    ifHandle_t* pIfHandle;
    ipAddr_t remoteAddr;
    list_t optionList;
    bool_t isSecured;
}coapSession_t;

typedef struct coapOptionDetails_tag
{
    uint8_t optName;
    uint8_t optValue[COAP_MAX_OPTION_VALUE_SIZE];
    uint8_t optValueLen;
}coapOptionDetails_t;

typedef enum
{
    gCoapSuccess_c,     
    gCoapFailure_c
}coapSessionStatus_t;


typedef enum
{
    gCoapConfirmable_c          = 0,
    gCoapNonConfirmable_c       = 1,
    gCoapAcknowledgement_c      = 2,
    gCoapReset_c                = 3
}coapMessageTypes_t;

typedef enum
{
    gCoapRequest_c              = 0,
    gCoapSuccessResponse_c      = 2,
    gCoapClientErrorResponse_c  = 4,
    gCoapServerErrorRespons_c   = 5
}coapClassCodes_t;

typedef enum
{
    gCoapGET_c                  = 1,
    gCoapPOST_c                 = 2,
    gCoapPUT_c                  = 3,
    gCoapDELETE_c               = 4
}coapMethodCodes_t;

typedef enum
{    
    gCreated_c                  = 1,
    gDeleted_c                  = 2,    
    gValid_c                    = 3,
    gChanged_c                  = 4,
    gContent_c                  = 5    
}coapResponseCodes_t;


typedef struct coapCallbackStruct_tag
{
    uint8_t uriPath[COAP_MAX_URI_PATH_OPT_SIZE];
    coapCallback_t pCallback;     
}coapCallbackStruct_t;


typedef struct coapStartSecParams_tag
{   
    sockaddrIn6_t* pServerAddr;
    sockaddrIn6_t* pLocalAddr;
    uint8_t maxRetransmitCnt;
    uint32_t retransmitTimeUnits;    
#if DTLS_ENABLED
    void (*event)(dtlsPeerPtr_t pPeer, dtlsAlertLevel_t level, dtlsAlertCode_t code);
    void (*getJpakePasswd)(dtlsPeerPtr_t pPeer, dtlsJpakePasswd_t **pPasswd);
#endif    
}coapStartSecParams_t;


typedef struct coapRegCbParams_tag
{
    coapCallback_t pCallback;           /*!< pointer to the callback function */
    uint8_t* optName;                   /*!< array of tags that identify the waiting CoAP message */    
    uint32_t uriLen;                    /*!< the length of the array */
}coapRegCbParams_t;
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
\fn    void COAP_Init(taskMsgQueue_t *pTaskMsgQueue)                                                   
\brief  This function initializes the CoAP module.

\param [in]   pTaskMsgQueue     Pointer to message task queue.      
***************************************************************************************************/
void COAP_Init(taskMsgQueue_t *pTaskMsgQueue);

/*!*************************************************************************************************
\fn    void COAP_SendMsg(coapSession_t*  pSession, void* pData, uint32_t payloadLen)                                                   
\brief  This function builds and transmits a CoAP message. 

\param [in]     pSession        Pointer to CoAP session.   
\param [in]     pData           Pointer to data payload.     
\param [in]     payloadLen      Payload length.
***************************************************************************************************/
void COAP_SendMsg(coapSession_t*  pSession,void* pData,uint32_t payloadLen);

/*!*************************************************************************************************
\fn    bool_t COAP_Start(coapStartSecParams_t* pCoapStartParams)
\brief  This function creates the socket for CoAP module. Call this function with NULL parameter if 
        the transmission is not over DTLS
    
\param [in]   pCoapStartParams      Pointer to initialisation structure for a secure transmission 
                                    over DTLS
\retval       bool_t                TRUE if CoAP started successfully
                                    FALSE, otherwise
***************************************************************************************************/
bool_t COAP_Start(coapStartSecParams_t* pCoapStartParams);

#if 0
/*!*************************************************************************************************
\fn    void COAP_SendAck(coapSession_t*  pSession, void* pData, uint32_t payloadLen)                                                   
\brief  This function builds and transmits a CoAP message. 

\param [in]     pSession        Pointer to CoAP session.   
\param [in]     pData           Pointer to data payload.     
\param [in]     payloadLen      Payload length.
***************************************************************************************************/
void COAP_SendAck(coapSession_t*  pSession, void* pData,uint32_t payloadLen);
#endif

/*!*************************************************************************************************
\fn    void COAP_RegisterRcvCallbacks(coapCallback_t pCallback, uint16_t optName)                                                   
\brief  This function registers a callback for a given uri-path name. 

\param [in]  pCallback  Pointer to callback function.
\param [in]  optName    The name of the uri-path.
***************************************************************************************************/
void COAP_RegisterRcvCallbacks(coapRegCbParams_t* pCallbacksStruct,uint32_t nbOfCallbacks);

/*!*************************************************************************************************
\fn    void COAP_CloseSession(coapSession_t* pSession)                                                   
\brief  This function deletes a CoAP session when completed. 

\param [in]   pSession  Pointer to CoAP session to be deleted.     
***************************************************************************************************/
void COAP_CloseSession(coapSession_t* pSession);

/*!*************************************************************************************************
\fn    void COAP_OpenSession(void)                                                   
\brief  This function registers a CoAP session.

\param [in]   none      
***************************************************************************************************/
coapSession_t* COAP_OpenSession(void);

/*!*************************************************************************************************
\fn    void COAP_AddOptionToList(coapSession_t* pSession,uint8_t optName,uint8_t* optValue,uint8_t optValueLen)
\brief  This function adds the options named by application to a list.

\param [in]  pSession       Pointer to CoAP session.
\param [in]  optName        The name of the uri-path.
\param [in]  optValue       The value of the option.
\param [in]  optValueLen    The length of the option value.
***************************************************************************************************/
void COAP_AddOptionToList(coapSession_t* pSession, uint8_t optName, uint8_t* optValue,uint8_t optValueLen);

#endif /* COAP_ENABLED */
#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _COAP_H */

