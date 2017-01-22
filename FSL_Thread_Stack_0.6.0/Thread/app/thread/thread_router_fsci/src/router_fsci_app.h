/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file ProjectConfig.h
* This file holds type definitions that maps the standard c-types into types
* with guaranteed sizes. The types are target/platform specific and must be edited
* for each new target/platform.
* The header file also provides definitions for TRUE, FALSE and NULL.
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

#ifndef _ROUTER_FSCI_APP_H
#define _ROUTER_FSCI_APP_H
/*!=================================================================================================
\file       router_fsci_app.h
\brief      This is a header file for the router fsci application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"

#include "app_init.h"
#include "sockets.h"

/*==================================================================================================
Public macros
==================================================================================================*/

/* None */

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef enum
{
    gSockDemoDevTypeConcentrator_c = 0x00U,  
    gSockDemoDevTypeNode_c = 0x01U
} sockDemoDevType_t;

typedef struct userSock_tag
{
    int32_t userSockFd;     /* Socket file descriptor */
    tmrTimerID_t timerID;   /* ID for the timer used to poll on this socket */
    bool_t timerPaused;     /* Variable which allow a timer to send message for processing */
    uint32_t index;         /* Index of the file descriptor in the global file descriptors array */
    sockaddrStorage_t *pRemoteAddr;          /* Pointer to sockAddrStorage_t structure(only in tcp active mode) */
} userSock_t;

typedef struct pollInfo_tag
{
    uint8_t *pRequest;
    uint32_t delay;
    userSock_t *pSockList;
    tmrTimerID_t timerId;
    uint32_t currentSock;
    bool_t stop;            /*!< Used to force stop a poll and notify the timer callback to stop and
                                 free this structure */
}pollInfo_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
extern userSock_t       gaUserSockID[]; /* User socket list */
extern sockaddrIn6_t    ssTx;

/*==================================================================================================
Public function prototypes
==================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn     void APP_SockDemo(taskMsgQueue_t *pMainThreadMsgQueue, sockDemoDevType_t deviceType)
\brief  This function is used to initialize the SHELL commands module.

\param  [in]    pMainThreadMsgQueue pointer to the message queue
\param  [in]    deviceType          type of the device(gSockDemoDevTypeConcentrator_c or
                                    gSockDemoDevTypeNode_c)

\return         void
***************************************************************************************************/
void APP_SockDemo(taskMsgQueue_t * pMainThreadMsgQueue, sockDemoDevType_t deviceType);

/*!*************************************************************************************************
\fn     void APP_SockTcpConn(void *pParam)
\brief  Sockets demo application function used for tcp sockets. This function is used for blocking
        calls like connect/accept, which runs in the main task.

\param  [in]    pParam  pointer to the user socket structure

\return         void
***************************************************************************************************/
void APP_SockTcpConn(void *param);

/*!*************************************************************************************************
\fn     void KBD_Callback(uint8_t events)
\brief  This is a callback function called from the KBD module.

\param  [in]    events  value of the events

\return         void
***************************************************************************************************/
void KBD_Callback(uint8_t events);
/*!*************************************************************************************************
\fn     void APP_StartDevice(void)
\brief  This function is used to start device.

\param  [in]     param   pointer not used

\return         void
***************************************************************************************************/
void APP_StartDevice(void *param);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/


#endif /* _ROUTER_FSCI_APP_H */
