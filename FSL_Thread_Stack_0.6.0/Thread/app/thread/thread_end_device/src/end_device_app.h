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

#ifndef _END_DEVICE_APP_H
#define _END_DEVICE_APP_H

/*!=================================================================================================
\file       end_device_app.h
\brief      This is a header file for the end device demo application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "EmbeddedTypes.h"
#include "sockets.h"
#include "app_init.h"
#include "socket_app_utils.h"

/*==================================================================================================
Public macros
==================================================================================================*/

/* None */

/*==================================================================================================
Public type definitions
==================================================================================================*/

/* None */

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/* None */

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


#endif /* _END_DEVICE_APP_H */
