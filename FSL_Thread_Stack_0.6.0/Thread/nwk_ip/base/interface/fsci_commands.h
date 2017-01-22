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
#ifndef _FSCI_COMMANDS_H
#define _FSCI_COMMANDS_H
/*!=================================================================================================
\file       app_fsci_commands.h
\brief      This is a header file for the stack FSCI commands.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "sockets.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#define FSCI_FLIP_INTERFACE     (0U)

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef enum nwkOpGroup_tag
{
    gFSCI_FlipOpGReq_c          = 0xCEU,
    gFSCI_FlipOpGCnf_c          = 0xCFU
}nwkOpGroup_t;

typedef enum nwkOpCode_tag
{
    gFSCI_FlipBsdSocket_c       = 0x00U,
    gFSCI_FlipBsdShutdown_c     = 0x01U,
    gFSCI_FlipBsdBind_c         = 0x02U,
    gFSCI_FlipBsdSend_c         = 0x03U,
    gFSCI_FlipBsdSendto_c       = 0x04U,
    gFSCI_FlipBsdRecv_c         = 0x05U,
    gFSCI_FlipBsdRecvfrom_c     = 0x06U,
    gFSCI_FlipBsdConnect_c      = 0x07U,
#if BSDS_STREAM_SUPPORT
    gFSCI_FlipBsdListen_c       = 0x08U,
    gFSCI_FlipBsdAccept_c       = 0x09U,
#endif
    gFSCI_FlipBsdSetsockopt_c   = 0x0AU,
    gFSCI_FlipBsdGetsockopt_c   = 0x0BU,
    gFSCI_FlipIfconfigBind_c    = 0x0CU,
    gFSCI_FlipIfconfigAll_c     = 0x0DU,
    gFSCI_FlipPing_c            = 0x0EU,

    /* Thread Network Parameters */
    gFSCI_StartNetwork_c        = 0x10U,
    gFSCI_SetNwkParams_c        = 0x11U,
    gFSCI_MacFilter_c           = 0x12U,

    /* Thread Network Data */
    gFSCI_SetDHCPServer_c       = 0x20U,
    gFSCI_SetExtRoute_c         = 0x21U,
    gFSCI_IncrementVersion_c    = 0x22U,
    gFSCI_Register_c            = 0x23U,

    /* DTLS */
    gFSCI_DtlsOpen_c            = 0x30,
    gFSCI_DtlsCloseContext_c    = 0x31,
    gFSCI_DtlsClosePeer_c       = 0x32,
    gFSCI_DtlsConnect_c         = 0x33,
    gFSCI_DtlsClientConnected_c = 0x34,
    gFSCI_DtlsSend_c            = 0x35,
    gFSCI_DtlsReceive_c         = 0x36,

    gFSCI_FlipVtunOpen_c        = 0xF0U,    /* APP PROC --> FSCI --> VTUN --> 802.15.4 */
    gFSCI_FlipVtunClose_c       = 0xF1U,    /* APP PROC --> FSCI --> VTUN --> 802.15.4 */
    gFSCI_FlipVtunSend_c        = 0xF2U,    /* APP PROC --> FSCI --> VTUN --> 802.15.4 */
    gFSCI_FlipVtunReceive_c     = 0xF3U,    /* APP PROC <-- FSCI <-- VTUN <-- 802.15.4 */
}nwkOpCode_t;

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
\fn     void APP_FsciInterface(void)
\brief  This function is used to initialize the FSCI communication.

\param  [in]    pointer to the message queue

\return         void
***************************************************************************************************/
void APP_FsciInterface(taskMsgQueue_t *pMainThreadMsgQueue);


#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _FSCI_COMMANDS_H */


