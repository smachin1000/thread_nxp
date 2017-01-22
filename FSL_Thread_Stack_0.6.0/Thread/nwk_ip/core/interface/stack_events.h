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

#ifndef _STACK_EVENTS_H
#define _STACK_EVENTS_H
/*!=================================================================================================
\file       stack_events.h
\brief      This is a header file for the IP Stack.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "nd_events.h"

/*==================================================================================================
Public macros
==================================================================================================*/



/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef uint32_t stackEvmCodes_t;

typedef enum
{
    gStackEvJoinSuccess_c           = 0x00020000,    
    gStackEvJoinFailed_c            = 0x00020001,
    gStackEvDisconnected_c          = 0x00020002,
    gStackEvRequestingGlobalAddr_c  = 0x00020003,
    gStackEvGlobalAddrAssigned_c    = 0x00020004,
    gStackEvDeviceBecameLeader_c    = 0x00020005,
    gStackEvDeviceCreatedNwk_c      = 0x00020006,
    gStackEvRequestingRouterId_c    = 0x00020007
} stackEvmCodes_e;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/



/*==================================================================================================
Public function prototypes
==================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

/*================================================================================================*/
#endif  /* _STACK_EVENTS_H */
