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

#ifndef _EVENT_MANAGER_H
#define _EVENT_MANAGER_H

/*!=================================================================================================
\file       event_manager.h
\brief      This is a header file for the event_manager module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "EmbeddedTypes.h"
#include "network_utils.h"

/*==================================================================================================
Public macros
==================================================================================================*/

#if defined(__IAR_SYSTEMS_ICC__)
#pragma section="EVM_CODES"
#endif

#if defined(__IAR_SYSTEMS_ICC__)

#define EVM_RegisterStatic(code, pfFunction, ppMsgQueue, bNotifyAsync) _Pragma("location=\"EVM_CODES\"") __root const eventManagerEntry_t \
EvmEntry_##code = { code, pfFunction, ppMsgQueue, bNotifyAsync }

#elif defined(__GNUC__)

#define EVM_RegisterStatic(code, pfFunction, ppMsgQueue, bNotifyAsync ) \
    const eventManagerEntry_t EvmEntry#code __attribute__ ((section ("EVM_CODES"), used)) = \
    { code, pfFunction, ppMsgQueue, bNotifyAsync }

#else

#define EVM_RegisterStatic(code, pfFunction, ppMsgQueue, bNotifyAsync ) \
    const eventManagerEntry_t EvmEntry#code = \
    { code, pfFunction, ppMsgQueue, bNotifyAsync }

#endif

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef void (*pfFunction_t)(void* params);

typedef struct eventManagerEntry_tag
{
    uint32_t code;
    pfFunction_t pfFunction;
    taskMsgQueue_t ** ppMsgQueue;
    bool_t bNotifyAsync;
} eventManagerEntry_t;

typedef struct evmParams_tag
{
    uint32_t code;
    void* pData;
} evmParams_t;
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
\fn    void EVM_EventNotify(uint32_t code)
\brief Notifies the event manager of an event that occurred.

\param [in]   code    event code

\retval       none
***************************************************************************************************/
void EVM_EventNotify(uint32_t code);
/*!*************************************************************************************************
\fn    bool_t EVM_RegisterDynamic(uint32_t code, pfFunction_t pfFunction, taskMsgQueue_t ** ppMsgQueue
                                  bool_t bNotifyAsync)                                                
\brief Registers listening for an event dinamically(at run time). 

\param [in]   code          event code 
\param [in]   pfFunction    callback function for event 
\param [in]   ppMsgQueue    task message queue to use for the callback function
\param [in]   bNotifyAsync  if TRUE the notification callback function will be called 
                            asynchronously by sending a message to the provided task message queue.
                             
\retval       bool_t        TRUE if success
                            FALSE otherwise
***************************************************************************************************/
bool_t EVM_RegisterDynamic(uint32_t code, pfFunction_t pfFunction, taskMsgQueue_t ** ppMsgQueue,
                                        bool_t bNotifyAsync);
/*!*************************************************************************************************
\fn    void EVM_Unregister(uint32_t code, pfFunction_t pfFunction)
\brief Unregisters listening for an event dinamically(at run time).

\param [in]   code          event code
\param [in]   pfFunction    callback function for event

\retval       none
***************************************************************************************************/
void EVM_Unregister(uint32_t code, pfFunction_t pfFunction);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _EVENT_MANAGER_H */
