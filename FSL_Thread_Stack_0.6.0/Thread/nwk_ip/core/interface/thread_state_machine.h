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

#ifndef _THREAD_STATE_MACHINE_H
#define _THREAD_STATE_MACHINE_H
/*!=================================================================================================
\file       thread_state_machine.h
\brief      This is a header file for the Thread module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "stack_manager_if.h"

/*==================================================================================================
Public macros
==================================================================================================*/



/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef enum threadSmStates_tag
{
    mThreadSmIdle_c,
    mThreadSmNwkDiscovery_c,
    mThreadSmCommisioning_c,
    mThreadSmRouterDiscovery_c,
    mThreadSmEligibleRouterDiscovery_c,
    mThreadSmNwkAttach_c,
    mThreadSmAttachComplete_c,
    mThreadSmAttachFail_c
} threadSmStates_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

void Thread_MleDataIndHandler(void *pParams);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _THREAD_STATE_MACHINE_H */
