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

/*!=================================================================================================
\file       stack_start.c
\brief      This is a public source file for the initial system startup module. It contains
            the implementation of the interface functions.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_to_stack_config.h"
#include "stack_manager_if.h"
#include "nwk_params.h"
#include "app_init.h"
#include "stack_config.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/* None */

/*==================================================================================================
Private type definitions
==================================================================================================*/

/* None */

/*==================================================================================================
Private prototypes
==================================================================================================*/

/* None */

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

/* None */

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/* None */

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn     void Stack_Start( void* pStartStruct)
\brief  Start thread stack 

\param  [in]    argument    network start params

\return         void
***************************************************************************************************/
void Stack_Start
(
    void* pStartStruct
)
{  
  
    if (NULL != pStartStruct)
    {
        nwkStartParams_t* pNwkStartParams = (nwkStartParams_t*)pStartStruct;   
        if (FALSE == (stackConfig_t*)pNwkStartParams->pStack->isStarted)
        {
            pNwkStartParams->pStackConfig->StackStart((stackConfig_t*)pNwkStartParams->pStack, &appThreadMsgQueue);
        }
        MEM_BufferFree(pStartStruct);
    }
  
    else
    {            
        uint8_t cStackStartCfg = gSTACK_START_CFG_entries_d;
        stackStartConfig_t *pStartStackConfig = (stackStartConfig_t*)gSTACK_START_CFG_startAddr_d;

        /* Start all configurations requested by the user */
        while(cStackStartCfg--)
        {
            uint8_t idx = 0;
            while(pStartStackConfig->pStackParam[idx])
            {
                pStartStackConfig->StackStart(pStartStackConfig->pStackParam[idx++], &appThreadMsgQueue);
            }
        }
    }    
}

/*==================================================================================================
Private functions
==================================================================================================*/

/* None */

/*==================================================================================================
Private debug functions
==================================================================================================*/
