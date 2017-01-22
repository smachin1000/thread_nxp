
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
\file       thread_nwk_data_globals.c
\brief      This is the source file that contains parameters for the Thread Network Data module that can 
            be configured by the application
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_to_stack_config.h"
#include "thread_network_data.h"
#include "thread_router.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/* Default Values */

#ifndef THR_MAX_INTERFACES
    #define THR_MAX_INTERFACES         1
#endif

#ifndef THREAD_MLE_MAX_NEIGHBORS
    #define THREAD_MLE_MAX_NEIGHBORS         10
#endif

#ifndef THR_LEASE_QUERRY_CACHE_TBL_SIZE 
    #define THR_LEASE_QUERRY_CACHE_TBL_SIZE  5
#endif

#ifndef THREAD_FAST_POLLING_INTERVAL
    #define THREAD_FAST_POLLING_INTERVAL     100      /* Miliseconds */
#endif


#if (THREAD_MLE_MAX_NEIGHBORS != gNumKeyDeviceListEntries_c) && \
    (!MAC_FILTERING_ENABLED)
    #warning Please use the same value for THREAD_MLE_MAX_NEIGHBORS and gNumKeyDeviceListEntries_c
#endif


#if ((THR_MAX_ALLOWED_ROUTERS > THREAD_MLE_MAX_NEIGHBORS) && (!MAC_FILTERING_ENABLED)) || \
    (THR_MAX_ALLOWED_ROUTERS > 64)
    #warning Please update the THR_MAX_ALLOWED_ROUTERS value in app_to_stack_config.h. Note that the maxim values is 64.
#endif        
      
#if (THR_MAX_INTERFACES > IP_IF_NB)      
    #error Please use the same value for THR_MAX_INTERFACES and IP_IF_NB (app_to_stack_config.h)
#endif    
      

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*! Thread Instances Table */
threadInstance_t * maThreadInstances[THR_MAX_INTERFACES];
const uint32_t threadNbOfInstances = THR_MAX_INTERFACES;

/*! Thread MLE Neighbors Table */
mleNeighbor_t* gpMleNeighbors[THREAD_MLE_MAX_NEIGHBORS] = {NULL};
const uint32_t gMleNeighborsTblSize = THREAD_MLE_MAX_NEIGHBORS;

nwkDataInterfaceSet_t* maNwkDataIfTbl[THR_MAX_INTERFACES]= {NULL};
thrLqCacheEntry_t gaThrRouterLqTbl[THR_LEASE_QUERRY_CACHE_TBL_SIZE] = {0};
const uint32_t gThrRouterLqTblSize = THR_LEASE_QUERRY_CACHE_TBL_SIZE;

const uint32_t gThreadFastPollInterval = THREAD_FAST_POLLING_INTERVAL;

const uint32_t gMacSecPollEnabled = MAC_SEC_POLL_ENABLED;

/*==================================================================================================
Private functions
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/

