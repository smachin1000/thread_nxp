
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
\file       nd_globals.c
\brief      This is the source file that contains parameters for the ND module that can be configured
            by the application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_to_stack_config.h"
#include "nd.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/* Default Values */

/*! Maximum number of entries in the neighbor cache */
#ifndef ND_NEIGHBOR_CACHE_SIZE
   #define ND_NEIGHBOR_CACHE_SIZE       10
#endif

/*! Maximum number of entries in the Prefix list */
#ifndef ND_PREFIX_LIST_SIZE
   #define ND_PREFIX_LIST_SIZE          4
#endif

/*! Maximum number of entries in the Destination Cache */
#ifndef ND_DESTINATION_CACHE_SIZE
   #define ND_DESTINATION_CACHE_SIZE    4
#endif

/* Configuration macros consistency checks. */

#if ND_NEIGHBOR_CACHE_SIZE  < 1
    #error ND_NEIGHBOR_CACHE_SIZE should be > 0
#endif

#if ND_PREFIX_LIST_SIZE < 1
    #error ND_PREFIX_LIST_SIZE should be > 0
#endif

#if ND_DESTINATION_CACHE_SIZE < 1
    #error ND_DESTINATION_CACHE_SIZE should be > 0
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

/*! ND Configs Table */
ndCfg_t* aNdCfg[IP_IF_NB] = {NULL};

/*! ND Configs Table Size */
const uint32_t ndCfgTblSize = IP_IF_NB;


/*! Neighbor Cache */
ndNeighborEntry_t* aNeighborCache[ND_NEIGHBOR_CACHE_SIZE] = {NULL};

/*! Neighbor Cache Size */
const uint32_t ndNeighborCacheSize = ND_NEIGHBOR_CACHE_SIZE;


/*! Prefix List */
ndPrefixEntry_t* aPrefixList[ND_PREFIX_LIST_SIZE] = {NULL};

/*! Prefix List Size */
const uint32_t ndPrefixListSize = ND_PREFIX_LIST_SIZE;


/*! Destination Cache */
ndDstCacheEntry_t* aDestinationCache[ND_DESTINATION_CACHE_SIZE] = {NULL};

/*! Destination Cache Size */
const uint32_t ndDestinationCacheSize = ND_DESTINATION_CACHE_SIZE;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/

