
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
\file       sixlowpan_globals.c
\brief      This is the source file that contains parameters for the 6LOWPAN module that can
            be configured by the application
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_to_stack_config.h"
#include "sixlowpan.h"
#include "sixlowpan_tbl.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/* Default Values */

/*! Number of 6LoWPAN Instances */
#ifndef SLWPCFG_INSTANCES_NB
   #define SLWPCFG_INSTANCES_NB                 1
#endif

#ifndef SLWPCFG_RFC6282_CONTEXT_TABLE_SIZE
    #define SLWPCFG_RFC6282_CONTEXT_TABLE_SIZE  16
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

slwpStruct_t    *apSlwpStruct[SLWPCFG_INSTANCES_NB] = {NULL};
adpIb_t         *apAdpIb[SLWPCFG_INSTANCES_NB] = {NULL};
const uint32_t sixlowpanInstancesNb = SLWPCFG_INSTANCES_NB;

/*! Context Table */
ndContextEntry_t* aContextTable[SLWPCFG_RFC6282_CONTEXT_TABLE_SIZE] = {NULL};

/*! Context Table Size */
const uint32_t slwpContextTableSize = SLWPCFG_RFC6282_CONTEXT_TABLE_SIZE;

/*==================================================================================================
Private functions
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/

