/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MacGlobals.h
* This is the header file for the MacGlobals.c
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


#ifndef _MAC_GLOBALS_H_
#define _MAC_GLOBALS_H_

#include "MacConfig.h"
#include "MacInterface.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

#ifndef gMacTaskStackSize_c
#define gMacTaskStackSize_c         (1200)
#endif

#ifndef gMacTaskPriority_c
#define gMacTaskPriority_c          1
#endif

/*! *****************************/
/*** MAC Security Tables sizes **/
/****************************** */
#ifndef gNumKeyTableEntries_c
#define gNumKeyTableEntries_c                       2
#endif

#ifndef gNumKeyIdLookupListEntries_c
#define gNumKeyIdLookupListEntries_c                2
#endif

#ifndef gMAC2011_d
    #ifndef gNumKeyDeviceListEntries_c
    #define gNumKeyDeviceListEntries_c              2
    #endif
#else /* gMAC2011_d */
    #ifndef gNumDeviceDescriptorHandleListEntries_c
    #define gNumDeviceDescriptorHandleListEntries_c 2
    #endif
#endif /* gMAC2011_d */

#ifndef gNumKeyUsageListEntries_c
#define gNumKeyUsageListEntries_c                   2
#endif

#ifndef gNumDeviceTableEntries_c
#define gNumDeviceTableEntries_c                    2
#endif

#ifndef gNumSecurityLevelTableEntries_c
#define gNumSecurityLevelTableEntries_c             2
#endif

/*! ****************************************/
/*** MAC LE Tables sizes and transactions **/
/***************************************** */
#if gCslSupport_d
#define gMacCslTableSize_c          (10)
#define gMacCslMaxSequencesCnt_c    (12)
#endif

#if gRitSupport_d
#define gMacRitTableSize_c          (10)
#define gMacRitMaxSequencesCnt_c    (12)
#endif

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
extern uint8_t gMacData[gMacInstancesCnt_c][gMacInternalDataSize_c];
extern uint8_t gMacMaxIndirectTransactions;
extern const uint8_t gMacNoOfInstances;

/************************************************************************************
*************************************************************************************
* Public functions prototypes
*************************************************************************************
************************************************************************************/

#endif