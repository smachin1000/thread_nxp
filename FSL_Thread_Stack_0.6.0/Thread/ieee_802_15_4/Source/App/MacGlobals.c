/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MacGlobals.c
* This file contains various global variables definitions needed by the 802.15.4 MAC
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
*
*/

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "MacGlobals.h"

#include "MacInterface.h"
#include "PhyInterface.h"
#include "MpmInterface.h"
#include "ModuleInfo.h"
#include "Panic.h"

/************************************************************************************
*************************************************************************************
* Public memory definitions
*************************************************************************************
************************************************************************************/
extern void Mac_Task( task_param_t argument );
extern void Mac_InitializeData( void );

extern char * mMacVersionString;

#if defined(__IAR_SYSTEMS_ICC__)
#pragma location="VERSION_TAGS"
__root const moduleInfo_t MAC_version =
#elif defined(__GNUC__)
const moduleInfo_t MAC_version __attribute__ ((section ("VERSION_TAGS"), used)) =
#else
const moduleInfo_t MAC_version =
#endif
{
    &mMacVersionString,
    {0x85},  // moduleId
    {gMacVerMajor_c, gMacVerMinor_c, gMacVerPatch_c}, // version number
    gMacBuildNo_c,   // build number
};


/* MAC RTOS objects */
task_handler_t gMacTaskHandler;

#if defined(FSL_RTOS_MQX)
  uint32_t        Mac_Task_stack[(gMacTaskStackSize_c+3)/4];
  #define MacTaskStack(i) Mac_Task_stack
#else
  #define MacTaskStack(i) NULL
#endif

/* The maximum number of Indirect transactions */
uint8_t gMacMaxIndirectTransactions;

#if gCslSupport_d
/* The maximum number of CSL transactions */    
const uint8_t gMacMaxCslTransactions = gMacCslMaxSequencesCnt_c;

/* MAC CSL table used for sync */
#if gMacCslTableSize_c > 0
    macCslEntry_t macCslTable[gMacCslTableSize_c];
    const uint8_t gMacCslTableSize = gMacCslTableSize_c;
    #endif
#endif

#if gRitSupport_d
/* The maximum number of RIT transactions */    
const uint8_t gMacMaxRitTransactions = gMacRitMaxSequencesCnt_c;

/* MAC RIT table used for sync */
#if gMacRitTableSize_c > 0
    macRitEntry_t macRitTable[gMacRitTableSize_c];
    const uint8_t gMacRitTableSize = gMacRitTableSize_c;
    #endif
#endif

/* The maximum number MAC instances */
const uint8_t gMacNoOfInstances = gMacInstancesCnt_c;

/* Storage for MAC's internal data */
uint8_t gMacData[gMacInstancesCnt_c][gMacInternalDataSize_c];

/* MAC internal data size. Used for sanity check */
extern const uint16_t gMacLocalDataSize;

#if gMacSecurityEnable_d

const bool_t gMacWipeSecurityTables = TRUE;

/* Allocate memory for the MAC KeyTable and sub-tables */
#if gNumKeyTableEntries_c > 0
    keyDescriptor_t            gPIBKeyTable[gMacInstancesCnt_c][gNumKeyTableEntries_c];
    uint8_t                    gNumKeyTableEntries = gNumKeyTableEntries_c;
    
    #if gNumKeyIdLookupListEntries_c > 0
    keyIdLookupDescriptor_t    gPIBKeyIdLookupDescriptorTable[gMacInstancesCnt_c][gNumKeyIdLookupListEntries_c * gNumKeyTableEntries_c];
    uint8_t                    gNumKeyIdLookupListEntries = gNumKeyIdLookupListEntries_c;
    #endif

#ifndef gMAC2011_d    
    #if gNumKeyDeviceListEntries_c > 0
    keyDeviceDescriptor_t      gPIBKeyDeviceDescriptorTable[gMacInstancesCnt_c][gNumKeyDeviceListEntries_c * gNumKeyTableEntries_c];
    uint8_t                    gNumKeyDeviceListEntries = gNumKeyDeviceListEntries_c;
    #endif
#else /* gMAC2011_d */    
    #if gNumDeviceDescriptorHandleListEntries_c > 0
    uint8_t                    gPIBDeviceDescriptorHandleTable[gMacInstancesCnt_c][gNumDeviceDescriptorHandleListEntries_c * gNumKeyTableEntries_c];
    uint8_t                    gNumDeviceDescriptorHandleListEntries = gNumDeviceDescriptorHandleListEntries_c;
    #endif
#endif /* gMAC2011_d */
    
    #if gNumKeyUsageListEntries_c > 0
    keyUsageDescriptor_t       gPIBKeyUsageDescriptorTable[gMacInstancesCnt_c][gNumKeyUsageListEntries_c * gNumKeyTableEntries_c];
    uint8_t                    gNumKeyUsageListEntries = gNumKeyUsageListEntries_c;
    #endif
#endif

/* Allocate memory for the MAC DeviceTable */
#if gNumDeviceTableEntries_c > 0
    deviceDescriptor_t         gPIBDeviceTable[gMacInstancesCnt_c][gNumDeviceTableEntries_c];
    uint8_t                    gNumDeviceTableEntries = gNumDeviceTableEntries_c;
#endif

/* Allocate memory for the MAC SecurityLevelTable */
#if gNumSecurityLevelTableEntries_c > 0
    securityLevelDescriptor_t  gPIBSecurityLevelTable[gMacInstancesCnt_c][gNumSecurityLevelTableEntries_c];
    uint8_t                    gNumSecurityLevelTableEntries = gNumSecurityLevelTableEntries_c;
#endif

#endif //gMacSecurityEnable_d

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  This function will create the MAC task(s)
*
********************************************************************************** */
void MAC_Init( void )
{
    osa_status_t status;

    gMacMaxIndirectTransactions =
#if gMpmIncluded_d
    gPhyIndirectQueueSize_c/gMpmPhyPanRegSets_c;
#else
    gPhyIndirectQueueSize_c;
#endif
    
    if( gMacLocalDataSize > gMacInternalDataSize_c )
    {
        /* The value of gMacInternalDataSize_c define must be increased */
        panic(0,0,0,0);
        return;
    }

    Mac_InitializeData();

    /* The instance of the MAC is passed at task creaton */
    status = OSA_TaskCreate(Mac_Task, "MAC_Task", gMacTaskStackSize_c, MacTaskStack(i),
                            gMacTaskPriority_c, 0, FALSE, &gMacTaskHandler);
    if( kStatus_OSA_Success != status )
    {
        panic(0,0,0,0);
        return;
    }
}
