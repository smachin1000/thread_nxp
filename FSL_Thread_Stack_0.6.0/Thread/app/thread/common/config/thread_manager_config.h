
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

#ifndef _THREAD_MANAGER_CONFIG_H
#define _THREAD_MANAGER_CONFIG_H
/*!=================================================================================================
\file       thread_manager_config.h
\brief      This is a header file for the Thread stack configuration.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_to_stack_config.h"
#include "stack_config.h"

#include "stack_manager_if.h"
#include "network_utils.h"
#include "app_init.h"

#if STACK_THREAD

#include "thread_manager.h"

/*==================================================================================================
Public macros
==================================================================================================*/


/*******************************/
/* THREAD Stack Configurations */
/*******************************/

/* Network Common Configuration */

#define THREAD_PAN_ID                   0xFACE

#define THREAD_CHANNEL                  26

#define THREAD_RND_EXT_ADDR_ENABLED     TRUE

#define THREAD_ULA_PREFIX               0xFD, 0x00, 0x0D, 0xB8, 0x00, 0x00, 0x00, 0x00, \
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

#define THREAD_SECURITY_MATERIAL        .nodeKey = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, \
                                                    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}, \
                                        .keySequenceCounter = 0

/* Node Addresses */
#define THREAD_EXT_ADDR                 0x146E0A0000000001
                                          

/* Node Neighbors */
#define THREAD_NODE_NEIGHBORS            {0, 0, 0}

/*!=================================================================================================
   6LoWPAN Common Configuration 
==================================================================================================*/

#if (SLWPCFG_FRAGMENTATION_ENABLED)
    #define SLP_FRAGMENTATION_DEFAULT_CONFIGURATION \
                                        .datagramTag = 0,
#else
    #define SLP_FRAGMENTATION_DEFAULT_CONFIGURATION
#endif

#define SLP_BSTRAP_DEFAULT_CONFIGURATION
#define SLP_ROUTING_DEFAULT_CONFIGURATION

/*!=================================================================================================
   MPL Common Configuration 
==================================================================================================*/

#if MPL_MULTIPLE_SEED_ID_LENGTH_ENABLED
    #define THREAD_MPL_CFG_SEED_ID      .seedIdLength = 2
#else
    #define THREAD_MPL_CFG_SEED_ID
#endif

#define THREAD_STACK_MPL_CFG_ROUTER     .seedLifetime = 1800000,     /* 1800000 ms (30 min) */ \
                                        .Imin = 50,                  /* ms */ \
                                        .Imax = 250,                 /* ms */ \
                                        .k = 0xFF,                   /* infinite */ \
                                        .nbOfTimerExpirations = 3, \
                                        THREAD_MPL_CFG_SEED_ID

#define THREAD_STACK_MPL_CFG_ENDDEVICE  .seedLifetime = 1800000,     /* 1800000 ms (30 min) */ \
                                        .Imin = 50,                  /* ms */ \
                                        .Imax = 250,                 /* ms */ \
                                        .k = 0xFF,                   /* infinite */ \
                                        .nbOfTimerExpirations = 0, \
                                        THREAD_MPL_CFG_SEED_ID

#define THREAD_STACK_MPL_ADDR_ROUTER    IN6ADDR_LINKLOCAL_ALLNODES_INIT, \
                                        IN6ADDR_LINKLOCAL_ALLROUTERS_INIT, \
                                        IN6ADDR_REALMLOCAL_ALLNODES_INIT, \
                                        IN6ADDR_REALMLOCAL_ALLROUTERS_INIT, \
                                        INADDR_ANY_INIT

#define THREAD_STACK_MPL_ADDR_ENDDEVICE IN6ADDR_LINKLOCAL_ALLNODES_INIT, \
                                        IN6ADDR_REALMLOCAL_ALLNODES_INIT, \
                                        INADDR_ANY_INIT

/*================================================================================================*/

                                          
/*==================================================================================================
 Private macros
==================================================================================================*/

/* Select the device type */
#if THREAD_DEFAULT_CAN_CREATE_NEW_NETWORK
    #define THREAD_DEVICE_TYPE          gThreadDevTypeRouter_c
#elif THREAD_DEFAULT_CAN_BECOME_ACTIVE_ROUTER
    #define THREAD_DEVICE_TYPE          gThreadDevTypeEligibleRouter_c
#else
    #define THREAD_DEVICE_TYPE          gThreadDevTypeEndDevice_c     
#endif                                          
                 

/* MPL configuration */          
#if THREAD_DEFAULT_IS_POLLING_END_DEVICE             
    #define THREAD_STACK_MPL_CFG        THREAD_STACK_MPL_CFG_ENDDEVICE 
#else
    #define THREAD_STACK_MPL_CFG        THREAD_STACK_MPL_CFG_ROUTER
#endif                                          
                                       
#if THREAD_DEFAULT_IS_POLLING_END_DEVICE             
    #define THREAD_STACK_MPL_ADDR        THREAD_STACK_MPL_ADDR_ENDDEVICE 
#else
    #define THREAD_STACK_MPL_ADDR        THREAD_STACK_MPL_ADDR_ROUTER
#endif    

#if THREAD_DEFAULT_IS_POLLING_END_DEVICE                                           
    #define RX_ON_IDLE                  FALSE
#else  
    #define RX_ON_IDLE                  TRUE
#endif    
    
#define THREAD_MAC_CFG \
                                        .extendedAddr = THREAD_EXT_ADDR, \
                                        .panId = THREAD_PAN_ID, \
                                        .randomExt = THREAD_RND_EXT_ADDR_ENABLED, \
                                        .rxOnIdle = RX_ON_IDLE, \
                                        .pollInterval = 3000, \
                                        .channel = THREAD_CHANNEL, \
                                        .pMacFilteringTbl = (uint8_t *)&mThreadICanHearYouTbl

#if (SLWPCFG_MESH_ENABLED)                                 
    #define THREAD_SLP_MESH_CONFIGURATION \
                                        .adpMeshEnable = gAdpMeshOnly_c, \
                                        .adpMeshIb.adpMaxHops = SLWPIB_ADP_MAX_HOPS, \
                                        .adpFwdIb.init = NULL, \
                                        .adpFwdIb.isMeshNeeded = adpThreadIsMeshNeeded, \
                                        .adpFwdIb.ucastGetNextHop = adpThreadGetParent, \
                                        .adpFwdIb.mcastForward = NULL, \
                                        .adpRouteTbl = NULL                                                   
#else
    #define THREAD_SLP_MESH_CONFIGURATION
#endif
                                        
                                          
                                          
#define THREAD_ADP_IB                   .adpDeviceType = gAdpDeviceTypeNone_c, \
                                        .adpHcEnable = gAdpHcRFC6282_c, \
                                        .adpIIDType = gAdpIIDWithoutPanId_c, \
                                        .adpActiveKeyIndex = 0, \
                                        .msduHandle = 0, \
                                        SLP_FRAGMENTATION_DEFAULT_CONFIGURATION \
                                        THREAD_SLP_MESH_CONFIGURATION \
                                        SLP_BSTRAP_DEFAULT_CONFIGURATION \
                                        SLP_ROUTING_DEFAULT_CONFIGURATION

#define THREAD_CONFIGURATION \
                                        .stackType = gThread_c, \
                                        .ifType = gIfType80154_c, \
                                        .deviceType = THREAD_DEVICE_TYPE, \
                                        .deviceRole = gThreadRoleNormalNode_c, \
                                        .isStarted = FALSE,\
                                        /* 802.15.4 */ \
                                        .pMacCfg = (macCfg_t *)&mThreadMacCfg, \
                                        .pAdpIb = (adpIb_t *)&mThreadAdpIb, \
                                        .contextAddrLen = 64U, \
                                        /* IP */ \
                                        .ipUlaPrefix = THREAD_ULA_PREFIX, \
                                        .ipUlaPrefixLen = 64, \
                                        .apMcastAddreses = NULL, \
                                        .pNdPib = NULL, \
                                        /* Security */ \
                                        .pSecurityMaterial = (securityMaterial_t*)&mThreadSecMaterial, \
                                        /* MPL */ \
                                        .pMplInstanceCfg = (mplInstanceCfg_t *)&mThreadMplCfg, \
                                        .pMulticastAddresses = (ipAddr_t*)&mThreadMcastAddreses, \
                                        /* MLE */ \
                                        .challengeSize = 8, \
                                        .dhcpMacSec = gMacAbsMacSecurityEncMic32_c, \
                                        .fullNwkData = TRUE

/*==================================================================================================
Public type definitions
==================================================================================================*/

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

#endif /* STACK_THREAD */

#endif  /* _THREAD_MANAGER_CONFIG_H */
