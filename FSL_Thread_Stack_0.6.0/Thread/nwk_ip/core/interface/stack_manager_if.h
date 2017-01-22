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

#ifndef _STACK_MANAGER_IF_H
#define _STACK_MANAGER_IF_H
/*!=================================================================================================
\file       stack_manager_if.h
\brief      This is a header file for stack configuration structures.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "network_utils.h"
#include "sixlowpan_ib.h"
#include "nd.h"
#include "mpl.h"
#include "dhcp6.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#if defined(__IAR_SYSTEMS_ICC__)
#pragma section="STACK_START_CFG"
#endif

#if defined(__GNUC__)
  #define gSTACK_START_CFG_startAddr_d ((stackStartConfig_t*)__start_STACK_START_CFG)
  #define gSTACK_START_CFG_endAddr_d   ((stackStartConfig_t*)__stop_STACK_START_CFG)
#elif defined(__IAR_SYSTEMS_ICC__)
  #define gSTACK_START_CFG_startAddr_d ((stackStartConfig_t*)__section_begin("STACK_START_CFG"))
  #define gSTACK_START_CFG_endAddr_d   ((stackStartConfig_t*)__section_end("STACK_START_CFG"))
#else
  #define gSTACK_START_CFG_startAddr_d ((stackStartConfig_t*)0)
  #define gSTACK_START_CFG_endAddr_d   ((stackStartConfig_t*)0)
  #warning Flip Start Config will not be stored!
#endif

#define gSTACK_START_CFG_entries_d  ( ((uint32_t)gSTACK_START_CFG_endAddr_d - \
                                        (uint32_t)gSTACK_START_CFG_startAddr_d)/ \
                                        sizeof(stackStartConfig_t) )

#define SET_HANDLER_STRUCT_NAME_CONCAT(structName, moduleName, line) g##structName##_##moduleName##_##line
#define SET_HANDLER_STRUCT_NAME(structName, moduleName, line) SET_HANDLER_STRUCT_NAME_CONCAT(structName, moduleName, line)

#if defined(__IAR_SYSTEMS_ICC__)
#define StackStartCfg_RegisterStatic(module, ...) \
    _Pragma("location=\"STACK_START_CFG\"") __root \
    const stackStartConfig_t SET_HANDLER_STRUCT_NAME(StackStartCfg, module, __LINE__) = {__VA_ARGS__};

#elif defined(__GNUC__)

#define StackStartCfg_RegisterStatic(module, StackStart, pStackParam) \
    const stackStartConfig_t SET_HANDLER_STRUCT_NAME(StackStartCfg, module, __LINE__) __attribute__ ((section ("STACK_START_CFG"), used)) = \
    {StackStart, pStackParam}

#else
    #warning shell command will not be stored!
#endif




/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef enum stackType_tag
{
    gThread_c,
    gFlip_c,
    gStatic_c
}stackType_t;

typedef enum interfaceType_tag
{
    gIfTypeEth_c,
    gIfType80154_c,
    gIfTypeWifi_c,
    gIfTypeVtun_c
} interfaceType_t;

typedef struct macCfg_tag
{
    uint64_t  extendedAddr;
    uint16_t  shortAddr;
    uint16_t  panId;
    uint8_t  *pMacFilteringTbl;
    uint32_t  pollInterval;                 /* Miliseconds */
    bool_t    randomExt;
    bool_t    rxOnIdle;        
    uint8_t   channel;
} macCfg_t;

typedef struct securityMaterial_tag
{
    uint8_t   nodeKey[16];
    uint32_t  keySequenceCounter;
} securityMaterial_t;

typedef struct stackConfig_tag
{   
    stackType_t         stackType;
    interfaceType_t     ifType;
    uint8_t             deviceType;
    uint8_t             deviceRole;

    /* Stack Management */
    bool_t              isStarted;

    /* 802.15.4 */
    macCfg_t            *pMacCfg;
    adpIb_t             *pAdpIb;

    /* Ethernet / Virtual Enet */
    llAddr_t            macAddress;

    /* Wifi */
    void                *wifiCfg;

    /* IP */
    ipAddr_t            ipUlaPrefix;
    ipAddr_t            ipGlobalPrefix;

    uint32_t            ipUlaPrefixLen;
    uint32_t            ipGlobalPrefixLen;

    ipAddr_t            *apMcastAddreses;      /*!< Variable length array containing device multicast addresses. Must be NULL terminated. */

    /* PAN config */
    uint16_t            shortAddress;
    ipAddr_t            contextAddr;
    uint8_t             contextAddrLen;


    /* ND */
    ndPib_t             *pNdPib;

    /* Security */
    securityMaterial_t  *pSecurityMaterial;

    /* MPL */
    void                *pMplInstanceCfg;
    ipAddr_t            *pMulticastAddresses;

    /* DHCPv6 */
    dhcp6ServerIpCfg_t  **pDhcp6ServerCfg;
    ipAddr_t            **pDhcp6DNSServers;
    bool_t              dhcp6ClientEnable;
    bool_t              dhcp6ServerEnable;
    uint8_t             dhcpMacSec;

    /* DHCPv4 */
    bool_t              dhcp4ClientEnable;
    bool_t              dhcp4ServerEnable;
    uint32_t            dhcp4ServerIpAddr;
    uint32_t            dhcp4ServerStartIpAddr;
    uint32_t            dhcp4ServerEndIpAddr;

    /* MLE */
    uint32_t            timeout;        /*!< timeout a parent can wait for this child to respond */
    uint8_t             challengeSize;  /*!< number of bytes sent in a challenge(minimum 4) */
    bool_t              fullNwkData;    /*!< specifies if the device requires the full network data */
}stackConfig_t;


typedef struct stackStartConfig_tag
{
    void                (*StackStart)(stackConfig_t* pStackParam, taskMsgQueue_t *pTaskMsgQueue);
    stackConfig_t       **pStackParam;
}stackStartConfig_t;


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
#endif  /* _STACK_MANAGER_IF_H */
