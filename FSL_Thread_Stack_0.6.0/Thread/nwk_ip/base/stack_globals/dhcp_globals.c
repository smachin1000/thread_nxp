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
\file       dhcp_globals.c
\brief      This is the source file that contains parameters for the DHCP modules that can be
            configured by the application
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "app_to_stack_config.h"

#include "dhcp_client.h"
#include "dhcp_server.h"

#include "dhcp6_client.h"
#include "dhcp6_server.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/* Default values */

#ifndef DHCP_SERVER_MAX_CLIENTS
    #define DHCP_SERVER_MAX_CLIENTS         5
#endif

#ifndef DHCP6_SERVER_MAX_INSTANCES
    #define DHCP6_SERVER_MAX_INSTANCES      2
#endif

#ifndef DHCP6_SERVER_MAX_CLIENTS
    #define DHCP6_SERVER_MAX_CLIENTS        5
#endif

#ifndef DHCP6_CLIENT_MAX_INSTANCES
    #define DHCP6_CLIENT_MAX_INSTANCES      2
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

/* DHCP4 Server*/
#if DHCP4_SERVER_ENABLED && IP_IP4_ENABLE
dhcpServerCfgStruct_t   *aServerCfgStruct[IP_IF_NB] = {NULL};
dhcpServerData_t        *aServerBindingTbl[DHCP_SERVER_MAX_CLIENTS+1] = {NULL};
#endif

/* DHCP6 Server*/
dhcp6ServerCfg_t*        aDhcp6ServerCfgStruct[DHCP6_SERVER_MAX_INSTANCES];
dhcp6ServerBindingTbl_t *mDhcp6ServerBindingTbl[DHCP6_SERVER_MAX_CLIENTS] = {NULL};


/* DHCP4 Client*/
#if DHCP4_CLIENT_ENABLED && IP_IP4_ENABLE
dhcpClientData_t        *aClientParamsTbl[IP_IF_NB] = {NULL};
#endif

/* DHCP6 Client*/
dhcp6ClientData_t       *aDhcp6ClientParams[DHCP6_CLIENT_MAX_INSTANCES] = {NULL};

const uint32_t dhcp6ServerMaxClients = DHCP6_SERVER_MAX_CLIENTS;
const uint32_t dhcp4ServerMaxClients = DHCP_SERVER_MAX_CLIENTS;

const uint32_t dhcp6ServerMaxInstances = DHCP6_SERVER_MAX_INSTANCES;
const uint32_t dhcp6ClientMaxInstances = DHCP6_CLIENT_MAX_INSTANCES;

/*==================================================================================================
Private functions
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/

