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


#ifndef _FLIP_STACK_CONFIG_H
#define _FLIP_STACK_CONFIG_H
/*!
\file       flip_stack_config.h

\copyright  (c) Copyright 2014, Freescale, Inc.  All rights reserved.

\brief      This is a header file for flip stack configuration structures.

*/

/*==================================================================================================
Include Files
==================================================================================================*/

/*==================================================================================================
Public macros
==================================================================================================*/

/* Stacks */
#ifndef
    #define STACK_FLIP                  1    
#endif
    #define STACK_STATIC                0


/* Device Type */
#if STACK_FLIP
    #define FLIP_ENET_ROUTER            0
    #define FLIP_VTUN_ROUTER            0

    #define FLIP_802154_BROUTER         0
    #define FLIP_802154_ROUTER          1
    #define FLIP_802154_HOST            0
#endif

#if STACK_STATIC
    #define STATIC_ENET_ROUTER          0
    #define STATIC_ENET_HOST            0

    #define STATIC_802154_BROUTER       1
    #define STATIC_802154_HOST          0
#endif



/*================================================================================================*/

#define STACK_MAX_DEVICES               100

#define DHCP6_SERVER_MAX_CLIENTS        STACK_MAX_DEVICES
#define IP_IP6_ROUTING_TBL_SIZE         STACK_MAX_DEVICES
#define ND_NEIGHBOR_CACHE_SIZE          STACK_MAX_DEVICES
#define ND_DESTINATION_CACHE_SIZE       STACK_MAX_DEVICES
#define BSDS_MAX_SOCKETS                STACK_MAX_DEVICES
#define BSDS_SELECT_MAX_FDS             STACK_MAX_DEVICES
#define SESS_SOCK_ENT_LIST_SIZE         STACK_MAX_DEVICES
#define MAX_TCP_CONNECTIONS             STACK_MAX_DEVICES
#define MAX_UDP_CONNECTIONS             STACK_MAX_DEVICES

/* Network Configuration */



#if STACK_FLIP
    /* Stack Variables */
    #define IP_IF_NB                    2

    /* Library Settings (Read Only) */

    /* Modules */
    #define IP_IP6_ENABLE               1
    #define IP_IP6_ROUTING_ENABLE       1
    #define IP_IP4_ENABLE               1
    #define ND_ENABLED                  1
    #define TCP_ENABLED                 1
    #define MPL_ENABLED                 1
    #define RIPNG_ENABLE                1
    #define DHCP6_CLIENT_ENABLED        1
    #define DHCP6_SERVER_ENABLED        1
    #define DHCP4_SERVER_ENABLED        1
    #define DHCP4_CLIENT_ENABLED        1
    #define TRICKLE_ENABLED             1
    #define NVM_NG_ENABLED              1

    /* Module Settings */
    #define DHCP6_RAPID_COMMIT          0
#endif

#if STACK_STATIC
    /* Stack Variables */
    #define IP_IF_NB                    2

    /* Library Settings (Read Only) */

    /* Modules */
    #define IP_IP6_ENABLE               1
    #define IP_IP6_ROUTING_ENABLE       1
    #define IP_IP4_ENABLE               1
    #define ND_ENABLED                  1
    #define TCP_ENABLED                 1
    #define MPL_ENABLED                 1
    #define RIPNG_ENABLE                1
    #define DHCP6_CLIENT_ENABLED        1
    #define DHCP6_SERVER_ENABLED        1
    #define DHCP4_SERVER_ENABLED        1
    #define DHCP4_CLIENT_ENABLED        1
    #define TRICKLE_ENABLED             1
    #define NVM_NG_ENABLED              1

    /* Module Settings */


#endif

#define BLACK_BOX_ENABLED               1

#if BLACK_BOX_ENABLED
    #define CONST
#else
   #define CONST const
#endif

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
#endif  /* _FLIP_STACK_CONFIG_H */
