/*!=================================================================================================
\file       dns_globals.c
\brief      This is the source file that contains parameters for the DNS module that can
            be configured by the application

\copyright  (c) Copyright 2013, Freescale, Inc.  All rights reserved.

==================================================================================================*/

#include "app_to_stack_config.h"
#include "dns_client.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/* Default Values */

/*! Maximum DNS requests */
#ifndef DNS_MAX_REQUESTS
    #define DNS_MAX_REQUESTS        5U        
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

/*! UDP Connections table */
dnsCacheEntry_t *aDnsCache[DNS_MAX_REQUESTS];
const uint32_t mDnsCacheSize = DNS_MAX_REQUESTS;

/*==================================================================================================
Private functions
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/