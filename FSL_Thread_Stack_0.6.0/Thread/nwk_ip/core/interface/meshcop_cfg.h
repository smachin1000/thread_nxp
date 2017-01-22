#ifndef _MESHCOP_CFG_H
#define _MESHCOP_CFG_H


/*!=================================================================================================
\file       meshcop_cfg.h
\brief      This is a header file for the MESHCOP module.

\copyright  Freescale Confidential Proprietary. No part of this document must be reproduced in any
            form - including copied, transcribed, printed or by any electronic means - without
            specific written permission from Freescale.
            (c) Copyright 2014, Freescale, Inc.  All rights reserved.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/*==================================================================================================
Public macros
==================================================================================================*/
#ifndef MESHCOP_ENABLED
#   define MESHCOP_ENABLED                  (0)
#endif

/* The maximum amount of time between when a device successfully petitions to become a Commissioner
 * and a new Joiner device may attempt to join the network.  This sets a maximum term limit for any
 * one Commissioner */
#ifndef MESHCOP_TIMEOUT_PERMIT_JOIN
#   define MESHCOP_TIMEOUT_PERMIT_JOIN      (10000U)
#endif

/* The maximum amount of unsecured packets a Joiner Router will forward per second. Default = 10 */
#ifndef MESHCOP_JOINER_RELAY_RATE_LIMIT
#   define MESHCOP_JOINER_RELAY_RATE_LIMIT  (10U)
#endif

#ifndef MESHCOP_TIMEOUT
#   define MESHCOP_TIMEOUT                  (100U)
#endif

/* The maximum characters Commissioner ID can have */
#ifndef MESHCOP_COMM_SESS_ID_MAX
#   define MESHCOP_COMM_SESS_ID_MAX         (10)
#endif

#ifndef JOINER_URL
#   define JOINER_URL                       "www.threadgroup.org"
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

#endif /* _MESHCOP_CFG_H */
