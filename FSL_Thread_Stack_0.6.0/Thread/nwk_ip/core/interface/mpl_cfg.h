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

#ifndef _MPL_CFG_H
#define _MPL_CFG_H

/*!=================================================================================================
\file       mpl_cfg.h
\brief      This is a configuration header file for the mpl module.

\details    This file contains the folowing configuration options:

            MPL_ENABLED                             0 | 1       (default is 1)
            MPL_INSTANCE_SET_SIZE                   0 | 255     (default is 3)
            MPL_SEED_SET_SIZE                       0 | 255     (default is 5)
            MPL_BUFFERED_MESSAGE_SET_SIZE           0 | 255     (default is 5)
            MPL_M_FLAG_VERIFICATION_ENABLED         0 | 1       (default is 0)
            MPL_MULTIPLE_SEED_ID_LENGTH_ENABLED     0 | 1       (default is 0)
            MPL_MAX_SEED_ID_LENGTH                  2, 8, 16    (default is 2)
            MPL_DEBUG                               0 | 1       (default is 0)
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "stack_config.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/*! Description of the MPL_ENABLED configuration option: set to 1 to enable the MPL
    functionality */
#ifndef MPL_ENABLED
  #define MPL_ENABLED                               0
#endif


/*! Description of the MPL_M_FLAG_VERIFICATION_ENABLED configuration option: set to 1 to enable verification
    of M flag from MPL header */
#ifndef MPL_M_FLAG_VERIFICATION_ENABLED
    #define MPL_M_FLAG_VERIFICATION_ENABLED         0
#endif


/*! Description of the MPL_MULTIPLE_SEED_ID_LENGTH_ENABLED configuration option: set to 1 to permit different
    seed identifier length */
#ifndef MPL_MULTIPLE_SEED_ID_LENGTH_ENABLED
    #define MPL_MULTIPLE_SEED_ID_LENGTH_ENABLED     0
#endif


/*! Description of the MPL_MAX_SEED_ID_LENGTH configuration option: maximum supported length of seed identifier.
    It can be 2, 8 or 16 */
#ifndef MPL_MAX_SEED_ID_LENGTH
    #define MPL_MAX_SEED_ID_LENGTH                  2
#endif


/*! Description of the MPL_DEBUG configuration option: set to 1 to enable the MPL
    debug functionality */
#ifndef MPL_DEBUG
  #define MPL_DEBUG                                 0
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

#endif  /*_MPL_CFG_H */
