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

#ifndef _FILENAME_PUBLIC_H
#define _FILENAME_PUBLIC_H
/*!=================================================================================================
\file       C_header_template_public.h
\brief      This is a header file for the filename module. It contains ...
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "stack_manager_if.h"

/*==================================================================================================
Public macros
==================================================================================================*/


/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef struct nwkStartParams_tag
{
    stackStartConfig_t* pStackConfig;
    stackConfig_t *pStack;
}nwkStartParams_t;

typedef struct stackParams_tag
{
    uint8_t stackID;
    uint8_t deviceID; 
    uint8_t deviceRole;
}stackParams_t;

typedef enum
{
    gNwkParamSetChannel_c,
    gNwkParamSetPanId_c,
    gNwkParamSetExtAddr_c,
    gNwkParamSetShortAddr_c,
    gNwkParamsSetRndExtAddr_c,
    gNwkParamsSetRxOnIdle_c,
    gNwkParamsSetMLPrefix_c,
    gNwkParamsSetMLPrefixLen_c
}nwkParamCode_t;
/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif
void NWK_Params_Set(uint8_t attributeID, uint8_t* pValue, stackConfig_t * pStackConfig);
void NWK_GetStack(stackParams_t* stackID, stackStartConfig_t **pStackConfig, stackConfig_t **pStack);
bool_t NWK_SetStackParams(stackParams_t* stackID);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _FILENAME_PUBLIC_H */
