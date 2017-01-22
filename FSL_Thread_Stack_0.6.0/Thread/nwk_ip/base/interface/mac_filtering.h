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
#ifndef _MAC_FILTERING_H_
#define _MAC_FILTERING_H_

/*!=================================================================================================
\file       mac_filtering.h
\brief      This is a header file for the filtering module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* Network includes */
#include "mac_abs_types.h"

/*==================================================================================================
Public macros
==================================================================================================*/

/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef struct macFilteringConfig_tag
{
    uint64_t    extendedAddress;
    uint16_t    shortAddress;
    uint8_t     linkIndicator;
}macFilteringConfig_t;



typedef struct macFilteringNeighborData_tag
{
    uint64_t    extendedAddress;
    uint16_t    shortAddress;
    uint8_t     linkIndicator;
}macFilteringNeighborData_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

void MacFiltering_AddNeighbor(uint64_t extendedAddress, uint16_t shortAddress, uint8_t linkIndicator);

void MacFiltering_RemoveNeighbor(uint64_t extendedAddress);

bool_t MacFiltering_KeepPacket(macAbsAddrModeType_t addressMode, uint64_t address, uint8_t *pLinkIndicator);

void MacFiltering_Active(bool_t bActive);

bool_t MacFiltering_IsActive(void);

macFilteringNeighborData_t** MacFiltering_GetEntryByIdx(uint32_t index);

#ifdef __cplusplus
}
#endif

/*================================================================================================*/

#endif /* _MAC_FILTERING_H_ */
