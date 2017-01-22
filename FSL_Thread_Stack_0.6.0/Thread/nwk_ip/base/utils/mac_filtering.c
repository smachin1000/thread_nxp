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
\file       mac_filtering.c
\brief      This is a private source file for the filtering module implementation
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* Framework includes */
#include "Panic.h"
#include "MemManager.h"

/* Network includes */
#include "mac_filtering_cfg.h"
#include "mac_filtering.h"

#include "nvm_adapter.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/*==================================================================================================
Private type definitions
==================================================================================================*/

#if MAC_FILTERING_ENABLED

    static macFilteringNeighborData_t** MacFiltering_GetEntry(macAbsAddrModeType_t addressMode, uint64_t address, bool_t bAcceptFreeEntry);
    static bool_t MacFiltering_IsTableEmpty(void);

#endif /* MAC_FILTERING_ENABLED */

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

#if MAC_FILTERING_ENABLED

    macFilteringNeighborData_t*  macFilteringTable[MAC_FILTERING_TABLE_SIZE]={NULL};
    bool_t mbMacFilteringActive = FALSE;

#endif /* MAC_FILTERING_ENABLED */

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Private functions
==================================================================================================*/

#if MAC_FILTERING_ENABLED

static macFilteringNeighborData_t** MacFiltering_GetEntry
(
    macAbsAddrModeType_t addressMode, 
    uint64_t address, 
    bool_t bAcceptFreeEntry
)
{
    macFilteringNeighborData_t**    ppNeighborData = NULL;
    uint8_t                         iCount;

    if((gMacAbsAddrModeExtendedAddress_c == addressMode) ||
       (gMacAbsAddrModeShortAddress_c == addressMode))
    {
        for(iCount = 0; iCount < MAC_FILTERING_TABLE_SIZE; iCount++)
        {
            if(NULL != macFilteringTable[iCount])
            {
                uint64_t neighborAddress;

                if(addressMode == gMacAbsAddrModeExtendedAddress_c)
                {
                    neighborAddress = macFilteringTable[iCount]->extendedAddress;
                }
                else if(addressMode == gMacAbsAddrModeShortAddress_c)
                {
                    address &= 0xFFFF;
                    neighborAddress = macFilteringTable[iCount]->shortAddress;
                }

                if(address == neighborAddress)
                {
                    ppNeighborData = &macFilteringTable[iCount];
                    break;
                }
            }
            else if((TRUE == bAcceptFreeEntry) &&
                    (NULL == ppNeighborData))
            {
                ppNeighborData = &macFilteringTable[iCount];
            }
        }
    }

    return ppNeighborData;
}

static bool_t MacFiltering_IsTableEmpty(void)
{
    bool_t  bTableEmpty = TRUE;
    uint8_t iCount;

    for(iCount = 0; iCount < MAC_FILTERING_TABLE_SIZE; iCount++)
    {
        if(NULL != macFilteringTable[iCount])
        {
            bTableEmpty = FALSE;
            break;
        }
    }

    return bTableEmpty;
}

#endif /* MAC_FILTERING_ENABLED */

/*==================================================================================================
Public functions
==================================================================================================*/

void MacFiltering_AddNeighbor(uint64_t extendedAddress, uint16_t shortAddress, uint8_t linkIndicator)
{
#if MAC_FILTERING_ENABLED
    if (mbMacFilteringActive)
    {
        macFilteringNeighborData_t** ppNeighborData = MacFiltering_GetEntry(gMacAbsAddrModeExtendedAddress_c, extendedAddress, TRUE);

        if(NULL == ppNeighborData)
        {
            panic(0, (uint32_t)MacFiltering_AddNeighbor, 0, 0);
        }

        if(NULL == *ppNeighborData)
        {
            *ppNeighborData = (macFilteringNeighborData_t*)MEM_BufferAlloc(sizeof(macFilteringNeighborData_t));
        }
        else
        {
            NVNG_MoveToRam((void**)ppNeighborData, sizeof(macFilteringNeighborData_t));
        }

        (*ppNeighborData)->extendedAddress  = extendedAddress;
        (*ppNeighborData)->shortAddress     = shortAddress;
        if (linkIndicator)
        {
            (*ppNeighborData)->linkIndicator    = linkIndicator;
        }

        /* Save to NVM */
        NVNG_Save((void**)ppNeighborData, sizeof(macFilteringNeighborData_t));
    }
#else
    (void)extendedAddress;
    (void)shortAddress;
    (void)linkIndicator;
#endif /* MAC_FILTERING_ENABLED */
}

void MacFiltering_RemoveNeighbor(uint64_t extendedAddress)
{
#if MAC_FILTERING_ENABLED
    macFilteringNeighborData_t** ppNeighborData = MacFiltering_GetEntry(gMacAbsAddrModeExtendedAddress_c, extendedAddress, FALSE);

    if(NULL != ppNeighborData)
    {
        NVNG_MoveToRam((void**)ppNeighborData, sizeof(macFilteringNeighborData_t));
        MEM_BufferFree(*ppNeighborData);
        *ppNeighborData = NULL;
    }
#else
    (void)extendedAddress;
#endif /* MAC_FILTERING_ENABLED */
}

bool_t MacFiltering_KeepPacket(macAbsAddrModeType_t addressMode, uint64_t address, uint8_t *pLinkIndicator)
{
    bool_t bKeepPacket = TRUE;

#if MAC_FILTERING_ENABLED
    if (mbMacFilteringActive)
    {
        if(FALSE == MacFiltering_IsTableEmpty())
        {
            macFilteringNeighborData_t** ppNeighborData = MacFiltering_GetEntry(addressMode, address, FALSE);

            if (NULL != ppNeighborData)
            {
                *pLinkIndicator = (*ppNeighborData)->linkIndicator;
            }
            else
            {
                bKeepPacket = FALSE;
            }
        }
        else
        {
            bKeepPacket = FALSE;
        }
    }
#else
    (void)addressMode;
    (void)address;
    (void)pLinkIndicator;
#endif /* MAC_FILTERING_ENABLED */

    return bKeepPacket;
}

void MacFiltering_Active
(
    bool_t bActive
)
{
#if MAC_FILTERING_ENABLED  
    mbMacFilteringActive = bActive;
#endif    
}

bool_t MacFiltering_IsActive
(
    void
)
{
    bool_t bRetStatus = FALSE;
#if MAC_FILTERING_ENABLED  
    bRetStatus = mbMacFilteringActive;
#endif
    return bRetStatus;
}

macFilteringNeighborData_t** MacFiltering_GetEntryByIdx
(
    uint32_t index
)
{
    macFilteringNeighborData_t **ppMacFilterEntry = NULL;

#if MAC_FILTERING_ENABLED    
    if ((index < MAC_FILTERING_TABLE_SIZE) && (NULL != macFilteringTable[index]))
    {
        ppMacFilterEntry = &macFilteringTable[index];
    }
#endif    
    
    return ppMacFilterEntry;
}
