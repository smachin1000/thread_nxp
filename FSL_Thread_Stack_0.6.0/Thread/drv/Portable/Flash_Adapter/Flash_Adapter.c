/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file Flash.c
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


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "Flash_Adapter.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"

/*! *********************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
FLASH_SSD_CONFIG gFlashConfig =
{
    FTFx_REG_BASE,          /* FTFx control register base */
    P_FLASH_BASE,           /* base address of PFlash block */
    P_FLASH_SIZE,           /* size of PFlash block */
    FLEXNVM_BASE,           /* base address of DFlash block */
    0,                      /* size of DFlash block */
    EERAM_BASE,             /* base address of EERAM block */
    0,                      /* size of EEE block */
    DEBUGENABLE,            /* background debug mode enable bit */
    NULL_CALLBACK           /* pointer to callback function */
};

pFLASHCOMMANDSEQUENCE gFlashLaunchCommand = NULL;
uint16_t __ram_func[LAUNCH_CMD_SIZE/2];

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */
void NV_Init(void)
{
    if( !gFlashLaunchCommand )
    {
        FlashInit(&gFlashConfig);
        gFlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)RelocateFunction((uint32_t)__ram_func , 
                                                                      LAUNCH_CMD_SIZE,
                                                                      (uint32_t)FlashCommandSequence);
    }
}

uint32_t NV_FlashProgramUnaligned(PFLASH_SSD_CONFIG pSSDConfig,
                                  uint32_t dest,
                                  uint32_t size,
                                  uint8_t* pData,
                                  pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    uint8_t  buffer[PGM_SIZE_BYTE];
    uint16_t bytes = dest & (PGM_SIZE_BYTE-1);
    uint32_t status;

    if( bytes )
    {
        uint16_t unalignedBytes = PGM_SIZE_BYTE - bytes;
        
        if( unalignedBytes > size )
        {
            unalignedBytes = size;
        }

        FLib_MemCpy(buffer, (void*)(dest - bytes), PGM_SIZE_BYTE);
        FLib_MemCpy(&buffer[bytes], pData, unalignedBytes);
#if gNvDisableIntCmdSeq_c
        OSA_EnterCritical(kCriticalDisableInt);
#endif
        status = FlashProgram(pSSDConfig, dest - bytes, PGM_SIZE_BYTE, buffer, pFlashCommandSequence);
#if gNvDisableIntCmdSeq_c
        OSA_ExitCritical(kCriticalDisableInt);
#endif
        if( status != FTFx_OK )
            return status;

        dest += PGM_SIZE_BYTE - bytes;
        pData += unalignedBytes;
        size -= unalignedBytes;
    }

    bytes = size & ~(PGM_SIZE_BYTE-1);

    if( bytes )
    {
#if gNvDisableIntCmdSeq_c
        OSA_EnterCritical(kCriticalDisableInt);
#endif
        status = FlashProgram(pSSDConfig, dest, bytes, pData, pFlashCommandSequence);
#if gNvDisableIntCmdSeq_c
        OSA_ExitCritical(kCriticalDisableInt);
#endif
        if( status != FTFx_OK )
            return status;

        dest  += bytes;
        pData += bytes;
        size  -= bytes;
    }
    
    if( size )
    {
        FLib_MemCpy(buffer, (void*)dest, PGM_SIZE_BYTE);
        FLib_MemCpy(buffer, pData, size);
#if gNvDisableIntCmdSeq_c
        OSA_EnterCritical(kCriticalDisableInt);
#endif
        status = FlashProgram(pSSDConfig, dest, PGM_SIZE_BYTE, buffer, pFlashCommandSequence);
#if gNvDisableIntCmdSeq_c
        OSA_ExitCritical(kCriticalDisableInt);
#endif
        if( status != FTFx_OK )
            return status;
    }
    
    return FTFx_OK;
}

uint32_t NV_FlashEraseSector(PFLASH_SSD_CONFIG pSSDConfig,
                                  uint32_t dest,
                                  uint32_t size,
                                  pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
    uint32_t status;
#if gNvDisableIntCmdSeq_c
    OSA_EnterCritical(kCriticalDisableInt);
#endif
    status = FlashEraseSector(pSSDConfig, dest, size, pFlashCommandSequence);
#if gNvDisableIntCmdSeq_c
    OSA_ExitCritical(kCriticalDisableInt);
#endif
    return status;
}