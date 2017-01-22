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
\file       C_source_template_public.c
\brief      This is a public source file for the new generation non volatile flash storage module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_to_stack_config.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "stack_config.h"
#include "NVM_Interface.h"
#include "fsl_os_abstraction.h"
#include "nvm_adapter.h"


#if (gNvStorageIncluded_d == TRUE)
  #include "panic.h"
#endif
/*==================================================================================================
Private macros
==================================================================================================*/
#if NVM_NG_ENABLED
    #if (gNvStorageIncluded_d == FALSE)
        #error "*** ERROR: NVM module is not enabled"
    #endif
    #if (gUnmirroredFeatureSet_d != TRUE)
        #error "*** ERROR: gUnmirroredFeatureSet_d from NVM module is not enabled"
    #endif
#endif
/*==================================================================================================
Private type definitions
==================================================================================================*/

/* None */

/*==================================================================================================
Private prototypes
==================================================================================================*/

/* None */

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

/* None */


/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/* None */

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn void NVNG_Init( bool_t formatNVM)
\brief  Interface function for the NVNG module. It performes the initialization of the module.

\param [in]     formatNVM       if TRUE - formats the non volatile memory, else restores the memory 
     
\retval      none
***************************************************************************************************/
void NVNG_Init
(
    bool_t  formatNVM
)
{
#if NVM_NG_ENABLED
  /* Non volatile memory module init */
  if(gNVM_OK_c != NvModuleInit())
  {
     panic(0,(uint32_t)NVNG_Init,0,0);
  }
  if(TRUE == formatNVM)
  {
    if(gNVM_OK_c != NvFormat())
    {
      panic(0,(uint32_t)NVNG_Init,0,0);
    }
  }
#endif /* gNvStorageIncluded_d */   
}
/*!*************************************************************************************************
\fn void NVNG_MoveToRam(void ** ppRam, uint32_t size)
\brief  Erases from NVM.

\param [in]  ppRam     double pointer to the entity to be moved from flash to RAM
\param [in]  size      the size

\retval      none
***************************************************************************************************/
void NVNG_MoveToRam
(
    void ** ppRam,
    uint32_t size
)
{
#if NVM_NG_ENABLED
  (void)size;
  if(gNVM_OK_c != NvMoveToRam(ppRam))
  {
    panic(0,(uint32_t)NVNG_MoveToRam,0,0);
  } 
#endif    
}

/*!*************************************************************************************************
\fn void NVNG_Save(void ** ppRam, uint32_t size)
\brief  Saves a structure in NVM.

\param [in]  ppRam     double pointer to the entity to be saved
\param [in]  size      the size
     
\retval      none
***************************************************************************************************/
void NVNG_Save
(
    void ** ppRam, 
    uint32_t size
)
{
#if NVM_NG_ENABLED
  
    (void)size;
    if(gNVM_OK_c != NvSyncSave(ppRam,FALSE,TRUE))
    {
      panic(0,(uint32_t)NVNG_Save,0,0);
    }
#endif    
}

/*!*************************************************************************************************
\fn void NVNG_Erase(void ** ppRam)
\brief  Erases from NVM.

\param [in]  ppRam     double pointer to the entity to be erased
     
\retval      none
***************************************************************************************************/
void NVNG_Erase
(
    void ** ppRam
)
{
#if !NVM_NG_ENABLED  
    /* make a local copy of the pointer */
    void * tempPram = *ppRam;

    OSA_EnterCritical(kCriticalDisableInt);
    *ppRam = NULL;
    OSA_ExitCritical(kCriticalDisableInt); 
    {
        MEM_BufferFree(tempPram);
    }
#else
    if(gNVM_OK_c != NvErase(ppRam))
    {
      panic(0,(uint32_t)NVNG_Erase,0,0);
    }  
#endif    
}

/* Delimiters */
/*================================================================================================*/


/*==================================================================================================
Private debug functions
==================================================================================================*/
