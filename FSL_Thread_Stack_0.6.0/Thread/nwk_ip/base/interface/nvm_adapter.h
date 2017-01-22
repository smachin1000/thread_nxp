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

#ifndef _NVM_ADAPTER_H
#define _NVM_ADAPTER_H

/*!=================================================================================================
\file       nvm_adapter.h
\brief      This is the header file for the  stack adapter module of non volatile flash 
            storage module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "EmbeddedTypes.h"

/*==================================================================================================
Public macros
==================================================================================================*/

/*==================================================================================================
Public type definitions
==================================================================================================*/

/* None */

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn void NVNG_Init(bool_t formatNVM)
\brief  Interface function for the NVNG module. It performes the initialization of the module.

\param [in]     formatNVM       if TRUE - formats the non volatile memory, else restores the memory

\retval      none
***************************************************************************************************/
void NVNG_Init(bool_t formatNVM);

/*!*************************************************************************************************
\fn void NVNG_Save(void ** ppRam, uint32_t size)
\brief  Saves a structure in NVM.

\param [in]  ppRam     double pointer to the entity to be saved
\param [in]  size      the size

\retval      none
***************************************************************************************************/
void NVNG_Save(void ** ppRam, uint32_t size);
/*!*************************************************************************************************
\fn void NVNG_Erase(void ** ppRam)
\brief  Erases from NVM.

\param [in]  ppRam     double pointer to the entity to be erased

\retval      none
***************************************************************************************************/
void NVNG_Erase(void ** ppRam);
/*!*************************************************************************************************
\fn void NVNG_MoveToRam(void ** ppRam, uint32_t size)
\brief  Erases from NVM.

\param [in]  ppRam     double pointer to the entity to be moved from flash to RAM
\param [in]  size      the size

\retval      none
***************************************************************************************************/
void NVNG_MoveToRam(void ** ppRam, uint32_t size);


#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _NVM_ADAPTER_H */
