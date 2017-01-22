/******************************************************************************
 * Filename: NV_Data.h
 *
 * Description: Declarations for the application client of the NV  
 *              storage module (NVM)
 * 
 *
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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

#ifndef _NV_DATA_H_
#define _NV_DATA_H_

 /*!=================================================================================================
\file       nvm_adapter.h
\brief      This is the header file for the  stack non volatile data module.
==================================================================================================*/
   
#ifdef __cplusplus
    extern "C" {
#endif

/*==================================================================================================
Include Files
==================================================================================================*/      
#include "EmbeddedTypes.h"
#include "NVM_Interface.h"   


/*==================================================================================================
Public macros
==================================================================================================*/

/* Unique Nvm Data  Id's */
/* WARNING WARNING  WARNING  WARNING  WARNING  WARNING  WARNING  WARNING*/
/* For instantiable stacks  the most significant nibble is used for pan Id */ 

/*PAN0*/
#define nvmId_SlwpStruct_c                      0x0000
#define nvmId_ContextTable_c                    0x0001
#define nvmId_Dhcp6ClientParams_c               0x0002
#define nvmId_mDhcp6ServerBindingTbl_c          0x0003
#define nvmId_ClientParamsTbl_c                 0x0004
#define nvmId_InterfaceTable_c                  0x0005
#define nvmId_GlobalAddrTable6_c                0x0006
#define nvmId_mMplInstanceSet_c                 0x0007
#define nvmId_NdCfg_c                           0x0008
#define nvmId_mTrickleInstanceSet_c             0x0009
#define nvmId_ip6RoutingTblEntry_c              0x000A
#define nvmId_macFilteringTable_c               0x000B
#define nvmId_ndCfgData_c                       0x000C      
#define nvmId_ndPrefixList_c                    0x000D


/*==================================================================================================
Public type definitions
==================================================================================================*/

/* None */
      
/*==================================================================================================
Public global variables declarations
==================================================================================================*/

extern const NVM_DataEntry_t NVM_DataTable[];


#ifdef __cplusplus
}
#endif

#endif //_NV_DATA_H_
