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
\file       nv_data.c
\brief      This is the source file for the  stack non volatile data module.
==================================================================================================*/
   
/*==================================================================================================
Include Files
==================================================================================================*/   
   
#include "EmbeddedTypes.h"

#include "app_to_stack_config.h"

#include "NVM_Interface.h"
#include "NV_Data.h"

#include "stack_config.h"
#include "ip.h"
#include "network_utils.h"
#include "sixlowpan_cfg.h"
#include "sixlowpan_interface.h"
#include "sixlowpan_tbl.h"
#include "sixlowpan.h"
#include "mac_abs_types.h"
#include "ip_cfg.h"
#include "dhcp_cfg.h"
#include "dhcp6_cfg.h"

#include "dhcp_client.h"
#include "dhcp_server.h"

#include "dhcp6_client.h"
#include "dhcp6_server.h"

#include "mpl_cfg.h"
#include "mpl.h"

#include "nd.h"

#include "trickle.h"
#include "mac_filtering.h"

#include "MemManager.h"
#include "FunctionLib.h"
#include "nvm_adapter.h"


/* gSIXLOWPAN_DATA_SET_FOR_NVM */
extern slwpStruct_t                *apSlwpStruct[];
extern ndContextEntry_t            *aContextTable[];

/* gDhcp6Server_DATA_SET_FOR_NVM */
extern dhcp6ServerBindingTbl_t     *mDhcp6ServerBindingTbl[];

/* gDhcp6Client_DATA_SET_FOR_NVM */ 
extern dhcp6ClientData_t           *aDhcp6ClientParams[];

/* IP_IP6_DATA_SET_FOR_NVM */
extern ipIfStruct_t                *aInterfaceTable[];
extern ip6IfAddrData_t             *aGlobalAddrTable6[];

/* MPL_DATA_SET_FOR_NVM */
extern mplInstanceSetEntry_t       *mMplInstanceSet[];

/* TRICKLE_DATA_SET_FOR_NVM */
extern trickleInstanceConfig_t     *mTrickleInstanceSet[];

/* IP_IP6_ROUTING_DATA_SET_FOR_NVM */
extern ip6RoutingTblEntry_t        *aIp6RoutingTable[];

/* MAC_FILTERING_DATA_SET_FOR_NVM */
extern macFilteringNeighborData_t*  macFilteringTable[];

/* ND_DATA_SET_FOR_NVM */
extern ndCfg_t* aNdCfg[];
extern ndPrefixEntry_t* aPrefixList[];

/*==================================================================================================
Private macros
==================================================================================================*/
#define gSIXLOWPAN_DATA_SET_FOR_NVM\
      {apSlwpStruct,                SLWPCFG_INSTANCES_NB,               sizeof(slwpStruct_t),                nvmId_SlwpStruct_c,             gNVM_NotMirroredInRam_c},\
      {aContextTable,               SLWPCFG_RFC6282_CONTEXT_TABLE_SIZE, sizeof(ndContextEntry_t),            nvmId_ContextTable_c,           gNVM_NotMirroredInRam_c}

#define gDhcp6Server_DATA_SET_FOR_NVM\
      {mDhcp6ServerBindingTbl,      DHCP6_SERVER_MAX_CLIENTS,           sizeof(dhcp6ServerBindingTbl_t),     nvmId_mDhcp6ServerBindingTbl_c, gNVM_NotMirroredInRam_c} 
      
#define gDhcp6Client_DATA_SET_FOR_NVM\
      {aDhcp6ClientParams,          DHCP6_CLIENT_MAX_INSTANCES,        sizeof(dhcp6ClientData_t),           nvmId_Dhcp6ClientParams_c,      gNVM_NotMirroredInRam_c}      

#define IP_IP6_DATA_SET_FOR_NVM\
      {aInterfaceTable,             IP_IF_NB+1,                         sizeof(ipIfStruct_t),                nvmId_InterfaceTable_c,         gNVM_NotMirroredInRam_c},\
      {aGlobalAddrTable6,           IP_IF_IP6_ADDR_NB+1,                sizeof(ip6IfAddrData_t),             nvmId_GlobalAddrTable6_c,       gNVM_NotMirroredInRam_c}

#define IP_IP6_ROUTING_DATA_SET_FOR_NVM\
      {aIp6RoutingTable,            IP_IP6_ROUTING_TBL_SIZE+1,          sizeof(ip6RoutingTblEntry_t ),       nvmId_ip6RoutingTblEntry_c,     gNVM_NotMirroredInRam_c}
         
#define MPL_DATA_SET_FOR_NVM\
      {mMplInstanceSet,             MPL_INSTANCE_SET_SIZE,              sizeof(mplInstanceSetEntry_t),       nvmId_mMplInstanceSet_c,        gNVM_NotMirroredInRam_c}

#define TRICKLE_DATA_SET_FOR_NVM\
      {mTrickleInstanceSet,         TRICKLE_INSTANCE_SET_SIZE,          sizeof(trickleInstanceConfig_t),     nvmId_mTrickleInstanceSet_c,    gNVM_NotMirroredInRam_c}
        
#define MAC_FILTERING_DATA_SET_FOR_NVM\
      {macFilteringTable,           MAC_FILTERING_TABLE_SIZE,           sizeof(macFilteringNeighborData_t),  nvmId_macFilteringTable_c,      gNVM_NotMirroredInRam_c}

#define ND_DATA_SET_FOR_NVM\
      {aNdCfg,                      IP_IF_NB,                           sizeof(ndCfg_t),                     nvmId_ndCfgData_c,              gNVM_NotMirroredInRam_c},\
      {aPrefixList,                 ND_PREFIX_LIST_SIZE,                sizeof(ndPrefixEntry_t),             nvmId_ndPrefixList_c,           gNVM_NotMirroredInRam_c }                  
      
#define gMaxNVDataTableEntries_c gNvTableEntriesCountMax_c
/*==================================================================================================
Private functions
==================================================================================================*/

/* None */

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

#if gNvStorageIncluded_d
/*
 * Name: NvDataTable
 * Description: NVM data table. Contains entries of datasets.
 *              Defined by appication.
 */

const NVM_DataEntry_t NVM_DataTable[] =
{
  gSIXLOWPAN_DATA_SET_FOR_NVM,
  gDhcp6Server_DATA_SET_FOR_NVM,
  gDhcp6Client_DATA_SET_FOR_NVM,
  IP_IP6_DATA_SET_FOR_NVM,
#if IP_IP6_ROUTING_ENABLE
  IP_IP6_ROUTING_DATA_SET_FOR_NVM,
#endif
#if MPL_ENABLED
  MPL_DATA_SET_FOR_NVM,
#endif
#if TRICKLE_ENABLED
   TRICKLE_DATA_SET_FOR_NVM,
#endif
#if MAC_FILTERING_ENABLED  
  MAC_FILTERING_DATA_SET_FOR_NVM,
#endif  
#if ND_ENABLED
 ND_DATA_SET_FOR_NVM, 
#endif 

 /* Required end-of-table marker. */
  {NULL,0,0,gNvEndOfTableId_c,0}  
};

/*
 * Name: pNVM_DataTable
 * Description: Pointer to NVM table. The content of the table
 * is defined by the application code. See NvDataTable.
 */
NVM_DataEntry_t* pNVM_DataTable = (NVM_DataEntry_t*)NVM_DataTable;

#endif /*gNvStorageIncluded_d */

