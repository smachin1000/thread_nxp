/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file FsciMacCommands.h
* This is the private header file for the MAC FSCI cmmands.
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


#ifndef _FSCI_MAC_COMMANDS_H_
#define _FSCI_MAC_COMMANDS_H_


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "FsciInterface.h"


/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */

/*
 * Operation Codes used in packets sent/received from the Serial Interface
 */
enum { /* NWK_MLME_SapHandler() OG = 0x85*/
    mFsciNwkMlmeAssociateReq_c    = 0x00,
    mFsciNwkMlmeAssociateRes_c    = 0x01,
    mFsciNwkMlmeDisassociateReq_c = 0x02,
    mFsciNwkMlmeGetReq_c          = 0x03,
    mFsciNwkMlmeGtsReq_c          = 0x04,
    mFsciNwkMlmeOrphanRes_c       = 0x05,
    mFsciNwkMlmeResetReq_c        = 0x06,
    mFsciNwkMlmeRxEnableReq_c     = 0x07,
    mFsciNwkMlmeScanReq_c         = 0x08,
    mFsciNwkMlmeSetReq_c          = 0x09,
    mFsciNwkMlmeStartReq_c        = 0x0A,
    mFsciNwkMlmeSyncReq_c         = 0x0B,
    mFsciNwkMlmePollReq_c         = 0x0C,
};

enum { /* MLME_NWK_SapHandler() OG = 0x84 */
    mFsciMlmeNwkAssociateInd_c       = 0x00,
    mFsciMlmeNwkAssociateCnf_c       = 0x01,
    mFsciMlmeNwkDisassociateIng_c    = 0x02,
    mFsciMlmeNwkDisassociateCnf_c    = 0x03,
    mFsciMlmeNwkBeaconNotifyInd_c    = 0x04,
    mFsciMlmeNwkGetCnf_c             = 0x05,
    mFsciMlmeNwkGtsInd_c             = 0x06,
    mFsciMlmeNwkGtsCnf_c             = 0x07,
    mFsciMlmeNwkOrphanInd_c          = 0x08,
    mFsciMlmeNwkRxEnableCnf_c        = 0x0A,
    mFsciMlmeNwkScanCnf_c            = 0x0B,
    mFsciMlmeNwkCommStatusInd_c      = 0x0C,
    mFsciMlmeNwkSetCnf_c             = 0x0D,
    mFsciMlmeNwkStartCnf_c           = 0x0E,
    mFsciMlmeNwkSyncLossInd_c        = 0x0F,
    mFsciMlmeNwkPollCnf_c            = 0x10,
    mFsciMlmeNwkErrorCnf_c           = 0x11,
    mFsciMlmeNwkBeaconStartInd_c     = 0x12,
    mFsciMlmeNwkMaintenanceScanCnf_c = 0x13,
    mFsciMlmeNwkPollNotifyInd_c      = 0x14,
};

enum { /* NWK_MCPS_SapHandler() OG = 0x87 */
    mFsciNwkMcpsDataReq_c  = 0x00,
    mFsciNwkMcpsPurgeReq_c = 0x01,
};

enum { /* MCPS_NWK_SapHandler() OG = 0x86 */
    mFsciMcpsNwkDataCnf_c  = 0x00,
    mFsciMcpsNwkDataInd_c  = 0x01,
    mFsciMcpsNwkPurgeCnf_c = 0x02,
    mFsciMcpsNwkPromInd_c  = 0x03,
};

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#define gFSCI_MlmeNwkOpcodeGroup_c       0x84    /* MLME_NWK_SapHandler          */
#define gFSCI_NwkMlmeOpcodeGroup_c       0x85    /* NWK_MLME_SapHandler          */
#define gFSCI_McpsNwkOpcodeGroup_c       0x86    /* MCPS_NWK_SapHandler          */
#define gFSCI_NwkMcpsOpcodeGroup_c       0x87    /* NWK_MCPS_SapHandler          */


/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
void fsciRegisterMac( instanceId_t macInterface, uint32_t fsciInterfaceId );
uint32_t fsciGetMacInterfaceId( instanceId_t macInstance );
instanceId_t fsciGetMacInstanceId( uint32_t interfaceId );
void MAC_Monitor(uint8_t sapId, void *pMsg, void* param, uint8_t instanceId );

void fsciMcpsReqHandler(void *pData, void* param, uint32_t interfaceId);
void fsciMlmeReqHandler(void *pData, void* param, uint32_t interfaceId);

void MlmeSapMonitor(void *pData, void* param, uint32_t interfaceId);
void McpsSapMonitor(void *pData, void* param, uint32_t interfaceId);

#endif /* _FSCI_MAC_COMMANDS_H_ */