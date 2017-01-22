/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MpmInterface.h
* This is a header file for the Multiple PAN Manager.
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

#ifndef __MPM_H__
#define __MPM_H__

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "PhyInterface.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

/* The maximun number of MAC instances that can be registered. 
 * If set to 0, the MPM is disabled!
 */
#if !defined(gMpmMaxPANs_c)
#define gMpmMaxPANs_c 1
#endif

#define gMpmIncluded_d (gMpmMaxPANs_c > 1)

#define gMpmPhyPanRegSets_c          (2)
#define gMpmUseDifferentTxPwrLevel_c (0)
#define gMpmAcquireIsBlocking_d      (0)
#define gMpmInvalidRegSet_c (gMpmPhyPanRegSets_c)
     
/*        Dual Pan Dwell settings
   +-----------------+---------------------+
   | PRESCALER       |    RANGE            |
   | bits [1:0]      |  bits [7:2]         |
   +------+----------+---------------------+
   |value | timebase |  min - max          |
   +------+----------+---------------------+
   |  00  |  0.5 ms  |  0.5 - 32  ms       |
   |  01  |  2.5 ms  |  2.5 - 160 ms       |
   |  10  |  10  ms  |   10 - 640 ms       |
   |  11  |  50  ms  |   50 - 3.2 seconds  |
   +------+----------+---------------------+
*/

  /* Dwell Time prescaller (0 to 3) */
  #define mDefaultDualPanDwellPrescaller_c (0x00) // 0,5 ms
  #define mDualPanDwellPrescallerMask_c    (0x03)
  #define mDualPanDwellPrescallerShift_c   (0)

  /* Dwell Time value (0 to 63) */
  #define mDefaultDualPanDwellTime_c       (0x06)
  #define mDualPanDwellTimeMask_c          (0xFC)
  #define mDualPanDwellTimeShift_c         (2)


#if (gMpmMaxPANs_c > gMpmPhyPanRegSets_c)
    #error The number of PANs exceeds the number of HW registry sets! This feature is not supported yet.
#endif

/* MPM flags */
#define gMpmFlagPanCoord_c      (1 << 0)
#define gMpmFlagPromiscuous_c   (1 << 1)
#define gMpmFlagRxOnWhenIdle_c  (1 << 2)

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

typedef PACKED_STRUCT mpmConfig_tag{
    bool_t  autoMode;
    uint8_t dwellTime;
    uint8_t activeMAC;
}mpmConfig_t;

typedef union panFlags_tag{
    uint16_t all;
    struct{
        uint16_t panCoordinator: 1;
        uint16_t promiscuous:    1;
        uint16_t rxOnWhenIdle:   1;
        uint16_t reserved:       13;
    };
}panFlags_t;

typedef struct panInfo_tag{
    uint8_t        flags;
    uint8_t        macInstance;
    uint8_t        phyRegSet;
    int8_t         locked;
#if (gMpmMaxPANs_c > gMpmPhyPanRegSets_c)
    uint64_t       longAddr;
    uint16_t       shortAddr;
    uint16_t       panId;
    uint8_t        channel;
#endif
#if gMpmUseDifferentTxPwrLevel_c
    uint8_t        pwrLevel;
#endif
}panInfo_t;

#ifdef __cplusplus
extern "C" {
#endif 

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
#if gMpmIncluded_d

void MPM_Init( void );
void MPM_SetConfig( mpmConfig_t *pCfg );
void MPM_GetConfig( mpmConfig_t *pCfg );

int32_t  MPM_GetPanIndex( instanceId_t macInstance );
uint32_t MPM_GetRegSet(uint8_t panIdx);
uint32_t MPM_GetMacInstanceFromRegSet(uint32_t regSet);
#define MPM_isPanActive( panIdx ) (MPM_GetRegSet(panIdx) != gMpmInvalidRegSet_c)

phyStatus_t MPM_PrepareForTx( instanceId_t macInstance );
phyStatus_t MPM_PrepareForRx( instanceId_t macInstance );
phyStatus_t MPM_GetPIB(phyPibId_t pibId, void *pValue, uint8_t panIdx);
phyStatus_t MPM_SetPIB(phyPibId_t pibId, void *pValue, uint8_t panIdx);

#else /* #if gMpmIncluded_d */

#define MPM_Init()
#define MPM_SetConfig( prescaller, dwellTime )

#define MPM_GetPanIndex( macInstance )          0
#define MPM_GetRegSet( panIdx )                 0
#define MPM_GetMacInstanceFromRegSet( regSet )  0
#define MPM_isPanActive( panIdx )               1

#define MPM_PrepareForTx( macInstance )        gPhySuccess_c
#define MPM_PrepareForRx( macInstance )        gPhySuccess_c
#define MPM_GetPIB( pibId, pibValue, panIdx )  gPhySuccess_c
#define MPM_SetPIB( pibId, pibValue, panIdx )  gPhySuccess_c

#endif /* #if gMpmIncluded_d */

phyStatus_t MPM_AcquirePAN( instanceId_t macInstance );
phyStatus_t MPM_ReleasePAN( instanceId_t macInstance );

#ifdef __cplusplus
}
#endif 

#endif /*__MPM_H__ */
