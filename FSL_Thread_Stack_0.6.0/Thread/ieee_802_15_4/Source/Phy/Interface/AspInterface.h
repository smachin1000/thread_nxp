/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file ASP.h
* This is a header file for the ASP module.
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

#ifndef __ASP_H__
#define __ASP_H__

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "PhyInterface.h"
#include "MpmInterface.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
/* Enable/Disable the ASP module */
#ifndef gAspCapability_d
#ifndef gPHY_802_15_4g_d
#define gAspCapability_d (1)
#else
#define gAspCapability_d (0)
#endif // gPHY_802_15_4g_d
#endif // gAspCapability_d

#define gAspInterfaceId (0)

/* Allowed ASP Power Levels */
#define gAspPowerLevel_m40dBm_c (uint8_t)gAspPowerLevel_m40dBm
#define gAspPowerLevel_m38dBm_c (uint8_t)gAspPowerLevel_m38dBm
#define gAspPowerLevel_m36dBm_c (uint8_t)gAspPowerLevel_m36dBm
#define gAspPowerLevel_m34dBm_c (uint8_t)gAspPowerLevel_m34dBm
#define gAspPowerLevel_m32dBm_c (uint8_t)gAspPowerLevel_m32dBm
#define gAspPowerLevel_m30dBm_c (uint8_t)gAspPowerLevel_m30dBm
#define gAspPowerLevel_m28dBm_c (uint8_t)gAspPowerLevel_m28dBm
#define gAspPowerLevel_m26dBm_c (uint8_t)gAspPowerLevel_m26dBm
#define gAspPowerLevel_m24dBm_c (uint8_t)gAspPowerLevel_m24dBm
#define gAspPowerLevel_m22dBm_c (uint8_t)gAspPowerLevel_m22dBm
#define gAspPowerLevel_m20dBm_c (uint8_t)gAspPowerLevel_m20dBm
#define gAspPowerLevel_m18dBm_c (uint8_t)gAspPowerLevel_m18dBm
#define gAspPowerLevel_m16dBm_c (uint8_t)gAspPowerLevel_m16dBm
#define gAspPowerLevel_m14dBm_c (uint8_t)gAspPowerLevel_m14dBm
#define gAspPowerLevel_m12dBm_c (uint8_t)gAspPowerLevel_m12dBm
#define gAspPowerLevel_m10dBm_c (uint8_t)gAspPowerLevel_m10dBm
#define gAspPowerLevel_m8dBm_c  (uint8_t)gAspPowerLevel_m8dBm
#define gAspPowerLevel_m6dBm_c  (uint8_t)gAspPowerLevel_m6dBm
#define gAspPowerLevel_m4dBm_c  (uint8_t)gAspPowerLevel_m4dBm
#define gAspPowerLevel_m2dBm_c  (uint8_t)gAspPowerLevel_m2dBm
#define gAspPowerLevel_0dBm_c   (uint8_t)gAspPowerLevel_0dBm
#define gAspPowerLevel_2dBm_c   (uint8_t)gAspPowerLevel_2dBm
#define gAspPowerLevel_4dBm_c   (uint8_t)gAspPowerLevel_4dBm
#define gAspPowerLevel_6dBm_c   (uint8_t)gAspPowerLevel_6dBm
#define gAspPowerLevel_8dBm_c   (uint8_t)gAspPowerLevel_8dBm
#define gAspPowerLevel_10dBm_c  (uint8_t)gAspPowerLevel_10dBm
#define gAspPowerLevel_12dBm_c  (uint8_t)gAspPowerLevel_12dBm
#define gAspPowerLevel_14dBm_c  (uint8_t)gAspPowerLevel_14dBm
#define gAspPowerLevel_16dBm_c  (uint8_t)gAspPowerLevel_16dBm

/* OpGroup codes used with FSCI */
#define gFSCI_AspAppOpcodeGroup_c        0x94    /* ASP_APP_SapHandler           */
#define gFSCI_AppAspOpcodeGroup_c        0x95    /* APP_ASP_SapHandler           */

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
/* ASP status messages */
typedef enum{
    gAspSuccess_c          = 0x00,
    gAspInvalidRequest_c   = 0xC2,
    gAspDenied_c           = 0xE2,
    gAspTooLong_c          = 0xE5,
    gAspInvalidParameter_c = 0xE8
}AspStatus_t;

/* Supported Power Levels */
enum {
    gAspPowerLevel_m40dBm = 0x03,
    gAspPowerLevel_m38dBm = 0x04,
    gAspPowerLevel_m36dBm = 0x05,
    gAspPowerLevel_m34dBm = 0x06,
    gAspPowerLevel_m32dBm = 0x07,
    gAspPowerLevel_m30dBm = 0x08,
    gAspPowerLevel_m28dBm = 0x09,
    gAspPowerLevel_m26dBm = 0x0A,
    gAspPowerLevel_m24dBm = 0x0B,
    gAspPowerLevel_m22dBm = 0x0C,
    gAspPowerLevel_m20dBm = 0x0D,
    gAspPowerLevel_m18dBm = 0x0E,
    gAspPowerLevel_m16dBm = 0x0F,
    gAspPowerLevel_m14dBm = 0x10,
    gAspPowerLevel_m12dBm = 0x11,
    gAspPowerLevel_m10dBm = 0x12,
    gAspPowerLevel_m8dBm  = 0x13,
    gAspPowerLevel_m6dBm  = 0x14,
    gAspPowerLevel_m4dBm  = 0x15,
    gAspPowerLevel_m2dBm  = 0x16,
    gAspPowerLevel_0dBm   = 0x17,
    gAspPowerLevel_2dBm   = 0x18,
    gAspPowerLevel_4dBm   = 0x19,
    gAspPowerLevel_6dBm   = 0x1A,
    gAspPowerLevel_8dBm   = 0x1B,
    gAspPowerLevel_10dBm  = 0x1C,
    gAspPowerLevel_12dBm  = 0x1D,
    gAspPowerLevel_14dBm  = 0x1E,
    gAspPowerLevel_16dBm  = 0x1F
};

/* Radio test modes */
enum {
    gTestForceIdle_c               = 0,
    gTestPulseTxPrbs9_c            = 1,
    gTestContinuousRx_c            = 2,
    gTestContinuousTxMod_c         = 3,
    gTestContinuousTxNoMod_c       = 4,
    gTestContinuousTx2Mhz_c        = 5,
    gTestContinuousTx200Khz_c      = 6,
    gTestContinuousTx1MbpsPRBS9_c  = 7,
    gTestContinuousTxExternalSrc_c = 8,  
    gTestContinuousTxNoModZero_c   = 9,
    gTestContinuousTxNoModOne_c    = 10
};

/* This enum matches with the FSCI OpCode used by ASP*/
typedef enum {
    aspMsgTypeGetTimeReq_c          = 0x00,
    aspMsgTypeGetInactiveTimeReq_c  = 0x01,
    aspMsgTypeGetMacStateReq_c      = 0x02,
    aspMsgTypeDozeReq_c             = 0x03,
    aspMsgTypeAutoDozeReq_c         = 0x04,
    aspMsgTypeAcomaReq_c            = 0x05,
    aspMsgTypeHibernateReq_c        = 0x06,
    aspMsgTypeWakeReq_c             = 0x07,
    aspMsgTypeEventReq_c            = 0x08,
    aspMsgTypeClkoReq_c             = 0x09,
    aspMsgTypeTrimReq_c             = 0x0A,
    aspMsgTypeDdrReq_c              = 0x0B,
    aspMsgTypePortReq_c             = 0x0C,
    aspMsgTypeSetMinDozeTimeReq_c   = 0x0D,
    aspMsgTypeSetNotifyReq_c        = 0x0E,
    aspMsgTypeSetPowerLevel_c       = 0x0F,
    aspMsgTypeGetPowerLevel_c       = 0x1F,
    aspMsgTypeTelecTest_c           = 0x10,
    aspMsgTypeTelecSetFreq_c        = 0x11,
    aspMsgTypeGetInactiveTimeCnf_c  = 0x12,
    aspMsgTypeGetMacStateCnf_c      = 0x13,
    aspMsgTypeDozeCnf_c             = 0x14,
    aspMsgTypeAutoDozeCnf_c         = 0x15,
    aspMsgTypeTelecSendRawData_c    = 0x16,
    aspMsgTypeSetFADState_c         = 0x17,
    aspMsgTypeSetFADThreshold_c     = 0x18,
    aspMsgTypeGetFADThreshold_c     = 0x19,
    aspMsgTypeGetFADState_c         = 0x1A,
    aspMsgTypeSetActivePromState_c  = 0x1B,
    aspMsgTypeXcvrWriteReq_c        = 0x1C,
    aspMsgTypeXcvrReadReq_c         = 0x1D,
    aspMsgTypeSetANTXState_c        = 0x20,
    aspMsgTypeGetANTXState_c        = 0x21,
    aspMsgTypeSetLQIMode_c          = 0x22,
    aspMsgTypeGetRSSILevel_c        = 0x23,
    aspMsgTypeSetMpmConfig_c        = 0x24,
    aspMsgTypeGetMpmConfig_c        = 0x25
}AppAspMsgType_t;

typedef PACKED_STRUCT aspEventReq_tag
{   /* AspEvent.Request              */
    uint32_t eventTime;
} aspEventReq_t;

typedef PACKED_STRUCT aspGetTimeReq_tag
{   /* AspGetTime.Request            */
    uint32_t time;
} aspGetTimeReq_t;

typedef PACKED_STRUCT aspSetNotifyReq_tag
{   /* AspSetNotify.Request          */
    uint8_t notifications;
} aspSetNotifyReq_t;


typedef PACKED_STRUCT aspSetPowerLevelReq_tag
{   /* AspSetPowerLevel.Request      */
    uint8_t powerLevel;
} aspSetPowerLevelReq_t;


typedef PACKED_STRUCT aspGetPowerLevelReq_tag
{   /* AspGetPowerLevel.Request      */
    uint8_t powerLevel;
} aspGetPowerLevelReq_t;


typedef PACKED_STRUCT aspTelecTest_tag
{   /* AspTelecTest.Request          */
    uint8_t mode;
} aspTelecTest_t;


typedef PACKED_STRUCT aspTelecsetFreq_tag
{   /* AspTelecSetFreq.Request       */
    uint8_t channel;
} aspTelecsetFreq_t;


typedef PACKED_STRUCT aspTelecSendRawData_tag
{   /* AspTelecSendRawData.Request   */
    uint8_t  length;
    uint8_t* dataPtr;
} aspTelecSendRawData_t;

    /* AspSetFADThreshold.Request   */
typedef uint8_t aspFADThreshold_t;
    /* AspSetLQIMode.Request    */
typedef uint8_t aspLQIMode_t;

typedef PACKED_STRUCT aspXcvrReq_tag
{   /* AspXcvrWrite.Request / AspXcvrRead.Request   */
    uint8_t  mode;
    uint16_t addr;
    uint8_t  len;
    uint8_t  data[4]; /* more than 4 bytes can be read/written */
} aspXcvrReq_t;


typedef PACKED_STRUCT AppToAspMessage_tag
{
    AppAspMsgType_t msgType;
    PACKED_UNION
    {
        aspEventReq_t           aspEventReq;
        aspGetTimeReq_t         aspGetTimeReq;
        aspSetPowerLevelReq_t   aspSetPowerLevelReq;
        aspGetPowerLevelReq_t   aspGetPowerLevelReq;
        aspTelecTest_t          aspTelecTest;
        aspTelecsetFreq_t       aspTelecsetFreq;
        aspTelecSendRawData_t   aspTelecSendRawData;
        aspFADThreshold_t       aspFADThreshold;
        bool_t                  aspFADState;
        bool_t                  aspANTXState;
        aspLQIMode_t            aspLQIMode;
        bool_t                  aspActivePromState;
        aspXcvrReq_t            aspXcvrData;
        mpmConfig_t             MpmConfig;
    }msgData;
} AppToAspMessage_t;

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
#if gAspCapability_d

void ASP_Init( instanceId_t phyInstance, uint8_t interfaceId );

AspStatus_t APP_ASP_SapHandler(AppToAspMessage_t *pMsg, instanceId_t instanceId);

void Asp_GetTimeReq(uint32_t *time);

AspStatus_t Asp_XcvrWriteReq (uint8_t mode, uint16_t addr, uint8_t len, uint8_t* pData);
AspStatus_t Asp_XcvrReadReq  (uint8_t mode, uint16_t addr, uint8_t len, uint8_t* pData);

AspStatus_t Asp_SetPowerLevel(uint8_t powerLevel);
uint8_t     Asp_GetPowerLevel(void);

AspStatus_t Asp_SetActivePromState(bool_t state);

AspStatus_t Asp_SetFADState(bool_t state);
AspStatus_t Asp_SetFADThreshold(uint8_t thresholdFAD);

AspStatus_t Asp_SetANTXState(bool_t state);
uint8_t     Asp_GetANTXState(void);
uint8_t     Asp_SetANTPadStateRequest(bool_t antAB_on, bool_t rxtxSwitch_on);
uint8_t     Asp_SetANTPadStrengthRequest(bool_t hiStrength);
uint8_t     Asp_SetANTPadInvertedRequest(bool_t invAntA, bool_t invAntB, bool_t invTx, bool_t invRx);

AspStatus_t Asp_SetLQIMode(bool_t mode);
uint8_t     Asp_GetRSSILevel(void);

AspStatus_t ASP_TelecSetFreq    (uint8_t channel);
AspStatus_t ASP_TelecSendRawData(uint8_t* dataPtr);
AspStatus_t ASP_TelecTest       (uint8_t mode);

#else /* gAspCapability_d */

#define ASP_Init(phyInstance,interfaceId)
#define Asp_GetTimeReq(time)

#define APP_ASP_SapHandler(pMsg)                  (gAspDenied_c)
#define Asp_XcvrWriteReq(mode, addr, len, pData)  (gAspDenied_c)
#define Asp_XcvrReadReq(mode, addr, len, pData)   (gAspDenied_c)
#define Asp_SetPowerLevel(powerLevel)             (gAspDenied_c)
#define Asp_SetActivePromState(state)             (gAspDenied_c)
#define Asp_SetFADState(state)                    (gAspDenied_c)
#define Asp_SetFADThreshold(thresholdFAD)         (gAspDenied_c)
#define Asp_SetANTXState(state)                   (gAspDenied_c)
#define Asp_SetLQIMode(mode)                      (gAspDenied_c)
#define ASP_TelecSetFreq(channel)                 (gAspDenied_c)
#define ASP_TelecSendRawData(dataPtr)             (gAspDenied_c)
#define ASP_TelecTest(mode)                       (gAspDenied_c)

#define Asp_GetPowerLevel() (0)
#define Asp_GetANTXState()  (0)
#define Asp_GetRSSILevel()  (0)
#endif /* gAspCapability_d */

#ifdef __cplusplus
}
#endif 

#endif /*__ASP_H__ */