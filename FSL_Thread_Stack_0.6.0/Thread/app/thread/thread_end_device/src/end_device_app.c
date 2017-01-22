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
\file       end_device_app.c
\brief      This is a public source file for the end device demo application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* General Includes */
#include "EmbeddedTypes.h"
#include <string.h>
#include <stdio.h>

/* Application */
#include "app_init.h"
#include "app_to_stack_config.h"
#include "stack_config.h"
#include "end_device_app.h"
#include "socket_app_utils.h"
#include "thread_manager_config.h"

/* FSL Framework */
#include "MemManager.h"
#include "TimersManager.h"
#include "FunctionLib.h"
#include "shell.h"
#include "Led.h"
#include "Keyboard.h"

/* Network */
#include "stack_events.h"
#include "event_manager.h"
#include "nwk_params.h"

#include "shell_commands.h"

#if USE_TEMPERATURE_SENSOR
  #include "app_temp_sensor.h"
#endif
/*==================================================================================================
Private macros
==================================================================================================*/

#if (THREAD_USE_SHELL == FALSE)
    #define shell_write(a)
    #define shell_refresh()
    #define shell_printf(a,...)
    #define SHELL_PrintIpAddr(a)
#endif

#if (SOCK_DEMO_SERVER == 1)
#error "*** ERROR: SOCKET DEMO SERVER IS IMPLEMENTED JUST ON THREAD ROUTER DEMO"
#endif
/*==================================================================================================
Private type definitions
==================================================================================================*/

typedef enum{
    gOffTheNetwork_c,
    gStartJoinTheNetwork_c,
    gOnTheNetwork_c
}nwkDeviceStatus_t;


/*==================================================================================================
Private global variables declarations
==================================================================================================*/
/* Network status */
static nwkDeviceStatus_t  nwkDeviceStatus = gOffTheNetwork_c; 

/* Pointer application task message queue */
static taskMsgQueue_t * mpAppThreadMsgQueue;

/* remote information 
  - used to report the current temperature when the SW2(Twr)/SW1(USB) is pressed */    
static ipAddr_t gAppRemoteAddress;
static uint16_t gAppRemoteUdpPort = 0;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

extern bool_t gEnable802154TxLed;

/*==================================================================================================
Private prototypes
==================================================================================================*/
static void Stack_to_APP_Handler(void* param);
static void APP_RegisterToStackEvents(void);
static void APP_ReportTemp(void *param);
static void APP_SetRemoteNodeInf(uint16_t updPort, ipAddr_t ipAddress);

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn     void App_Task(osaTaskParam_t argument)
\brief  Application task.

\param  [in]    argument    task private data

\return         void
***************************************************************************************************/
void App_Task(osaTaskParam_t argument)
{
  
    /* Init pointer to app task message queue */
    mpAppThreadMsgQueue = &appThreadMsgQueue;
    
    /* Register application to stack events */
    APP_RegisterToStackEvents();
  
    /* Init demo application sockets */
    APP_InitUserSockets(mpAppThreadMsgQueue);
  
    shell_write("\rEnd Device Application Demo\n\r");
    shell_write("Press a board switch or enter 'startnwk' to create or join a Thread network!");
    shell_refresh();
    
    #if (AUTO_STACK_START)
        /* Start thread stack */
        NWKU_SendMsg(APP_StartDevice, NULL, mpAppThreadMsgQueue);
    #endif
    
    #if USE_TEMPERATURE_SENSOR
        APP_InitADC(ADC_0);
        APP_SetRemoteNodeInf(UDP_PORT, in6addr_sitelocal_allrouters);
    #endif    
        
    /* Application Task Loop */
    while(1)
    {
        NWKU_RecvMsg(&appThreadMsgQueue);
    }
}

/*!*************************************************************************************************
\fn     void KBD_Callback(uint8_t events)
\brief  This is a callback function called from the KBD module.

\param  [in]    events  value of the events

\return         void
***************************************************************************************************/
void KBD_Callback
(
    uint8_t events
)
{
    if(gOffTheNetwork_c == nwkDeviceStatus)
    {
        #if !AUTO_STACK_START
            NWKU_SendMsg(APP_StartDevice, NULL, mpAppThreadMsgQueue);
        #endif
        return;    
    }    
    switch(events)
    {
        case gKBD_EventPB1_c:    
            #if gUSBKW24D512Dongle  
                NWKU_SendMsg(APP_ReportTemp, NULL, mpAppThreadMsgQueue);    
            #endif
            break;
        case gKBD_EventPB2_c:
            NWKU_SendMsg(APP_ReportTemp, NULL, mpAppThreadMsgQueue);
            break;
        case gKBD_EventPB3_c:
        case gKBD_EventPB4_c:                     
        case gKBD_EventLongPB1_c:
        case gKBD_EventLongPB2_c:
        case gKBD_EventLongPB3_c:
        case gKBD_EventLongPB4_c:   
            break;
        default:
            break;
  }
}

/*!*************************************************************************************************
\fn     void APP_StartDevice(void)
\brief  This function is used to start device.

\param  [in]     param   pointer not used

\return         void
***************************************************************************************************/
void APP_StartDevice
(
  void *param
)
{
    nwkStartParams_t nwkStartParams;
    (void)param;
    if(gOffTheNetwork_c != nwkDeviceStatus)
    {
        shell_write("\rNetwork already started!");
        shell_refresh();
        return;
    }
    shell_write("\rStarting network...");
    shell_refresh();
    Stack_Start(param);
    nwkDeviceStatus = gStartJoinTheNetwork_c;

    
    NWK_GetStack(NULL, &nwkStartParams.pStackConfig, &nwkStartParams.pStack);
    shell_printf("\rAttaching to Thread network on channel %d\n\r",nwkStartParams.pStack->pMacCfg->channel);
    
    #if gUSBKW24D512Dongle
        /* Stop LEDs Flashing */
        LED_StopFlash(LED1|LED2);
        /* LED1 flashing */
        Led1Flashing();
    #else
        /* Stop LEDs Flashing */
        LED_StopFlashingAllLeds();
        /* start serial flashing*/
        LED_StartSerialFlash(LED1); 
    #endif  
        
}
/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn     void APP_SetRemoteNodeInf(void)
\brief  This function is used to initialize the remote node information.

\param  [in]    uint16_t updPort
                ipAddr_t ipAddress

\return         void
***************************************************************************************************/
static void APP_SetRemoteNodeInf
(
    uint16_t updPort,
    ipAddr_t ipAddress
)
{
    /* set remote information 
       - used to report the current temperature when the SW2(Twr)/SW1(USB) is pressed */       
    gAppRemoteUdpPort = updPort;
    IP_AddrCopy(&gAppRemoteAddress, &ipAddress);
}
/*!*************************************************************************************************
\private
\fn     void APP_RegisterToStackEvents(void)
\brief  This function is used to register an application callback to stack events.

\param  [in]    void

\return         void
***************************************************************************************************/
static void APP_RegisterToStackEvents
(
  void
)
{
  
    EVM_RegisterDynamic(gStackEvJoinSuccess_c,          Stack_to_APP_Handler, &mpAppThreadMsgQueue, FALSE);
    EVM_RegisterDynamic(gStackEvJoinFailed_c,           Stack_to_APP_Handler, &mpAppThreadMsgQueue, FALSE);
    EVM_RegisterDynamic(gStackEvDisconnected_c,         Stack_to_APP_Handler, &mpAppThreadMsgQueue, FALSE);
    EVM_RegisterDynamic(gStackEvRequestingGlobalAddr_c, Stack_to_APP_Handler, &mpAppThreadMsgQueue, FALSE);
    EVM_RegisterDynamic(gStackEvGlobalAddrAssigned_c,   Stack_to_APP_Handler, &mpAppThreadMsgQueue, FALSE);
    EVM_RegisterDynamic(gStackEvDeviceBecameLeader_c,   Stack_to_APP_Handler, &mpAppThreadMsgQueue, FALSE);  
    EVM_RegisterDynamic(gStackEvDeviceCreatedNwk_c,     Stack_to_APP_Handler, &mpAppThreadMsgQueue, FALSE);   
    EVM_RegisterDynamic(gStackEvRequestingRouterId_c,   Stack_to_APP_Handler, &mpAppThreadMsgQueue, FALSE);    
}

/*!*************************************************************************************************
\private
\fn     void Stack_to_APP_Handler(void)
\brief  This function is used to handle stack events in asyncronous mode.

\param  [in]    param   pointer to stack event

\return         void
***************************************************************************************************/
static void Stack_to_APP_Handler
(
  void* param
)
{
    uint32_t stackEvent = *(uint32_t*)param;
    nwkStartParams_t nwkStartParams;
    
    NWK_GetStack(NULL, &nwkStartParams.pStackConfig, &nwkStartParams.pStack);   
    
    switch(stackEvent)
    {
        case gStackEvJoinSuccess_c:
            #if (THREAD_DEFAULT_IS_POLLING_END_DEVICE)
                shell_printf("\rAttached to network with PAN ID: 0x%x \n\r", nwkStartParams.pStack->pMacCfg->panId);
                shell_write("Node started as Polling End Device\n\r");
            #endif
            #if gUSBKW24D512Dongle
                LED_StopFlash(LED1|LED2);
                #if (!THREAD_DEFAULT_IS_POLLING_END_DEVICE)               
                    Led1On();
                #endif  
            #else
                LED_StopFlashingAllLeds();
                #if (!THREAD_DEFAULT_IS_POLLING_END_DEVICE)               
                    Led4On();
                #endif  
            #endif          
            #if THREAD_DEFAULT_CAN_CREATE_NEW_NETWORK
                shell_write("Success\n\r");
            #endif
            SHELL_PrintIpAddr(gMeshLocalAddr_c);	
            shell_refresh();
            nwkDeviceStatus = gOnTheNetwork_c;
            gEnable802154TxLed = TRUE;
            break;
        case gStackEvJoinFailed_c:
            shell_write("Cannot find an existing network\n\r");
            shell_refresh();
        case gStackEvDisconnected_c:
            #if gUSBKW24D512Dongle
                LED_StartFlash(LED1|LED2);
            #else
                LED_StartFlash(LED_ALL);
            #endif
             nwkDeviceStatus = gOffTheNetwork_c;
             break;
        case gStackEvRequestingGlobalAddr_c:
            shell_write("\rRequesting Global Address...\n\r"); 
            break;
        case gStackEvGlobalAddrAssigned_c:
            shell_write("Global Address has been assigned");
            shell_refresh();
            break;
        case gStackEvDeviceBecameLeader_c:
            #if !gUSBKW24D512Dongle
                Led1On();
            #endif 
            shell_write("\n\rNode has taken the Leader role");
            shell_refresh();
            break;
        case gStackEvDeviceCreatedNwk_c: 
            nwkDeviceStatus = gOnTheNetwork_c;
            gEnable802154TxLed = TRUE;
            #if gUSBKW24D512Dongle
                LED_StopFlash(LED1|LED2);
                Led1On();
            #else
                LED_StopFlashingAllLeds();      
                Led4On();
            #endif            
            shell_printf("\rCreated a new Thread network on channel %d and PAN ID:0x%x \n\r",nwkStartParams.pStack->pMacCfg->channel, nwkStartParams.pStack->pMacCfg->panId);
            SHELL_PrintIpAddr(gMeshLocalAddr_c);
            break;
        case gStackEvRequestingRouterId_c:
            shell_printf("Attached to network with PAN ID: 0x%x\n\r", nwkStartParams.pStack->pMacCfg->panId);      
            shell_write("Requesting to become Active Router...\n\r");
            break;
        default:
            break;
    } 
}

/*!*************************************************************************************************
\private
\fn     void APP_ReportTemp(void)
\brief  This open a socket and report the temperature to gAppRemoteAddress.

\param  [in]    param   pointer to stack event

\return         void
***************************************************************************************************/
static void APP_ReportTemp(void *param)
{
    
    static int32_t mSockfd = gBsdsSockInvalid_c;
    uint32_t macSecurityLevel = 5;
  
    if(gBsdsSockInvalid_c == mSockfd)
    {
        /* Create socket */
        mSockfd = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
        setsockopt(mSockfd, SOL_MAC, MAC_SECURITY_LEVEL, (void*)&macSecurityLevel, sizeof(uint32_t));

    }
    if(gBsdsSockInvalid_c != mSockfd)
    {
        uint8_t * pSendData;
        /* Set local information */
        sockaddrIn6_t mSsTx;
        mSsTx.sin6_family = AF_INET6;
        mSsTx.sin6_port   = gAppRemoteUdpPort;
        IP_AddrCopy(&mSsTx.sin6_addr, &gAppRemoteAddress);
        
        pSendData = App_GetTempDataString();
        if(NULL == pSendData)
        {
            return;
        }
    
        shell_write("\r");
        shell_write((char *)pSendData);
        shell_refresh();
            
        /* Send data via socket */     
        sendto(mSockfd,pSendData,strlen((char*)pSendData) + 1,0,(sockaddrStorage_t *)&mSsTx,sizeof(sockaddrIn6_t));
        MEM_BufferFree(pSendData);
        
    }
    else
    {
        shell_write("\r\nCan not create socket \r\n");
    }
}
/*==================================================================================================
Private debug functions
==================================================================================================*/
