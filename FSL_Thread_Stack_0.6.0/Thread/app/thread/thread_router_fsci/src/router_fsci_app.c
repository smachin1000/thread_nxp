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
\file       router_fsci_app.c
\brief      This is a public source file for the router fsci application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* General Includes */
#include "EmbeddedTypes.h"

/* Application */
#include "app_to_stack_config.h"
#include "app_init.h"
#include "router_fsci_app.h"

/* FSL Framework */
#include "Led.h"
#include "Keyboard.h"

/* Network */
#include "stack_events.h"
#include "event_manager.h"

/*==================================================================================================
Private macros
==================================================================================================*/

#if (SOCK_DEMO == 1)
#error "*** ERROR: SOCKET DEMO IS NOT IMPLEMENTED IN THIS APPLICATION"
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
static nwkDeviceStatus_t  nwkDeviceStatus = gOffTheNetwork_c;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

taskMsgQueue_t *pAppThreadMsgQueue = NULL;
extern bool_t gEnable802154TxLed;

/*==================================================================================================
Private prototypes
==================================================================================================*/
static void Stack_to_APP_Handler(void* param);
static void APP_RegisterToStackEvents(void);

/*==================================================================================================
Public functions
==================================================================================================*/
void App_Task(osaTaskParam_t argument)
{
    
  pAppThreadMsgQueue = &appThreadMsgQueue;
  
  /* Register application to stack events */
  APP_RegisterToStackEvents();
  
    #if (AUTO_STACK_START)
        /* Start thread stack */
        NWKU_SendMsg(APP_StartDevice, NULL, &appThreadMsgQueue);
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
            NWKU_SendMsg(APP_StartDevice, NULL, &appThreadMsgQueue);
        #endif
        return;    
    }    
    switch(events)
    {
        case gKBD_EventPB1_c:            
        case gKBD_EventPB2_c:
        case gKBD_EventPB3_c:
        case gKBD_EventPB4_c:                     
        case gKBD_EventLongPB1_c:
        case gKBD_EventLongPB2_c:
        case gKBD_EventLongPB3_c:
        case gKBD_EventLongPB4_c:   
        default:
            break;
  }
}
/*!*************************************************************************************************
\fn     void APP_StartDevice(void)
\brief  This function is used to start device.

\param  [in]    void

\return         void
***************************************************************************************************/
void APP_StartDevice
(
  void *param
)
{
    (void)param;
    if(gOffTheNetwork_c != nwkDeviceStatus)
    {
        return;
    }
    Stack_Start(param);
    nwkDeviceStatus = gStartJoinTheNetwork_c;
    
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
  EVM_RegisterDynamic(gStackEvJoinSuccess_c  ,Stack_to_APP_Handler, &pAppThreadMsgQueue, FALSE);
  EVM_RegisterDynamic(gStackEvJoinFailed_c   ,Stack_to_APP_Handler, &pAppThreadMsgQueue, FALSE);
  EVM_RegisterDynamic(gStackEvDisconnected_c ,Stack_to_APP_Handler, &pAppThreadMsgQueue, FALSE);
  EVM_RegisterDynamic(gStackEvRequestingGlobalAddr_c ,Stack_to_APP_Handler, &pAppThreadMsgQueue, FALSE);
  EVM_RegisterDynamic(gStackEvGlobalAddrAssigned_c ,Stack_to_APP_Handler, &pAppThreadMsgQueue, FALSE);
  EVM_RegisterDynamic(gStackEvDeviceBecameLeader_c ,Stack_to_APP_Handler, &pAppThreadMsgQueue, FALSE);  
  EVM_RegisterDynamic(gStackEvDeviceCreatedNwk_c ,Stack_to_APP_Handler, &pAppThreadMsgQueue, FALSE);   
  EVM_RegisterDynamic(gStackEvRequestingRouterId_c ,Stack_to_APP_Handler, &pAppThreadMsgQueue, FALSE);   
  
  
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
    switch(stackEvent)
    {
        case gStackEvJoinSuccess_c:
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
            gEnable802154TxLed = TRUE;
            nwkDeviceStatus = gOnTheNetwork_c;
            break;
        case gStackEvJoinFailed_c:
        case gStackEvDisconnected_c:
            #if gUSBKW24D512Dongle
                LED_StartFlash(LED1|LED2);
            #else
                LED_StartFlash(LED_ALL);
            #endif
             nwkDeviceStatus = gOffTheNetwork_c;
            break;
        case gStackEvRequestingGlobalAddr_c:
            break;
        case gStackEvGlobalAddrAssigned_c:       
            break;
        case gStackEvDeviceBecameLeader_c:          
            #if !gUSBKW24D512Dongle
                Led1On();
            #endif 
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
            break;
        case gStackEvRequestingRouterId_c:
            break;
        default:
            break;
    } 
  
}

/*==================================================================================================
Private debug functions
==================================================================================================*/

