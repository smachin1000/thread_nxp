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
\file       app_init.c
\brief      This is a public source file for the initial system startup module. It contains
            the implementation of the interface functions.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "MK21DA5.h"

#include "PhyInterface.h"
#include "MacInterface.h"

/* FSL Framework */
#include "RNG_Interface.h"
#include "Led.h"
#include "nvm_adapter.h"
#include "NVM_Interface.h"
#include "TimersManager.h"
#include "Keyboard.h"

#include "fsl_osa_ext.h"
#include "app_init.h"
#include "app_config.h"

#if THREAD_USE_FSCI
    #include "fsci_commands.h"
    #include "FsciInterface.h"
#endif

#if THREAD_USE_SHELL
    #include "shell_commands.h"
#endif

extern void App_Task(osaTaskParam_t argument);
extern void KBD_Callback(uint8_t events);
/*==================================================================================================
Private macros
==================================================================================================*/

/* None */

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

static void APP_Init(void);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

#if THREAD_USE_FSCI
/* FSCI Interface Configuration structure */
static const gFsciSerialConfig_t mFsciSerials[] = {
    /* Baudrate,           interface type,   channel number */
#if FSCI_USB_ENABLE
    {
        .baudrate = gUARTBaudRate115200_c,
        .interfaceType = gSerialMgrUSB_c,
        .interfaceChannel = 0
    },
#else
    {
        .baudrate = gUARTBaudRate115200_c,
        .interfaceType = gSerialMgrUart_c,
        .interfaceChannel = 1
    },
#endif
};
#endif

OSA_EXT_TASK_DEFINE( App_Task, APP_TASK_PRIORITY,  1, APP_TASK_STACK_SIZE, 0);
/*==================================================================================================
Public global variables declarations
==================================================================================================*/
volatile uint32_t gaUniqueId[4];
taskMsgQueue_t  appThreadMsgQueue;

/*==================================================================================================
Public functions
==================================================================================================*/
/*==================================================================================================
==================================================================================================*/
void main_task(uint32_t param)
{
    /* Initialize framework and platform drivers */
    
    /* Init memory blocks manager */
    MEM_Init();    
    
    /* Init  timers module */
    TMR_Init();
    TMR_TimeStampInit();
    
    /* Init Led module */
    LED_Init();
   
    NVNG_Init(TRUE);

    /* Init phy module */  
    Phy_Init();
    
    /* RNG must be initialized after the PHY is Initialized */
    RNG_Init(); 
    
    /* Init mac module */
    MAC_Init();
    

    /* Initialize Keyboard (Switches) Module */
    KBD_Init(KBD_Callback);


    /* Get board UniqueID */
    gaUniqueId[0] = SIM_UIDH;
    gaUniqueId[1] = SIM_UIDMH;
    gaUniqueId[2] = SIM_UIDML;
    gaUniqueId[3] = SIM_UIDL;
        
    #if THREAD_USE_SHELL
        SHELLComm_Init(&appThreadMsgQueue);
    #endif
    
    #if THREAD_USE_FSCI
        SerialManager_Init();
    
        FSCI_Init((void*)&mFsciSerials);
    
        APP_FsciInterface(&appThreadMsgQueue);
    #endif
    
    /* Device is not started */
    #if gUSBKW24D512Dongle
        LED_StartFlash(LED1|LED2);
    #else
        LED_StartFlash(LED_ALL);
    #endif
      
    /* Start demo application */
    APP_Init();  
    
    /* Main Application Loop */
    while(1)
    {   
        #if NVM_NG_ENABLED
            /* Process NV Storage save-on-idle, save-on-count and save-on-interval requests */
            NvIdle();
        #endif
    }
}


/*==================================================================================================
Private functions
==================================================================================================*/

/*!*************************************************************************************************
\private
\fn     void APP_Init(void)
\brief  This function is used to create application task and init app msg queue.

\param  [in]    void

\return         void
***************************************************************************************************/
static void APP_Init
(
  void
)
{
    /* Initialize main thread message queue */
    ListInit(&appThreadMsgQueue.msgQueue,20);
    appThreadMsgQueue.taskEventId = OSA_EXT_EventCreate(TRUE);
    appThreadMsgQueue.taskId = OSA_EXT_TaskCreate(OSA_EXT_TASK(App_Task), (osaTaskParam_t)NULL);
}
/*==================================================================================================
Private debug functions
==================================================================================================*/
