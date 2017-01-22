/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file ProjectConfig.h
* This file holds type definitions that maps the standard c-types into types
* with guaranteed sizes. The types are target/platform specific and must be edited
* for each new target/platform.
* The header file also provides definitions for TRUE, FALSE and NULL.
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

#ifndef _PROJECTCONFIG_H_ 
#define _PROJECTCONFIG_H_ 


/************************************************************************************
*
*       CONFIG APPLICATION
*
************************************************************************************/
#define THREAD_USE_SHELL                                0
#define THREAD_USE_FSCI                                 1
#define gFsciIncluded_c                                 1

#if gUSBKW24D512Dongle
#define FSCI_USB_ENABLE                                 1
#else
#define FSCI_UART_ENABLE                                1
#endif

#include "app_config.h"

/************************************************************************************
*
*       CONFIG STACK
*
************************************************************************************/
#define STACK_THREAD                                    1

/* Thread Router which can start network and/or become its Leader and/or act as Active Router. 
    FSCI enabled by default */
#define THREAD_DEFAULT_CAN_CREATE_NEW_NETWORK           1 
#define THREAD_DEFAULT_CAN_BECOME_ACTIVE_ROUTER         1
#define THREAD_DEFAULT_IS_POLLING_END_DEVICE            0

//#include "app_to_stack_config.h"

/************************************************************************************
*
*       CONFIG FRAMEWORK
*
************************************************************************************/

#define gKeyBoardSupported_d 1

#include "app_to_fwk_config.h"

/************************************************************************************
*
*       CONFIG Mac/Phy
*
************************************************************************************/
#include "app_to_mac_config.h"
  
#endif /* _PROJECTCONFIG_H_ */
