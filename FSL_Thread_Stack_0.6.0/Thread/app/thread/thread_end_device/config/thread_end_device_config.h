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

#ifndef _THREAD_SOCKET_CLIENT_CONFIG_H_ 
#define _THREAD_SOCKET_CLIENT_CONFIG_H_ 

/*!=================================================================================================
\file       thread_socket_server_config.h
\brief      This is the header file for the configuration of the thread socket server demo 
            application.
==================================================================================================*/

/*==================================================================================================
CONFIG APPLICATION
==================================================================================================*/

#define SOCK_DEMO                  1

#define USE_TEMPERATURE_SENSOR     1

#define THREAD_USE_SHELL           1
#define THREAD_USE_FSCI            0
#include "app_config.h"

/*!=================================================================================================
  CONFIG STACK
==================================================================================================*/
#define STACK_THREAD 1

/* Node which cannot start network nor become its Leader nor act as Active Router */
#define THREAD_DEFAULT_CAN_CREATE_NEW_NETWORK    0
#define THREAD_DEFAULT_CAN_BECOME_ACTIVE_ROUTER  0
#define THREAD_DEFAULT_IS_POLLING_END_DEVICE     1 


//#include "app_to_stack_config.h"

/*!=================================================================================================
     CONFIG FRAMEWORK
==================================================================================================*/

#define gKeyBoardSupported_d 1

#include "app_to_fwk_config.h"

/*!=================================================================================================
        CONFIG Mac/Phy
==================================================================================================*/
#include "app_to_mac_config.h"
  
#endif /* _THREAD_SOCKET_CLIENT_CONFIG_H_ */
