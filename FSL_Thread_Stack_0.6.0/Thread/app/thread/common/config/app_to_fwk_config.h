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

#ifndef _APP_TO_FWK_CONFIG_H_ 
#define _APP_TO_FWK_CONFIG_H_

/*!=================================================================================================
\file       app_to_fwk_config.h
\brief      This file is a for framework configuration of all thread demo applications.
            If it is required to configure just one application use the appllication config. file.
            Ex: for thread router application use thread_router_config.h 
==================================================================================================*/ 

/*!=================================================================================================
       CONFIG FRAMEWORK
==================================================================================================*/ 

/*!=================================================================================================
* MEMORY MANAGER
==================================================================================================*/ 

#ifndef PoolsDetails_c
#define PoolsDetails_c \
         _block_size_  16    _number_of_blocks_    26 _eol_  \
         _block_size_  32    _number_of_blocks_    26 _eol_  \
         _block_size_  64    _number_of_blocks_    70 _eol_  \
         _block_size_  128   _number_of_blocks_    16 _eol_  \
         _block_size_  256   _number_of_blocks_     8 _eol_  \
         _block_size_  512   _number_of_blocks_     4 _eol_  \
         _block_size_  1600  _number_of_blocks_     4 _eol_  
#endif
/*!=================================================================================================
  SHELL
==================================================================================================*/ 
#ifndef SHELL_USE_PRINTF          
    #define SHELL_USE_PRINTF     1
#endif       
#ifndef SHELL_CB_SIZE          
    #define SHELL_CB_SIZE       80
#endif           
#ifndef SHELL_MAX_COMMANDS          
    #define SHELL_MAX_COMMANDS  16
#endif
#ifndef SHELL_MAX_ARGS
    #define SHELL_MAX_ARGS      10
#endif           
/*!=================================================================================================
  SEC_LIB
==================================================================================================*/ 
#ifndef gSecLib_HWSupport_d           
    #define gSecLib_HWSupport_d gSecLib_MMCAUSupport_d
#endif           
/*!=================================================================================================
  NVM
==================================================================================================*/         
#ifndef gNvStorageIncluded_d           
    #define gNvStorageIncluded_d        1
#endif
#ifndef gNvFragmentation_Enabled_d          
    #define gNvFragmentation_Enabled_d  1
#endif
#ifndef gNvUseExtendedFeatureSet_d          
    #define gNvUseExtendedFeatureSet_d  1
#endif           
#ifndef gUnmirroredFeatureSet_d         
    #define gUnmirroredFeatureSet_d     1
#endif           
#ifndef gNvDisableIntCmdSeq_c          
    #define gNvDisableIntCmdSeq_c       TRUE           
#endif
#ifndef gNvmEnableFSCIMonitoring_c           
  #define gNvmEnableFSCIMonitoring_c      (0)
#endif           
/*!=================================================================================================
  SERIAL MANAGER
==================================================================================================*/  
#ifndef gSerialMgrUseUSB_c          
  #define gSerialMgrUseUSB_c     0
#endif
#ifndef gSerialMgrUseUart_c          
    #define gSerialMgrUseUart_c    1
#endif
#ifndef gSerialTaskStackSize_c          
    /* key exchange require 300 bytes stack */           
    #define gSerialTaskStackSize_c 1500
#endif           
/*!=================================================================================================
  FSCI
==================================================================================================*/            
#ifndef gFsciIncluded_c           
    #define gFsciIncluded_c         0
#endif           
#ifndef gFsciLenHas2Bytes_c          
    #define gFsciLenHas2Bytes_c  1
#endif
#ifndef gFsciMaxPayloadLen_c          
    #define gFsciMaxPayloadLen_c 1500
#endif                
/*!=================================================================================================
  TMR
==================================================================================================*/            
#ifndef gTmrTaskStackSize_c
    #define gTmrTaskStackSize_c 1024
#endif
#ifndef gTmrStackTimers_c           
    #define gTmrStackTimers_c   15
#endif
/*!=================================================================================================
* OSA EXT
==================================================================================================*/                
#ifndef osNumberOfSemaphores
    #define osNumberOfSemaphores 5
#endif
#ifndef osNumberOfMutexes          
    #define osNumberOfMutexes    4
#endif
#ifndef osNumberOfMessageQs          
    #define osNumberOfMessageQs  1
#endif     
#ifndef osNumberOfMessages          
    #define osNumberOfMessages   20
#endif
#ifndef osNumberOfEvents          
    #define osNumberOfEvents     10
#endif           
#ifndef gMainThreadPriority_c          
    #define gMainThreadPriority_c  (OSA_PRIORITY_IDLE + 2)
#endif
#ifndef gMainThreadStackSize_c          
    #define gMainThreadStackSize_c 1280
#endif           
         
#endif /* _APP_TO_FWK_CONFIG_H_  */
