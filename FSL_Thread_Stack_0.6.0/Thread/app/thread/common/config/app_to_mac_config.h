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

#ifndef _APP_TO_MAC_CONFIG_H_ 
#define _APP_TO_MAC_CONFIG_H_  

/*!=================================================================================================
\file       app_to_mac_config.h
\brief      This file is a for mac/phy configuration of all thread demo applications.
            If it is required to configure just one application use the appllication config. file.
            Ex: for thread router application use thread_router_config.h 
==================================================================================================*/ 

           
/*!=================================================================================================
    CONFIG Mac/Phy
==================================================================================================*/
#ifndef gFSCI_IncludeMacCommands_c
  #define gFSCI_IncludeMacCommands_c 0
#endif

#ifndef gMpmMaxPANs_c
    #define gMpmMaxPANs_c       1
#endif    
#ifndef gMacInstancesCnt_c
    #define gMacInstancesCnt_c  1
#endif
#ifndef gNumKeyTableEntries_c
    #define gNumKeyTableEntries_c        2
#endif
#ifndef gNumKeyIdLookupListEntries_c
    #define gNumKeyIdLookupListEntries_c 1
#endif
#ifndef gNumKeyDeviceListEntries_c
    #define gNumKeyDeviceListEntries_c   60
#endif
#ifndef gNumKeyUsageListEntries_c
    #define gNumKeyUsageListEntries_c    2
#endif
#ifndef gNumDeviceTableEntries_c
    #define gNumDeviceTableEntries_c     gNumKeyTableEntries_c*gNumKeyDeviceListEntries_c
#endif
         
#endif /* _APP_TO_MAC_CONFIG_H_   */
