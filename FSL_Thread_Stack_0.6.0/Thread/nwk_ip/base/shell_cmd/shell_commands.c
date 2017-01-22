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
\file       shell_commands.c
\brief      This is a public source file for the shell application. It contains the implementation
            of the shell commands used in the application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_to_stack_config.h"
#include "stack_config.h"
#include "app_init.h"

#include "EmbeddedTypes.h"
#include <string.h>
#include "stdio.h"
#include "shell.h"
#include "sockets.h"
#include "ip_if_management.h"
#include "icmp.h"
#include "sixlowpan.h"
#include "FunctionLib.h"
#include "session.h"
#include "virtual_enet_driver.h"
#include "panic.h"
#include "dhcp6_lq_client.h"
#include "nd.h"
#include "shell_commands.h"
#include "mac_filtering.h"
#include "nwk_params.h"

#include "dtls.h"



#if THROUGHPUT_DEMO
#include "app_throughput_demo.h"
#endif

#if STACK_FLIP
#include "flip_manager.h"
#endif

#if STACK_STATIC
#include "static_manager.h"
#endif

#if STACK_THREAD
#include "thread_routing.h"
#include "thread_network_data.h"
#include "thread_security.h"
#endif


#if SOCK_DEMO
    #include "socket_app_utils.h"
#endif


//#include "wmiconfig_custom.h"

#if THREAD_USE_SHELL
extern void ResetMCU(void);
extern void APP_StartDevice(void *param);

/*==================================================================================================
Private macros
==================================================================================================*/
#define PING_ID                     (1U)
#define PING_SEQ_NB                 (1U)
#define PING_PAYLOAD_DEFAULT_SIZE   (32U)
#define PING_PAYLOAD_START          '@'
#define PING_PAYLOAD_END            'W'
#define SHELL_CMD_MAX_ARGS          (10U)
#define PING_HEADER_SIZE            (4U)
#define DEFAULT_TIMEOUT             (2000U)
#define SHELL_PING_MIN_TIMEOUT      (2000U)
#define PING_DELAY                  (500U)


#if STACK_THREAD

#define SHELL_WRITE_STACK_CONFIG shell_write("THREAD Configuration\n\r\n\r")

#else

#define SHELL_WRITE_STACK_CONFIG shell_write("FlexibleIP Configuration\n\r\n\r")

#endif

/*==================================================================================================
Private type definitions
==================================================================================================*/


/*==================================================================================================
Private prototypes
==================================================================================================*/
#if SOCK_DEMO
    static int8_t SHELL_Socket(uint8_t argc, char *argv[]);
#endif
static int8_t SHELL_Ifconfig(uint8_t argc, char *argv[]);
static int8_t SHELL_Ping(uint8_t argc, char *argv[]);
static int8_t SHELL_Ping6(uint8_t argc, char *argv[]);
static int8_t SHELL_Reboot(uint8_t argc, char *argv[]);

#if ECHO_PROTOCOL
static int8_t SHELL_EchoUdp(uint8_t argc, char *argv[]);
#endif

#if AJ_THIN_CLIENT
static int8_t SHELL_AJ(uint8_t argc, char *argv[]);
#endif
static void SHELL_Resume(void);
static void PING_EchoReplyReceiveAsync(ipPktInfo_t *pRxIpPktInfo);
static void PING_EchoReplyReceive(void *pParam);
static void PING_TimerCallback(void *param);
static void PING_HandleTimerCallback(void *param);
static void PING_RetransmitCallback(void *param);
static void PING_RetransmitHandle(void *param);
static ipPktInfo_t *PingCreatePktInfo(ipAddr_t *pDstAddr, uint32_t payloadLen);


#if THROUGHPUT_DEMO
static int8_t SHELL_Iptest(uint8_t argc, char *argv[]);
#endif

#if DUAL_PAN_ZPRO_NWKIP
static int8_t SHELL_Tcpi(uint8_t argc, char *argv[]);
#endif

#if MAC_FILTERING_ENABLED
static int8_t SHELL_Filter(uint8_t argc, char *argv[]);
static void SHELL_Filtering_Print(void);
#endif
static int8_t SHELL_Remove(uint8_t argc, char *argv[]);
static int8_t SHELL_SetNwkParams(uint8_t argc, char *argv[]);
static int8_t SHELL_SetStackParams(uint8_t argc, char *argv[]);
static int8_t SHELL_StartStack(uint8_t argc, char *argv[]);

static int8_t SHELL_GetNwkParams(uint8_t argc, char *argv[]);


#if STACK_THREAD
#if 0
static int8_t SHELL_SwitchKeys(uint8_t argc, char *argv[]);
#endif
static int8_t SHELL_NwkData(uint8_t argc, char *argv[]);
#endif

#if DTLS_ENABLED
static int8_t SHELL_Dtls(uint8_t argc, char *argv[]);

static void DTLS_GetJpakePasswd(dtlsPeerPtr_t pPeer, dtlsJpakePasswd_t **pPasswd);
static void DTLS_Received(dtlsPeerPtr_t pPeer, uint8_t *pData, uint32_t len);
static void DTLS_Event(dtlsPeerPtr_t pPeer, dtlsAlertLevel_t level, dtlsAlertCode_t code);
#endif
/*==================================================================================================
Private global variables declarations
==================================================================================================*/

ShellComm_RegisterStatic(shellComm, "                   ", SHELL_CMD_MAX_ARGS, 0, NULL
#if SHELL_USE_HELP
    ,"",
    ""
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

ShellComm_RegisterStatic(shellComm, "ifconfig", SHELL_CMD_MAX_ARGS, 0, SHELL_Ifconfig
#if SHELL_USE_HELP
    ,"IP Stack interfaces configuration",
    "IP Stack interfaces configuration\r\n"
    "   ifconfig all - displays all interfaces and addresses configured on the device\r\n"
#if ND_ENABLED
    "   ifconfig ncache - displays the ND neighbors\r\n"
#endif
#if DHCP6_CLIENT_ENABLED
    "   ifconfig <interface ID> dhcpleasequery <EUI64 - 0x1122334455667788>\r\n"
    "   ifconfig <interface ID> dhcprelease\r\n"
#endif
#if 0
    "   ifconfig <interface ID> dhcp6start <clientMode [0-GlobalAddr/6-RouterIDAssign]> <serverIpAddress>\r\n"
    "   ifconfig <interface ID> dhcp6stop <clientMode [0-GlobalAddr/6-RouterIDAssign]>\r\n"
#endif

#endif /* SHELL_USE_HELP */
    "   ifconfig <interface ID> ip <IP address>", NULL);

ShellComm_RegisterStatic(shellComm, "ping", SHELL_CMD_MAX_ARGS, 0, SHELL_Ping
#if SHELL_USE_HELP
    ,"IP Stack ping IPv4/IPv6 addresses",
    "IP Stack ping IPv4/IPv6 addresses\r\n"
    "   ping <ip address> <timeout>\r\n"
    "   ping <ip address> <timeout> <t> -infinite ping"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

ShellComm_RegisterStatic(shellComm, "ping6", SHELL_CMD_MAX_ARGS, 0, SHELL_Ping6
#if SHELL_USE_HELP
    ,"IP Stack ping IPv6 only",
    "IP Stack ping IPv6 only\r\n"
    "   ping6 -s <size> -c <count> -i <timeout> -S <ipv6 source address> -h <hop limit> -macsec <MAC security \
    level> <ipv6 destination address>"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

#if THROUGHPUT_DEMO
ShellComm_RegisterStatic(shellComm, "iptest", SHELL_CMD_MAX_ARGS, 0, SHELL_Iptest
#if SHELL_USE_HELP
    ,"IP Stack throughput test",
    "IP Stack throughput test\r\n"
    "   iptest rx <protocol> <ip version>\r\n"
    "   iptest tx <protocol> <ip version> <ip address>\r\n"
    "   iptest tx <protocol> <ip version> <ip address> <size>\r\n"
    "   iptest tx <protocol> <ip version> <ip address> <size> <number of packets>\r\n"
    "   iptest tx <protocol> <ip version> <ip address> <size> <number of packets> <iterations>"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif

ShellComm_RegisterStatic(shellComm, "reboot", SHELL_CMD_MAX_ARGS, 0, SHELL_Reboot
#if SHELL_USE_HELP
    , "MCU Reset",
    "MCU Reset"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

#if DUAL_PAN_ZPRO_NWKIP
ShellComm_RegisterStatic(shellComm, "tcpi", SHELL_CMD_MAX_ARGS, 0, SHELL_Tcpi
#if SHELL_USE_HELP
    ,"TCPi Bulb Control",
    "TCPi Bulb Control\r\n"
    "   tcpi set\r\n"
    "   tcpi toggle\r\n"
    "   tcpi up\r\n"
    "   tcpi down"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif

#if AJ_THIN_CLIENT
ShellComm_RegisterStatic(shellComm, "aj", SHELL_CMD_MAX_ARGS, 0, SHELL_AJ
#if SHELL_USE_HELP
    ,"IP Stack All Joyn commands",
    "IP Stack All Joyn commands\r\n"
    "   commands not yet done"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif /* AJ_THIN_CLIENT */

#if ECHO_PROTOCOL
ShellComm_RegisterStatic(shellComm, "echoudp", SHELL_CMD_MAX_ARGS, 0, SHELL_EchoUdp
#if SHELL_USE_HELP
    ,"Echo udp client",
    "Echo udp client\r\n"
    "   echoudp <size> <ip address>\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif

#if 0
ShellComm_RegisterStatic(shellComm, "wmiconfig", SHELL_CMD_MAX_ARGS, 0, wmiconfig_handler
#if SHELL_USE_HELP
    ," Type wimiconfig --help for a list of all available commands"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif
 
ShellComm_RegisterStatic(shellComm, "setstackparams", SHELL_CMD_MAX_ARGS, 0, SHELL_SetStackParams
#if SHELL_USE_HELP
    ,"Set Stack Parameters",
    "Set Stack Parameters\r\n"
     "setstackparams -stack <stackID> -device <deviceID> -role <roleID>"
     "\r\n"
     "- stackID: 0 - THREAD\r\n"
//     "- stackID: 0 - THREAD, 1- FLIP, 2 - STATIC\r\n"
     "- deviceID for THREAD: 0 - END DEVICE, 1 - ELIGIBLE ROUTER, 2 - ROUTER\r\n"
     "- roleID: 0 - normal, 1 - leader\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

ShellComm_RegisterStatic(shellComm, "getnwkparams", SHELL_CMD_MAX_ARGS, 0, SHELL_GetNwkParams
#if SHELL_USE_HELP
    ,"Get Network Parameters",
    "Get Network Parameters\r\n"
     "getnwkparams <interfaceID>"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

ShellComm_RegisterStatic(shellComm, "setnwkparams", SHELL_CMD_MAX_ARGS, 0, SHELL_SetNwkParams
#if SHELL_USE_HELP
    ,"Set Network Parameters",
    "Set Network Parameters\r\n"
     "setnwkparams -ch <channel> -panID <panID> -extended <EUI64> -short <shortAddr> -rxonidle <0/1>"
#if STACK_THREAD
     " - mlprefix <Mesh Local Prefix>"
     " - mlprefixlen <Mesh Local Prefix Length>"
#endif
     "\r\n"
     "- channel: decimal format\r\n"
     "- panID format: 0xabcd\r\n"
     "- short address format: 0xabcd\r\n"
     "- extended address format: 0x1122334455667788\r\n"

#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#if MAC_FILTERING_ENABLED
ShellComm_RegisterStatic(shellComm, "macfilter", SHELL_CMD_MAX_ARGS, 0, SHELL_Filter
#if SHELL_USE_HELP
    ,"MAC filtering commands",
    "MAC filtering commands\r\n"
    "   macfilter add <neighbor extended address> <neighbor short address> <neighbor link indicator>\r\n"
    "   macfilter remove <neighbor extended address>\r\n"
    "   macfilter enable\r\n"
    "   macfilter disable\r\n"
    "   macfilter table\r\n"    
    "   \r\n"
    "   Example: macfilter add 0x1122334455667788 0x0401 25 \r\n"
    "   Neighbor link indicator values: \r\n"
    "       Good Link: 20 - 255 \r\n"
    "       Medium Link: 11 - 20 \r\n"
    "       Bad Link: 3 - 10  \r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif
ShellComm_RegisterStatic(shellComm, "startnwk", SHELL_CMD_MAX_ARGS, 0, SHELL_StartStack
#if SHELL_USE_HELP
    ,"Starts Network",
    "Starts Network\r\n"
#if THREAD_COMBO_DEVICE
     "startnwk -stack<stackID> -device<deviceID> -role<roleID>\r\n"
     "- stackID: 0 - THREAD, 1- FLIP, 2 - STATIC\r\n"
     "- deviceID for THREAD: 0 - END DEVICE, 1 - ELIGIBLE ROUTER, 2 - ROUTER, 3 - LEADER\r\n"
     "- roleID: 0 - normal, 1 - leader\r\n"
#endif
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

#if STACK_THREAD
ShellComm_RegisterStatic(shellComm, "nwkdata", SHELL_CMD_MAX_ARGS, 0, SHELL_NwkData
#if SHELL_USE_HELP
    ,"Network data operations",
    "Add/removes Network Data\r\n"
     "nwkdata add dhcpserver <interfaceID> <Prefix> <prefixLength> <stable[0/1]>\r\n"
     "nwkdata add extroute <interfaceID> <Prefix> <prefixLength> <stable[0/1]>\r\n"
     "nwkdata propagate <interfaceID> <stable[0/1]> - used for Leader\r\n"
     "nwkdata register <interfaceID> - used for Router, EndDevice\r\n"

     "nwkdata removeall <interfaceID>\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

#endif

ShellComm_RegisterStatic(shellComm, "remove", SHELL_CMD_MAX_ARGS, 0, SHELL_Remove
#if SHELL_USE_HELP
    ,"Remove commands",
    "Remove commands\r\n"
    "   remove <interface id> router <router MAC short address>\r\n"
    "   Example: remove 0 router 0x0400 \r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );

#if 0
#if STACK_THREAD
ShellComm_RegisterStatic(shellComm, "swkeys", SHELL_CMD_MAX_ARGS, 0, SHELL_SwitchKeys
#if SHELL_USE_HELP
    ,"Switch Key",
    "Switch Key command\r\n"
    "   switch key <interface Id>\r\n"
    "   switch key <interface Id> -seq <sequence Id>\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif
#endif

#if SOCK_DEMO
ShellComm_RegisterStatic(shellComm, "socket", SHELL_CMD_MAX_ARGS, 0, SHELL_Socket
#if SHELL_USE_HELP
    ,"IP Stack BSD Sockets commands",
    "IP Stack BSD Sockets commands\r\n"
    "   socket open <protocol> <remote ip addr> <remote port> <ip version>\r\n"
    //"   socket open tcp <ip version>- open a socket and wait for tcp connections\r\n"
    "   socket close <socket id>\r\n"
    "   socket send <socket id> <command>\r\n"
    //"   socket receive <socket id>\r\n"
    //"   socket select <socket id>\r\n"
    //"   socket poll <socket id> <timeout>\r\n"
      "   socket post <socket id> <timeout>\r\n"
    //"   socket poll selected <timeout>\r\n"
    //"   socket connect <socket id>\r\n"
    //"   socket accept <socket id>"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
    ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif

#if DTLS_ENABLED
ShellComm_RegisterStatic(shellComm, "dtls", SHELL_CMD_MAX_ARGS, 0, SHELL_Dtls
#if SHELL_USE_HELP
    ,"DTLS Commands",
     "dtls open -i <retransmission interval> -c <max retransmissions> -port <port>\r\n"
     "dtls close -ctx <context #>\r\n"
     "dtls connect -ctx <context> -addr <server IP> -port <server port>\r\n"
     "dtls closepeer -peer <peer #>\r\n"
     "dtls send -peer <peer> -size <size> <data>\r\n"
#endif /* SHELL_USE_HELP */
#if SHELL_USE_AUTO_COMPLETE
        ,NULL
#endif /* SHELL_USE_AUTO_COMPLETE */
    );
#endif /* DTLS_ENABLED */

static const icmpProtMsgTypeHandler_t mShellProtMsgTypeHandlerTbl6[] =
{
    {PING_EchoReplyReceiveAsync, 129U, ICMP_CODE_DEFAULT}
};

static const icmpProtMsgTypeHandler_t mShellProtMsgTypeHandlerTbl4[] =
{
    {PING_EchoReplyReceiveAsync, 0U, ICMP_CODE_DEFAULT}
};

/* Register ICMP receive callback */
IMCP_RegisterHandler(SHELL_APP, IPPROTO_ICMPV6, NULL, NumberOfElements(mShellProtMsgTypeHandlerTbl6),
                             (icmpProtMsgTypeHandler_t*)&mShellProtMsgTypeHandlerTbl6);

IMCP_RegisterHandler(SHELL_APP, IPPROTO_ICMP, NULL, NumberOfElements(mShellProtMsgTypeHandlerTbl4),
                             (icmpProtMsgTypeHandler_t*)&mShellProtMsgTypeHandlerTbl4);

/* Ping variables */
uint64_t                pingTimeStamp = 0;
tmrTimerID_t            pingTimerID = gTmrInvalidTimerID_c;
static bool_t           mContinuousPing = FALSE;
static uint32_t         mPingTimeoutMs;
static uint16_t         defaultSeqNb = PING_SEQ_NB;
static uint16_t         pingSize = 0;
static uint16_t         pingCounter = 0;
static ipAddr_t         mDstIpAddr;
static uint32_t         mPingMacSec = 0;

static tmrTimerID_t     mDelayTimerID = gTmrInvalidTimerID_c;
static char             addrStr[INET6_ADDRSTRLEN];
static tmrTimerID_t     timerIDSelect = gTmrInvalidTimerID_c;

#if THROUGHPUT_DEMO
static throughputBenchTxParams_t txParams;
static throughputBenchRxParams_t rxParams;
#endif

static taskMsgQueue_t * pmMainThreadMsgQueue;   /*!< Pointer to main thread message queue */

static bool_t           shellCommandsEnabled = FALSE; /*!< Avoid initializing the module multiple times */
ipAddr_t                *pSrcIpAddr = NULL;; /* Used for keeping ping source address */

#if DTLS_ENABLED
static dtlsCallbacks_t  mDtlsCallbacks;
static int32_t maDtlsContexts[DTLS_MAX_CONTEXTS];
static int32_t maDtlsPeers[DTLS_MAX_PEERS];
#endif /* DTLS_ENABLED */

/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn     void SHELLComm_Init(void)
\brief  This function is used to initialize the SHELL commands module.

\param  [in]    pMsgQueue   pointer to the message queue

\return         void
***************************************************************************************************/
void SHELLComm_Init
(
    taskMsgQueue_t *pMsgQueue
)
{
    if(!shellCommandsEnabled)
    {
        // TODO: check the pointer to media interface.

        pmMainThreadMsgQueue = pMsgQueue;

        /* Initialize Shell module */
        shell_init("$ ");

        /* Register functions */
        shell_register_function_array((cmd_tbl_t*)gSHELL_COMMANDS_startAddr_d,
            gSHELL_COMMANDS_entries_d);
        shellCommandsEnabled = TRUE;

#if DTLS_ENABLED
        DTLS_Init(pmMainThreadMsgQueue);
#endif
    }
}


/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn     void SHELL_Resume(void)
\brief  This function is used to resume SHELL execution after a pause was issued.

\return void
***************************************************************************************************/
static void SHELL_Resume
(
    void
)
{
#if SOCK_DEMO
    App_ShellSocketResume();
#endif

    /* Stop Ping timer */
    if(pingTimerID != gTmrInvalidTimerID_c)
    {
        TMR_FreeTimer(pingTimerID);
        pingTimerID = gTmrInvalidTimerID_c;
    }

    mContinuousPing = FALSE;
    if(timerIDSelect != gTmrInvalidTimerID_c)
    {
        TMR_FreeTimer(timerIDSelect);
        timerIDSelect = gTmrInvalidTimerID_c;
    }
    if(mDelayTimerID != gTmrInvalidTimerID_c)
    {
        TMR_FreeTimer(mDelayTimerID);
        mDelayTimerID = gTmrInvalidTimerID_c;
    }


    if(pSrcIpAddr)
    {
        MEM_BufferFree(pSrcIpAddr);
        pSrcIpAddr = NULL;
    }

    shell_refresh();
}

/*!*************************************************************************************************
\private
\fn     int8_t  SHELL_Ping(uint8_t argc, char *argv[])
\brief  This function is used for "ping" shell command.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_Ping(uint8_t argc, char *argv[])
{
    ipPktInfo_t *pIpPktInfo;
    command_ret_t ret = CMD_RET_SUCCESS;

    /* Stop infinite ping */
    if(argc == 0)
    {
        if(mContinuousPing)
        {
            SHELL_Resume();
        }
        else
        {
            /* Update the prompt */
            shell_refresh();
        }
    }
    /* Check number of arguments according to the shellComm table */
    else if(argc == 3 || argc == 4)
    {
        uint8_t ap = AF_INET6;
        mPingMacSec = 0;

        /* Get timeout */
        mPingTimeoutMs = atoi((char const*)argv[2]);
        if (mPingTimeoutMs < SHELL_PING_MIN_TIMEOUT)
        {
            mPingTimeoutMs = SHELL_PING_MIN_TIMEOUT;
        }

        /* Verify IP address (v4 or v6) */
        uint8_t *pText;
        uint8_t *pStr;
        pText = MEM_BufferAlloc(INET6_ADDRSTRLEN);
        if(pText)
        {
            FLib_MemSet(pText, 0, INET6_ADDRSTRLEN);
            FLib_MemCpy(pText, argv[1], strlen((const char*)argv[1]));
            pStr = pText;
            while(*pStr)
            {
                if(*pStr == '.')
                {
                    ap = AF_INET;
                    break;
                }
                pStr++;
            }
            MEM_BufferFree(pText);
        }
        else
        {
            shell_write("No memory for verifying IP address type");
            panic(0,0,0,0);
        }
        pton(ap,argv[1],&mDstIpAddr);
        /* The fourth argument means continuous ping */
        if(argc == 4)
        {
            mContinuousPing = TRUE;
            pingCounter = 0xFFFFU;
        }
        /* Pause SHELL command parsing */
        ret = CMD_RET_ASYNC;

        /* Create Ping packet */
        pingSize = PING_PAYLOAD_DEFAULT_SIZE;
        pIpPktInfo = PingCreatePktInfo(&mDstIpAddr, pingSize);

        /* Send packet to ICMP for transmission */
        if(ap == AF_INET6)
        {
            ICMP_Send(pIpPktInfo, gIcmp6TypeEchoRequest_c, ICMP_CODE_DEFAULT);
        }
        else
        {
            ICMP_Send(pIpPktInfo, gIcmp4TypeEchoRequest_c, ICMP_CODE_DEFAULT);
        }

        /* Get timestamp */
        pingTimeStamp = TMR_GetTimestamp();

        if (gTmrInvalidTimerID_c == pingTimerID)
        {
            /* Allocate ping timer */
            pingTimerID = TMR_AllocateTimer();

            if (pingTimerID != gTmrInvalidTimerID_c)
            {
                /* Start timer */
                TMR_StartSingleShotTimer(pingTimerID, mPingTimeoutMs, PING_TimerCallback, NULL);

                shell_printf("Pinging %s with %u bytes of data\r\n", argv[1], PING_PAYLOAD_DEFAULT_SIZE);
            }
            else
            {
                shell_write("Timer cannot be allocated!");
                ret = CMD_RET_SUCCESS;
            }
        }
        else
        {
            shell_write("Timer already allocated!");
            ret = CMD_RET_SUCCESS;
        }

    } /* Correct number of arguments */
    else
    {
        shell_write("Wrong number of parameters");
    }

    return ret;
}

/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Ping6(uint8_t argc, char *argv[])
\brief  This function is used for "ping6" shell command.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_Ping6(uint8_t argc, char *argv[])
{
    ipPktInfo_t *pIpPktInfo;
    int32_t validIpAddr = 0;
    uint16_t count;
    char *pIpAddr;
    char *pValue;
    command_ret_t ret = CMD_RET_SUCCESS;
    uint32_t hopLimit = ICMP_DEFAULT_HOP_LIMIT;

    /* Stop infinite ping6 */
    if(argc == 0)
    {
        if(mContinuousPing)
        {
            SHELL_Resume();
        }
    }
    else
    {
        /* Reset the size of the ping */
        pingSize = 0;
        mPingTimeoutMs = DEFAULT_TIMEOUT;
        count = 0; /* infinite ping */

        /* Get option value for -s */
        pValue = shell_get_opt(argc, argv, "-s");
        if(pValue)
        {
            pingSize = (uint16_t)atoi(pValue);
        }

        /* Get option value for -i */
        pValue = shell_get_opt(argc, argv, "-i");
        if(pValue)
        {
            mPingTimeoutMs = (uint16_t)atoi(pValue);
            if (mPingTimeoutMs < SHELL_PING_MIN_TIMEOUT)
            {
                mPingTimeoutMs = SHELL_PING_MIN_TIMEOUT;
            }
        }

        /* Get option value for -c */
        pValue = shell_get_opt(argc, argv, "-c");
        if(pValue)
        {
            count = (uint16_t)atoi(pValue);
        }

        /* Get option value for -src */
        pValue = shell_get_opt(argc, argv, "-S");
        if(pValue)
        {
            pSrcIpAddr = MEM_BufferAlloc(sizeof(ipAddr_t));
            pton(AF_INET6, pValue, pSrcIpAddr);
        }

        /* Get option value for macsec */
        pValue = shell_get_opt(argc, argv, "-macsec");
        if(pValue)
        {
            mPingMacSec = atoi(pValue);
        }
        else
        {
            mPingMacSec = 5;
        }

        /* Get option value for hop limit */
        pValue = shell_get_opt(argc, argv, "-h");
        if(pValue)
        {
            hopLimit = atoi(pValue);
        }

        /* Check for the destination ip address */
        if(strchr(argv[argc - 1], ':'))
        {
            validIpAddr = pton(AF_INET6, argv[argc - 1], &mDstIpAddr);
            pIpAddr = argv[argc - 1];
        }

        /* Check if the parameters are for a valid command */
        if(validIpAddr)
        {
            /* Check ping payload size */
            pingSize = (pingSize)?pingSize:PING_PAYLOAD_DEFAULT_SIZE;

            /* Check number of counts */
            mContinuousPing = TRUE;
            if(count == 0)
            {
                pingCounter = 0xFFFFU;
            }
            else if(count == 1)
            {
                mContinuousPing = FALSE;
                pingCounter = 0xFFFFU;
            }
            else
            {
                pingCounter = count - 1;
            }

            /* Avoid showing the prompt after this function completes, instead, show the prompt
             * after the last echo reply message was received */
            ret = CMD_RET_ASYNC;

            /* Create Ping packet */
            pIpPktInfo = PingCreatePktInfo(&mDstIpAddr, pingSize);
            pIpPktInfo->pIpPktOptions->hopLimit = hopLimit;

            /* If we have a specified source address: set it */
            if(pSrcIpAddr)
            {
                IP_AddrCopy(pIpPktInfo->pIpSrcAddr, pSrcIpAddr);
            }

            /* Send packet to ICMP for transmission */
            ICMP_Send(pIpPktInfo, gIcmp6TypeEchoRequest_c, ICMP_CODE_DEFAULT);

            /* Get timestamp */
            pingTimeStamp = TMR_GetTimestamp();

            if (gTmrInvalidTimerID_c == pingTimerID)
            {
                /* Allocate ping timer */
                pingTimerID = TMR_AllocateTimer();

                if (pingTimerID != gTmrInvalidTimerID_c)
                {
                    /* Start timer */
                    TMR_StartSingleShotTimer(pingTimerID, mPingTimeoutMs, PING_TimerCallback, NULL);
                    shell_printf("Pinging %s with %d bytes of data:\n\r", pIpAddr, pingSize);
                }
                else
                {
                    shell_write("Timer cannot be allocated!");
                    ret = CMD_RET_SUCCESS;
                }
            }
            else
            {
                shell_write("Timer already allocated!");
                ret = CMD_RET_SUCCESS;
            }
        }
        else
        {
            shell_write("Invalid IP address\n\r");
        }
    }

    return ret;
}

#if SOCK_DEMO
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Socket(uint8_t argc, char *argv[])
\brief  This function is used for "open" shell command.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_Socket(uint8_t argc, char *argv[])
{

    command_ret_t ret = CMD_RET_ASYNC;
    appSockCmdParams_t* pAppSockCmdParams = MEM_BufferAlloc(sizeof(appSockCmdParams_t));

    FLib_MemSet(pAppSockCmdParams, 0, sizeof(appSockCmdParams_t));
    /* Stop any socket pending commands */
    if((argc == 0) || (pAppSockCmdParams == NULL))
    {
        SHELL_Resume();
        ret = CMD_RET_SUCCESS;
    }
    /* socket open */
    else if(!strcmp(argv[1], "open"))
    {
        /* Check number of arguments according to the shellComm table */
        if((argc == 6) || (argc == 4))
        {
            pAppSockCmdParams->appSocketsCmd = gSockOpenUdp;
            /* socket open udp */
            if(!strcmp(argv[2], "udp") && (argc == 6))
            {

                uint32_t result = -1;

                pAppSockCmdParams->appSocketsTrans = gSockUdp;
                /* Set local information */
                pAppSockCmdParams->ipVersion = (atoi((const char*)argv[5]) == 4)?AF_INET:AF_INET6;
                /* Set remote information for easier send */
                if(pAppSockCmdParams->ipVersion == AF_INET6)
                {
                   pAppSockCmdParams->sin6_port = atoi((char const*)argv[4]);
                   result = pton(AF_INET6, argv[3], &pAppSockCmdParams->sin6_addr);
                }
                else
                {
                   pAppSockCmdParams->sin_port = atoi((char const*)argv[4]);
                   result = pton(AF_INET, argv[3], &pAppSockCmdParams->sin_addr);
                }
                if(result == -1)
                {
                  shell_write("IP address has a wrong format");
                  SHELL_NEWLINE();
                  ret = CMD_RET_FAILURE;
                }
            }
#if TCP_ENABLED
            /* socket open tcp */
            else if(!strcmp(argv[2], "tcp"))
            {

                uint32_t result = -1;
                pAppSockCmdParams->appSocketsCmd = gSockOpenTcp;
                pAppSockCmdParams->appSocketsTrans = gSockTcp;
                /* Remote address is not specified */
                if(argc == 4)
                {
                    pAppSockCmdParams->ipVersion = (atoi((const char*)argv[3]) == 4)?AF_INET:AF_INET6;
                    FLib_MemSet(&pAppSockCmdParams->sockAddr, 0, sizeof(sockaddrStorage_t));
                }
                /* Remote address is specified so we have to initiate an active tcp connection:
                 * create socket */
                else if(argc == 6)
                {
                    pAppSockCmdParams->ipVersion = (atoi((const char*)argv[5]) == 4)?AF_INET:AF_INET6;

                    /* IPV6 address */
                    if(pAppSockCmdParams->ipVersion == AF_INET6)
                    {

                        pAppSockCmdParams->sin6_port = atoi((char const*)argv[4]);
                        result = pton(AF_INET6, argv[3], &pAppSockCmdParams->sockAddr.ss_addr);
                    }
                    /* IPV4 address */
                    else
                    {
                        pAppSockCmdParams->sin_port = atoi((char const*)argv[4]);
                        result = pton(AF_INET, argv[3], &pAppSockCmdParams->sockAddr.ss_addr);
                    }

                    if(result == -1)
                    {
                        shell_write("IP address has a wrong format");
                        SHELL_NEWLINE();
                        ret = CMD_RET_FAILURE;
                    }
                }
                else
                {
                    ret = CMD_RET_FAILURE;
                    shell_write("ERROR\n\rA new socket could not be created");
                }

            }
#endif /* TCP_ENABLED */
            else
            {
              ret = CMD_RET_FAILURE;
              shell_write("Wrong number of parameters");
            }
        } /* correct number of parameters */
        else
        {
           ret = CMD_RET_FAILURE;
           shell_write("Wrong number of parameters");
        }
    }
    /* socket connect */
#if TCP_ENABLED
    else if(!strcmp(argv[1], "connect") && (argc == 3))
    {
        pAppSockCmdParams->appSocketsCmd = gSockConnect;
        pAppSockCmdParams->sock32 = atoi((const char*)argv[2]);

    }
    /* socket accept */
    else if(!strcmp(argv[1], "accept") && (argc == 3))
    {
        pAppSockCmdParams->appSocketsCmd = gSockAccept;
        pAppSockCmdParams->sock32 = atoi((const char*)argv[2]);
    }
#endif /* TCP_ENABLED */
    /* socket close */
    else if(!strcmp(argv[1], "close"))
    {
        pAppSockCmdParams->appSocketsCmd = gSockClose;
        pAppSockCmdParams->sock32 = atoi((const char*)argv[2]);
    }
    /* socket send */
    else if(!strcmp(argv[1], "send"))
    {
        pAppSockCmdParams->appSocketsCmd = gSockSend;

        /* Check number of arguments according to the shellComm table */
        if(argc == 4)
        {
            /* Get socket id */
            pAppSockCmdParams->sock32 = atoi((const char *)argv[2]);

            pAppSockCmdParams->pData = MEM_BufferAlloc(sizeof(pollInfo_t));
            if(NULL != pAppSockCmdParams->pData)
            {
              pAppSockCmdParams->dataLen = strlen((char const*)argv[3]);
              FLib_MemCpy(pAppSockCmdParams->pData,argv[3],pAppSockCmdParams->dataLen);
            }
            else
            {
              ret = CMD_RET_FAILURE;
            }
        }
        else
        {
           ret = CMD_RET_FAILURE;
           shell_write("Wrong number of parameters");
        }
    }
    /* socket select */
    else if(!strcmp(argv[1], "select"))
    {
        pAppSockCmdParams->appSocketsCmd = gSockSelect;

        /* Check number of arguments according to the shellComm table */
        if(argc == 3)
        {
            /* Get socket ID from command */
            pAppSockCmdParams->sock32 = atoi((char const*)argv[2]);
        }
        else
        {
           ret = CMD_RET_FAILURE;
           shell_write("Wrong number of parameters");
        }
    }
    /* socket post */
    else if(!strcmp(argv[1], "post"))
    {
        pAppSockCmdParams->appSocketsCmd = gSockPost;

        /* Check number of arguments according to the shellComm table */
        if(argc == 4 || argc == 3 || argc == 5)
        {

            pAppSockCmdParams->bSelectedFlag = FALSE;
            /* socket poll <socket id> <timeout> */
            if(NWKU_IsNUmber(argv[2]))
            {
                pAppSockCmdParams->sock32 = atoi((char const*)argv[2]);
                if(argc == 4)
                {
                    /* Get time from command */
                    pAppSockCmdParams->timeMs = atoi((char const*)argv[3]);
                }
                else
                {
                    pAppSockCmdParams->timeMs = DEFAULT_TIMEOUT;
                }
            }
            /* socket poll selected */
            else if(!strcmp((const char*)argv[2], "selected"))
            {
                pAppSockCmdParams->bSelectedFlag = TRUE;

                /* Get time from command */
                pAppSockCmdParams->timeMs = atoi((char const*)argv[3]);
            }
        }
        else
        {
            shell_write("Wrong number of parameters");
            ret = CMD_RET_FAILURE;
        }
    }
    else
    {
        shell_write("Wrong command syntax");
        ret = CMD_RET_FAILURE;
    }
    
    if(ret == CMD_RET_ASYNC)
    {
        App_SocketSendAsync(pAppSockCmdParams);
    }
    else
    {
        MEM_BufferFree(pAppSockCmdParams);
    }
    return ret;
}

#endif /* SOCK_DEMO */

/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Reboot(uint8_t argc, char *argv[])
\brief  This function is used to reset the device.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_Reboot(uint8_t argc, char *argv[])
{
#if USE_VENET
    VIRTUAL_ENET_reset();
#endif /* USE_VENET */
#if !gUSBKW24D512Dongle
    ResetMCU();
#else
    shell_write("The reboot command is not available on USB-KW24.");
#endif

    return CMD_RET_SUCCESS;
}

#if AJ_THIN_CLIENT
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_AJ(uint8_t argc, char *argv[])
\brief  This function is used for All Joyn interface.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_AJ(uint8_t argc, char *argv[])
{
    AJ_UtilCmdLine(argc, argv);
    return CMD_RET_SUCCESS;
}
#endif /* AJ_THIN_CLIENT */

#if ECHO_PROTOCOL
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_EchoUdp(uint8_t argc, char *argv[])
\brief  This function is used to send packets with echo protocol.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_EchoUdp(uint8_t argc, char *argv[])
{
    return ECHO_ShellUdp(argc, argv);
}
#endif
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_GetNwkParams(uint8_t argc, char *argv[])
\brief  This function is used to get the stack parameters.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_GetNwkParams(uint8_t argc, char *argv[])
{   
    int32_t interfaceId = 0;
    command_ret_t ret = CMD_RET_SUCCESS;
    bool_t bGetNwkParams = FALSE;
    
    if (2 == argc)
    {     
        /* Get interface ID */
        interfaceId = atoi(argv[1]);
        if(interfaceId >= IP_IF_NB)
        {
           shell_write("Wrong interface Id!");
        }
        else
        {
            bGetNwkParams = TRUE;
        }
            
    }
    else if (1 == argc)
    {
        bGetNwkParams = TRUE;
    }
    else
    {
        shell_write("Wrong number of parameters!");
    }

    if (bGetNwkParams)
    {
        stackStartConfig_t *pStackConfig = NULL;
        stackConfig_t* pStack = NULL;
        stackConfig_t* pStackParam = NULL;
        /* Get the first stack available (assuming only one stack is available) */
        NWK_GetStack(NULL, &pStackConfig, &pStack);

        /* print interface id */
        shell_printf("Interface Id: %d\n\r",interfaceId);

        /* print channel */
        shell_printf("Channel: %d\n\r",pStack->pMacCfg->channel);
        /* print PANID */
        shell_printf("PanID: 0x%x\n\r",pStack->pMacCfg->panId);  

        pStackParam = *pStackConfig->pStackParam;

        /* short address */
        if(pStackParam->isStarted)
        {
          shell_printf("Short address: 0x%04x\n\r",pStack->shortAddress);
        }
        else
        {
          shell_write("Short address: Not initialized. Stack is not started\r\n");
        }
         /* extended address */           
        if(pStack->pMacCfg->randomExt && (!pStackParam->isStarted))
        {
          shell_write("Extended address: Not initialized. Stack is not started\r\n");
        }
        else
        {
          shell_printf("Extended address: 0x%016llx\n\r",pStack->pMacCfg->extendedAddr);
        }
#if MAC_FILTERING_ENABLED  
        if(MacFiltering_IsActive())
        {
            shell_write("Mac filtering: Enabled\r\n");
        }
        else
        {
            shell_write("Mac filtering: Disabled\r\n");
        }
#endif        
        /* Device Role*/
        if(pStackParam->isStarted)
        {
            threadInstance_t *pThreadInstance;
            ifHandle_t * pIfHandle;
            
            /* Get ifHandle*/
            pIfHandle = IP_IF_GetIfByNr(interfaceId);

            /* Get Thread Instance */
            pThreadInstance = Thread_GetInstanceByIf(pIfHandle);
            
            if(gThreadDevTypeRouter_c == pThreadInstance->currentDevType)
            {
                shell_write("Device Type: Router\r\n");
                if(gThreadRoleLeader_c == pThreadInstance->currentDevRole)
                {
                    shell_write("Device Role: Leader\r\n");
                }
                
            }
            else
            {
                shell_write("Device Type: End Device\r\n");
            }
              
        }
    }

    return ret;
}

/*!*************************************************************************************************
\private
\fn     int8_t SHELL_SetNwkParams(uint8_t argc, char *argv[])
\brief  This function is used to set the stack parameters.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_SetNwkParams(uint8_t argc, char *argv[])
{
    char *pValue;
    stackStartConfig_t *pStackConfig;
    command_ret_t ret = CMD_RET_SUCCESS;
    stackConfig_t* pStack = NULL;
    ipAddr_t* pMLprefix = MEM_BufferAlloc(sizeof(ipAddr_t));

    /* Get the first stack available (assuming only one stack is available) */
    NWK_GetStack(NULL, &pStackConfig, &pStack);
    if (NULL == pStack)
    {
        shell_write("Please insert a valid device ID and a stack ID");
    }
    else
    {
        /* Get option value for -channel */
        pValue = shell_get_opt(argc, argv, "-ch");
        if(pValue)
        {
            uint8_t channel = (uint8_t)atoi(pValue);

            NWK_Params_Set(gNwkParamSetChannel_c,&channel,pStack);
        }

        /* Get option value for -panID */
        pValue = shell_get_opt(argc, argv, "-panID");
        if(pValue)
        {
            uint16_t temp;
            uint8_t panID[2];

            sscanf(pValue, "%hx", &temp);
            htonas(panID, temp);

            NWK_Params_Set(gNwkParamSetPanId_c,(uint8_t*)panID,pStack);
        }

        /* Get option value for -short */
        pValue = shell_get_opt(argc, argv, "-short");
        if(pValue)
        {
            uint16_t temp;
            uint8_t shortAddr[2];

            sscanf(pValue, "%hx", &temp);
            htonas(shortAddr, temp);

            NWK_Params_Set(gNwkParamSetShortAddr_c,(uint8_t*)shortAddr,pStack);
        }

        /* Get option value for -extended */
        pValue = shell_get_opt(argc, argv, "-extended");

        if(pValue)
        {
            uint64_t extendedAddress;
            uint8_t aExtendedAddr[8];
            sscanf((char const*)(pValue + 2), "%llx", &extendedAddress);
            htonall(aExtendedAddr, extendedAddress);

            NWK_Params_Set(gNwkParamSetExtAddr_c,aExtendedAddr,pStack);
        }

        /* Get option value for -mlprefix */
        pValue = shell_get_opt(argc, argv, "-mlprefix");
        if (pValue)
        {
            pton(AF_INET6,pValue,pMLprefix);
            NWK_Params_Set(gNwkParamsSetMLPrefix_c,(uint8_t*)pMLprefix,pStack);
        }

        /* Get option value for -mlprefixlen */
        pValue = shell_get_opt(argc, argv, "-lenmlprefix");
        if (pValue)
        {
            int32_t prefixLen = atoi(pValue);
            NWK_Params_Set(gNwkParamsSetMLPrefixLen_c,(uint8_t*)&prefixLen,pStack);
        }

        /* Get option value for -rxonidle */
        pValue = shell_get_opt(argc, argv, "-rxonidle");
        if (pValue)
        {
            int32_t rxonidle = atoi(pValue);
            NWK_Params_Set(gNwkParamsSetRxOnIdle_c,(uint8_t*)&rxonidle,pStack);
        }

        shell_write("Parameters set");
    }

    return ret;
}

/*!*************************************************************************************************
\private
\fn     int8_t SHELL_SetNwkParams(uint8_t argc, char *argv[])
\brief  This function is used to set the stack parameters.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_SetStackParams(uint8_t argc, char *argv[])
{
    char *pValue;
    command_ret_t ret = CMD_RET_SUCCESS;
    stackParams_t stackID;

    pValue = shell_get_opt(argc, argv, "-device");
    if(pValue)
    {
        stackID.deviceID = (uint8_t)atoi(pValue);
    }

    pValue = shell_get_opt(argc, argv, "-stack");
    if(pValue)
    {
        stackID.stackID = (uint8_t)atoi(pValue);
    }

    pValue = shell_get_opt(argc, argv, "-role");
    if(pValue)
    {
        stackID.deviceRole = (uint8_t)atoi(pValue);
    }

    /*  */
    if (NWK_SetStackParams(&stackID))
    {
        shell_write("Parameters set!");
    }
    return ret;
}

/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Ifconfig(uint8_t argc, char *argv[])
\brief  This function is used for ifconfig.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_Ifconfig(uint8_t argc, char *argv[])
{
#if IP_IP6_ENABLE
    int32_t interfaceId;
    ipAddr_t inIpAddr;
#if IP_IP4_ENABLE
    ip4IfAddrData_t *pIp4AddrData;
#endif
    ifHandle_t* pIfHandle;
    uint32_t bindStatus = gIpOk_c;
#if ND_ENABLED
    uint32_t iIf;
    ndNeighborEntry_t *pNdNeighbor = NULL;
    char addrStr[INET6_ADDRSTRLEN];
#endif

    uint8_t queryEui[8];
    uint64_t extendedAddress;

    /* ifconfig all */
    if((argc == 2) && !strcmp(argv[1], "all"))
    {
        SHELL_WRITE_STACK_CONFIG;

        SHELL_PrintIpAddr(gAllIpAddr_c);
    }
#if ND_ENABLED
    /* ifconfig ncache */
    else if((argc == 2) && !strcmp(argv[1], "ncache"))
    {
        interfaceId = atoi(argv[1]);
        shell_write("ND Neighbor Cache:\n\r");
        /* Get interfaces by index */
        iIf = 0;
        pIfHandle = IP_IF_GetIfByNr(iIf);
        while(*pIfHandle)
        {
            shell_printf("\n\r\tInterface %d: %s\n\r", iIf,(*pIfHandle)->ifNamePtr);

            for (uint32_t iCount = 0U; iCount < ND_NEIGHBOR_CACHE_SIZE; iCount++)
            {
                pNdNeighbor = ND_NeighborCacheGetByIdx(pIfHandle, iCount);
                if (pNdNeighbor)
                {
                    ntop(AF_INET6, &pNdNeighbor->ipAddr, addrStr, INET6_ADDRSTRLEN);
                    shell_printf("\t%s\n\r", addrStr);
                }
            }
            /* Go to the next interface */
            iIf++;
            pIfHandle = IP_IF_GetIfByNr(iIf);
        }
    }
#endif /* ND_ENABLED */
#if DHCP6_CLIENT_ENABLED
    /* ifconfig <interface ID> leasequery <EUI64> */
    else if((argc == 4) && !strcmp(argv[2], "dhcpleasequery"))
    {

        /* Get extended address */
        sscanf((char const*)(argv[3]+2), "%llx", &extendedAddress);

        htonall(queryEui, extendedAddress);

        interfaceId = atoi(argv[1]);
        SHELL_WRITE_STACK_CONFIG;
        pIfHandle = IP_IF_GetIfByNr(interfaceId);
        DHCP6_Client_SendLeaseQuery(pIfHandle,queryEui, NULL,(ipAddr_t*)&in6addr_any);
    }
    /* ifconfig <interface ID> dhcprelease */
    else if((argc == 3) && !strcmp(argv[2], "dhcprelease"))
    {
        interfaceId = atoi(argv[1]);
        SHELL_WRITE_STACK_CONFIG;
        pIfHandle = IP_IF_GetIfByNr(interfaceId);
        DHCP6_Client_SendRelease(pIfHandle);
    }
#endif
#if 0
    /* ifconfig <interface ID> dhcp6start */
    else if((argc == 5) && !strcmp(argv[2], "dhcp6start"))
    {
        dhcp6ClientStartParams_t dhcp6ClientStartParams;
        interfaceId = atoi(argv[1]);
        uint32_t clientMode = atoi(argv[3]);
        ipAddr_t* pPrefix = (ipAddr_t*)MEM_BufferAlloc(sizeof(ipAddr_t));
        pton(AF_INET6,argv[4],pPrefix);
        SHELL_WRITE_STACK_CONFIG;
        pIfHandle = IP_IF_GetIfByNr(interfaceId);

        dhcp6ClientStartParams.pIfPtr = pIfHandle;
        dhcp6ClientStartParams.clientStartMode = clientMode;
        dhcp6ClientStartParams.deviceType = DHCP6_HW_TYPE_EUI64;
        dhcp6ClientStartParams.relayAddr = NULL;
        dhcp6ClientStartParams.pPrefix = pPrefix;
        dhcp6ClientStartParams.macSec = 5;
        DHCP6_Client_Init(pmMainThreadMsgQueue);
        DHCP6_Client_Start(&dhcp6ClientStartParams);
        DHCP6_Client_SolicitAddress(&dhcp6ClientStartParams.clientStartMode);
        MEM_BufferFree(pPrefix);
    }
    /* ifconfig <interface ID> dhcp6stop */
    else if((argc == 4) && !strcmp(argv[2], "dhcp6stop"))
    {
        interfaceId = atoi(argv[1]);
        uint32_t clientMode = atoi(argv[3]);
        SHELL_WRITE_STACK_CONFIG;
        pIfHandle = IP_IF_GetIfByNr(interfaceId);
        DHCP6_Client_Stop(pIfHandle, clientMode);
    }
#endif
    /* ifconfig <interface ID> ip <IP address> */
    else if((argc == 4) && !strcmp(argv[2], "ip"))
    {
        interfaceId = atoi(argv[1]);

        /* Check IP address */
        if(pton(AF_INET6, argv[3], &inIpAddr) != 1)
        {
            shell_write("Malformed IP address");
        }
        else
        {
            pIfHandle = IP_IF_GetIfByNr(interfaceId);
            bindStatus = IP_IF_BindAddr6(pIfHandle, &inIpAddr,ip6AddrTypeManual_c,
                IP6_ADDRESS_LIFETIME_INFINITE, 64U);
            switch (bindStatus)
            {
                case gIpOk_c:
                    shell_write("interface configured");
                    break;

                case gIpNoAddressSpaceError_c:
                    shell_write("address limit reached");
                    break;

                default:
                    shell_write("error");
                    break;
            }
        }
    }
    else
    {
        shell_write("Unknown command\n\r");
    }
#endif

    return CMD_RET_SUCCESS;
}
/*!*************************************************************************************************
\fn     void SHELL_PrintIpAddr(threadAddrTypes_t addrType)
\brief  This function is used for printing in shell terminal the IP addresses of a certain type.

\param  [in]    threadAddrTypes_t    the type of the IP addresses to be printed

\return         void
***************************************************************************************************/
void SHELL_PrintIpAddr
(
    threadAddrTypes_t addrType
)
{
    uint32_t iIf, iAddr;
    char addrStr[INET6_ADDRSTRLEN];
    ip6IfAddrData_t *pIp6AddrData;
#if IP_IP4_ENABLE
    ip4IfAddrData_t *pIp4AddrData;
#endif
    ifHandle_t* pIfHandle;
    
    /* Get interfaces by index */
    iIf = 0;
    pIfHandle = IP_IF_GetIfByNr(iIf);
    
    while(*pIfHandle)
    {
        shell_printf("Interface %d: %s", iIf,(*pIfHandle)->ifNamePtr);
#if IP_IP4_ENABLE
        /* Get IPv4 addresse for an interface */
        pIp4AddrData = IP_IF_GetAddrByIf4(pIfHandle);
        if(pIp4AddrData)
        {
            NWKU_ConvertIp4Addr(pIp4AddrData->ip4Addr,&inIpAddr);
            ntop(AF_INET, &inIpAddr, addrStr, INET_ADDRSTRLEN);
            shell_printf("\tIPv4 Address: %s\n\r", addrStr);
        }
#endif
        /* Get IPv6 addresses for an interface */
        iAddr = 0;
        pIp6AddrData = IP_IF_GetAddrByIf6(pIfHandle, iAddr);
        while(pIp6AddrData)
        {                          
            ntop(AF_INET6, &pIp6AddrData->ip6Addr, addrStr, INET6_ADDRSTRLEN);
            if (((addrType == gMeshLocalAddr_c)||(addrType == gAllIpAddr_c))
                && (IP6_IsUniqueLocalAddr(&pIp6AddrData->ip6Addr))) 
            {  
                uint8_t addrTypeIndex = 16;
                ifHandle_t ifHandle = *pIfHandle;
                if (FLib_MemCmp(&pIp6AddrData->ip6Addr.addr8[9], &ifHandle->ifDevAddrTbl[0].eui[1],ifHandle->ifDevAddrTbl[0].addrSize-1))
                {                    
                    addrTypeIndex = 64;
                }
                shell_printf("\n\r\tMesh local address (ML%d): %s", addrTypeIndex, addrStr);
            }
            if (((addrType == gGlobalAddr_c) ||(addrType == gAllIpAddr_c) )
                && (IP6_IsGlobalAddr(&pIp6AddrData->ip6Addr)))
            {
                shell_printf("\n\r\tGlobal address: %s", addrStr);   
            }
            if (((addrType == gLinkLocalAddr_c) ||(addrType == gAllIpAddr_c) )
                && (IP6_IsLinkLocalAddr(&pIp6AddrData->ip6Addr)))
            {
                shell_printf("\n\r\tLink local address: %s", addrStr);   
            }

            /* Go to the next IP address */
            iAddr++;
            pIp6AddrData = IP_IF_GetAddrByIf6(pIfHandle, iAddr);
        }

        /* Go to the next interface */
        iIf++;
        pIfHandle = IP_IF_GetIfByNr(iIf);
    }    
}
#if THROUGHPUT_DEMO
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Iptest(uint8_t argc, char *argv[])
\brief  This function is used for throughput Rx benchmark.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
 ***************************************************************************************************/
int8_t SHELL_Iptest(uint8_t argc, char *argv[])
{
    command_ret_t ret = CMD_RET_SUCCESS;

#if THROUGHPUT_DEMO
    if(argc == 0)
    {
        osSignalSet(pmMainThreadMsgQueue->taskId, 0x40);
    }
    /* iptest rx */
    else if((argc == 4) && !strcmp(argv[1], "rx"))
    {
        rxParams.family = (atoi(argv[3]) == 4) ? AF_INET : AF_INET6;

        /* TCP */
        if(FLib_MemCmp("tcp", argv[2], 3))
        {
            ret = CMD_RET_ASYNC;
            rxParams.protocol= IPPROTO_TCP;
            NWKU_SendMsg(APP_ThroughputBenchRx, &rxParams, pmMainThreadMsgQueue);
        }
        /* UDP */
        else
        {
            if( FLib_MemCmp("udp", argv[2], 3))
            {
                ret = CMD_RET_ASYNC;
                rxParams.protocol= IPPROTO_UDP;
                NWKU_SendMsg(APP_ThroughputBenchRx, &rxParams, pmMainThreadMsgQueue);
            }
            else
            {
                shell_printf("Parameter error: %s", argv[2]);
            }
        }
    }
    /* iptest tx */
    else if((argc == 7 || argc == 8) && !strcmp(argv[1], "tx"))
    {
        uint8_t error = 0;
        FLib_MemSet(&txParams.foreignAddr, 0, sizeof(sockaddrStorage_t));
        txParams.foreignAddr.ss_family = (atoi(argv[3]) == 4) ? AF_INET : AF_INET6;

        pton(txParams.foreignAddr.ss_family, argv[4], &txParams.foreignAddr.ss_addr);
        if(txParams.foreignAddr.ss_family == AF_INET)
        {
            sockaddrIn_t *pSockAddr = (sockaddrIn_t *)(&txParams.foreignAddr);
            pSockAddr->sin_port = APP_BENCH_PORT;
        } else
        {
            sockaddrIn6_t *pSockAddr = (sockaddrIn6_t *)(&txParams.foreignAddr);
            pSockAddr->sin6_port = APP_BENCH_PORT;
        }

        txParams.packetSize = APP_BENCH_TX_PACKET_SIZE_DEFAULT;
        txParams.packetNo = APP_BENCH_TX_PACKET_NUMBER_DEFAULT;
        txParams.iterationsNo = APP_BENCH_TX_ITERATION_NUMBER_DEFAULT;

        if(argc > 5)
        {
            /* Packet size.*/
            txParams.packetSize = (uint16_t)atoi(argv[5]);
            if ((txParams.packetSize == 0) || (txParams.packetSize > APP_BENCH_PACKET_SIZE_MAX))
            {
                shell_write("Packet size error."); /* Print error mesage. */
                SHELL_NEWLINE();
                error = 1;
            } else
            {
                /* Number of packets.*/
                if(argc > 6)
                {
                    txParams.packetNo = (uint32_t)atoi(argv[6]);
                    if (txParams.packetNo == 0)
                    {
                        shell_write("Packet number error."); /* Print error mesage. */
                        SHELL_NEWLINE();
                        error = 1;
                    } else
                    {
                        /* Number of iterations.*/
                        if(argc > 7)
                        {
                            txParams.iterationsNo = (uint16_t)atoi(argv[7]);
                            if ((txParams.iterationsNo < 1) ||
                                (txParams.iterationsNo > APP_BENCH_TX_ITERATION_NUMBER_MAX) )
                            {
                                shell_write("Packet number error."); /* Print error mesage. */
                                SHELL_NEWLINE();
                                error = 1;
                            }
                        }
                    }

                }
            }
        }

        if (!error)
        {
            /* TCP */
            if(!strcmp(argv[2], "tcp"))
            {
                shell_write("Work in Progress");
                SHELL_NEWLINE();
            }
            /* UDP */
            else if(!strcmp(argv[2], "udp"))
            {
                ret = CMD_RET_ASYNC;
                NWKU_SendMsg(APP_ThroughputBenchUdpTx,&txParams,pmMainThreadMsgQueue);
            }
            else
            {
                shell_printf("Parameter error: %s", argv[2]);
            }
        }
    }
    else
    {
        shell_write("Malformed command");
    }
#else
    shell_write("Command was not enabled");
#endif /* THROUGHPUT_DEMO */

    return ret;
}
#endif

#if DUAL_PAN_ZPRO_NWKIP
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Tcpi(uint8_t argc, char *argv[])
\brief  This function is used for controlling TCPi bulb.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/

int8_t SHELL_Tcpi(uint8_t argc, char *argv[])
{
    command_ret_t ret = CMD_RET_SUCCESS;

    if(argc == 2)
    {
        if(!strcmp(argv[1], "set"))
        {
            shell_write("Not yet implemented\r\n");
        }
        else if(!strcmp(argv[1], "toggle"))
        {
            shell_write("Not yet implemented\r\n");
        }
        else if(!strcmp(argv[1], "up"))
        {
            shell_write("Not yet implemented\r\n");
        }
        else if(!strcmp(argv[1], "down"))
        {
            shell_write("Not yet implemented\r\n");
        }
    }

    return ret;
}
#endif

#if MAC_FILTERING_ENABLED
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Filter(uint8_t argc, char *argv[])
\brief  This function is used for "filter" shell command.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_Filter(uint8_t argc, char *argv[])
{
    command_ret_t ret = CMD_RET_SUCCESS;

    #if (FALSE == MAC_FILTERING_ENABLED)
    shell_write("MAC_FILTERING_ENABLED is set FALSE!\r\n");
    return ret;
    #else
    /* Stop any socket pending commands */
    if(argc > 0)
    {
        if(!strcmp(argv[1], "add"))
        {
            if(argc == 5)
            {
                uint64_t extendedAddress;
                uint16_t shortAddress;
                uint8_t  linkIndicator;

                sscanf((char const*)(argv[2] + 2), "%llx", &extendedAddress);
                sscanf((char const*)(argv[3] + 2), "%hx", &shortAddress);
                linkIndicator = (uint8_t)atoi((char const*)argv[4]);

                MacFiltering_AddNeighbor(extendedAddress, shortAddress, linkIndicator);

                shell_write("Entry has been added!\r\n");
            }
            else
            {
                ret = CMD_RET_ASYNC;
            }
        }
        else if(!strcmp(argv[1], "remove"))
        {
            if(argc == 3)
            {
                uint64_t extendedAddress;

                sscanf((argv[2] + 2), "%llx", &extendedAddress);

                MacFiltering_RemoveNeighbor(extendedAddress);
                shell_write("Entry has been removed!\r\n");
            }
            else
            {
                ret = CMD_RET_ASYNC;
            }
        }
        else if(!strcmp(argv[1], "enable"))
        {   
            MacFiltering_Active(TRUE);
            shell_write("MAC Filtering Enabled!\r\n");
        }
        else if(!strcmp(argv[1], "disable"))
        {
            MacFiltering_Active(FALSE);
            shell_write("MAC Filtering Disabled!\r\n");
        }
        else if(!strcmp(argv[1], "table"))
        {            
            shell_printf("MAC Filtering Status: ");
            if (MacFiltering_IsActive())
            {
                shell_printf("Enabled.\r\n\r\n");
            }
            else
            {
                shell_printf("Disabled.\r\n\r\n");
            }
            SHELL_Filtering_Print();            
        }
        else
        {
            ret = CMD_RET_ASYNC;
        }
    }
    else
    {
        SHELL_Resume();
    }

    return ret;
    #endif
}

static void SHELL_Filtering_Print
(
    void
)
{
    uint32_t iCount = MAC_FILTERING_TABLE_SIZE;
    macFilteringNeighborData_t **ppMacFilterEntry = NULL;

    shell_printf("Idx   Extended Address     Short Address   Link Quality\n\r");
    do
    {
        iCount--;
        ppMacFilterEntry = MacFiltering_GetEntryByIdx(iCount);
        if (NULL != ppMacFilterEntry)
        {
            shell_printf("%d", iCount);
            shell_printf("     ");
            shell_printf("0x%016llX", (*ppMacFilterEntry)->extendedAddress);
            shell_printf("   ");            
            shell_printf("0x%04X", (*ppMacFilterEntry)->shortAddress);
            shell_printf("          ");            
            shell_printf("%d", (*ppMacFilterEntry)->linkIndicator);
        }
    }
    while (iCount);

    shell_printf("\r\nEnd of MAC Filtering Table.\r\n");
}

#endif
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Remove(uint8_t argc, char *argv[])
\brief  This function is used for removing actions.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_Remove(uint8_t argc, char *argv[])
{
    if(argc == 4)
    {
        if(!strcmp(argv[2], "router"))
        {
#if STACK_THREAD
            ifHandle_t *pIfHandle;
            uint16_t shortAddress;
            pIfHandle = IP_IF_GetIfByNr(atoi(argv[1]));
            sscanf((char const*)(argv[3] + 2), "%hx", &shortAddress);
            THR_LeaderRemoveRouter(pIfHandle, THR_SHORT_ADDR_TO_R_ID(shortAddress));
#endif
        }
    }
    else
    {
        shell_printf("Wrong number of parameters\n\r");
    }
    return CMD_RET_SUCCESS;
}

/*!*************************************************************************************************
\private
\fn     int8_t SHELL_StartStack(uint8_t argc, char *argv[])
\brief  This function is used for starting a network.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_StartStack(uint8_t argc, char *argv[])
{
    char *pValue;
    stackParams_t stackID;

    if (1 == argc)
    {
        NWKU_SendMsg(APP_StartDevice, NULL, pmMainThreadMsgQueue);
    }
    else if (4 == argc)
    {
        pValue = shell_get_opt(argc, argv, "-device");
        if(pValue)
        {
            stackID.deviceID = (uint8_t)atoi(pValue);

        }

        pValue = shell_get_opt(argc, argv, "-stack");
        if(pValue)
        {
            stackID.stackID = (uint8_t)atoi(pValue);

        }

        pValue = shell_get_opt(argc, argv, "-role");
        if(pValue)
        {
            stackID.deviceRole = (uint8_t)atoi(pValue);
        }

        nwkStartParams_t* pNwkStartParams = MEM_BufferAlloc(sizeof(nwkStartParams_t));
        NWK_GetStack(&stackID, &pNwkStartParams->pStackConfig, &pNwkStartParams->pStack);

        if (NULL == pNwkStartParams->pStack)
        {
            shell_write("Please insert a valid device ID and a stack ID");
            MEM_BufferFree(pNwkStartParams);
        }
        else
        {
            NWKU_SendMsg(APP_StartDevice, pNwkStartParams, pmMainThreadMsgQueue);
        }
    }

    return CMD_RET_SUCCESS;
}

#if STACK_THREAD
/*!*************************************************************************************************
\private
\fn     int8_t  SHELL_NwkData(uint8_t argc, char *argv[])
\brief  This function is used for setting the network data.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_NwkData(uint8_t argc, char *argv[])
{
    ifHandle_t * pIfHandle;
    int32_t interfaceId;
    bool_t isStable;

    if (argc == 7)
    {
        threadInstance_t *pThreadInstance;
        int32_t prefixLength;
        ipAddr_t* pPrefix = (ipAddr_t*)MEM_BufferAlloc(sizeof(ipAddr_t));
        uint64_t extendedAddress;
        uint16_t shortAddr;

        ///TODO: Perform sanity checks.

        /* Get interface ID */
        interfaceId = atoi(argv[3]);
        pIfHandle = IP_IF_GetIfByNr(interfaceId);

        /* Get Thread Instance */
        pThreadInstance = Thread_GetInstanceByIf(pIfHandle);
        slwpStruct_t *pSlwpStruct = *((slwpStruct_t**)(*pIfHandle)->ifDriverHandle);

        /* Get DHCP prefix */
        pton(AF_INET6,argv[4],pPrefix);

        /* Get prefix length */
        prefixLength = atoi(argv[5]);

        isStable = atoi(argv[6]);

        /* Get extended address */
        extendedAddress = pThreadInstance->pStackCfg->pMacCfg->extendedAddr;

        /* Get short address */
        shortAddr = pSlwpStruct->pMacAbsReq->GetShortAddress(pSlwpStruct->macInstanceId);

        if (!strcmp(argv[1], "add"))
        {
            if (!strcmp(argv[2], "dhcpserver"))
            {
                dhcpServerSet_t * pDhcpInfo = MEM_BufferAlloc(sizeof(dhcpServerSet_t));
                htonall(pDhcpInfo->dhcpEui,extendedAddress);
                pDhcpInfo->dhcpShortAddr = shortAddr;
                pDhcpInfo->dhcpStableRoute = isStable;
                Thread_DataSetDhcpServer(pIfHandle,(uint8_t*)pPrefix,(uint32_t)prefixLength,pDhcpInfo);
                shell_write("DHCP Server Added!");
            }
            if (!strcmp(argv[2], "extroute"))
            {
                externalRouteSet_t *pExternalRouteSet = MEM_BufferAlloc(sizeof(externalRouteSet_t));
                uint8_t *pRoutePrefix = (uint8_t *)pPrefix;
                htonall(pExternalRouteSet->brEui,extendedAddress);
                pExternalRouteSet->brShortAddr = shortAddr;
                pExternalRouteSet->brStableRoute = isStable;
                if (FLib_MemCmp(pPrefix, (uint8_t *)&in6addr_any.addr8[0], 16))
                {
                    pRoutePrefix = NULL;
                }
                Thread_DataSetExtRoute(pIfHandle,(uint8_t*)pRoutePrefix,(uint32_t)prefixLength,pExternalRouteSet);
                shell_write("External Route Added!");
            }
        }
        else if (!strcmp(argv[1], "remove"))
        {

        }

        MEM_BufferFree(pPrefix);
    }
    else if (argc == 4)
    {
        /* Get interface ID */
        interfaceId = atoi(argv[2]);
        pIfHandle = IP_IF_GetIfByNr(interfaceId);
        isStable = atoi(argv[3]);

        if (!strcmp(argv[1], "propagate"))
        {
            Thread_NwkDataIncrementVersion(pIfHandle, isStable);
            Thread_NwkDataPropagate(pIfHandle, NULL, FALSE, FALSE);
            Thread_NwkDataSleepyNodesPropagate(pIfHandle);
            shell_write("Network Data version incremented!\r\n");
            shell_write("Network Data propagated!");
        }
    }
    else if (argc == 3)
    {
        /* Get interface ID */
        interfaceId = atoi(argv[2]);
        pIfHandle = IP_IF_GetIfByNr(interfaceId);

        if (!strcmp(argv[1], "register"))
        {
            Thread_ServerDataRegister(pIfHandle);
            shell_write("Registered Server Data!");
        }
        if (!strcmp(argv[1], "removeall"))
        {
            Thread_ServerDataRemoveAll(pIfHandle);
            shell_write("Removed Server Data!");
        }
    }
    else
    {
        shell_write("Invalid number of parameters");
    }

    return CMD_RET_SUCCESS;
}
#endif

#if 0
#if STACK_THREAD
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_SwitchKeys(uint8_t argc, char *argv[])
\brief  This function is used for switching the MAC and MLE keys.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         CMD_RET_SUCCESS
***************************************************************************************************/
static int8_t SHELL_SwitchKeys(uint8_t argc, char *argv[])
{
    char *pValue;

    if ((argc >= 1) && (argc < 4))
    {
        uint32_t interfaceId = atoi(argv[2]);
        ifHandle_t *pIfHandle = IP_IF_GetIfByNr(interfaceId);
        instanceId_t slwpInstanceId = (instanceId_t)(*pIfHandle)->ifDriverHandle;
        threadInstance_t *pThreadInstance = Thread_GetInstanceByIf(pIfHandle);
        securityMaterial_t *pSecurityMaterial = pThreadInstance->pStackCfg->pSecurityMaterial;

        if(argc == 2)
        {
            /* Switch to the next thrKeySequenceCounter key set */
            pSecurityMaterial->keySequenceCounter++;
            Thread_ConfigureSecurity(slwpInstanceId, pSecurityMaterial);
        }
        else if(argc == 4)
        {
            pValue = shell_get_opt(argc, argv, "-seq");

            if(pValue)
            {
                pSecurityMaterial->keySequenceCounter = atoi(pValue);
                Thread_ConfigureSecurity(slwpInstanceId, pSecurityMaterial);
            }
        }
    }
    else
    {
        shell_write("Incorect number of parameters!");
    }

    return CMD_RET_SUCCESS;
}
#endif
#endif

#if DTLS_ENABLED
/*!*************************************************************************************************
\private
\fn     int8_t SHELL_Dtls(uint8_t argc, char *argv[])
\brief  This function is used for DTLS.

\param  [in]    argc    number of arguments the command was called with
\param  [in]    argv    pointer to a list of pointers to the arguments

\return         void
***************************************************************************************************/
static int8_t SHELL_Dtls(uint8_t argc, char *argv[])
{
    command_ret_t ret = CMD_RET_SUCCESS;
    char *pValue;
    dtlsContext_t *pCtx = NULL;
    dtlsPeerPtr_t pPeer = NULL;

    /* dtls open -i<retransmission interval> -c<max retransmissions> -port<port> */
    if(!strcmp(argv[1], "open"))
    {
        dtlsInitParams_t initParms;
        uint16_t port;
        uint32_t timeUnits = 0;
        uint32_t cRetr;

        /* Time units */
        pValue = shell_get_opt(argc, argv, "-i");
        if(pValue)
        {
            timeUnits = atoi(pValue);
        }
        else
        {
            timeUnits = 50;
        }

        /* Number of retransmissions */
        pValue = shell_get_opt(argc, argv, "-c");
        if(pValue)
        {
            cRetr = atoi(pValue);
        }
        else
        {
            cRetr = 3;
        }

        /* Initialization */
        pValue = shell_get_opt(argc, argv, "-port");
        if(pValue)
        {
            sockaddrIn6_t addrStorage;
            uint8_t iCtx;

            port = atoi(pValue);

            /* Init a context */
            initParms.maxRetransmitCnt = cRetr;
            initParms.retransmitTimeUnits = timeUnits;
            mDtlsCallbacks.event = DTLS_Event;
            mDtlsCallbacks.rcvd = DTLS_Received;
            mDtlsCallbacks.getJpakePasswd = DTLS_GetJpakePasswd;

            addrStorage.sin6_family = AF_INET6;
            addrStorage.sin6_port = port;
            IP_AddrCopy(&addrStorage.sin6_addr, &in6addr_any);
            pCtx = DTLS_NewContext(&initParms, &mDtlsCallbacks, &addrStorage);
            if(pCtx)
            {
                iCtx = NWKU_AddTblEntry((uint32_t)pCtx, (uint32_t*)maDtlsContexts,
                    NumberOfElements(maDtlsContexts));

                if(iCtx != (uint8_t)(-1))
                {
                    shell_printf("Context created with id #%d\n\r", iCtx);
                }
            }
        }
    }
    /* dtls close -ctx<context #> */
    else if(!strcmp(argv[1], "close"))
    {
        pValue = shell_get_opt(argc, argv, "-ctx");
        if(pValue)
        {
            pCtx = (dtlsContext_t*)NWKU_GetTblEntry(atoi(pValue), (uint32_t*)maDtlsContexts,
                NumberOfElements(maDtlsContexts));

            if(pCtx)
            {
                shell_printf("Context was cleared\n\r");
                maDtlsContexts[atoi(pValue)] = 0;
                DTLS_FreeContext(pCtx);
            }
        }
    }
    /* dtls closepeer -peer<peer #> */
    else if(!strcmp(argv[1], "closepeer"))
    {
        pValue = shell_get_opt(argc, argv, "-peer");
        if(pValue)
        {
            pPeer = (dtlsPeerPtr_t)NWKU_GetTblEntry(atoi(pValue), (uint32_t*)maDtlsPeers,
                NumberOfElements(maDtlsPeers));
            if(pPeer)
            {
                shell_printf("Peer was cleared\n\r");
                maDtlsPeers[atoi(pValue)] = 0;
                DTLS_Close(pPeer);
            }
        }
    }
    /* dtls connect -ctx<context> -addr<server IP> -port<server port> */
    else if(!strcmp(argv[1], "connect"))
    {
        pValue = shell_get_opt(argc, argv, "-port");
        if(pValue)
        {
            uint8_t *pAddr;
            sockaddrIn6_t addrInfo;

            /* Check IP address */
            pAddr = (uint8_t*)shell_get_opt(argc, argv, "-addr");
            if(pAddr && pton(AF_INET6, (char*)pAddr, &addrInfo.sin6_addr))
            {
                addrInfo.sin6_family = AF_INET6;
                addrInfo.sin6_port = atoi(pValue);

                /* Get context */
                pValue = shell_get_opt(argc, argv, "-ctx");
                if(pValue)
                {
                    pCtx = (dtlsContext_t*)NWKU_GetTblEntry(atoi(pValue), (uint32_t*)maDtlsContexts,
                        NumberOfElements(maDtlsContexts));
                    if(pCtx)
                      (void)DTLS_Connect(pCtx, &addrInfo);

                }
            }
        }
    }
    /* dtls send -peer<peer> -size<size> <data> */
    else if(!strcmp(argv[1], "send"))
    {
        pValue = shell_get_opt(argc, argv, "-peer");
        if(pValue)
        {
            pPeer = (dtlsPeerPtr_t)NWKU_GetTblEntry(atoi(pValue), (uint32_t*)maDtlsPeers,
                NumberOfElements(maDtlsPeers));
            if(pPeer)
            {
                pValue = shell_get_opt(argc, argv, "-size");
                if(pValue)
                {
                    uint8_t *pOut;
                    uint32_t idx;
                    uint32_t outLen = atoi(pValue);
                    uint8_t iData = 0;
                    uint32_t tmp32;

                    pOut = MEM_BufferAlloc(outLen);
                    for(idx=0;idx<outLen;idx++)
                    {


                         sscanf(argv[argc - outLen + iData], "%02x", &tmp32);
                         *(pOut + idx) = tmp32;

                        iData++;
                    }

                    DTLS_Send(pPeer, pOut, outLen);
                    shell_printf("Sent %d bytes\n\r",outLen);
                    MEM_BufferFree(pOut);
                }
            }
        }
    }

    return ret;
}
#endif
/*!*************************************************************************************************
\private
\fn     ipPktInfo_t *PingCreatePktInfo(uint32_t payloadLen)
\brief  This function is used to create the packet info structure needed by ICMP ping.

\param  [in]    pDstAddr    pointer to the destination IP address
\param  [in]    payloadLen  the size of the payload

\return         ipPktInfo_t pointer to a packet info structure
***************************************************************************************************/
static ipPktInfo_t *PingCreatePktInfo(ipAddr_t *pDstAddr, uint32_t payloadLen)
{
    uint16_t echoId = PING_ID;
    ipPktInfo_t *pIpPktInfo = NWKU_CreateIpPktInfo();
    uint16_t idx;
    uint16_t iPayload;
    uint8_t *pPayload;

    /* Allocate and populate pIpPktInfo->pIpDstAddr */
    pIpPktInfo->pIpDstAddr = NWKU_CreateIpAddr();
    IP_AddrCopy(pIpPktInfo->pIpDstAddr, pDstAddr);

    /* Allocate and populate pIpPktInfo->pIpSrcAddr */
    pIpPktInfo->pIpSrcAddr = NWKU_CreateIpAddr();
    /* Determine IP source address based on IP destination address */
    if(IP_IsAddrIPv6(pIpPktInfo->pIpDstAddr))
    {
#if IP_IP6_ENABLE

      ipAddr_t*  pSrcAddr = IP_IF_SelSrcAddr6(NULL, pIpPktInfo->pIpDstAddr);
      IP_AddrCopy(pIpPktInfo->pIpSrcAddr, pSrcAddr);
#endif /* IP_IP6_ENABLE */
    }
    else
    {
#if IP_IP4_ENABLE
      NWKU_ConvertIp4Addr(IP_IF_SelSrcAddr4(pIpPktInfo->pIpDstAddr, pIpPktInfo->pIpPktOptions->ifHandle),
                          pIpPktInfo->pIpSrcAddr);
#endif /* IP_IP4_ENABLE */
    }

    /* Populate pIpPktInfo->pIpPktOptions */
    // TODO: call function to determine interface hop limit
    pIpPktInfo->pIpPktOptions->hopLimit = ICMP_DEFAULT_HOP_LIMIT;
    pIpPktInfo->pIpPktOptions->security = mPingMacSec;
    // TODO: pIpPktInfo->pIpPktOptions->lqi
    // TODO: pIpPktInfo->pIpPktOptions->qos


    /* Allocate and populate pIpPktInfo->pNwkBuff using Echo request payload and
           echo request identifier and sequence number */
    pIpPktInfo->pNwkBuff = NWKU_CreateNwkBuffer(payloadLen + sizeof(echoId) + sizeof(defaultSeqNb));

    /* Populate first the echo request identifier */
    htonas(pIpPktInfo->pNwkBuff->pData, echoId);

    /* Populate the echo request sequence number with a default value */
    htonas(pIpPktInfo->pNwkBuff->pData + sizeof(echoId), defaultSeqNb++);

    /* Set ping payload: 0x61..0x77(a..w) */
    pPayload = pIpPktInfo->pNwkBuff->pData + sizeof(echoId) + sizeof(defaultSeqNb);
    iPayload = 0;
    for(idx=0;idx<payloadLen;idx++,iPayload++)
    {
        if(iPayload > (PING_PAYLOAD_END - PING_PAYLOAD_START))
        {
            iPayload = 0;
        }
        pPayload[idx] = iPayload + PING_PAYLOAD_START;
    }


    return pIpPktInfo;
}

/*!*************************************************************************************************
\private
\fn     void PING_EchoReplyReceiveAsync(ipPktInfo_t ipPktInfo)
\brief  Interface function for the user app. It handles a received Ping Echo Reply message.

\param  [in]    pIpPktInfo      Pointer to the packet information structure.
***************************************************************************************************/
static void PING_EchoReplyReceiveAsync
(
    ipPktInfo_t *pIpPktInfo
)
{
    NWKU_SendMsg(PING_EchoReplyReceive, (void *)pIpPktInfo, pmMainThreadMsgQueue);
}

/*!*************************************************************************************************
\private
\fn     void PING_EchoReplyReceive(ipPktInfo_t ipPktInfo)
\brief  Interface function for the user app. It handles a received Ping Echo Reply message.

\param  [in]    pIpPktInfo      Pointer to the packet information structure.
***************************************************************************************************/
static void PING_EchoReplyReceive
(
    void *pParam
)
{
    uint16_t echoId;
    uint16_t seqNb;
    uint32_t payloadLen;
    uint64_t tempTimestamp;
    ipPktInfo_t *pIpPktInfo = (ipPktInfo_t *)pParam;

    /* Reply from desired IP address */
    if(IP_IsAddrEqual(&mDstIpAddr, pIpPktInfo->pIpSrcAddr) || IP6_IsMulticastAddr(&mDstIpAddr))
    {
        /* Get first the echo request identifier */
        //htonas(pIpPktInfo->pNwkBuff->pData, echoId);
        echoId = ntohas(pIpPktInfo->pNextProt);
        if(echoId == PING_ID)
        {
            /* Get the echo request sequence number */
            //htonas(pIpPktInfo->pNwkBuff->pData + sizeof(echoId), defaultSeqNb);
            seqNb = ntohas(pIpPktInfo->pNextProt + sizeof(echoId));
            if(seqNb == defaultSeqNb-1)
            {
                /* Get payload length from the ICMP packet.
                 * The ping payload is with an offset of 4 */
                payloadLen = pIpPktInfo->nextProtLen - PING_HEADER_SIZE;

                /* Compare payload */
                /*if(FLib_MemCmp(pIpPktInfo->pNextProt + sizeof(echoId) + sizeof(seqNb),
                            (void*)PING_PAYLOAD, payloadLen - (sizeof(echoId) + sizeof(seqNb))))
            {*/
                if(IP_IsAddrIPv6(pIpPktInfo->pIpSrcAddr))
                {
                    ntop(AF_INET6, pIpPktInfo->pIpSrcAddr, addrStr, INET6_ADDRSTRLEN);
                }
                else
                {
                    ntop(AF_INET, pIpPktInfo->pIpSrcAddr, addrStr, INET_ADDRSTRLEN);
                }

                /* Free the input pIpPktInfo  */
                NWKU_FreeIpPktInfo(&pIpPktInfo);

                shell_write("Reply from ");
                shell_write(addrStr);
                shell_write(": bytes=");
                shell_writeDec(payloadLen);
                shell_write(" time=");
                tempTimestamp = TMR_GetTimestamp();
                tempTimestamp -= pingTimeStamp;
                tempTimestamp /= 1000;
                shell_writeDec(tempTimestamp);
                shell_write("ms");
                SHELL_NEWLINE();

                /* Stop timer */
                if(pingTimerID != gTmrInvalidTimerID_c)
                {
                    TMR_StopTimer(pingTimerID);
                }

                /* Continuous ping: restart */
                if(mContinuousPing)
                {

                    /* Start timer */
                    if(mDelayTimerID == gTmrInvalidTimerID_c)
                    {
                        mDelayTimerID = TMR_AllocateTimer();
                    }
                    TMR_StartSingleShotTimer(mDelayTimerID, PING_DELAY, PING_RetransmitCallback, NULL);
                }
                else
                {
                    SHELL_Resume();
                }
                /*else
            {
                shell_write("Reply payload not matching\n\r");
            }*/
            }
            else
            {
                shell_write("Reply sequence number not matching\n\r");
                /* Free the input pIpPktInfo  */
                NWKU_FreeIpPktInfo(&pIpPktInfo);
            }
        }
        else
        {
            shell_write("Reply PING ID not matching\n\r");
            /* Free the input pIpPktInfo  */
            NWKU_FreeIpPktInfo(&pIpPktInfo);
        }
    }
    else
    {
        shell_write("Reply IP source address not matching\n\r");
        /* Free the input pIpPktInfo  */
        NWKU_FreeIpPktInfo(&pIpPktInfo);
    }

}

/*!*************************************************************************************************
\private
\fn     void PING_TimerCallback(void *param)
\brief  This function sets the timeout expire value.

\param  [in]    param   unused

\return         void
***************************************************************************************************/
static void PING_TimerCallback(void *param)
{
    NWKU_SendMsg(PING_HandleTimerCallback, param, pmMainThreadMsgQueue);
}

/*!*************************************************************************************************
\private
\fn     void PING_HandleTimerCallback(void *param)
\brief  This function sets the timeout expire value.

\param  [in]    param   unused

\return         void
***************************************************************************************************/
static void PING_HandleTimerCallback(void *param)
{
    /* Ping reply was not received */
    shell_write("Request timed out");
    SHELL_NEWLINE();

    if(mContinuousPing)
    {
        ipPktInfo_t *pPingIpPktInfo;

        /* Create Ping packet */
        pPingIpPktInfo = PingCreatePktInfo(&mDstIpAddr, pingSize);

        if(pSrcIpAddr)
        {
            IP_AddrCopy(pPingIpPktInfo->pIpSrcAddr, pSrcIpAddr);
        }

        /* Send packet to ICMP for transmission */
        ICMP_Send(pPingIpPktInfo, gIcmp6TypeEchoRequest_c, ICMP_CODE_DEFAULT);

        if(pingCounter > 0 && pingCounter != 0xFFFFU)
        {
            pingCounter--;
        }

        /* Counter have reached 0: stop pinging */
        if(pingCounter == 0)
        {
            mContinuousPing = FALSE;
        }

        /* Get timestamp */
        pingTimeStamp = TMR_GetTimestamp();

        if (pingTimerID != gTmrInvalidTimerID_c)
        {
            /* Start timer */
            TMR_StartSingleShotTimer(pingTimerID, mPingTimeoutMs, PING_TimerCallback, NULL);
        }
        else
        {
            shell_write("Invalid Timer ID!");
        }
    }
    else
    {
        SHELL_Resume();
    }
}
#if DTLS_ENABLED
static dtlsJpakePasswd_t password =
{
    .pSecret = "threadjpaketest",
    .secretLen = sizeof("threadjpaketest") - 1
};
static void DTLS_GetJpakePasswd(dtlsPeerPtr_t pPeer, dtlsJpakePasswd_t **pPasswd)
{
    *pPasswd = &password;
}

static void DTLS_Received(dtlsPeerPtr_t pPeer, uint8_t *pData, uint32_t len)
{

  uint8_t iPeer;

  iPeer = NWKU_GetTblEntry((uint32_t)pPeer, (uint32_t*)maDtlsPeers,
                           NumberOfElements(maDtlsPeers));
  if(iPeer != (uint8_t)(-1))
  {
    int n = 0;
    while (len--)
    {
      shell_printf("%02X ", *pData++);

      n++;
      if (n % 8 == 0)
      {
        if (n % 16 == 0)
          shell_printf("\n\r");
        else
          shell_printf(" ");
      }
    }
  }


  shell_refresh();

}

static void DTLS_Event(dtlsPeerPtr_t pPeer, dtlsAlertLevel_t level, dtlsAlertCode_t code)
{
    /* Check alerts */
    if((level == DTLS_ALERT_LEVEL_OK) && (code == DTLS_ALERT_CONNECTED))
    {
        uint8_t iPeer;

        iPeer = NWKU_AddTblEntry((uint32_t)pPeer, (uint32_t*)maDtlsPeers,
            NumberOfElements(maDtlsPeers));
        if(iPeer != (uint8_t)(-1))
        {
            shell_printf("Received Client connection on peer #%d\n\r", iPeer);
        }
    }
    else
    {
        shell_printf("Connection error(%02d, %02d) on peer #%d\n\r", level, code);
    }

    SHELL_Resume();
}

#endif /* DTLS_ENABLED */

/*!*************************************************************************************************
\private
\fn     void PING_RetransmitCallback(void *param)
\brief  This function is called when 500ms ping timer expires. This timer is used to send another
        Ping.Request after a Ping.Reply packet was received.

\param  [in]    param   unused

\return         void
***************************************************************************************************/
static void PING_RetransmitCallback
(
    void *param
)
{
    NWKU_SendMsg(PING_RetransmitHandle, param, pmMainThreadMsgQueue);
}

/*!*************************************************************************************************
\private
\fn     void PING_RetransmitHandle(void *param)
\brief  This function is handles the 500ms timeout. This timer is used to send another Ping.Request
        after a Ping.Reply packet was received.

\param  [in]    param   unused

\return         void
***************************************************************************************************/
static void PING_RetransmitHandle
(
    void *param
)
{
    ipPktInfo_t *pPingIpPktInfo;

    /* Create Ping packet */
    pPingIpPktInfo = PingCreatePktInfo(&mDstIpAddr, pingSize);

    /* If we have a specified source address: set it */
    if(pSrcIpAddr)
    {
        IP_AddrCopy(pPingIpPktInfo->pIpSrcAddr, pSrcIpAddr);
    }

    /* Send packet to ICMP for transmission */
    if(IP_IsAddrIPv6(&mDstIpAddr))
    {
        ICMP_Send(pPingIpPktInfo, gIcmp6TypeEchoRequest_c, ICMP_CODE_DEFAULT);
    }
    else
    {
        ICMP_Send(pPingIpPktInfo, gIcmp4TypeEchoRequest_c, ICMP_CODE_DEFAULT);
    }

    /* Get timestamp */
    pingTimeStamp = TMR_GetTimestamp();

    /* Start timer */
    TMR_StartSingleShotTimer(pingTimerID, mPingTimeoutMs, PING_TimerCallback, NULL);

    if(pingCounter > 0 && pingCounter != 0xFFFFU)
    {
        pingCounter--;
    }

    /* Counter have reached 0: stop pinging */
    if(pingCounter == 0)
    {
        mContinuousPing = FALSE;
        MEM_BufferFree(pSrcIpAddr);
        pSrcIpAddr = NULL;
    }

    TMR_FreeTimer(mDelayTimerID);
    mDelayTimerID = gTmrInvalidTimerID_c;
}

#endif /* THREAD_USE_SHELL */

/*==================================================================================================
Private debug functions
==================================================================================================*/
