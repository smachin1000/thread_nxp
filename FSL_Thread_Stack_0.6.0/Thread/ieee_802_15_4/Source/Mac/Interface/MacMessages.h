/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file MacMessages.h
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

#ifndef _MAC_MESSAGES_H
#define _MAC_MESSAGES_H

    /* 802.15.4-2006 standard MAC MCPS and MLME API messages */
    gMcpsDataReq_c,
    gMcpsDataCnf_c,
    gMcpsDataInd_c,

    gMcpsPurgeReq_c,
    gMcpsPurgeCnf_c,

    gMcpsPromInd_c,

    gMlmeAssociateReq_c,
    gMlmeAssociateInd_c,
    gMlmeAssociateRes_c,
    gMlmeAssociateCnf_c,

    gMlmeDisassociateReq_c,
    gMlmeDisassociateInd_c,
    gMlmeDisassociateCnf_c,
    gMlmeBeaconNotifyInd_c,

    gMlmeGetReq_c,
    gMlmeGetCnf_c,

    gMlmeGtsReq_c,
    gMlmeGtsCnf_c,
    gMlmeGtsInd_c,

    gMlmeOrphanInd_c,
    gMlmeOrphanRes_c,

    gMlmeResetReq_c,
    gMlmeResetCnf_c,

    gMlmeRxEnableReq_c,
    gMlmeRxEnableCnf_c,

    gMlmeScanReq_c,
    gMlmeScanCnf_c,

    gMlmeCommStatusInd_c,

    gMlmeSetReq_c,
    gMlmeSetCnf_c,

    gMlmeStartReq_c,
    gMlmeStartCnf_c,

    gMlmeSyncReq_c,

    gMlmeSyncLossInd_c,

    gAutoPollReq_c,
    gMlmePollReq_c,
    gMlmePollCnf_c,

    gMlmePollNotifyInd_c,

#define gMsgMacFirst_c (gMcpsDataReq_c)
#define gMsgMacLast_c (gMlmePollNotifyInd_c)

#endif  /* _MAC_MESSAGES_H */
