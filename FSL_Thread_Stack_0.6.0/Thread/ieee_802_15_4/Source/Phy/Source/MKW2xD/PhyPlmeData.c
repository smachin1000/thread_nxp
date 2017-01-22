/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyPlmeData.c
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


/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "MCR20Drv.h"
#include "MCR20Reg.h"
#include "Phy.h"
#include "PhyTypes.h"
#include "PhyInterface.h"


/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define PHY_PARAMETERS_VALIDATION 1

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
                                     //2405   2410    2415    2420    2425    2430    2435    2440    2445    2450    2455    2460    2465    2470    2475    2480
static const uint8_t  pll_int[16] =  {0x0B,   0x0B,   0x0B,   0x0B,   0x0B,   0x0B,   0x0C,   0x0C,   0x0C,   0x0C,   0x0C,   0x0C,   0x0D,   0x0D,   0x0D,   0x0D};
static const uint16_t pll_frac[16] = {0x2800, 0x5000, 0x7800, 0xA000, 0xC800, 0xF000, 0x1800, 0x4000, 0x6800, 0x9000, 0xB800, 0xE000, 0x0800, 0x3000, 0x5800, 0x8000};

extern Phy_PhyLocalStruct_t     phyLocal[];
static uint8_t gPhyCurrentChannelPAN0 = 0x0B;
static uint8_t gPhyCurrentChannelPAN1 = 0x0B;


/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void PhyRxRetry( uint32_t param );


/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  This function will start a TX sequence. The packet will be sent OTA
*
* \param[in]  pTxPacket   pointer to the TX packet structure
* \param[in]  pRxParams   pointer to RX parameters
* \param[in]  pTxParams   pointer to TX parameters
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPdDataRequest( pdDataReq_t *pTxPacket,
                              volatile phyRxParams_t *pRxParams,
                              volatile phyTxParams_t *pTxParams )
{
    uint8_t phyRegs[5], phyCtrl4Reg;
    uint8_t *pTmpPsdu, *tmp;

#ifdef PHY_PARAMETERS_VALIDATION
    // null pointer
    if(NULL == pTxPacket)
    {
        return gPhyInvalidParameter_c;
    }

    // if CCA required ...
    if( (pTxPacket->CCABeforeTx == gPhyCCAMode3_c) || (pTxPacket->CCABeforeTx == gPhyEnergyDetectMode_c))
    { // ... cannot perform other types than MODE1 and MODE2
        return gPhyInvalidParameter_c;
    }

#endif // PHY_PARAMETERS_VALIDATION

    if( gIdle_c != PhyPpGetState() )
    {
        return gPhyBusy_c;
    }

    // load data into PB
    tmp = pTxPacket->pPsdu;
    
    pTmpPsdu = (uint8_t *) ((&pTxPacket->pPsdu[0])-1);
    *pTmpPsdu = pTxPacket->psduLength + 2; /* including 2 bytes of FCS */
    MCR20Drv_PB_SPIBurstWrite( pTmpPsdu, (uint8_t) (pTxPacket->psduLength + 1)); /* including psduLength */

    pTxPacket->pPsdu = tmp;

    phyCtrl4Reg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL4);
    phyRegs[0] = MCR20Drv_DirectAccessSPIMultiByteRead(IRQSTS2, &phyRegs[1], 4);
    
    // perform CCA before TX if required
    phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_CCABFRTX);
    phyCtrl4Reg &= (uint8_t) ~(cPHY_CTRL4_CCATYPE << cPHY_CTRL4_CCATYPE_Shift_c);
    
    if( pTxPacket->CCABeforeTx != gPhyNoCCABeforeTx_c )
    {
#if (gUseStandaloneCCABeforeTx_d == 0)
        phyRegs[PHY_CTRL1] |= (uint8_t) (cPHY_CTRL1_CCABFRTX);
#endif
        phyCtrl4Reg |= (uint8_t) ((cPHY_CTRL4_CCATYPE & pTxPacket->CCABeforeTx) << (cPHY_CTRL4_CCATYPE_Shift_c));
    }

    // slotted operation
    if( pTxPacket->slottedTx == gPhySlottedMode_c )
    {
        phyRegs[PHY_CTRL1] |= (uint8_t) (cPHY_CTRL1_SLOTTED);
    }
    else
    {
        phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_SLOTTED);
    }

    // perform TxRxAck sequence if required by phyTxMode
    if(pTxPacket->ackRequired == gPhyRxAckRqd_c)
    {
        PhyIsrPassRxParams(pRxParams);
        
        phyRegs[PHY_CTRL1] |= (uint8_t) (cPHY_CTRL1_RXACKRQD);
        phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
        phyRegs[PHY_CTRL1] |=  gTR_c;
    }
    else
    {
        PhyIsrPassRxParams(NULL);
        
        phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_RXACKRQD);
        phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
        phyRegs[PHY_CTRL1] |=  gTX_c;
    }
    
#if gUseStandaloneCCABeforeTx_d
    if( pTxPacket->CCABeforeTx != gPhyNoCCABeforeTx_c )
    {
        // start the CCA or ED sequence (this depends on CcaType used)
        // immediately or by TC2', depending on a previous PhyTimeSetEventTrigger() call)
        if( pTxPacket->slottedTx == gPhySlottedMode_c )
            pTxParams->numOfCca = 2;
        else
            pTxParams->numOfCca = 1;
        pTxParams->ackRequired = pTxPacket->ackRequired;
        phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
        phyRegs[PHY_CTRL1] |= gCCA_c;
        // at the end of the scheduled sequence, an interrupt will occur:
        // CCA , SEQ or TMR3
    }
    else
    {
        pTxParams->numOfCca = 0;
    }
#endif
    
    phyRegs[PHY_CTRL2] &= (uint8_t) ~(cPHY_CTRL2_SEQMSK); // unmask SEQ interrupt
    
    // ensure that no spurious interrupts are raised
    phyRegs[IRQSTS3] &= 0xF0;                     // do not change IRQ status
    phyRegs[IRQSTS3] |= (uint8_t) (cIRQSTS3_TMR3MSK |
                                   cIRQSTS3_TMR2IRQ |
                                   cIRQSTS3_TMR3IRQ);   // mask TMR3 interrupt
    
    MCR20Drv_DirectAccessSPIMultiByteWrite(IRQSTS1, phyRegs, 3);
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL2, phyRegs[PHY_CTRL2]);
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL4, phyCtrl4Reg);
    
    // start the TX or TRX sequence
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyRegs[PHY_CTRL1]);

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will start a RX sequence
*
* \param[in]  phyRxMode   slotted/unslotted
* \param[in]  pRxParams   pointer to RX parameters
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeRxRequest( phySlottedMode_t phyRxMode, phyRxParams_t *  pRxParams )
{
    uint8_t phyRegs[5];

#ifdef PHY_PARAMETERS_VALIDATION
    if(NULL == pRxParams)
    {
        return gPhyInvalidParameter_c;
    }
#endif // PHY_PARAMETERS_VALIDATION

    if( gIdle_c != PhyPpGetState() )
    {
        return gPhyBusy_c;
    }

    pRxParams->phyRxMode = phyRxMode;

    if( NULL == pRxParams->pRxData )
    {
        pRxParams->pRxData = MEM_BufferAlloc(sizeof(pdDataToMacMessage_t) + gMaxPHYPacketSize_c);
    }

    if( NULL == pRxParams->pRxData )
    {
        phyTimeEvent_t event = {
            .timestamp = PhyTime_GetTimestamp() + gPhyRxRetryInterval_c,
            .parameter = (uint32_t)pRxParams,
            .callback  = PhyRxRetry,
        };

        PhyTime_ScheduleEvent( &event );
        return gPhyTRxOff_c;   
    }

    PhyIsrPassRxParams(pRxParams);

    pRxParams->pRxData->msgData.dataInd.pPsdu = 
        (uint8_t*)&pRxParams->pRxData->msgData.dataInd.pPsdu +
        sizeof(pRxParams->pRxData->msgData.dataInd.pPsdu);

    phyRegs[0] = MCR20Drv_DirectAccessSPIMultiByteRead(IRQSTS2, &phyRegs[1], 4);

    // slotted operation
    if(gPhySlottedMode_c == phyRxMode)
    {
        phyRegs[PHY_CTRL1] |= (uint8_t) (cPHY_CTRL1_SLOTTED);
    }
    else
    {
        phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_SLOTTED);
    }

    // program the RX sequence
    phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
    phyRegs[PHY_CTRL1] |=  gRX_c;

    phyRegs[PHY_CTRL2] &= (uint8_t) ~(cPHY_CTRL2_SEQMSK); // unmask SEQ interrupt

    // ensure that no spurious interrupts are raised    
    phyRegs[IRQSTS3] &= 0xF0;                     // do not change IRQ status
    phyRegs[IRQSTS3] |= (uint8_t) (cIRQSTS3_TMR3MSK |
                                   cIRQSTS3_TMR2IRQ |
                                   cIRQSTS3_TMR3IRQ);   // mask TMR3 interrupt

    MCR20Drv_DirectAccessSPIMultiByteWrite(IRQSTS1, phyRegs, 3);    
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL2, phyRegs[PHY_CTRL2]);

    // start the RX sequence
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyRegs[PHY_CTRL1]);

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will start a CCA / CCCA sequence
*
* \param[in]  ccaParam   the type of CCA
* \param[in]  cccaMode   continuous or single CCA
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeCcaEdRequest( phyCCAType_t ccaParam, phyContCCAMode_t cccaMode )
{
    uint8_t phyRegs[5];

#ifdef PHY_PARAMETERS_VALIDATION
    // illegal CCA type
    if( (ccaParam != gPhyCCAMode1_c) && (ccaParam != gPhyCCAMode2_c) && (ccaParam != gPhyCCAMode3_c) && (ccaParam != gPhyEnergyDetectMode_c))
    {
        return gPhyInvalidParameter_c;
    }

    // cannot perform Continuous CCA using ED type
    if( (ccaParam == gPhyEnergyDetectMode_c) && (cccaMode == gPhyContCcaEnabled) )
    {
        return gPhyInvalidParameter_c;
    }
#endif // PHY_PARAMETERS_VALIDATION

    if( gIdle_c != PhyPpGetState() )
    {
        return gPhyBusy_c;
    }

    // write in PHY CTRL4 the desired type of CCA
    phyRegs[0] = MCR20Drv_DirectAccessSPIRead(PHY_CTRL4);
    phyRegs[0] &= (uint8_t) ~(cPHY_CTRL4_CCATYPE << cPHY_CTRL4_CCATYPE_Shift_c);
    phyRegs[0] |= (uint8_t) ((cPHY_CTRL4_CCATYPE & ccaParam) << (cPHY_CTRL4_CCATYPE_Shift_c));
    MCR20Drv_DirectAccessSPIWrite( (uint8_t)PHY_CTRL4, phyRegs[0]);
    
    phyRegs[0] = MCR20Drv_DirectAccessSPIMultiByteRead(IRQSTS2, &phyRegs[1], 4);
    
    // TODO: slotted operation in CCA/ED mode. Slotted Mode, for beacon-enabled networks. Applies only to Sequences T, TR,and R.
#if 0
    if(gSlottedEnMask_c & ccaParam)
    {
        phyCtrl1Reg |= (uint8_t) (cPHY_CTRL1_SLOTTED);
    }
    else
    {
        phyCtrl1Reg &= (uint8_t) ~(cPHY_CTRL1_SLOTTED);
    }
#endif

    // continuous CCA
    if(cccaMode == gPhyContCcaEnabled)
    {
        // start the continuous CCA sequence
        // immediately or by TC2', depending on a previous PhyTimeSetEventTrigger() call)
        phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
        phyRegs[PHY_CTRL1] |= gCCCA_c;
        // at the end of the scheduled sequence, an interrupt will occur:
        // CCA , SEQ or TMR3
    }
    // normal CCA (not continuous)
    else
    {
        // start the CCA or ED sequence (this depends on CcaType used)
        // immediately or by TC2', depending on a previous PhyTimeSetEventTrigger() call)
        phyRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
        phyRegs[PHY_CTRL1] |= gCCA_c;
        // at the end of the scheduled sequence, an interrupt will occur:
        // CCA , SEQ or TMR3
    }
    
    phyRegs[PHY_CTRL2] &= (uint8_t) ~(cPHY_CTRL2_SEQMSK); // unmask SEQ interrupt
    
    // ensure that no spurious interrupts are raised
    phyRegs[IRQSTS3] &= 0xF0;                     // do not change IRQ status
    phyRegs[IRQSTS3] |= (uint8_t) (cIRQSTS3_TMR3MSK |
                                   cIRQSTS3_TMR2IRQ |
                                   cIRQSTS3_TMR3IRQ);   // mask TMR3 interrupt
    
    MCR20Drv_DirectAccessSPIMultiByteWrite(IRQSTS1, phyRegs, 3);
    
    // start the CCA/ED or CCCA sequence
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL2, phyRegs[PHY_CTRL2]);
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyRegs[PHY_CTRL1]);

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will set the channel number for the specified PAN
*
* \param[in]   channel   new channel number
* \param[in]   pan       the PAN registers (0/1)
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeSetCurrentChannelRequest
(
  uint8_t channel,
  uint8_t pan
)
{

#ifdef PHY_PARAMETERS_VALIDATION
  if((channel < 11) || (channel > 26))
  {
    return gPhyInvalidParameter_c;
  }
#endif // PHY_PARAMETERS_VALIDATION

  if( !pan )
  {
      gPhyCurrentChannelPAN0 = channel;
      MCR20Drv_DirectAccessSPIWrite(PLL_INT0, pll_int[channel - 11]);
      MCR20Drv_DirectAccessSPIMultiByteWrite(PLL_FRAC0_LSB, (uint8_t *) &pll_frac[channel - 11], 2);
  }
  else
  {
      gPhyCurrentChannelPAN1 = channel;
      MCR20Drv_IndirectAccessSPIWrite(PLL_INT1, pll_int[channel - 11]);
      MCR20Drv_IndirectAccessSPIMultiByteWrite(PLL_FRAC1_LSB, (uint8_t *) &pll_frac[channel - 11], 2);
  }
  return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function will return the current channel for a specified PAN
*
* \param[in]   pan   the PAN registers (0/1)
*
* \return  uint8_t  current channel number
*
********************************************************************************** */
uint8_t PhyPlmeGetCurrentChannelRequest
(
  uint8_t pan
)
{
    if( !pan )
        return gPhyCurrentChannelPAN0;
    else
        return gPhyCurrentChannelPAN1;
}

/*! *********************************************************************************
* \brief  This function will set the radio Tx power
*
* \param[in]   pwrStep   the Tx power
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeSetPwrLevelRequest
(
  uint8_t pwrStep
)
{
#ifdef PHY_PARAMETERS_VALIDATION
  if((pwrStep < 3) || (pwrStep > 31)) //-40 dBm to 16 dBm
  {
    return gPhyInvalidParameter_c;
  }
#endif // PHY_PARAMETERS_VALIDATION

  MCR20Drv_DirectAccessSPIWrite(PA_PWR, (uint8_t)(pwrStep & 0x1F));
  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeSetLQIModeRequest
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PhyPlmeSetLQIModeRequest(uint8_t lqiMode)
{
  uint8_t currentMode;

  currentMode = MCR20Drv_IndirectAccessSPIRead(CCA_CTRL);
  lqiMode ? (currentMode |= cCCA_CTRL_LQI_RSSI_NOT_CORR) : (currentMode &= (~((uint8_t)cCCA_CTRL_LQI_RSSI_NOT_CORR)));
  MCR20Drv_IndirectAccessSPIWrite(CCA_CTRL, currentMode);

  return gPhySuccess_c;
}

/*---------------------------------------------------------------------------
 * Name: PhyPlmeGetRSSILevelRequest
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PhyPlmeGetRSSILevelRequest(void)
{
  return MCR20Drv_IndirectAccessSPIRead(RSSI);
}

/*! *********************************************************************************
* \brief  This function will set the value of PHY PIBs
*
* \param[in]   pibId            the Id of the PIB
* \param[in]   pibValue         the new value of the PIB
* \param[in]   phyRegistrySet   the PAN registers (0/1)
* \param[in]   instanceId       the instance of the PHY
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeSetPIBRequest(phyPibId_t pibId, uint64_t pibValue, uint8_t phyRegistrySet, instanceId_t instanceId)
{
  phyStatus_t result = gPhySuccess_c;

  switch(pibId)
  {
    case gPhyPibCurrentChannel_c:
    {
        result = PhyPlmeSetCurrentChannelRequest((uint8_t) pibValue, phyRegistrySet);
    }
    break;
    case gPhyPibTransmitPower_c:
    {
        result = PhyPlmeSetPwrLevelRequest((uint8_t) pibValue);
    }
    break;
    case gPhyPibLongAddress_c:
    {
        uint64_t longAddr = pibValue;
        result = PhyPpSetLongAddr((uint8_t *) &longAddr, phyRegistrySet);
    }
    break;
    case gPhyPibShortAddress_c:
    {
        uint16_t shortAddr = (uint16_t) pibValue;
        result = PhyPpSetShortAddr((uint8_t *) &shortAddr, phyRegistrySet);
    }
    break;
    case gPhyPibPanId_c:
    {
        uint16_t panId = (uint16_t) pibValue;
        result = PhyPpSetPanId((uint8_t *) &panId, phyRegistrySet);
    }
    break;
    case gPhyPibPanCoordinator_c:
    {
        bool_t macRole = (bool_t) pibValue;
        result = PhyPpSetMacRole(macRole, phyRegistrySet);
    }
    break;
    case gPhyPibCurrentPage_c:
    {
        /* Nothinh to do... */
    }
    break;
    case gPhyPibPromiscuousMode_c:
    {
        PhyPpSetPromiscuous((uint8_t)pibValue);
    }
    break;
    case gPhyPibRxOnWhenIdle:
    {
        PhyPlmeSetRxOnWhenIdle( (bool_t)pibValue, instanceId );
    }
    break;
    case gPhyPibFrameWaitTime_c:
    {
        PhyPlmeSetFrameWaitTime( (uint32_t)pibValue, instanceId );
    }
    break;
    default:
    {
        result = gPhyUnsupportedAttribute_c;
    }
    break;
  }

  return result;
}

/*! *********************************************************************************
* \brief  This function will return the value of PHY PIBs
*
* \param[in]   pibId            the Id of the PIB
* \param[out]  pibValue         pointer to a location where the value will be stored
* \param[in]   phyRegistrySet   the PAN registers (0/1)
* \param[in]   instanceId       the instance of the PHY
*
* \return  phyStatus_t
*
********************************************************************************** */
phyStatus_t PhyPlmeGetPIBRequest(phyPibId_t pibId, uint64_t * pibValue, uint8_t phyRegistrySet, instanceId_t instanceId)
{
    phyStatus_t result = gPhySuccess_c;
    switch(pibId)
    {
      case gPhyPibCurrentChannel_c:
      {
          *((uint8_t*)pibValue) = (uint64_t) PhyPlmeGetCurrentChannelRequest(phyRegistrySet);
      }
      break;
      case gPhyPibTransmitPower_c:
      {
          *((uint8_t*)pibValue) = MCR20Drv_DirectAccessSPIRead(PA_PWR);
      }
      break;
      case gPhyPibLongAddress_c:
      {
          if( !phyRegistrySet )
              MCR20Drv_IndirectAccessSPIMultiByteRead( MACLONGADDRS0_0, (uint8_t*)pibValue, 8);
          else
              MCR20Drv_IndirectAccessSPIMultiByteRead( MACLONGADDRS1_0, (uint8_t*)pibValue, 8);
      }
      break;
      case gPhyPibShortAddress_c:
      {
          if( !phyRegistrySet )
              MCR20Drv_IndirectAccessSPIMultiByteRead( MACSHORTADDRS0_LSB, (uint8_t*)pibValue, 2);
          else
              MCR20Drv_IndirectAccessSPIMultiByteRead( MACSHORTADDRS1_LSB, (uint8_t*)pibValue, 2);
      }
      break;
      case gPhyPibPanId_c:
      {
          if( !phyRegistrySet )
              MCR20Drv_IndirectAccessSPIMultiByteRead( MACPANID0_LSB, (uint8_t*)pibValue, 2);
          else
              MCR20Drv_IndirectAccessSPIMultiByteRead( MACPANID1_LSB, (uint8_t*)pibValue, 2);
      }
      break;
      case gPhyPibPanCoordinator_c:
      {
          uint8_t phyReg;

          if( !phyRegistrySet )
          {
              phyReg = MCR20Drv_DirectAccessSPIRead( PHY_CTRL4);
              phyReg = (phyReg & cPHY_CTRL4_PANCORDNTR0) == cPHY_CTRL4_PANCORDNTR0;
          }
          else
          {
              phyReg = MCR20Drv_IndirectAccessSPIRead( (uint8_t) DUAL_PAN_CTRL);
              phyReg = (phyReg & cDUAL_PAN_CTRL_PANCORDNTR1) == cDUAL_PAN_CTRL_PANCORDNTR1;
          }

          *((uint8_t*)pibValue) = phyReg;
      }
      break;
      case gPhyPibRxOnWhenIdle:
      {
          *((uint8_t*)pibValue) = !!(phyLocal[instanceId].flags & gPhyFlagRxOnWhenIdle_c);
      }
      break;
      case gPhyPibFrameWaitTime_c:
      {
          *((uint8_t*)pibValue) = phyLocal[instanceId].maxFrameWaitTime;
      }
      break;
      default:
      {
          result = gPhyUnsupportedAttribute_c;
      }
      break;
    }

    return result;

}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
static void PhyRxRetry( uint32_t param )
{
    PhyPlmeRxRequest( ((phyRxParams_t*)param)->phyRxMode, (phyRxParams_t*)param );
}