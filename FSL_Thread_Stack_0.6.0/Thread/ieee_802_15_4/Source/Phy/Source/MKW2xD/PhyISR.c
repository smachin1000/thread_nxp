/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyISR.c
* PHY ISR Functions
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
#include "board.h"
#include "MCR20Drv.h"
#include "MCR20Reg.h"
#include "Phy.h"
#include "PhyInterface.h"

#include "fsl_os_abstraction.h"
#include "fsl_gpio_driver.h"

extern const IRQn_Type g_portIrqId[HW_PORT_INSTANCE_COUNT];

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#if defined(MCU_MKL46Z4)
  #define MCR20_Irq_Priority     (0xC0)
#else
  #define MCR20_Irq_Priority     (0x80)
#endif

#define PHY_IRQSTS1_INDEX_c     0x00
#define PHY_IRQSTS2_INDEX_c     0x01
#define PHY_IRQSTS3_INDEX_c     0x02
#define PHY_CTRL1_INDEX_c       0x03
#define PHY_CTRL2_INDEX_c       0x04
#define PHY_CTRL3_INDEX_c       0x05
#define PHY_RX_FRM_LEN_INDEX_c  0x06
#define PHY_CTRL4_INDEX_c       0x07

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
extern Phy_PhyLocalStruct_t     phyLocal[];
static volatile phyRxParams_t * mpRxParams = NULL;
static uint32_t                 mPhyTaskInstance;
uint8_t                         mStatusAndControlRegs[8];
uint8_t                         mPhyLastRxLQI = 0;
uint8_t                         mPhyLastRxRSSI = 0;

void (*gpfPhyPreprocessData)(uint8_t *pData) = NULL;
static void* pfOldIsr = NULL;

#if gUsePBTransferThereshold_d
static uint8_t mPhyWatermarkLevel;
#define mPhyGetPBTransferThreshold(len) ((len) - 2)
//#define mPhyGetPBTransferThreshold(len) ((len)*93/100)
//#define mPhyGetPBTransferThreshold(len) (((len) < 20) ? ((len) - 2) : ((len) * 93 / 100))
#endif

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  Sets the current PHY instance waiting for an IRQ
*
* \param[in]  instanceId instance of the PHY
*
********************************************************************************** */
void PhyIsrPassTaskParams
(
  instanceId_t instanceId
)
{
    mPhyTaskInstance = instanceId;
}

/*! *********************************************************************************
* \brief  Sets the location of the Rx parameters
*
* \param[in]  pRxParam pointer to Rx parameters
*
********************************************************************************** */
void PhyIsrPassRxParams
(
  volatile phyRxParams_t * pRxParam
)
{
    mpRxParams = pRxParam;
}

/*! *********************************************************************************
* \brief  Clear and mask PHY IRQ, set sequence to Idle
*
********************************************************************************** */
void PhyIsrSeqCleanup
(
  void
)
{
    mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] &= 0xF0;
    mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] &= (uint8_t) ~( cIRQSTS3_TMR3MSK ); // unmask TMR3 interrupt
    mStatusAndControlRegs[PHY_CTRL1_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL1_XCVSEQ );
    mStatusAndControlRegs[PHY_CTRL2_INDEX_c]   |= (uint8_t)  ( cPHY_CTRL2_CCAMSK | \
                                                               cPHY_CTRL2_RXMSK  | \
                                                               cPHY_CTRL2_TXMSK  | \
                                                               cPHY_CTRL2_SEQMSK );

    // clear transceiver interrupts, mask SEQ, RX, TX and CCA interrupts and set the PHY sequencer back to IDLE
    MCR20Drv_DirectAccessSPIMultiByteWrite(IRQSTS1, mStatusAndControlRegs, 5);
}

/*! *********************************************************************************
* \brief  Clear and mask PHY IRQ, disable timeout, set sequence to Idle
*
********************************************************************************** */
void PhyIsrTimeoutCleanup
(
  void
)
{
    mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] &= 0xF0;
    mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] |= (uint8_t)  ( cIRQSTS3_TMR3MSK | \
                                                               cIRQSTS3_TMR3IRQ); // mask and clear TMR3 interrupt
    mStatusAndControlRegs[PHY_CTRL1_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL1_XCVSEQ );
    mStatusAndControlRegs[PHY_CTRL2_INDEX_c]   |= (uint8_t)  ( cPHY_CTRL2_CCAMSK | \
                                                               cPHY_CTRL2_RXMSK  | \
                                                               cPHY_CTRL2_TXMSK  | \
                                                               cPHY_CTRL2_SEQMSK );

    // disable TMR3 comparator and timeout
    mStatusAndControlRegs[PHY_CTRL3_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL3_TMR3CMP_EN );
    mStatusAndControlRegs[PHY_CTRL4_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL4_TC3TMOUT );

    // clear transceiver interrupts, mask mask SEQ, RX, TX, TMR3 and CCA interrupts interrupts and set the PHY sequencer back to IDLE
    MCR20Drv_DirectAccessSPIMultiByteWrite(IRQSTS1, mStatusAndControlRegs, 8);
}

/*! *********************************************************************************
* \brief  Scales energy level to 0-255
*
* \param[in]  energyLevel  the energ level reported by HW
*
* \return  uint8_t  the energy level scaled in 0x00-0xFF
*
********************************************************************************** */
uint8_t Phy_GetEnergyLevel
(
uint8_t energyLevel /* db */
)
{
    if(energyLevel >= 90)
    {
        /* ED value is below minimum. Return 0x00. */
        energyLevel = 0x00;
    }
    else if(energyLevel <= 26)
    {
        /* ED value is above maximum. Return 0xFF. */
        energyLevel = 0xFF;
    }
    else
    {
        /* Energy level (-90 dBm to -26 dBm ) --> varies form 0 to 64 */
        energyLevel = (90 - energyLevel);
        /* Rescale the energy level values to the 0x00-0xff range (0 to 64 translates in 0 to 255) */
        /* energyLevel * 3.9844 ~= 4 */
        /* Multiply with 4=2^2 by shifting left.
        The multiplication will not overflow beacause energyLevel has values between 0 and 63 */
        energyLevel <<= 2;
    }

    return energyLevel;
}

/*! *********************************************************************************
* \brief  Scales LQI to 0-255
*
* \param[in]  hwLqi  the LQI reported by HW
*
* \return  uint8_t  the LQI scaled in 0x00-0xFF
*
********************************************************************************** */
static uint8_t Phy_LqiConvert
(
  uint8_t hwLqi
)
{
  uint32_t tmpLQI;

  /* LQI Saturation Level */
  if (hwLqi >= 230)
  {
    return 0xFF;
  }
  else
  {
    /* Rescale the LQI values from min to saturation to the 0x00 - 0xFF range */
    /* The LQI value mst be multiplied by ~1.1087 */
    /* tmpLQI =  hwLqi * 7123 ~= hwLqi * 65536 * 0.1087 = hwLqi * 2^16 * 0.1087*/
    tmpLQI = ((uint32_t)hwLqi * (uint32_t)7123 );
    /* tmpLQI =  (tmpLQI / 2^16) + hwLqi */
    tmpLQI = (uint32_t)(tmpLQI >> 16) + (uint32_t)hwLqi;

    return (uint8_t)tmpLQI;
  }
}

/*! *********************************************************************************
* \brief  This function returns the LQI for the las received packet
*
* \return  uint8_t  LQI value
*
********************************************************************************** */
uint8_t PhyGetLastRxLqiValue(void)
{
    return mPhyLastRxLQI;
}

/*! *********************************************************************************
* \brief  This function returns the RSSI for the las received packet
*
* \return  uint8_t  RSSI value
*
********************************************************************************** */
uint8_t PhyGetLastRxRssiValue(void)
{
  return mPhyLastRxRSSI;
}

/*! *********************************************************************************
* \brief  PHY ISR
*
********************************************************************************** */
void PHY_InterruptHandler(void)
{
    uint8_t xcvseqCopy;

    /* The ISR may be called even if another PORTx pin has changed */
    if( !MCR20Drv_IsIrqPending() )
    {
        if( pfOldIsr ) /* Run other ISRs that are chained */
            ((void (*)(void))pfOldIsr)();

        return;
    }

    /* Disable and clear transceiver(IRQ_B) interrupt */
    MCR20Drv_IRQ_Disable();
    MCR20Drv_IRQ_Clear();

    /* Read transceiver interrupt status and control registers */
    mStatusAndControlRegs[PHY_IRQSTS1_INDEX_c] =
        MCR20Drv_DirectAccessSPIMultiByteRead(IRQSTS2, &mStatusAndControlRegs[1], 7);
    xcvseqCopy = mStatusAndControlRegs[PHY_CTRL1_INDEX_c] & cPHY_CTRL1_XCVSEQ;
    /* clear transceiver interrupts */
    MCR20Drv_DirectAccessSPIMultiByteWrite(IRQSTS1, mStatusAndControlRegs, 3);

    if( (mStatusAndControlRegs[PHY_IRQSTS2_INDEX_c] & cIRQSTS2_WAKE_IRQ) &&
       !(mStatusAndControlRegs[PHY_CTRL3_INDEX_c] & cPHY_CTRL3_WAKE_MSK) )
    {
#ifdef MAC_PHY_DEBUG
        Radio_Phy_UnexpectedTransceiverReset(mPhyTaskInstance);
#endif
        MCR20Drv_IRQ_Enable();
        return;
    }

    /* Flter Fail IRQ */
    if( (mStatusAndControlRegs[PHY_IRQSTS1_INDEX_c] & cIRQSTS1_FILTERFAIL_IRQ) &&
       !(mStatusAndControlRegs[PHY_CTRL2_INDEX_c] & cPHY_CTRL2_FILTERFAIL_MSK) )
    {
#if gUsePBTransferThereshold_d
        /* Reset the RX_WTR_MARK level since packet was dropped. */
        mPhyWatermarkLevel = 0;
        MCR20Drv_IndirectAccessSPIWrite(RX_WTR_MARK, mPhyWatermarkLevel);
#endif
        Radio_Phy_PlmeFilterFailRx(mPhyTaskInstance);
    }
    /* Rx Watermark IRQ */
    else if( (mStatusAndControlRegs[PHY_IRQSTS1_INDEX_c] & cIRQSTS1_RXWTRMRKIRQ) &&
            !(mStatusAndControlRegs[PHY_CTRL2_INDEX_c] & cPHY_CTRL2_RX_WMRK_MSK) )
    {
#if gUsePBTransferThereshold_d
        if( 0 == mPhyWatermarkLevel )
        {
            /* Check if this is a standalone RX because we could end up here during a TR sequence also. */
            if( xcvseqCopy == gRX_c )
            {
                /* Set the thereshold packet length at which to start the PB Burst Read.*/
                mPhyWatermarkLevel = mPhyGetPBTransferThreshold( mStatusAndControlRegs[PHY_RX_FRM_LEN_INDEX_c] );
                MCR20Drv_IndirectAccessSPIWrite(RX_WTR_MARK, mPhyWatermarkLevel);
            }
#endif
            Radio_Phy_PlmeRxSfdDetect(mPhyTaskInstance, mStatusAndControlRegs[PHY_RX_FRM_LEN_INDEX_c]);
#if gUsePBTransferThereshold_d
        }
        else
        {
            /* Reset RX_WTR_MARK here, because if the FCS fails, no other IRQ will arrive
            * and the RX will restart automatically. */
            mPhyWatermarkLevel = 0;
            MCR20Drv_IndirectAccessSPIWrite(RX_WTR_MARK, mPhyWatermarkLevel);

            if( mpRxParams )
            {
                // Read data from PB
                MCR20Drv_PB_SPIBurstRead(mpRxParams->pRxData->msgData.dataInd.pPsdu, (uint8_t)(mStatusAndControlRegs[PHY_RX_FRM_LEN_INDEX_c] - 2));
                if( gpfPhyPreprocessData )
                    gpfPhyPreprocessData(mpRxParams->pRxData->msgData.dataInd.pPsdu);
            }
        }
#endif
    }

    /* Timer 1 Compare Match */
    if( (mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] & cIRQSTS3_TMR1IRQ) &&
       !(mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] & cIRQSTS3_TMR1MSK))
    {
        // disable TMR1 comparator
        mStatusAndControlRegs[PHY_CTRL3_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL3_TMR1CMP_EN);
        MCR20Drv_DirectAccessSPIWrite(PHY_CTRL3, mStatusAndControlRegs[PHY_CTRL3_INDEX_c]);

        Radio_Phy_TimeWaitTimeoutIndication(mPhyTaskInstance);
    }

    /* Sequencer interrupt, the autosequence has completed */
    if( (mStatusAndControlRegs[PHY_IRQSTS1_INDEX_c] & cIRQSTS1_SEQIRQ) &&
       !(mStatusAndControlRegs[PHY_CTRL2_INDEX_c] & cPHY_CTRL2_SEQMSK) ) 
    {
        // PLL unlock, the autosequence has been aborted due to PLL unlock
        if( mStatusAndControlRegs[PHY_IRQSTS1_INDEX_c] & cIRQSTS1_PLL_UNLOCK_IRQ )
        {
            PhyIsrSeqCleanup();
            Radio_Phy_PlmeSyncLossIndication(mPhyTaskInstance);
            MCR20Drv_IRQ_Enable();
            return;
        }

        // TMR3 timeout, the autosequence has been aborted due to TMR3 timeout
        if( (mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] & cIRQSTS3_TMR3IRQ) &&
           !(mStatusAndControlRegs[PHY_IRQSTS1_INDEX_c] & cIRQSTS1_RXIRQ) &&
            (gTX_c != xcvseqCopy) )
        {
            PhyIsrTimeoutCleanup();

            Radio_Phy_TimeRxTimeoutIndication(mPhyTaskInstance);
            MCR20Drv_IRQ_Enable();
            return;
        }

        PhyIsrSeqCleanup();

        switch(xcvseqCopy)
        {
        case gTX_c:
            if( (mStatusAndControlRegs[PHY_IRQSTS2_INDEX_c] & cIRQSTS2_CCA) &&
                (mStatusAndControlRegs[PHY_CTRL1_INDEX_c]   & cPHY_CTRL1_CCABFRTX) )
            {
                Radio_Phy_PlmeCcaConfirm(gPhyChannelBusy_c, mPhyTaskInstance);
            }
            else
            {
                Radio_Phy_PdDataConfirm(mPhyTaskInstance, FALSE);
            }
            break;

        case gTR_c:
            if( (mStatusAndControlRegs[PHY_IRQSTS2_INDEX_c] & cIRQSTS2_CCA) &&
                (mStatusAndControlRegs[PHY_CTRL1_INDEX_c]   & cPHY_CTRL1_CCABFRTX) )
            {
                Radio_Phy_PlmeCcaConfirm(gPhyChannelBusy_c, mPhyTaskInstance);
            }
            else
            {
                if(NULL != mpRxParams)
                {
                    // reports value of 0x00 for -105 dBm of received input power and 0xFF for 0 dBm of received input power
                    mPhyLastRxRSSI = MCR20Drv_DirectAccessSPIRead((uint8_t) LQI_VALUE);
                    mpRxParams->linkQuality = Phy_LqiConvert(mPhyLastRxRSSI);
                    mPhyLastRxLQI = mpRxParams->linkQuality;
                    MCR20Drv_DirectAccessSPIMultiByteRead( (uint8_t) TIMESTAMP_LSB, (uint8_t *)&mpRxParams->timeStamp, 3);
                    mpRxParams->psduLength = (uint8_t)(mStatusAndControlRegs[PHY_RX_FRM_LEN_INDEX_c]); //Including FCS (2 bytes)
                }
                if( (mStatusAndControlRegs[PHY_IRQSTS1_INDEX_c] & cIRQSTS1_RX_FRM_PEND) == cIRQSTS1_RX_FRM_PEND )
                {
                    Radio_Phy_PdDataConfirm(mPhyTaskInstance, TRUE);
                }
                else
                {
                    Radio_Phy_PdDataConfirm(mPhyTaskInstance, FALSE);
                }
            }
            break;

        case gRX_c:
            if( NULL != mpRxParams )
            {
                // reports value of 0x00 for -105 dBm of received input power and 0xFF for 0 dBm of received input power
                mPhyLastRxRSSI = MCR20Drv_DirectAccessSPIRead((uint8_t) LQI_VALUE);
                mpRxParams->linkQuality = Phy_LqiConvert(mPhyLastRxRSSI);
                mPhyLastRxLQI = mpRxParams->linkQuality;
                MCR20Drv_DirectAccessSPIMultiByteRead( (uint8_t) TIMESTAMP_LSB, (uint8_t *)&mpRxParams->timeStamp, 3);
                mpRxParams->psduLength = (uint8_t)(mStatusAndControlRegs[PHY_RX_FRM_LEN_INDEX_c]); //Including FCS (2 bytes)
            }
            Radio_Phy_PdDataIndication(mPhyTaskInstance);
            break;

        case gCCA_c:
            if( (mStatusAndControlRegs[PHY_CTRL4_INDEX_c] & (cPHY_CTRL4_CCATYPE << cPHY_CTRL4_CCATYPE_Shift_c)) == (gCcaED_c << cPHY_CTRL4_CCATYPE_Shift_c) )
            {
                // Ed
                Radio_Phy_PlmeEdConfirm(MCR20Drv_DirectAccessSPIRead((uint8_t) CCA1_ED_FNL), mPhyTaskInstance);
            }
            else
            {
                // CCA
                if( mStatusAndControlRegs[PHY_IRQSTS2_INDEX_c] & cIRQSTS2_CCA )
                {
#if (gUseStandaloneCCABeforeTx_d == 1)
                    phyLocal[mPhyTaskInstance].txParams.numOfCca = 0;
#endif
                    Radio_Phy_PlmeCcaConfirm(gPhyChannelBusy_c, mPhyTaskInstance);
                }
                else
                {
#if (gUseStandaloneCCABeforeTx_d == 1)
                    if( phyLocal[mPhyTaskInstance].txParams.numOfCca > 0 )
                    {
                        mStatusAndControlRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);

                        if( --phyLocal[mPhyTaskInstance].txParams.numOfCca == 0 )
                        {
                            // perform TxRxAck sequence if required by phyTxMode
                            if( gPhyRxAckRqd_c == phyLocal[mPhyTaskInstance].txParams.ackRequired )
                            {
                                mStatusAndControlRegs[PHY_CTRL1] |= (uint8_t) (cPHY_CTRL1_RXACKRQD);
                                mStatusAndControlRegs[PHY_CTRL1] |=  gTR_c;
                            }
                            else
                            {
                                mStatusAndControlRegs[PHY_CTRL1] &= (uint8_t) ~(cPHY_CTRL1_RXACKRQD);
                                mStatusAndControlRegs[PHY_CTRL1] |=  gTX_c;
                            }
                        }
                        else
                        {
                            mStatusAndControlRegs[PHY_CTRL1] |= gCCA_c;
                        }

                        mStatusAndControlRegs[PHY_CTRL2] &= (uint8_t) ~(cPHY_CTRL2_SEQMSK); // unmask SEQ interrupt
                        // start the sequence immediately
                        MCR20Drv_DirectAccessSPIMultiByteWrite(PHY_CTRL1,
                                                                 &mStatusAndControlRegs[PHY_CTRL1],
                                                                 2);
                    }
                    else
#endif
                    {
                        Radio_Phy_PlmeCcaConfirm(gPhyChannelIdle_c, mPhyTaskInstance);
                    }
                }
            }
            break;

        case gCCCA_c:
            Radio_Phy_PlmeCcaConfirm(gPhyChannelIdle_c, mPhyTaskInstance);
            break;

        default:
            Radio_Phy_PlmeSyncLossIndication(mPhyTaskInstance);
            break;
        }
    }
    // timers interrupt
    else
    {
        if( mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] & cIRQSTS3_TMR2IRQ )
        {
            // disable TMR2 comparator and time triggered action
            mStatusAndControlRegs[PHY_CTRL3_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL3_TMR2CMP_EN);
            mStatusAndControlRegs[PHY_CTRL1_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL1_TMRTRIGEN);

            MCR20Drv_DirectAccessSPIWrite(PHY_CTRL3, mStatusAndControlRegs[PHY_CTRL3_INDEX_c]);
            MCR20Drv_DirectAccessSPIWrite(PHY_CTRL1, mStatusAndControlRegs[PHY_CTRL1_INDEX_c]);

            Radio_Phy_TimeStartEventIndication(mPhyTaskInstance);
        }

        if( mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] & cIRQSTS3_TMR3IRQ )
        {
            /* disable TMR3 comparator and timeout */
            mStatusAndControlRegs[PHY_CTRL3_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL3_TMR3CMP_EN);
            mStatusAndControlRegs[PHY_CTRL4_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL4_TC3TMOUT);

            MCR20Drv_DirectAccessSPIWrite(PHY_CTRL3, mStatusAndControlRegs[PHY_CTRL3_INDEX_c]);
            MCR20Drv_DirectAccessSPIWrite(PHY_CTRL4, mStatusAndControlRegs[PHY_CTRL4_INDEX_c]);

            /* Ensure that we're not issuing TimeoutIndication while the Automated sequence is still in progress */
            /* TMR3 can expire during R-T turnaround for example, case in which the sequence is not interrupted */
            if( gIdle_c == xcvseqCopy )
            {
                Radio_Phy_TimeRxTimeoutIndication(mPhyTaskInstance);
            }
        }

        /* Timer 4 Compare Match */
        if( mStatusAndControlRegs[PHY_IRQSTS3_INDEX_c] & cIRQSTS3_TMR4IRQ )
        {
            /* disable TMR4 comparator */
            mStatusAndControlRegs[PHY_CTRL3_INDEX_c]   &= (uint8_t) ~( cPHY_CTRL3_TMR4CMP_EN);
            MCR20Drv_DirectAccessSPIWrite(PHY_CTRL3, mStatusAndControlRegs[PHY_CTRL3_INDEX_c]);
        }
    }

    MCR20Drv_IRQ_Enable();
}

/*! *********************************************************************************
* \brief  This function installs the PHY ISR
*
********************************************************************************** */
void PHY_InstallIsr( void )
{
    pfOldIsr = OSA_InstallIntHandler(g_portIrqId[GPIO_EXTRACT_PORT(kGpioXcvrIrqPin_d)], PHY_InterruptHandler);
    /* Handle other PORTx ISRs */
    if(OSA_DEFAULT_INT_HANDLER == pfOldIsr)
        pfOldIsr = NULL;
    
    /* enable transceiver SPI interrupt request */
    NVIC_ClearPendingIRQ(g_portIrqId[GPIO_EXTRACT_PORT(kGpioXcvrIrqPin_d)]);
    NVIC_EnableIRQ(g_portIrqId[GPIO_EXTRACT_PORT(kGpioXcvrIrqPin_d)]);
    
    /* set transceiver interrupt priority */
    NVIC_SetPriority(g_portIrqId[GPIO_EXTRACT_PORT(kGpioXcvrIrqPin_d)], 
                     MCR20_Irq_Priority >> (8 - __NVIC_PRIO_BITS));
}