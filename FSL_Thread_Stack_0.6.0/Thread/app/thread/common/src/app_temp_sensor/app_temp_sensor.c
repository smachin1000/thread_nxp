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
\file       app_temp_sensor.c
\brief      This is a public source file for the application temperature sensor.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"
#include <string.h>
#include <stdio.h>

/* Drivers */
#include "fsl_adc16_driver.h"
#include "fsl_pmc_hal.h"

#include "app_temp_sensor.h"
#include "FunctionLib.h"
#include "MemManager.h"

/*==================================================================================================
Private macros
==================================================================================================*/
/* These values are used to get the temperature */
#define V_REFH                          (3075U)         /* [mV] */
#define V_TEMP25                        (716U)          /* [mV] */
#define ADC_MAX_SCALE                   (65535U)        /* 2^16-1 bits */
#define ADC_TEMP_SLOPE                  (1620U)         /* (1715U) [(mV*1000)/C] */
#define TEMP_OFFSET                     (200U)          /* [C*100] */
#define DEFAULT_TEMP                    (2500U)         /*[C*100] */

         
#define CHANNEL_0                       (0U)            /* ADC channel group */
#define ADC_CHANNEL_TEMPERATURE         (26U)           /* ADC channel of temperature sensor */

/* Temp sensor calibration */
#define ADC_CHANNEL_BANDGAP             (27U)           /* ADC channel of BANDGAP */
#define V_BG                            (1000U)         /* BANDGAP voltage in mV (trim to 1.0V) */


#define TEMP_BUFF_SIZE     (20U)
/*==================================================================================================
Private global variables declarations
==================================================================================================*/


/* Temp sensor calibration */
static uint32_t gADCrTemp25 = 0;                 /* Calibrated temp value */
static uint32_t gADCr100m = 0;


/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/* None */


/*==================================================================================================
Private prototypes
==================================================================================================*/

static void APP_TempSensorCalibrateParams(void);

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn       void APP_InitADC(uint32_t instance)
\brief    ADC module initialization
\return   void
***************************************************************************************************/
void APP_InitADC(uint32_t instance)
{
#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif
    adc16_user_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfig;

    APP_TempSensorCalibrateParams();
    
#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    /* Auto calibration */
    ADC16_DRV_GetAutoCalibrationParam(instance, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(instance, &adcCalibraitionParam);
#endif

    /* Initialization ADC for 16bit resolution
       normal convert speed, VREFH/L as reference,
       continuous convert mode. */
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.continuousConvEnable = TRUE;
    adcUserConfig.resolutionMode = kAdcResolutionBitOf16;
    ADC16_DRV_Init(instance, &adcUserConfig);
    
    adcChnConfig.chnNum = ADC_CHANNEL_TEMPERATURE;
    adcChnConfig.diffEnable = FALSE;
    adcChnConfig.intEnable = FALSE;
    adcChnConfig.chnMux = kAdcChnMuxOfDefault;

    // Configure channel 0
    ADC16_DRV_ConfigConvChn(instance, CHANNEL_0, &adcChnConfig);
 
}

/*!*************************************************************************************************
\fn       int32_t APP_GetCurrentTempValue(void))
\brief    Calculate the current temperature
\return   int32_t
***************************************************************************************************/
int32_t APP_GetCurrentTempValue(void)
{
    uint32_t adcValue = 0;
    int32_t currentTemperature = 0;
    
    /* get current adc value */
    ADC16_DRV_WaitConvDone(ADC_0, CHANNEL_0);
    adcValue = ADC16_DRV_GetConvValueRAW(ADC_0, CHANNEL_0);
    
    /* compute temperature*/   
    currentTemperature = (int32_t)(DEFAULT_TEMP - ((int32_t)adcValue - (int32_t)gADCrTemp25) * 10000 / (int32_t)gADCr100m);
   
    currentTemperature -= TEMP_OFFSET;
    #if gUSBKW24D512Dongle
        /* TODO Temperature sensor on the dongle*/
        currentTemperature = 2400;
    #endif
    return currentTemperature;
}

/*!*************************************************************************************************
\fn     void* App_GetTempDataString(void)
\brief  Return post data.

\param  [in]    none

\return         return data to be send through post
***************************************************************************************************/
void* App_GetTempDataString
(
    void
)
{
#if USE_TEMPERATURE_SENSOR
    /* Compute temperature */
    int32_t temperature = APP_GetCurrentTempValue();
    uint8_t * sendTemperatureData = MEM_BufferAlloc(TEMP_BUFF_SIZE);
    if(NULL == sendTemperatureData)
    {
      return sendTemperatureData;
    }
    
    /* Clear data and reset buffers */
    FLib_MemSet(sendTemperatureData, 0, TEMP_BUFF_SIZE);
    
    /* Compute output */
    sprintf((char*)sendTemperatureData, "Temp:%i.%.02d", temperature/100, abs(temperature)%100);
    return sendTemperatureData;
#else
    return NULL;
#endif    
}
/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn    static void APP_TempSensorCalibrateParams(void)
\brief Private function: Set calibration params
\return       void
***************************************************************************************************/
static void APP_TempSensorCalibrateParams(void)
{
#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    adc16_calibration_param_t adcCalibraitionParam;
#endif
    adc16_user_config_t adcUserConfig;
    adc16_chn_config_t adcChnConfig;
    uint32_t bandgapValue = 0;  /*! ADC value of BANDGAP */
    uint32_t vdd = 0;           /*! VDD in mV */

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    /* Auto calibration */
    ADC16_DRV_GetAutoCalibrationParam(ADC_0, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC_0, &adcCalibraitionParam);
#endif

    /* Enable BANDGAP reference voltage */
    PMC_HAL_SetBandgapBufferCmd(PMC_BASE, true);

    /* Initialization ADC for 16bit resolution, hw trigger disabled.
     normal convert speed, VREFH/L as reference */
    ADC16_DRV_StructInitUserConfigDefault(&adcUserConfig);
    adcUserConfig.resolutionMode = kAdcResolutionBitOf16;
    adcUserConfig.continuousConvEnable = FALSE;
    //adcUserConfig.clkSrcMode = kAdcClkSrcOfAsynClk;
    ADC16_DRV_Init(ADC_0, &adcUserConfig);

#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    ADC16_DRV_EnableHwAverage(ADC_0, kAdcHwAverageCountOf32);
#endif 

    adcChnConfig.chnNum = ADC_CHANNEL_BANDGAP;
    adcChnConfig.diffEnable = false;
    adcChnConfig.intEnable = false;
    adcChnConfig.chnMux = kAdcChnMuxOfA;
    ADC16_DRV_ConfigConvChn(ADC_0, CHANNEL_0, &adcChnConfig);

    /* Wait for the conversion to be done */
    ADC16_DRV_WaitConvDone(ADC_0, CHANNEL_0);

    /* Get current ADC BANDGAP value */
    bandgapValue = ADC16_DRV_GetConvValueRAW(ADC_0, CHANNEL_0);
    bandgapValue = ADC16_DRV_ConvRAWData(bandgapValue, false, adcUserConfig.resolutionMode);

    /* ADC stop conversion */
    ADC16_DRV_PauseConv(ADC_0, CHANNEL_0);

    /* Get VDD value measured in mV: VDD = (ADC_MAX_SCALE x V_BG) / ADCR_BG */
    vdd = ADC_MAX_SCALE * V_BG / bandgapValue;
    /* Calibrate ADCR_TEMP25: ADCR_TEMP25 = ADC_MAX_SCALE x V_TEMP25 / VDD */
    gADCrTemp25 = ADC_MAX_SCALE * V_TEMP25 / vdd;
    /* ADCR_100M = ADC_MAX_SCALE x ADC_TEMP_SLOPE x 100 / VDD */
    gADCr100m = (ADC_MAX_SCALE * ADC_TEMP_SLOPE) / (vdd * 10);

#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    ADC16_DRV_DisableHwAverage(ADC_0);
#endif 

    /* Disable BANDGAP reference voltage */
    PMC_HAL_SetBandgapBufferCmd(PMC_BASE, false);
}

/*==================================================================================================
Private debug functions
==================================================================================================*/

