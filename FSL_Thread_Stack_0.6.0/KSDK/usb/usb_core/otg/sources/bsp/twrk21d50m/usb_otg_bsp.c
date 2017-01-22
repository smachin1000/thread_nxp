/**HEADER********************************************************************
*
* Copyright (c) 2013 - 2014 Freescale Semiconductor;
* All Rights Reserved
*
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: usb_otg_bsp.c$
* $Version :
* $Date    :
*
* Comments:
*
*
*****************************************************************************/
#include "adapter.h"
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_i2c_shared_function.h"
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
#include "MK22F51212.h"
#endif

extern uint8_t soc_get_usb_vector_number(uint8_t controller_id);

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM) || (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include "usb_otg.h"
#define BSP_USB_INT_LEVEL                (4)
#define BSP_USB_OTG_MAX3353_INT_LEVEL    (4)
/* TODO: Move this structure to other place */
/* struct contains max3353 init params */
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
#define KHCI_BASE_PTR                USB0_BASE_PTR      /* KHCI_BASE_PTR */
#define KHCI_VECTOR                  INT_USB0           /* KHCI_VECTOR */
#define MAX_3353_INT_PORT            PORTA_BASE_PTR     /* MAX_3353_INT_PORT */
#define MAX3353_VECTOR               INT_PORTA          /* MAX3353_VECTOR */
#define I2C_CHANNEL                  "i2c1:"            /* I2C Channel */
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
#define KHCI_BASE_PTR                USB0_BASE          /* KHCI_BASE_PTR */
#define KHCI_VECTOR                  (USB0_IRQn + 16)   /* KHCI_VECTOR */
#define MAX_3353_INT_PORT            HW_PORTA         /* MAX_3353_INT_PORT */
#define MAX3353_VECTOR               (PORTA_IRQn + 16)        /* MAX3353_VECTOR */
#define I2C_CHANNEL                  0                  /* I2C Channel */
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
extern void I2C1_IRQHandler(void);
#define KHCI_BASE_PTR                USB0_BASE          /* KHCI_BASE_PTR */
#define KHCI_VECTOR                  USB0_IRQn          /* KHCI_VECTOR */
#define MAX_3353_INT_PORT            PORTA_BASE         /* MAX_3353_INT_PORT */
#define MAX3353_VECTOR               PORTA_IRQn         /* MAX3353_VECTOR */
#define I2C_CHANNEL                  1                  /* I2C Channel */
#endif
#define MAX3353_INT_PIN              16 /*MAX3353_INT_PIN*/
static const usb_khci_otg_int_struct_t g_khci0_otg_init_param =
{
    (void*)KHCI_BASE_PTR,
    KHCI_VECTOR,
    BSP_USB_INT_LEVEL ,
};

static const usb_otg_max3353_init_struct_t g_otg_max3353_init_param =
{
    (void*)MAX_3353_INT_PORT,
    MAX3353_INT_PIN,
    MAX3353_VECTOR,
    BSP_USB_OTG_MAX3353_INT_LEVEL,
    I2C_CHANNEL,
    0x2C
};
/* Private functions definitions *********************************************/
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : max3353_i2c1_IRQHandler
* Returned Value   :
* Comments         : max3353_i2c1_IRQHandler
*
*
*END*----------------------------------------------------------------------*/
#if defined(FSL_RTOS_MQX)
void max3353_i2c1_IRQHandler(void)
#else
void I2C1_IRQHandler(void)
#endif
{
    I2C_DRV_IRQHandler(1);
}
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_usb_otg_max3353_pin_int_clear
* Returned Value   : none
* Comments         :
*    This function clears the pin interrupt flag associated with the max3353 interrupt pin
*
*END*----------------------------------------------------------------------*/
void _bsp_usb_otg_max3353_clear_pin_int_flag()
{
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
    if (PORT_ISFR_REG(MAX_3353_INT_PORT) & (1<<MAX3353_INT_PIN))
    {
        PORT_ISFR_REG(MAX_3353_INT_PORT) |= 1<<MAX3353_INT_PIN;
    }
#else
     if (HW_PORT_ISFR_RD(MAX_3353_INT_PORT) & (1<<MAX3353_INT_PIN))
    {
        HW_PORT_ISFR_SET(MAX_3353_INT_PORT,1<<MAX3353_INT_PIN);
    }
#endif
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_usb_otg_max3353_set_pin_int
* Returned Value   : none
* Comments         :
*    This function enables/disables the pin interrupt associated with the max3353 interrupt pin
*
*END*----------------------------------------------------------------------*/
void _bsp_usb_otg_max3353_set_pin_int
(
    bool level,
    bool enable
)
{
#if ((OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)||(OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK))
    if (enable)
    {
        if (level)/* interrupt is triggered  by low level */
        {
            HW_PORT_PCRn_WR(MAX_3353_INT_PORT,MAX3353_INT_PIN,0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x08));
        }
        else/* interrupt is triggered by falling edge */
        {
            HW_PORT_PCRn_WR(MAX_3353_INT_PORT,MAX3353_INT_PIN,0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x0A));
        }
    }
    else
    {
        HW_PORT_PCRn_WR(MAX_3353_INT_PORT,MAX3353_INT_PIN,0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x00));
    }
#else
    if (enable)
    {
        if (level)/* interrupt is triggered  by low level */
        {
           PORT_PCR_REG(MAX_3353_INT_PORT,MAX3353_INT_PIN) |=  0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x08);
        }
        else/* interrupt is triggered by falling edge */
        {
            PORT_PCR_REG(MAX_3353_INT_PORT,MAX3353_INT_PIN) |=  0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x0A);
        }
    }
    else
    {
       PORT_PCR_REG(MAX_3353_INT_PORT,MAX3353_INT_PIN) |=  0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x00);
    }
#endif
}

void* bsp_usb_otg_get_init_param
(
    uint8_t controller_id
)
{
    if (controller_id == USB_CONTROLLER_KHCI_0)
    {
        return (void*)(&g_khci0_otg_init_param);
    }
    else
    {
        return NULL;
    }
}

void* bsp_usb_otg_get_peripheral_init_param
(
    uint8_t peripheral_id
)
{
    if (peripheral_id == USB_OTG_PERIPHERAL_MAX3353)
    {
        return (void*)(&g_otg_max3353_init_param);
    }
    else
    {
        return NULL;
    }
}

static int32_t bsp_usb_otg_io_init
(
   int32_t i
)
{
    if (i == 0)
    {
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
        /* USB clock divider */
        CLOCK_SYS_SetUsbfsDiv(i, 1U, 0U);

        /* PLL/FLL selected as CLK source */
        CLOCK_SYS_SetUsbfsSrc(i, kClockUsbfsSrcPllFllSel);
        CLOCK_SYS_SetPllfllSel(kClockPllFllSelPll);

        /* USB Clock Gating */
        CLOCK_SYS_EnableUsbfsClock(i);
        /* Souce the P5V0_K22_USB. Set PTC9 to high */
        BW_PORT_PCRn_MUX(PORTC_BASE, 9, 1); /* GPIO mux */
        HW_GPIO_PDDR_SET(PTC_BASE, 1<<9);        /* Set output */
        HW_GPIO_PSOR_WR(PTC_BASE, HW_GPIO_PSOR_RD(PTC_BASE) | 1<<9);   /* Output high */
#endif
    }
    else
    {
        return -1; /* unknow controller */
    }
    return 0;
}

int32_t bsp_usb_otg_init(uint8_t controller_id)
{
    int32_t result = 0;

    result = bsp_usb_otg_io_init(controller_id);

    if (result != 0)
    {
        return result;
    }

    if (0 == controller_id)
    {
#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
        /* Configure enable USB regulator for device */
        SIM_SOPT1CFG |= (SIM_SOPT1CFG_URWE_MASK);
        SIM_SOPT1 |= (SIM_SOPT1_USBREGEN_MASK);

        /* reset USB CTRL register */
        USB0_USBCTRL = 0;

        /* Enable internal pull-up resistor */
        USB0_CONTROL = (USB_CONTROL_DPPULLUPNONOTG_MASK);
        USB0_USBTRC0 |= (0x40); /* Software must set this bit to 1 */
        /* setup interrupt */
        OS_intr_init(soc_get_usb_vector_number(0), BSP_USB_INT_LEVEL, 0, TRUE);
#elif (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
        /* Configure enable USB regulator for device */
        HW_SIM_SOPT1CFG_SET(SIM_BASE, SIM_SOPT1CFG_URWE_MASK);
        HW_SIM_SOPT1_SET(SIM_BASE, SIM_SOPT1_USBREGEN_MASK);

        /* reset USB CTRL register */
        HW_USB_USBCTRL_WR(USB0_BASE, 0);

        /* Enable internal pull-up resistor */
        HW_USB_CONTROL_WR(USB0_BASE, USB_CONTROL_DPPULLUPNONOTG_MASK);
        HW_USB_USBTRC0_SET(USB0_BASE, 0x40); /* Software must set this bit to 1 */
        /* setup interrupt */
        OS_intr_init((IRQn_Type)soc_get_usb_vector_number(0), BSP_USB_INT_LEVEL, 0, TRUE);

        /* install i2c interrupt for MQX */
#if (defined (FSL_RTOS_MQX))
    /* install interrupt for i2c0 */
    OS_install_isr(I2C1_IRQn, (void (*)(void))max3353_i2c1_IRQHandler, &otg_max3353_call_ptr->init_param_ptr->channel);
#else
    OS_install_isr(I2C1_IRQn, (void (*)(void))I2C1_IRQHandler, &otg_max3353_call_ptr->init_param_ptr->channel);
#endif
        /* install i2c interrupt for FreeRTOS */
#if defined (FSL_RTOS_FREE_RTOS)
    NVIC_SetPriority(I2C1_IRQn,3);
#endif
#endif
    }
    else
    {
        /* unknown controller */
        result = -1;
    }
    return result;
}
#endif
/* EOF */
