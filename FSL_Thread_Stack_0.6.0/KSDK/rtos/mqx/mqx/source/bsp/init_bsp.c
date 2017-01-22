/*HEADER**********************************************************************
*
* Copyright 2014 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*   This file contains the source functions for functions required to
*   specifically initialize the card.
*
*END************************************************************************/

#include <assert.h>
#include "user_config.h"
#include "mqx_inc.h"
#include "bsp.h"

#if BSPCFG_ENABLE_IO_SUBSYSTEM
    #include "nio.h"
    #include "nio_serial.h"
    #include "nio_tty.h"
    #include "nio_dummy.h"

    /* Choose default debug serial module */
    #if defined(BOARD_USE_UART)
        #define NIO_SERIAL_DEF_MODULE kNioSerialUart
    #elif defined(BOARD_USE_LPUART)
        #define NIO_SERIAL_DEF_MODULE kNioSerialLpuart
    #elif defined(BOARD_USE_LPSCI)
        #define NIO_SERIAL_DEF_MODULE kNioSerialLpsci
    #else
      #errorc Default serial module is unsupported or undefined.
    #endif

    /* Create initialization structure for default serial module */
    const NIO_SERIAL_INIT_DATA_STRUCT nio_serial_default_init =
    {
        .SERIAL_INSTANCE     = BOARD_DEBUG_UART_INSTANCE,
        .BAUDRATE            = 115200,
        .PARITY_MODE         = kNioSerialParityDisabled,
        .STOPBIT_COUNT       = kNioSerialOneStopBit,
        .BITCOUNT_PERCHAR    = 8,
        .RXTX_PRIOR          = 4,
        .MODULE              = NIO_SERIAL_DEF_MODULE,
    /* Always second clock source configuration is used. It can be :kClockLpuartSrcPllFllSel, kClockLpsciSrcPllFllSel, kClockLpuartSrcIrc48M)*/
    /* For UART this is dummy value */
        .CLK_SOURCE          = 1, 
    };

#endif /* BSPCFG_ENABLE_IO_SUBSYSTEM */

//#if (__MPU_PRESENT == 1)
#ifdef HW_MPU_INSTANCE_COUNT
  #include <fsl_mpu_hal.h>
  #include <fsl_mpu_driver.h>
#endif
//#endif

hwtimer_t systimer;     /* System timer handle */
_mem_pool_id _BSP_sram_pool;
static uint32_t _bsp_get_hwticks(void *param);

/** Workaround for link error message with Keil. It complains that DbgConsole_Init
  * symbol is undefined (referenced by hardware_init.o, even thought it is not used at all
  * and after link processs the DbgConsole_Init is listed in "removed unused symbols".
 */
#ifdef __CC_ARM
#pragma weak DbgConsole_Init
#include "fsl_debug_console.h"
debug_console_status_t DbgConsole_Init(
  uint32_t uartInstance, uint32_t baudRate, debug_console_device_type_t device)
  {return kStatus_DEBUGCONSOLE_InvalidDevice;}
#endif


/*!
* \cond DOXYGEN_PRIVATE
* \brief Pre initialization - initializing requested modules for basic run of MQX.
*/
int _bsp_pre_init(void)
{
    uint32_t result;

/******************************************************************************
         Init gpio platform pins for LEDs, setup board clock source
******************************************************************************/
/* Macro PEX_MQX_KSDK used by PEX team */
#ifndef PEX_MQX_KSDK
    hardware_init();
    /* Configure PINS for default UART instance */
  #if defined(BOARD_USE_LPSCI)
    configure_lpsci_pins(BOARD_DEBUG_UART_INSTANCE);
  #elif defined(BOARD_USE_LPUART)
    configure_lpuart_pins(BOARD_DEBUG_UART_INSTANCE);
  #elif defined(BOARD_USE_UART)
    configure_uart_pins(BOARD_DEBUG_UART_INSTANCE);
  #else
    #errorc Default serial module is unsupported or undefined.
  #endif
#endif

    /* Set the CPU type */
    _mqx_set_cpu_type(MQX_CPU);

#if MQX_EXIT_ENABLED
    extern void  _bsp_exit_handler(void);
    /* Set the bsp exit handler, called by _mqx_exit */
    _mqx_set_exit_handler(_bsp_exit_handler);
#endif


#if MQXCFG_ALLOCATOR
    /* Memory splitter - prevent accessing both ram banks in one instruction */
    /* This is workaround and will be solved with [KPSDK-2733]. This fix will not work with TLSF allocators*/
    //_mem_alloc_at(0, (void*)0x20000000);
#endif

    result = _psp_int_init(BSP_FIRST_INTERRUPT_VECTOR_USED, BSP_LAST_INTERRUPT_VECTOR_USED);
    if (result != MQX_OK) {
        return result;
    }


/******************************************************************************
                        Init MQX tick timer
******************************************************************************/
    /* Initialize , set and run system hwtimer */
    result = HWTIMER_SYS_Init(&systimer, &BSP_SYSTIMER_DEV, BSP_SYSTIMER_ID, BSP_SYSTIMER_ISR_PRIOR, NULL);
    if (kStatus_OSA_Success != result) {
        return MQX_INVALID_POINTER;
    }
    result = HWTIMER_SYS_SetFreq(&systimer, BSP_SYSTIMER_SRC_CLK, BSP_ALARM_FREQUENCY);
    if (kStatus_OSA_Success != result) {
        HWTIMER_SYS_Deinit(&systimer);
        return MQX_INVALID_POINTER;
    }
    result = HWTIMER_SYS_RegisterCallback(&systimer,(hwtimer_callback_t)_time_notify_kernel, NULL);
    if (kStatus_OSA_Success != result) {
        HWTIMER_SYS_Deinit(&systimer);
        return MQX_INVALID_POINTER;
    }
    result = HWTIMER_SYS_Start(&systimer);
    if (kStatus_OSA_Success != result) {
        HWTIMER_SYS_Deinit(&systimer);
        return MQX_INVALID_POINTER;
    }
    /* Initialize the system ticks */
    _time_set_ticks_per_sec(BSP_ALARM_FREQUENCY);
    _time_set_hwticks_per_tick(HWTIMER_SYS_GetModulo(&systimer));
    _time_set_hwtick_function(_bsp_get_hwticks, (void *)NULL);

    return MQX_OK;
}
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Initialization - called from init task, usually for io initialization.
 */
int _bsp_init(void)
{
    uint32_t result = MQX_OK;

    /** Cache settings **/
    _DCACHE_ENABLE(0);
    _ICACHE_ENABLE(0);

    /* Disable MPU if present on device. This dirty hack is done to workaround missing MPU_DRV_Disable() function in KSDK */
#ifdef HW_MPU_INSTANCE_COUNT
    for(int i = 0; i < HW_MPU_INSTANCE_COUNT; i++)
    {
        MPU_HAL_Disable(g_mpuBaseAddr[i]);
    }
#endif

/******************************************************************************
    Install interrupts for UART driver and setup debug console
******************************************************************************/


#if BSPCFG_ENABLE_IO_SUBSYSTEM
    void *res;
    int fd;

    /* Install serial driver for default input and output */
    res = _nio_dev_install(BSP_DEFAULT_IO_CHANNEL, &nio_serial_dev_fn, (void*) &nio_serial_default_init);
    assert(NULL != res);

    /* Install and open dummy drivers for stdin, stdou and stderr. */
    res = _nio_dev_install("dummy:",&nio_dummy_dev_fn, (NULL));
    assert(NULL != res);

    fd = open("dummy:", 0);  // 0 - stdin
    assert(fd == 0);
    fd = open("dummy:", 0);  // 1 - stdout
    assert(fd == 1);
    fd = open("dummy:", 0);  // 2 - stderr
    assert(fd == 2);

    /* Instal and set tty driver */
    res = _nio_dev_install("tty:", &nio_tty_dev_fn, (void*)&(NIO_TTY_INIT_DATA_STRUCT){BSP_DEFAULT_IO_CHANNEL, 0});
    assert(NULL != res);

    close(0);
    fd = open("tty:", NIO_TTY_FLAGS_EOL_RN | NIO_TTY_FLAGS_ECHO);  // 0 - stdin
    assert(fd == 0);

    close(1);
    fd = open("tty:", NIO_TTY_FLAGS_EOL_RN | NIO_TTY_FLAGS_ECHO);  // 1 - stdout
    assert(fd == 1);

    close(2);
    fd = open("tty:", NIO_TTY_FLAGS_EOL_RN | NIO_TTY_FLAGS_ECHO);  // 2 - stderr
    assert(fd == 2);

    /* Dummy driver is not needed any more, therefore should be uninstalled */
    _nio_dev_uninstall("dummy:");

    /* Avoid warnings on fd and res variables in release targets */
    (void)fd;
    (void)res;


#if (defined(BOARD_SDHC_INSTANCE) && !defined(PEX_MQX_KSDK))
    /* SDHC module */
    configure_sdhc_pins(BOARD_SDHC_INSTANCE);
#endif

#else /* BSPCFG_ENABLE_IO_SUBSYSTEM */
#if  !defined(PEX_MQX_KSDK)
    dbg_uart_init();
#endif
#endif /* BSPCFG_ENABLE_IO_SUBSYSTEM */
/******************************************************************************
    Initialize ENET and install interrupts
******************************************************************************/
/* Check if enet is available */
#if (defined(BOARD_ENET_INSTANCE) && !defined(PEX_MQX_KSDK))
{
    /* ENET module */
    configure_enet_pins(BOARD_ENET_INSTANCE);

#if 0 //DM Done by ENET driver
    /* Open ENET clock gate */
    CLOCK_SYS_EnableEnetClock(0);
#endif
    /* Select the ptp timer  outclk */
    CLOCK_HAL_SetTimeSrc(SIM_BASE, 0, kClockTimeSrcOsc0erClk);

    extern void ENET_DRV_TxIRQHandler(void);
    extern void ENET_DRV_RxIRQHandler(void);
    extern void ENET_DRV_TsIRQHandler(void);
    INT_ISR_FPTR ret_temp;

    ret_temp = _int_install_isr(ENET_Transmit_IRQn,(INT_ISR_FPTR)ENET_DRV_TxIRQHandler, NULL);
    if(NULL == ret_temp)
    {
      return MQX_ERROR;
    }
    NVIC_SetPriority (ENET_Transmit_IRQn, BSP_MACNET0_INT_TX_LEVEL);
    NVIC_EnableIRQ(ENET_Transmit_IRQn);

    ret_temp = _int_install_isr(ENET_Receive_IRQn,(INT_ISR_FPTR)ENET_DRV_RxIRQHandler, NULL);
    if(NULL == ret_temp)
    {
      return MQX_ERROR;
    }
    NVIC_SetPriority (ENET_Receive_IRQn, BSP_MACNET0_INT_RX_LEVEL);
    NVIC_EnableIRQ(ENET_Receive_IRQn);

#if FSL_FEATURE_ENET_SUPPORT_PTP
    ret_temp = _int_install_isr(ENET_1588_Timer_IRQn,(INT_ISR_FPTR)ENET_DRV_TsIRQHandler, NULL);
    if(NULL == ret_temp)
    {
      return MQX_ERROR;
    }
    NVIC_SetPriority (ENET_1588_Timer_IRQn, BSP_MACNET0_INT_RX_LEVEL);
    NVIC_EnableIRQ(ENET_1588_Timer_IRQn);
#endif
}
#endif

    return result;
}
/*! \endcond */

/**FUNCTION********************************************************************
*
* Function Name    : _bsp_exit_handler
* Returned Value   : void
* Comments         :
*   This function is called when MQX exits
*
*END**************************************************************************/
/*!
 * \cond DOXYGEN_PRIVATE
 */
void _bsp_exit_handler(void)
{
    _DCACHE_DISABLE();
    _ICACHE_DISABLE();
}
/*! \endcond */

/**FUNCTION********************************************************************
 *
 * Function Name    : _bsp_get_hwticks
 * Returned Value   : none
 * Comments         :
 *    This function returns the number of hw ticks that have elapsed
 * since the last interrupt
 *
 *END**************************************************************************/

static uint32_t _bsp_get_hwticks(void *param) {
    hwtimer_time_t time;      //struct for storing time
    HWTIMER_SYS_GetTime(&systimer, &time);
    return time.subTicks;
}
