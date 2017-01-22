/*HEADER**********************************************************************
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
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
 *
 *END************************************************************************/

#include <stdarg.h>
#include <stdint.h>
#include <assert.h>
#include <fsl_os_abstraction.h>
/* Required to get info about available serial modules */
#include <fsl_device_registers.h>
/* The only one function from mqx api is _int_install_isr. The  reason is better performance and code size. */
#include "mqx.h"
#include "nio_serial.h"
#include "nio.h"
#include "ioctl.h"
#include "errno.h"

#if defined(FSL_FEATURE_UART_FIFO_SIZE)
  #include <fsl_uart_driver.h>
  extern void UART_DRV_IRQHandler(uint32_t instance);
#endif
#if defined(FSL_FEATURE_LPUART_FIFO_SIZE)
  #include <fsl_lpuart_driver.h>
  extern void LPUART_DRV_IrqHandler(uint32_t instance);
#endif
#if defined(FSL_FEATURE_LPSCI_FIFO_SIZE)
  #include <fsl_lpsci_driver.h>
  extern void LPSCI_DRV_IRQHandler(uint32_t instance);
#endif

#if defined ( __IAR_SYSTEMS_ICC__ )
/* MISRA C 2004 rule 20.5 suppress: Error[Pm101]: The error indicator errno shall not be used */
_Pragma ("diag_suppress= Pm101")
#endif

static int nio_serial_open(void *dev_context, const char *dev_name, int flags, void **fp_context);
static int nio_serial_read(void *dev_context, void *fp_context, void *buf, size_t nbytes);
static int nio_serial_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes);
static int nio_serial_ioctl(void *dev_context, void *fp_context, unsigned long int request, va_list ap);
static int nio_serial_close(void *dev_context, void *fp_context);
static int nio_serial_init(void *init_data, void **dev_context);
static int nio_serial_deinit(void *dev_context);

const NIO_DEV_FN_STRUCT nio_serial_dev_fn =
{
    .OPEN = nio_serial_open,
    .READ = nio_serial_read,
    .WRITE = nio_serial_write,
    .LSEEK = NULL,
    .IOCTL = nio_serial_ioctl,
    .CLOSE = nio_serial_close,
    .INIT = nio_serial_init,
    .DEINIT = nio_serial_deinit,
};

typedef struct
{
    mutex_t rlock;
    mutex_t wlock;
    void * serial_state;
    uint32_t instance;
    nio_serial_module_type_t module;

} NIO_SERIAL_DEV_CONTEXT_STRUCT;

#if defined(FSL_FEATURE_UART_FIFO_SIZE)
/* Local function to set up UART user config. */
static int get_uart_config (NIO_SERIAL_INIT_DATA_STRUCT *init, uart_user_config_t * uartConfig)
{
    uartConfig->baudRate = init->BAUDRATE;
    switch (init->BITCOUNT_PERCHAR)
    {
      case 8:
        uartConfig->bitCountPerChar = kUart8BitsPerChar;
        break;
      case 9:
        uartConfig->bitCountPerChar = kUart9BitsPerChar;
        break;
      default:
        return -1;
    }
    switch (init->PARITY_MODE)
    {
      case kNioSerialParityDisabled:
        uartConfig->parityMode = kUartParityDisabled;
        break;
      case kNioSerialParityEven:
        uartConfig->parityMode = kUartParityEven;
        break;
      case kNioSerialParityOdd:
        uartConfig->parityMode = kUartParityOdd;
        break;
      default:
        return -1;
    }
    switch (init->STOPBIT_COUNT)
    {
      case kNioSerialOneStopBit:
        uartConfig->stopBitCount = kUartOneStopBit;
        break;
      case kNioSerialTwoStopBit:
        uartConfig->stopBitCount = kUartTwoStopBit;
        break;
      default:
        return -1;
    }

    return 0;
}
#endif
#if defined(FSL_FEATURE_LPUART_FIFO_SIZE)
/* Local function to set up LPUART user config. */
static int get_lpuart_config (NIO_SERIAL_INIT_DATA_STRUCT *init, lpuart_user_config_t * lpuartConfig)
{
    lpuartConfig->clockSource = (clock_lpuart_src_t)init->CLK_SOURCE;
    lpuartConfig->baudRate = init->BAUDRATE;
    switch (init->BITCOUNT_PERCHAR)
    {
      case 8:
        lpuartConfig->bitCountPerChar = kLpuart8BitsPerChar;
        break;
      case 9:
        lpuartConfig->bitCountPerChar = kLpuart9BitsPerChar;
        break;
      case 10:
        lpuartConfig->bitCountPerChar = kLpuart10BitsPerChar;
        break;
      default:
        return -1;
    }
    switch (init->PARITY_MODE)
    {
      case kNioSerialParityDisabled:
        lpuartConfig->parityMode = kLpuartParityDisabled;
        break;
      case kNioSerialParityEven:
        lpuartConfig->parityMode = kLpuartParityEven;
        break;
      case kNioSerialParityOdd:
        lpuartConfig->parityMode = kLpuartParityOdd;
        break;
      default:
        return -1;
    }
    switch (init->STOPBIT_COUNT)
    {
      case kNioSerialOneStopBit:
        lpuartConfig->stopBitCount = kLpuartOneStopBit;
        break;
      case kNioSerialTwoStopBit:
        lpuartConfig->stopBitCount = kLpuartTwoStopBit;
        break;
      default:
        return -1;
    }

    return 0;
}
#endif
#if defined(FSL_FEATURE_LPSCI_FIFO_SIZE)
/* Local function to set up LPSCI user config. */
static int get_lpsci_config (NIO_SERIAL_INIT_DATA_STRUCT *init, lpsci_user_config_t * lpsciConfig)
{
    lpsciConfig->clockSource = (clock_lpsci_src_t)init->CLK_SOURCE;
    lpsciConfig->baudRate = init->BAUDRATE;
    switch (init->BITCOUNT_PERCHAR)
    {
      case 8:
        lpsciConfig->bitCountPerChar = kLpsci8BitsPerChar;
        break;
      case 9:
        lpsciConfig->bitCountPerChar = kLpsci9BitsPerChar;
        break;
      default:
        return -1;
    }
    switch (init->PARITY_MODE)
    {
      case kNioSerialParityDisabled:
        lpsciConfig->parityMode = kLpsciParityDisabled;
        break;
      case kNioSerialParityEven:
        lpsciConfig->parityMode = kLpsciParityEven;
        break;
      case kNioSerialParityOdd:
        lpsciConfig->parityMode = kLpsciParityOdd;
        break;
      default:
        return -1;
    }
    switch (init->STOPBIT_COUNT)
    {
      case kNioSerialOneStopBit:
        lpsciConfig->stopBitCount = kLpsciOneStopBit;
        break;
      case kNioSerialTwoStopBit:
        lpsciConfig->stopBitCount = kLpsciTwoStopBit;
        break;
      default:
        return -1;
    }

    return 0;
}
#endif



static int nio_serial_open(void *dev_context, const char *dev_name, int flags, void **fp_context)
{
    return 0;
}

static int nio_serial_read(void *dev_context, void *fp_context, void *buf, size_t nbytes)
{
    NIO_SERIAL_DEV_CONTEXT_STRUCT *serial_dev_context = (NIO_SERIAL_DEV_CONTEXT_STRUCT *)dev_context;
    size_t left = nbytes;
    osa_status_t status;

    status = OSA_MutexLock(&serial_dev_context->rlock, OSA_WAIT_FOREVER);
    assert (status == kStatus_OSA_Success);
    switch (serial_dev_context->module)
    {
#if defined(FSL_FEATURE_UART_FIFO_SIZE)
      case kNioSerialUart:
      {
          uart_status_t status;
          uart_state_t *uart_state = (uart_state_t *) serial_dev_context->serial_state;
          status = UART_DRV_ReceiveDataBlocking(serial_dev_context->instance, buf, nbytes, OSA_WAIT_FOREVER);
          assert(status == kStatus_UART_Success);
          (void)status; /* Avoid warning with unused variable */
          left = uart_state->rxSize;
          break;
      }
#endif
#if defined(FSL_FEATURE_LPUART_FIFO_SIZE)
      case kNioSerialLpuart:
      {
          lpuart_status_t status;
          lpuart_state_t *lpuart_state = (lpuart_state_t *) serial_dev_context->serial_state;
          status = LPUART_DRV_ReceiveDataBlocking(serial_dev_context->instance, buf, nbytes, OSA_WAIT_FOREVER);
          assert(status == kStatus_LPUART_Success);
          (void)status; /* Avoid warning with unused variable */
          left = lpuart_state->rxSize;
          break;
      }
#endif
#if defined(FSL_FEATURE_LPSCI_FIFO_SIZE)
      case kNioSerialLpsci:
      {
          lpsci_status_t status;
          lpsci_state_t *lpsci_state = (lpsci_state_t *) serial_dev_context->serial_state;
          status = LPSCI_DRV_ReceiveDataBlocking(serial_dev_context->instance, buf, nbytes, OSA_WAIT_FOREVER);
          assert(status == kStatus_LPSCI_Success);
          (void)status; /* Avoid warning with unused variable */
          left = lpsci_state->rxSize;
          break;
      }
#endif
      default:
          return -EINVAL;
    }
    assert(left <= nbytes);
    status = OSA_MutexUnlock(&serial_dev_context->rlock);
    assert (status == kStatus_OSA_Success);
    (void)status; /* Avoid warning with unused variable */
    return (nbytes - left);
}

static int nio_serial_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes)
{

    NIO_SERIAL_DEV_CONTEXT_STRUCT *serial_dev_context = (NIO_SERIAL_DEV_CONTEXT_STRUCT *)dev_context;
    size_t left = nbytes;
    osa_status_t status;

    status = OSA_MutexLock(&serial_dev_context->wlock, OSA_WAIT_FOREVER);
    assert (status == kStatus_OSA_Success);
    switch (serial_dev_context->module)
    {
#if defined(FSL_FEATURE_UART_FIFO_SIZE)
      case kNioSerialUart:
      {
          uart_status_t status;
          uart_state_t *uart_state = (uart_state_t *) serial_dev_context->serial_state;
          status = UART_DRV_SendDataBlocking(serial_dev_context->instance, buf, nbytes, OSA_WAIT_FOREVER);
          assert(status == kStatus_UART_Success);
          (void)status; /* Avoid warning with unused variable */
          left = uart_state->txSize;
          break;
      }
#endif
#if defined(FSL_FEATURE_LPUART_FIFO_SIZE)
      case kNioSerialLpuart:
      {
          lpuart_status_t status;
          lpuart_state_t *lpuart_state = (lpuart_state_t *) serial_dev_context->serial_state;
          status = LPUART_DRV_SendDataBlocking(serial_dev_context->instance, buf, nbytes, OSA_WAIT_FOREVER);
          assert(status == kStatus_LPUART_Success);
          (void)status; /* Avoid warning with unused variable */
          left = lpuart_state->txSize;
          break;
      }
#endif
#if defined(FSL_FEATURE_LPSCI_FIFO_SIZE)
      case kNioSerialLpsci:
      {
          lpsci_status_t status;
          lpsci_state_t *lpsci_state = (lpsci_state_t *) serial_dev_context->serial_state;
          status = LPSCI_DRV_SendDataBlocking(serial_dev_context->instance, buf, nbytes, OSA_WAIT_FOREVER);
          assert(status == kStatus_LPSCI_Success);
          (void)status; /* Avoid warning with unused variable */
          left = lpsci_state->txSize;
          break;
      }
#endif
      default:
          return -EINVAL;
    }
    assert(left <= nbytes);
    status = OSA_MutexUnlock(&serial_dev_context->wlock);
    assert (status == kStatus_OSA_Success);
    (void)status; /* Avoid warning with unused variable */
    return (nbytes - left);

}

static int nio_serial_ioctl(void *dev_context, void *fp_context, unsigned long int request, va_list ap) {

    NIO_SERIAL_DEV_CONTEXT_STRUCT *serial_dev_context = (NIO_SERIAL_DEV_CONTEXT_STRUCT *)dev_context;

    switch (request) {
    case IOCTL_ABORT:
        switch (serial_dev_context->module)
        {
#if defined(FSL_FEATURE_UART_FIFO_SIZE)
          case kNioSerialUart:
              UART_DRV_AbortSendingData(serial_dev_context->instance);
              UART_DRV_AbortReceivingData(serial_dev_context->instance);
              break;
#endif
#if defined(FSL_FEATURE_LPUART_FIFO_SIZE)
          case kNioSerialLpuart:
              LPUART_DRV_AbortSendingData(serial_dev_context->instance);
              LPUART_DRV_AbortReceivingData(serial_dev_context->instance);
              break;
#endif
#if defined(FSL_FEATURE_LPSCI_FIFO_SIZE)
          case kNioSerialLpsci:
              LPSCI_DRV_AbortSendingData(serial_dev_context->instance);
              LPSCI_DRV_AbortReceivingData(serial_dev_context->instance);
              break;
#endif
          default:
              return -EINVAL;
        }
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int nio_serial_close(void *dev_context, void *fp_context)
{
    return 0;
}

static int nio_serial_init(void *init_data, void **dev_context)
{
    NIO_SERIAL_DEV_CONTEXT_STRUCT *serial_dev_context = (NIO_SERIAL_DEV_CONTEXT_STRUCT *)dev_context;
    NIO_SERIAL_INIT_DATA_STRUCT *init = (NIO_SERIAL_INIT_DATA_STRUCT*)init_data;
    assert(init->SERIAL_INSTANCE < HW_UART_INSTANCE_COUNT);
    int32_t IRQNumber;



    serial_dev_context = (NIO_SERIAL_DEV_CONTEXT_STRUCT*) OSA_MemAlloc(sizeof(NIO_SERIAL_DEV_CONTEXT_STRUCT));
    if (NULL == serial_dev_context)
    {
        return -ENOMEM;
    }

    switch (init->MODULE)
    {
#if defined(FSL_FEATURE_UART_FIFO_SIZE)
      case kNioSerialUart:
      {
          uart_user_config_t uartConfig;
          if (0 != get_uart_config(init_data, &uartConfig))
          {
              OSA_MemFree(serial_dev_context);
              return -ENXIO;
          }

          serial_dev_context->serial_state = OSA_MemAlloc(sizeof(uart_state_t));
          if (NULL == serial_dev_context->serial_state)
          {
              OSA_MemFree(serial_dev_context);
              return -ENOMEM;
          }
          /* SDK HAL init */
          if ( kStatus_UART_Success != UART_DRV_Init(init->SERIAL_INSTANCE, (uart_state_t *) serial_dev_context->serial_state, &uartConfig))
          {
              OSA_MemFree(serial_dev_context->serial_state);
              OSA_MemFree(serial_dev_context);
              return -ENXIO;
          }
          IRQNumber = g_uartRxTxIrqId[init->SERIAL_INSTANCE];
          _int_install_isr(IRQNumber, (INT_ISR_FPTR)UART_DRV_IRQHandler, (void*)init->SERIAL_INSTANCE);
          break;
      }
#endif
#if defined(FSL_FEATURE_LPUART_FIFO_SIZE)
      case kNioSerialLpuart:
      {
          lpuart_user_config_t lpuartConfig;
          if (0 != get_lpuart_config(init_data, &lpuartConfig))
          {
              OSA_MemFree(serial_dev_context);
              return -ENXIO;
          }

          serial_dev_context->serial_state = OSA_MemAlloc(sizeof(lpuart_state_t));
          if (NULL == serial_dev_context->serial_state)
          {
              OSA_MemFree(serial_dev_context);
              return -ENOMEM;
          }
          /* SDK HAL init */
          if ( kStatus_LPUART_Success != LPUART_DRV_Init(init->SERIAL_INSTANCE, (lpuart_state_t *) serial_dev_context->serial_state, &lpuartConfig))
          {
              OSA_MemFree(serial_dev_context->serial_state);
              OSA_MemFree(serial_dev_context);
              return -ENXIO;
          }

          IRQNumber = g_lpuartRxTxIrqId[init->SERIAL_INSTANCE];
          _int_install_isr(IRQNumber, (INT_ISR_FPTR)LPUART_DRV_IrqHandler, (void*)init->SERIAL_INSTANCE);
          break;
      }
#endif
#if defined(FSL_FEATURE_LPSCI_FIFO_SIZE)
      case kNioSerialLpsci:
      {
          lpsci_user_config_t lpsciConfig;
          if (0 != get_lpsci_config(init_data, &lpsciConfig))
          {
              OSA_MemFree(serial_dev_context);
              return -ENXIO;
          }

          serial_dev_context->serial_state = OSA_MemAlloc(sizeof(lpsci_state_t));
          if (NULL == serial_dev_context->serial_state)
          {
              OSA_MemFree(serial_dev_context);
              return -ENOMEM;
          }
          /* SDK HAL init */
          if ( kStatus_LPSCI_Success != LPSCI_DRV_Init(init->SERIAL_INSTANCE, (lpsci_state_t *) serial_dev_context->serial_state, &lpsciConfig))
          {
              OSA_MemFree(serial_dev_context->serial_state);
              OSA_MemFree(serial_dev_context);
              return -ENXIO;
          }

          IRQNumber = g_lpsciRxTxIrqId[init->SERIAL_INSTANCE];
          _int_install_isr(IRQNumber, (INT_ISR_FPTR)LPSCI_DRV_IRQHandler, (void*)init->SERIAL_INSTANCE);
          break;
      }
#endif
      default:
          OSA_MemFree(serial_dev_context);
          return -ENXIO;
    }

    /* Semaphore initialization */
    if ( kStatus_OSA_Success != OSA_MutexCreate(&serial_dev_context->rlock))
    {
        OSA_MemFree(serial_dev_context->serial_state);
        OSA_MemFree(serial_dev_context);
        return -ENOMEM;
    }
    if ( kStatus_OSA_Success !=OSA_MutexCreate(&serial_dev_context->wlock))
    {
        OSA_MemFree(serial_dev_context->serial_state);
        OSA_MutexDestroy(&serial_dev_context->rlock);
        OSA_MemFree(serial_dev_context);
        return -ENOMEM;
    }
    /* SERIAL handler interrupt installation */
    NVIC_SetPriority((IRQn_Type)IRQNumber, init->RXTX_PRIOR);
    NVIC_EnableIRQ((IRQn_Type)IRQNumber);

    /* Device context initialization */
    serial_dev_context->module = init->MODULE;
    serial_dev_context->instance = init->SERIAL_INSTANCE;
    *dev_context = (void*)serial_dev_context;

    return 0;
}

static int nio_serial_deinit(void *dev_context)
{
    NIO_SERIAL_DEV_CONTEXT_STRUCT *serial_dev_context = (NIO_SERIAL_DEV_CONTEXT_STRUCT *)dev_context;

    switch (serial_dev_context->module)
    {
#if defined(FSL_FEATURE_UART_FIFO_SIZE)
      case kNioSerialUart:
          UART_DRV_Deinit(serial_dev_context->instance);
          break;
#endif
#if defined(FSL_FEATURE_LPUART_FIFO_SIZE)
      case kNioSerialLpuart:
          LPUART_DRV_Deinit(serial_dev_context->instance);
          break;
#endif
#if defined(FSL_FEATURE_LPSCI_FIFO_SIZE)
      case kNioSerialLpsci:
          LPSCI_DRV_Deinit(serial_dev_context->instance);
          break;
#endif
      default:
          return -EINVAL;
    }
    OSA_MutexDestroy(&serial_dev_context->rlock);
    OSA_MutexDestroy(&serial_dev_context->wlock);
    OSA_MemFree(serial_dev_context->serial_state);
    OSA_MemFree(dev_context);
    return 0;
}
