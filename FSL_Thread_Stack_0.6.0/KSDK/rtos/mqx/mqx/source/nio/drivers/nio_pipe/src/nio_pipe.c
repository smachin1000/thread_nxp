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

#include <stdint.h>
#include <fsl_os_abstraction.h>

#include "nio_pipe.h"
#include "nio.h"
#include "ioctl.h"
#include "fcntl.h"


static int nio_pipe_open(void *dev_context, const char *dev_name, int flags, void **fp_context);
static int nio_pipe_read(void *dev_context, void *fp_context, void *buf, size_t nbytes);
static int nio_pipe_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes);
//static int nio_pipe_ioctl(void *dev_context, void *fp_context, unsigned long int request, va_list ap);
static int nio_pipe_close(void *dev_context, void *fp_context);
static int nio_pipe_init(void *init_data, void **dev_context);
static int nio_pipe_deinit(void *dev_context);

const NIO_DEV_FN_STRUCT nio_pipe_dev_fn =
{
    .OPEN = nio_pipe_open,
    .READ = nio_pipe_read,
    .WRITE = nio_pipe_write,
    .LSEEK = NULL,
    .IOCTL = NULL,   // nio_pipe_ioctl,
    .CLOSE = nio_pipe_close,
    .INIT = nio_pipe_init,
    .DEINIT = nio_pipe_deinit,
};

typedef struct
{
    semaphore_t mutex;

    semaphore_t rlock;
    semaphore_t wlock;

    char *wr, *rd;
    int unread;
    int len;

    char buf[];
} NIO_PIPE_DEV_CONTEXT_STRUCT;

typedef struct
{
    int FLAGS;
} NIO_PIPE_FP_CONTEXT_STRUCT;

static int nio_pipe_open(void *dev_context, const char *dev_name, int flags, void **fp_context)
{
    *fp_context = OSA_MemAlloc(sizeof(NIO_PIPE_FP_CONTEXT_STRUCT));
    if (NULL == *fp_context)
    {
      return ENOMEM;
    }
    ((NIO_PIPE_FP_CONTEXT_STRUCT*)*fp_context)->FLAGS = flags;
    return 0;
}

static int nio_pipe_close(void *dev_context, void *fp_context)
{
    OSA_MemFree(fp_context);
    return 0;
}

static int nio_pipe_read(void *dev_context, void *fp_context, void *buf, size_t nbytes)
{
    NIO_PIPE_DEV_CONTEXT_STRUCT *devc = (NIO_PIPE_DEV_CONTEXT_STRUCT*)dev_context;
    NIO_PIPE_FP_CONTEXT_STRUCT *fpc = (NIO_PIPE_FP_CONTEXT_STRUCT*)fp_context;
    size_t remain = nbytes;
    char *p = (char*)buf;

    if ((NULL != buf) && (0 < nbytes))
    {
        osa_status_t osa_status;
        while ((remain) && (0 < devc->unread))
        {
            if (!((fpc->FLAGS) & (O_NONBLOCK)))
            {
                osa_status = OSA_SemaWait(&devc->rlock, OSA_WAIT_FOREVER);
                if (kStatus_OSA_Success != osa_status)
                {
                    return -ENOMEM;
                }
            }
            osa_status = OSA_SemaWait(&devc->mutex, OSA_WAIT_FOREVER);
            if (kStatus_OSA_Success != osa_status)
            {
                return -ENOMEM;
            }

            *p++ = *devc->rd++;
            devc->unread--;
            if (devc->rd >= &devc->buf[devc->len])
            {
                devc->rd = &devc->buf[0];
            }

            osa_status = OSA_SemaPost(&devc->mutex);
            if (kStatus_OSA_Success != osa_status)
            {
                return -ENOMEM;
            }

            osa_status = OSA_SemaPost(&devc->wlock);
            if (kStatus_OSA_Success != osa_status)
            {
                return -ENOMEM;
            }

            remain--;
        }
    }

    return nbytes - remain;
}

static int nio_pipe_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes)
{
    NIO_PIPE_DEV_CONTEXT_STRUCT *devc = (NIO_PIPE_DEV_CONTEXT_STRUCT*)dev_context;
    NIO_PIPE_FP_CONTEXT_STRUCT *fpc = (NIO_PIPE_FP_CONTEXT_STRUCT*)fp_context;
    size_t remain = nbytes;

    /* Double casting to supress MISRA C 2004 11.2 :
     * conversions shall not be performed between a pointer to object and any type other than an integral type, another pointer to object type or a pointer to void.
     */
    const uint32_t tmp =  (uint32_t) buf;
    const char *p = (const char*)tmp;

    if ((NULL != buf) && (0 < nbytes))
    {
        osa_status_t osa_status;
        while ((remain) && (devc->len > devc->unread))
        {
            if (!((fpc->FLAGS) & (O_NONBLOCK)))
            {
                osa_status = OSA_SemaWait(&devc->wlock, OSA_WAIT_FOREVER);
                if (kStatus_OSA_Success != osa_status)
                {
                return -ENOMEM;
                }
            }
            osa_status = OSA_SemaWait(&devc->mutex, OSA_WAIT_FOREVER);
            if (kStatus_OSA_Success != osa_status)
            {
                return -ENOMEM;
            }
            *devc->wr = *p++;
            devc->unread++;

            devc->wr++;

            if (devc->wr >= &devc->buf[devc->len])
            {
                devc->wr = &devc->buf[0];
            }

            osa_status = OSA_SemaPost(&devc->mutex);
            if (kStatus_OSA_Success != osa_status)
            {
                return -ENOMEM;
            }

            osa_status = OSA_SemaPost(&devc->rlock);
            if (kStatus_OSA_Success != osa_status)
            {
                return -ENOMEM;
            }

            remain--;
        }
    }

    return nbytes - remain;
}

static int nio_pipe_init(void *init_data, void **dev_context)
{
    NIO_PIPE_DEV_CONTEXT_STRUCT *devc;
    NIO_PIPE_INIT_DATA_STRUCT *init = (NIO_PIPE_INIT_DATA_STRUCT*)init_data;

    devc = OSA_MemAlloc(sizeof(NIO_PIPE_DEV_CONTEXT_STRUCT) + init->LEN);

    if (devc)
    {
        osa_status_t osa_status;

        *dev_context = (void*)devc;

        devc->len = init->LEN;
        devc->unread = 0;
        devc->rd = &devc->buf[0];
        devc->wr = &devc->buf[0];

        osa_status = OSA_SemaCreate(&devc->rlock, 0);
        if (kStatus_OSA_Success != osa_status)
        {
            OSA_MemFree(devc);
            return -ENOMEM;
        }
        osa_status = OSA_SemaCreate(&devc->wlock, devc->len);
        if (kStatus_OSA_Success != osa_status)
        {
            OSA_MemFree(devc);
            OSA_SemaDestroy(&devc->rlock);
            return -ENOMEM;
        }
        osa_status = OSA_SemaCreate(&devc->mutex, 1);
        if (kStatus_OSA_Success != osa_status)
        {
            OSA_MemFree(devc);
            OSA_SemaDestroy(&devc->rlock);
            OSA_SemaDestroy(&devc->wlock);
            return -ENOMEM;
        }
    }

    return (devc) ? 0 : -ENOMEM;
}

static int nio_pipe_deinit(void *dev_context)
{
    //NIO_PIPE_DEV_CONTEXT_STRUCT *devc = dev_context;

    OSA_SemaDestroy(&((NIO_PIPE_DEV_CONTEXT_STRUCT*)dev_context)->rlock);
    OSA_SemaDestroy(&((NIO_PIPE_DEV_CONTEXT_STRUCT*)dev_context)->wlock);
    OSA_SemaDestroy(&((NIO_PIPE_DEV_CONTEXT_STRUCT*)dev_context)->mutex);
    OSA_MemFree(dev_context);
    return 0;
}
