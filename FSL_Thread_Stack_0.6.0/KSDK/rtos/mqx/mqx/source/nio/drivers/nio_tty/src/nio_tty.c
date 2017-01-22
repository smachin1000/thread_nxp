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
#include <stdbool.h>
#include <stdarg.h>
#include <fsl_os_abstraction.h>

#include "nio_tty.h"
#include "nio.h"
#include "ioctl.h"
#include "errno.h"

#if defined ( __IAR_SYSTEMS_ICC__ )
/* MISRA C 2004 rule 20.5 suppress: Error[Pm101]: The error indicator errno shall not be used */
_Pragma ("diag_suppress= Pm101")
#endif

static int nio_tty_open(void *dev_context, const char *dev_name, int flags, void **fp_context);
static int nio_tty_read(void *dev_context, void *fp_context, void *buf, size_t nbytes);
static int nio_tty_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes);
static int nio_tty_ioctl(void *dev_context, void *fp_context, unsigned long int request, va_list ap);
static int nio_tty_close(void *dev_context, void *fp_context);
static int nio_tty_init(void *init_data, void **dev_context);
static int nio_tty_deinit(void *dev_context);

const NIO_DEV_FN_STRUCT nio_tty_dev_fn = {
    .OPEN = nio_tty_open,
    .READ = nio_tty_read,
    .WRITE = nio_tty_write,
    .LSEEK = NULL,
    .IOCTL = nio_tty_ioctl,
    .CLOSE = nio_tty_close,
    .INIT = nio_tty_init,
    .DEINIT = nio_tty_deinit,
};

typedef struct
{
    char dev_name[NIO_DEV_NAME_LEN]; /* Name of lower layer */
    uint32_t open_flags;            /* Flags for lower layer opening */
    int fd;                         /* File descriptor of lower layer */
    int opened;                     /* Count of opened tty */
} NIO_TTY_DEV_CONTEXT_STRUCT;

typedef struct
{
    uint32_t flags;     /* Flags determining behaviour of tty */
    bool     last_cr;   /* Mark if in last read \r was read */
} NIO_TTY_FP_CONTEXT_STRUCT;

static int nio_tty_open(void *dev_context, const char *dev_name, int flags, void **fp_context)
{
    *fp_context = OSA_MemAlloc(sizeof(NIO_TTY_FP_CONTEXT_STRUCT));
    NIO_TTY_DEV_CONTEXT_STRUCT *devc = (NIO_TTY_DEV_CONTEXT_STRUCT *)dev_context;
    NIO_TTY_FP_CONTEXT_STRUCT *fpc = (NIO_TTY_FP_CONTEXT_STRUCT *) *fp_context;
    int fd;
    /* Open lower layer fd. Only one fd is opened for all tty's. */
    if (!devc->opened)
    {
        fd = _nio_open(devc->dev_name, devc->open_flags);
        if (fd < 0)
        {
            return -EBADF;
        }
        devc->fd = fd;
    }
    fpc->flags = flags;
    fpc->last_cr = false;
    devc->opened++;
    return 0;
}

static int nio_tty_read(void *dev_context, void *fp_context, void *buf, size_t nbytes)
{
    NIO_TTY_DEV_CONTEXT_STRUCT *devc = (NIO_TTY_DEV_CONTEXT_STRUCT*)dev_context;
    NIO_TTY_FP_CONTEXT_STRUCT *fpc =  (NIO_TTY_FP_CONTEXT_STRUCT * )fp_context;
    char *s = (char *)buf;
    int ret;
    int i;
    /* Check input parameters */
    if (NULL == buf)
    {
        return -EFAULT;
    }

    if (0 > devc->fd)
    {
        return -EBADF;
    }

    /* Read data */
    for (i = 0; i < nbytes; i++)
    {

        /* Check previous end of line  */

        ret = _nio_read(devc->fd, s, 1);
        if (1 != ret)
        {
            if (-1 == ret)
            {
                return ret;
            }
            if (0 == ret)
            {
                break;
            }
            //assert that no other values of ret are possible
        }

        if (fpc->last_cr)
        {
            fpc->last_cr = false;
            if ('\n' == *s)
            {
                s++;
                /* \n has been discarded */
                continue;
            }
        }

        if (*s == '\r')
        {
            fpc->last_cr = true;
            if ((fpc->flags & NIO_TTY_FLAGS_ECHO) && (fpc->flags & NIO_TTY_FLAGS_EOL_RN))
            {
                _nio_write(devc->fd, s, 1);
            }
            *s = '\n';
        }

        if (fpc->flags & NIO_TTY_FLAGS_ECHO)
        {
            _nio_write(devc->fd, s, 1);
        }
        s++;
    }

    return i;
}

static int nio_tty_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes)
{
    NIO_TTY_DEV_CONTEXT_STRUCT *devc = (NIO_TTY_DEV_CONTEXT_STRUCT*)dev_context;
    NIO_TTY_FP_CONTEXT_STRUCT *fpc =  (NIO_TTY_FP_CONTEXT_STRUCT * )fp_context;

    /* Double casting to supress MISRA C 2004 11.2 :
     * conversions shall not be performed between a pointer to object and any type other than an integral type, another pointer to object type or a pointer to void.
     */
    const uint32_t tmp =  (uint32_t) buf;
    const char *s = (const char *)tmp;

    int i, ret;

    /* Check input parameters */
    if (NULL == buf)
    {
        return -EFAULT;
    }

    if (0 > devc->fd)
    {
        errno = -EBADF;
        return -1;
    }

    /* Write data */
    for (i = 0; i < nbytes; i++)
    {
        if ((fpc->flags & NIO_TTY_FLAGS_EOL_RN) && (*s == '\n'))
        {
            ret = _nio_write(devc->fd, "\r", 1);
            if (-1 == ret)
            {
                //TODO: errno should not be used in the low level driver. This is workaround for layering devices. _nio_write should not set errno, rather write should do it.
                return -errno;
            }
            if (0 == ret)
            {
                break;
            }
        }

        ret = _nio_write(devc->fd, s, 1);
        if (-1 == ret)
        {
            //TODO: errno should not be used in the low level driver. This is workaround for layering devices. _nio_write should not set errno, rather write should do it.
            return -errno;
        }
        if (0 == ret)
        {
            return i;
        }
        s++;
    }
    return i;
}

static int nio_tty_ioctl(void *dev_context, void *fp_context, unsigned long int request, va_list ap)
{
    NIO_TTY_DEV_CONTEXT_STRUCT *devc = (NIO_TTY_DEV_CONTEXT_STRUCT*)dev_context;
    NIO_TTY_FP_CONTEXT_STRUCT *fpc =  (NIO_TTY_FP_CONTEXT_STRUCT * )fp_context;

    /* Check input parameters */
    if (0 > devc->fd)
    {
        return -EBADF;
    }
    /* Handle request */
    switch (request)
    {
        case IOCTL_ABORT:
            _nio_ioctl(devc->fd, IOCTL_ABORT, ap);
            break;
        case IOCTL_NIO_TTY_SET_FLAGS:
            fpc->flags = va_arg(ap, uint32_t);
            break;

        default:
            return -ENOTTY;
    }

    return 0;
}

static int nio_tty_close(void *dev_context, void *fp_context)
{
    NIO_TTY_DEV_CONTEXT_STRUCT *devc = (NIO_TTY_DEV_CONTEXT_STRUCT*)dev_context;
    //NIO_TTY_FP_CONTEXT_STRUCT *fpc =  (NIO_TTY_FP_CONTEXT_STRUCT * )fp_context;

    /* Check input parameters */
    if (0 > devc->fd)
    {
        return -EBADF;
    }

    /* Close lower layer if last opened tty */
    devc->opened--;

    if (0 >= devc->opened)
    {
        _nio_close(devc->fd);
    }
    OSA_MemFree(fp_context);
    return 0;
}

static int nio_tty_init(void *init_data, void **dev_context)
{
    NIO_TTY_DEV_CONTEXT_STRUCT *devc;
    NIO_TTY_INIT_DATA_STRUCT *init = (NIO_TTY_INIT_DATA_STRUCT *)init_data;
    char *name = (char *)init->DEV_NAME;

    /* Create and initialize device context */
    devc = OSA_MemAllocZero(sizeof(NIO_TTY_DEV_CONTEXT_STRUCT));

    if (devc) {
        strncpy(devc->dev_name, name, (size_t)NIO_DEV_NAME_LEN);
        devc->open_flags = init->OPEN_FLAGS;
        devc->opened = 0;

        *dev_context = (void*)devc;
    }

    return devc ? 0 : -ENOMEM;
}

static int nio_tty_deinit(void *dev_context)
{
    //NIO_TTY_DEV_CONTEXT_STRUCT *devc = dev_context;

    OSA_MemFree(dev_context);
    return 0;
}
