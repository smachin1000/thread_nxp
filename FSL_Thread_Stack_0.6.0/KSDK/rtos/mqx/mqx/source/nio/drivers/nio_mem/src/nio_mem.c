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
#include "nio_mem.h"
#include "nio.h"
#include "errno.h"
#include "fcntl.h"
#include "ioctl.h"

static int nio_mem_open(void *dev_context, const char *dev_name, int flags, void **fp_context);
static int nio_mem_read(void *dev_context, void *fp_context, void *buf, size_t nbytes);
static int nio_mem_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes);
static _nio_off_t nio_mem_lseek(void *dev_context, void *fp_context, _nio_off_t offset, int whence);
//static int nio_mem_ioctl(void *dev_context, void *fp_context, unsigned long int request, va_list ap);
static int nio_mem_close(void *dev_context, void *fp_context);
static int nio_mem_init(void *init_data, void **dev_context);
static int nio_mem_deinit(void *dev_context);

const NIO_DEV_FN_STRUCT nio_mem_dev_fn = {
    .OPEN = nio_mem_open,
    .READ = nio_mem_read,
    .WRITE = nio_mem_write,
    .LSEEK = nio_mem_lseek,
    .IOCTL = NULL,   // nio_mem_ioctl,
    .CLOSE = nio_mem_close,
    .INIT = nio_mem_init,
    .DEINIT = nio_mem_deinit,
};

typedef struct {
    uint32_t BASE;
    uint32_t SIZE;
    semaphore_t LOCK;          ///< device lock - provide necessary atomic operations
} NIO_MEM_DEV_CONTEXT_STRUCT;

typedef struct {
    uint32_t LOCATION;         ///< actual position in memory
    int ERROR;                 ///< last error code
    semaphore_t LOCK;
} NIO_MEM_FP_CONTEXT_STRUCT;

/** Open memory device driver.
 * \param dev_context
 * \param dev_name
 * \param flags
 * \param fp_context
 * \return
 */
static int nio_mem_open(void *dev_context, const char *dev_name, int flags, void **fp_context) {
    NIO_MEM_FP_CONTEXT_STRUCT *fpc;
    int err = 0;

    assert((NULL != dev_context) && (NULL != dev_name) && (NULL != fp_context));

    if (NULL != (fpc = OSA_MemAlloc(sizeof(NIO_MEM_FP_CONTEXT_STRUCT)))) {
        // initialise information fields

        if (kStatus_OSA_Success != OSA_SemaCreate(&fpc->LOCK, 1)) {
            OSA_MemFree(fpc);
            err = -ENOMEM;
        }
        else {
            fpc->LOCATION = 0;
            *fp_context = fpc;
        }
    }

    return err;
}

static int nio_mem_close(void *dev_context, void *fp_context) {
    assert((NULL != dev_context) && (NULL != fp_context));

    OSA_SemaDestroy(&((NIO_MEM_FP_CONTEXT_STRUCT*)fp_context)->LOCK);
    OSA_MemFree(fp_context);
    return 0;
}

static int nio_mem_read(void *dev_context, void *fp_context, void *buf, size_t nbytes) {
    NIO_MEM_DEV_CONTEXT_STRUCT *devc = (NIO_MEM_DEV_CONTEXT_STRUCT*)dev_context;
    NIO_MEM_FP_CONTEXT_STRUCT *fpc = (NIO_MEM_FP_CONTEXT_STRUCT*)fp_context;
    size_t len;

    assert((NULL != dev_context) && (NULL != fp_context));

    if (buf) {
        if (nbytes) {
            // lock - only one rw can be processed in one time
            // TODO: check for return value
            OSA_SemaWait(&fpc->LOCK, OSA_WAIT_FOREVER);

            // check for EOF
            if (fpc->LOCATION < devc->SIZE) {
                // normalize size
                if (nbytes > devc->SIZE - fpc->LOCATION)
                {
                    len = devc->SIZE - fpc->LOCATION;
                }
                else
                {
                     len = nbytes;
                }

                // read data
                // TODO: check for return value
                OSA_SemaWait(&devc->LOCK, OSA_WAIT_FOREVER);
                memcpy(buf, (void*)(devc->BASE + fpc->LOCATION), len);
                OSA_SemaPost(&devc->LOCK);
                fpc->LOCATION += len;
            }
            else
            {
                // EOF
                len = 0;
            }

            OSA_SemaPost(&fpc->LOCK);
        }
        else
        {
             //no data to copy
            len = 0;
        }
    }
    else
    {
        return -EFAULT;
    }


    return len;
}

static int nio_mem_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes) {
    NIO_MEM_DEV_CONTEXT_STRUCT *devc = (NIO_MEM_DEV_CONTEXT_STRUCT*)dev_context;
    NIO_MEM_FP_CONTEXT_STRUCT *fpc = (NIO_MEM_FP_CONTEXT_STRUCT*)fp_context;
    size_t len;

    assert((NULL != dev_context) && (NULL != fp_context));

    if (buf) {
        if (nbytes) {
            // lock - only one rw can be processed in one time
            // TODO: check for return value
            OSA_SemaWait(&fpc->LOCK, OSA_WAIT_FOREVER);

            // check for EOF
            if (fpc->LOCATION < devc->SIZE) {
                // normalize size
                if (nbytes > devc->SIZE - fpc->LOCATION)
                {
                    len = devc->SIZE - fpc->LOCATION;
                }
                else
                {
                    len = nbytes;
                }

                // write data
                // TODO: check for return value
                OSA_SemaWait(&devc->LOCK, OSA_WAIT_FOREVER);
                memcpy((void*)(devc->BASE + fpc->LOCATION), buf, len);
                OSA_SemaPost(&devc->LOCK);
                fpc->LOCATION += len;
            }
            else
            {
                // EOF
                len = 0;
            }

            OSA_SemaPost(&fpc->LOCK);
        }
        else
        {
            //no data to write
            len = 0;
        }
    }
    else
    {
        return -EFAULT;
    }


    return len;
}

static _nio_off_t nio_mem_lseek(void *dev_context, void *fp_context, _nio_off_t offset, int whence) {
    NIO_MEM_DEV_CONTEXT_STRUCT *devc = (NIO_MEM_DEV_CONTEXT_STRUCT*)dev_context;
    NIO_MEM_FP_CONTEXT_STRUCT *fpc = (NIO_MEM_FP_CONTEXT_STRUCT*)fp_context;
    _nio_off_t res = 0;

    assert((NULL != dev_context) && (NULL != fp_context));

    // TODO: check for return value
    OSA_SemaWait(&fpc->LOCK, OSA_WAIT_FOREVER);

    switch (whence) {
    case SEEK_SET:
        if (0 > offset)
        {
            res = -EINVAL;
        }
        else
        {
            fpc->LOCATION = offset;
            res = fpc->LOCATION;
        }
        break;
    case SEEK_CUR:
        if (offset >= 0)
        {
            if ((UINT32_MAX - fpc->LOCATION) < offset)
            {
                res = -EOVERFLOW;
            }
            else
            {
                res = fpc->LOCATION = fpc->LOCATION + (uint32_t)offset;
            }
        }
        else
        {
            if (fpc->LOCATION < -offset)
            {
                res = -EOVERFLOW;
            }
            else
            {
                res = fpc->LOCATION = fpc->LOCATION - (uint32_t)-offset;
            }
        }
        break;
    case SEEK_END:
        if (offset >= 0)
        {
            if ((UINT32_MAX - devc->SIZE) < offset)
            {
                res = -EOVERFLOW;
            }
            else
            {
                res = fpc->LOCATION = devc->SIZE + (uint32_t)offset;
            }
        }
        else
        {
            if (devc->SIZE < -offset)
            {
                res = -EOVERFLOW;
            }
            else
            {
                res = fpc->LOCATION = devc->SIZE - (uint32_t)-offset;
            }
        }
        break;
    default:
        res = -EINVAL;
        break;
    }

    OSA_SemaPost(&fpc->LOCK);


    return res;
}

static int nio_mem_init(void *init_data, void **dev_context) {
    NIO_MEM_DEV_CONTEXT_STRUCT *devc;
    NIO_MEM_INIT_DATA_STRUCT *init = (NIO_MEM_INIT_DATA_STRUCT*)init_data;

    devc = OSA_MemAlloc(sizeof(NIO_MEM_DEV_CONTEXT_STRUCT));

    if (devc) {
        *dev_context = (void*)devc;

        devc->BASE = init->BASE;
        devc->SIZE = init->SIZE;

        // TODO: check for return value
        OSA_SemaCreate(&devc->LOCK, 1);
    }

    return (devc) ? 0 : -ENOMEM;
}

static int nio_mem_deinit(void *dev_context) {
    //NIO_MEM_DEV_CONTEXT_STRUCT *devc = dev_context;

    OSA_SemaDestroy(&((NIO_MEM_DEV_CONTEXT_STRUCT*)dev_context)->LOCK);
    OSA_MemFree(dev_context);
    return 0;
}
