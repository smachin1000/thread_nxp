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
*****************************************************************************
*
* Comments:
*
*   This file contains the nulldisk driver functions
*
*
*END************************************************************************/

#include <stdint.h>
#include "nio.h"



static int nio_null_open(void *dev_context, const char *dev_name, int flags, void **fp_context) {
    return 0;
}

static int nio_null_read(void *dev_context, void *fp_context, void *buf, size_t nbytes) {
    return 0;
}

static int nio_null_write(void *dev_context, void *fp_context, const void *buf, size_t nbytes) {
    return nbytes;
}

static _nio_off_t nio_null_lseek(void *dev_context, void *fp_context, _nio_off_t offset, int whence) {
    return 0;
}

static int nio_null_ioctl(void *dev_context, void *fp_context, unsigned long int request, va_list ap) {
    return -ENOTTY;
}

static int nio_null_close(void *dev_context, void *fp_context) {
    return 0;
}

static int nio_null_init(void *init_data, void **dev_context) {
    return 0;
}

static int nio_null_deinit(void *dev_context) {
    return 0;
}

const NIO_DEV_FN_STRUCT nio_null_dev_fn = {
    .OPEN = nio_null_open,
    .READ = nio_null_read,
    .WRITE = nio_null_write,
    .LSEEK = nio_null_lseek,
    .IOCTL = nio_null_ioctl,
    .CLOSE = nio_null_close,
    .INIT = nio_null_init,
    .DEINIT = nio_null_deinit,
};
