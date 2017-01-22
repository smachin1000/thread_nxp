
/*HEADER**********************************************************************
*
* Copyright 2008-2012 Freescale Semiconductor, Inc.
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
* See license agreement file for full license terms including other restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains runtime support for the RV Compiler.
*
*
*END************************************************************************/

#include "mqx.h"

// Do not include <stdio.h> !
//#include <stdio.h>

/*! \cond DOXYGEN_PRIVATE */
void __user_setup_stackheap(void);
/*! \endcond */

/*! \cond DOXYGEN_PRIVATE  */
int __aeabi_atexit(void *, void *, void *);
/*! \endcond */

void *malloc(_mem_size);
void *calloc(_mem_size, _mem_size);
void free(void *);

/*! \cond DOXYGEN_PRIVATE  */
void __main(void);
/*! \endcond */


/*!
 * \cond DOXYGEN_PRIVATE
 * \brief To create a version of __user_setup_stackheap() that inherits sp from the 
 *  execution environment and does not have a heap, set r0 and r2 to zero and 
 *  return.
 */
__asm void __user_setup_stackheap(void)
{
    movs r0, #0
    movs r2, #0
    bx lr
}
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Override C/C++ runtime __aeabi_atexit function in KEIL to avoid the call 
 *  to malloc and free before MQX starts http//infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka3951.html
 * 
 * \param object_ptr 
 * \param destructor_ptr 
 * \param __dso_handle_ptr 
 *
 * \return int
 */
int __aeabi_atexit(void *object_ptr, void *destructor_ptr, void *__dso_handle_ptr)
{
    return (1);
}
/*! \endcond */

#if MQX_USE_LWMEM_ALLOCATOR || MQX_USE_MEM || MQX_USE_TLSF_ALLOCATOR
/*!
 * \brief Override C/C++ runtime heap allocation function in KEIL
 *
 * \param bytes
 *
 * \return pointer
 */
void *malloc(_mem_size bytes)
{
    return _mem_alloc_system(bytes);
}

/*!
 * \brief Override C/C++ runtime heap allocation function in KEIL
 *
 * \param n
 * \param z
 *
 * \return pointer
 */
void *calloc(_mem_size n, _mem_size z)
{
    return _mem_alloc_system_zero(n*z);
}

/*!
 * \brief Override C/C++ runtime heap deallocation function in KEIL
 * 
 * \param p 
 */
void free(void *p)
{
    _mem_free(p);
}
#endif /* MQX_USE_LWMEM_ALLOCATOR, MQX_USE_MEM, MQX_USE_TLSF_ALLOCATOR */

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Workaround to override C/C++ runtime heap deallocation function in KEIL
 * 
 * Overload "_initio" at keil startup.
 * Linker has two phases.
 * - resolve the regular symbols (because of _fclose_internal)
 * - replace regular symbols with "$Sub$$" one
 */
int $Sub$$_initio(void)
{
    return 0;
}
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Dummy implementation of freopen required by KEIL startup
 * 
 * MQX_SDKLIB does not implement freopen and program should never reach this point.
 */
void * freopen( const char * filename, const char * mode, void * stream )
{
    return NULL;
}
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Dummy implementation of _fclose_internal required by KEIL startup
 * 
 * Application should not reach this point, if that happen it stops at infinite loop.
 */
int _fclose_internal(void * stream )
{
    volatile int i = 1;
    __asm("cpsid i");
    while(i);
    return -1;
}
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Perform necessary toolchain startup routines before main()
 */
void toolchain_startup(void)
{
    __main();
}
/*! \endcond */

/* EOF */
