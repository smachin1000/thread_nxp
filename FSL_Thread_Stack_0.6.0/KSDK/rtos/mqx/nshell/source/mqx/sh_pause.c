/*HEADER**********************************************************************
*
* Copyright 2008 Freescale Semiconductor, Inc.
* Copyright 2004-2008 Embedded Access Inc.
* Copyright 1989-2008 ARC International
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
*   This file contains the RTCS shell.
*
*
*END************************************************************************/

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <mqx.h>
#include "shell.h"
#include "sh_prv.h"


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name :  Shell_pause
*  Returned Value:  none
*  Comments  :  SHELL utility to pause the shell for specified time
*
*END*-----------------------------------------------------------------*/

int32_t  Shell_pause(int32_t argc, char *argv[] )
{
    SHELL_CONTEXT_PTR shell_ptr = Shell_get_context(argv);
   bool              print_usage, shorthelp = FALSE;
   int32_t               return_code = SHELL_EXIT_SUCCESS;
   uint32_t              pause=10*60*1000;

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
      if (argc == 1)  {
         fprintf(shell_ptr->STDOUT, "Pausing for 5 minutes...\n");
        _time_delay(5*60*1000);
          fprintf(shell_ptr->STDOUT, "Done\n");
      } else if (argc==2) {
         if (!Shell_parse_uint_32(argv[1], &pause  )) {
            fprintf(shell_ptr->STDOUT, "Error, invalid rule priority\n");
            return_code = SHELL_EXIT_ERROR;
         }
          fprintf(shell_ptr->STDOUT, "Pausing for %d minutes...\n", (unsigned int)pause);
          _time_delay(pause*60*1000);
          fprintf(shell_ptr->STDOUT, "Done\n");
      } else {
         fprintf(shell_ptr->STDOUT, "Error, %s invoked with incorrect number of arguments\n", argv[0]);
         return_code = SHELL_EXIT_ERROR;
         print_usage = TRUE;
      }
   }

   if (print_usage)  {
      if (shorthelp)  {
         fprintf(shell_ptr->STDOUT, "%s [<minutes>]\n", argv[0]);
      } else  {
         fprintf(shell_ptr->STDOUT, "Usage: %s <minutes>\n", argv[0]);
         fprintf(shell_ptr->STDOUT, "   <minutes>   = minutes to pause for\n");
      }
   }
   return return_code;
} /* Endbody */



/* EOF */
