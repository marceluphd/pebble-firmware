/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2020 SEGGER Microcontroller GmbH             *
*                                                                    *
*           www.segger.com     Support: support@segger.com           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* conditions are met:                                                *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this list of conditions and the following disclaimer.    *
*                                                                    *
* - Neither the name of SEGGER Microcontroller GmbH                  *
*   nor the names of its contributors may be used to endorse or      *
*   promote products derived from this software without specific     *
*   prior written permission.                                        *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED.                                                        *
* IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       RTT version: 5.12e                                           *
*                                                                    *
**********************************************************************
--------  END-OF-HEADER  ---------------------------------------------
File    : SEGGER_RTT_Syscalls_IAR.c
Purpose : Low-level functions for using printf() via RTT in IAR.
          To use RTT for printf output, include this file in your 
          application and set the Library Configuration to Normal.
----------------------------------------------------------------------
*/
#include <yfuns.h>
#include "SEGGER_RTT.h"
#pragma module_name = "?__write"

/*********************************************************************
*
*       Function prototypes
*
**********************************************************************
*/
size_t __write(int handle, const unsigned char * buffer, size_t size);

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
/*********************************************************************
*
*       __write()
*
* Function description
*   Low-level write function.
*   Standard library subroutines will use this system routine
*   for output to all files, including stdout.
*   Write data via RTT.
*/
size_t __write(int handle, const unsigned char * buffer, size_t size) {
  (void) handle;  /* Not used, avoid warning */
  SEGGER_RTT_Write(0, (const char*)buffer, size);
  return size;
}

/*********************************************************************
*
*       __write_buffered()
*
* Function description
*   Low-level write function.
*   Standard library subroutines will use this system routine
*   for output to all files, including stdout.
*   Write data via RTT.
*/
size_t __write_buffered(int handle, const unsigned char * buffer, size_t size) {
  (void) handle;  /* Not used, avoid warning */
  SEGGER_RTT_Write(0, (const char*)buffer, size);
  return size;
}
/****** End Of File *************************************************/
