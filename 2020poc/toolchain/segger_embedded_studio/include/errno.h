// **********************************************************************
// *                    SEGGER Microcontroller GmbH                     *
// *                        The Embedded Experts                        *
// **********************************************************************
// *                                                                    *
// *            (c) 2014 - 2020 SEGGER Microcontroller GmbH             *
// *            (c) 2001 - 2020 Rowley Associates Limited               *
// *                                                                    *
// *           www.segger.com     Support: support@segger.com           *
// *                                                                    *
// **********************************************************************
// *                                                                    *
// * All rights reserved.                                               *
// *                                                                    *
// * Redistribution and use in source and binary forms, with or         *
// * without modification, are permitted provided that the following    *
// * conditions are met:                                                *
// *                                                                    *
// * - Redistributions of source code must retain the above copyright   *
// *   notice, this list of conditions and the following disclaimer.    *
// *                                                                    *
// * - Neither the name of SEGGER Microcontroller GmbH                  *
// *   nor the names of its contributors may be used to endorse or      *
// *   promote products derived from this software without specific     *
// *   prior written permission.                                        *
// *                                                                    *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
// * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
// * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
// * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
// * DISCLAIMED.                                                        *
// * IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
// * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
// * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
// * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
// * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
// * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
// * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
// * DAMAGE.                                                            *
// *                                                                    *
// **********************************************************************

#ifndef __errno_h
#define __errno_h

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __ARM_EABI__
extern const int __aeabi_EDOM;
extern const int __aeabi_EILSEQ;
extern const int __aeabi_ERANGE;
#define EDOM     (__aeabi_EDOM)
#define EILSEQ   (__aeabi_EILSEQ)
#define ERANGE   (__aeabi_ERANGE)

#else

#define EDOM     0x01
#define EILSEQ   0x02
#define ERANGE   0x03

#endif

#define EHEAP    0x04


/*! \brief No memory available \ingroup Error numbers \synopsis

  \desc \b \this - no memory can be allocated by a function in the library.  Note that
  \b malloc, \b calloc, and \b realloc do not set \b errno to \b ENOMEM on failure,
  but other library routines (such as \b duplocale) may set \b errno to \b ENOMEM
  when memory allocation fails.
*/
#define ENOMEM    0x05


/*! \brief Invalid argument \ingroup Error numbers \synopsis

  \desc \b \this - An argument was invalid, or a combination of arguments was invalid.
*/
#define EINVAL    0x06


#ifdef __ARM_EABI__
volatile int *__aeabi_errno_addr(void);
#else
volatile int *__errno(void);
#endif

#ifdef __ARM_EABI__
#define errno (*__aeabi_errno_addr())
#else
#define errno (*__errno())
#endif

#ifdef __CROSSWORKS_DOCUMENTATION

/*! \brief Domain error \ingroup Error numbers \synopsis

  \desc \b \this - an input argument is outside the defined domain of a
    mathematical function.
*/
#define EDOM   ...


/*! \brief Illegal byte sequence \ingroup Error numbers \synopsis

  \desc \b \this - A wide-character code has been detected that does not correspond to a valid character, 
  or a byte sequence does not form a valid wide-character code.
*/
#define EILSEQ   ...


/*! \brief Result too large or too small \ingroup Error numbers \synopsis

  \desc \b \this - the result of the function is too large (overflow) or too small (underflow) to be represented 
  in the available space.
*/
#define ERANGE   ...


/*! \brief Last-set error condition \ingroup Macros \synopsis

  \desc \b \this is treated as an writable l-value, but the implementation of
  how the l-value is read an written is hidden from the user.

  The value of \b \this is zero at program startup, but is never set to 
  zero by any library function. The value of \b \this may be set to a nonzero 
  value by a library function, and this effect is documented in each function that 
  does so.

  \note The ISO standard does not specify whether \b \this is a macro or an identifier 
  declared with external linkage. Portable programs must not make assumptions 
  about the implementation of \b \this.

  In this implementation, \b \this expands to a function call to \b __errno
  (MSP430, AVR, MAXQ) or \b __aeabi_errno_addr (ARM) that returns a pointer to a
  volatile \b int. This function can be implemented by the application to provide
  a thread-specific \b errno.

*/
int errno;

#endif

#ifdef __cplusplus
}
#endif

#endif
