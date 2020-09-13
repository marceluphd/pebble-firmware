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

#ifndef __assert_h
#define __assert_h

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief User defined behaviour for the assert macro \ingroup Functions \synopsis

 \desc There is no default implementation of \b \this. Keeping \b \this 
  out of the library means that you can can customize its behaviour without rebuilding 
  the library. You must implement this function where \a expression is the stringized
  expression, \a filename is the filename of the source file and \a line
  is the linenumber of the failed assertion.
*/
extern void __assert(const char *__expression, const char *__filename, int __line);

#ifdef __cplusplus
}
#endif

#else
#undef assert
#endif

#ifdef NDEBUG
#define assert(ignore) ((void)0)
#else
#define assert(e) ((e) ? (void)0 : __assert(#e, __FILE__, __LINE__))
#endif

#if defined __GNUC__ && __STDC_VERSION__ >= 201112L && !defined __cplusplus
#define static_assert _Static_assert
#endif

#ifdef __CROSSWORKS_DOCUMENTATION

/*! \brief Allows you to place assertions and diagnostic tests into programs \ingroup Macros \synopsis

  \desc If \b NDEBUG is defined as a macro name at the point in the source file 
  where \b <assert.h> is included, the \b \this macro is defined 
  as: 
  
  \code #define assert(ignore) ((void)0) \endcode

  If \b NDEBUG is not defined as a macro name at the point in the source 
  file where \b <assert.h> is included, the \b \this macro expands 
  to a \b void expression that calls \b __assert. 
  
  \code #define assert(e) ((e) ? (void)0 : __assert(#e, __FILE__, __LINE__)) \endcode
  
  When such an \b \this 
  is executed and \b e is false, \b \this calls the \b __assert
  function with information about the particular call that failed: the text of 
  the argument, the name of the source file, and the source line number. These 
  are the stringized expression and the values of the preprocessing macros \b __FILE__
  and \b__LINE__.

  \note The \b \this macro is redefined according to the current state of \b NDEBUG
  each time that \b <assert.h> is included.
*/
#define assert(e) ...

#endif
