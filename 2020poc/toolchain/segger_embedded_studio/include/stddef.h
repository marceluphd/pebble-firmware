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

#ifndef __stddef_H
#define __stddef_H

#include "__crossworks.h"

#ifndef __RAL_SIZE_T_DEFINED
#define __RAL_SIZE_T_DEFINED
/*! \brief size_t type \ingroup Types \synopsis

  \desc \b \this is the unsigned integral type returned by the sizeof operator.
*/
typedef __RAL_SIZE_T size_t;
#endif

/*! \brief ptrdiff_t type \ingroup Types \synopsis

  \desc \b \this is the signed integral type of the result of subtracting two pointers.
*/
typedef __RAL_PTRDIFF_T ptrdiff_t;


#ifndef NULL
/*! \brief NULL pointer \ingroup Macros \synopsis

  \desc \b \this is the null pointer constant.
*/
#define NULL 0
#endif


#if !defined(__RAL_WCHAR_T_DEFINED) && !defined(__cplusplus)
#define __RAL_WCHAR_T_DEFINED

typedef __RAL_WCHAR_T wchar_t;

#endif

#if defined __GNUC__
#define offsetof(s,m) __builtin_offsetof(s, m)
#else
#define offsetof(s,m) ((size_t)&(((s *)0)->m))
#endif

#ifdef __CROSSWORKS_DOCUMENTATION

/*! \brief offsetof \ingroup Macros \synopsis

  \desc \b \this returns the offset in bytes to the structure \a member, from the beginning of its structure \a type. 
*/
#define offsetof(type, member)

#endif

#endif
