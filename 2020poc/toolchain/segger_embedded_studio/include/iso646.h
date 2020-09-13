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

#ifndef __iso646_h
#define __iso646_h


/*! \overview The header \<iso646.h\> defines macros that expand
  to the corresponding tokens to ease writing C programs with
  keyboards that do not have keys for frequently-used operators.
*/


#ifndef __cplusplus

/*! \brief Alternative spelling for logical and operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{&&}.
*/
#define and     &&


/*! \brief Alternative spelling for logical and-equals operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{&=}.
*/
#define and_eq  &=


/*! \brief Alternative spelling for bitwise and operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{&}.
*/
#define bitand  &


/*! \brief Alternative spelling for bitwise or operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{|}.
*/
#define bitor   |


/*! \brief Alternative spelling for bitwise complement operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{~}.
*/
#define compl   ~


/*! \brief Alternative spelling for logical not operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{!}.
*/
#define not     !


/*! \brief Alternative spelling for not-equal operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{!=}.
*/
#define not_eq  !=


/*! \brief Alternative spelling for logical or operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{||}.
*/
#define or      ||


/*! \brief Alternative spelling for bitwise or-equals operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{|=}.
*/
#define or_eq   |=


/*! \brief Alternative spelling for bitwise exclusive or operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{^}.
*/
#define xor     ^


/*! \brief Alternative spelling for bitwise exclusive-or-equals operator \ingroup Macros \synopsis

  \desc \b \this defines the alternative spelling for \c{^=}.
*/
#define xor_eq  ^=


#endif

#endif

