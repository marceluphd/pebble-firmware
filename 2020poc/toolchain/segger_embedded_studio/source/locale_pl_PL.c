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

// Generated automatically from the Unicode Character Database
// and Common Locale Data Repository.

#include <__crossworks.h>
#include <ctype.h>
#include <wchar.h>
#include <wctype.h>
#include <locale.h>
#include <limits.h>

const char __RAL_pl_PL_locale_day_names[] = 
{
  "niedziela\0"
  "poniedzia\305\202ek\0"
  "wtorek\0"
  "\305\233roda\0"
  "czwartek\0"
  "pi\304\205tek\0"
  "sobota\0"
};

const char __RAL_pl_PL_locale_abbrev_day_names[] = 
{
  "niedz.\0"
  "pon.\0"
  "wt.\0"
  "\305\233r.\0"
  "czw.\0"
  "pt.\0"
  "sob.\0"
};

const char __RAL_pl_PL_locale_month_names[] = 
{
  "stycznia\0"
  "lutego\0"
  "marca\0"
  "kwietnia\0"
  "maja\0"
  "czerwca\0"
  "lipca\0"
  "sierpnia\0"
  "wrze\305\233nia\0"
  "pa\305\272dziernika\0"
  "listopada\0"
  "grudnia\0"
};

const char __RAL_pl_PL_locale_abbrev_month_names[] = 
{
  "sty\0"
  "lut\0"
  "mar\0"
  "kwi\0"
  "maj\0"
  "cze\0"
  "lip\0"
  "sie\0"
  "wrz\0"
  "pa\305\272\0"
  "lis\0"
  "gru\0"
};

const char __RAL_pl_PL_locale_thousands_sep[] = "\302\240";
const char __RAL_pl_PL_locale_grouping[] = "3";
const char __RAL_pl_PL_locale_int_currency_symbol[] = "PLN";
const char __RAL_pl_PL_locale_currency_symbol[] = "z\305\202";
const char __RAL_pl_PL_locale_mon_thousands_sep[] = "\302\240";
const char __RAL_pl_PL_locale_mon_grouping[] = "3";
const char __RAL_pl_PL_locale_time_fmt[] = "%H:%M:%S";
const char __RAL_pl_PL_locale_date_fmt[] = "%d.%m.%Y";
const char __RAL_pl_PL_locale_date_time_fmt[] = "%e %B %Y, %H:%M:%S %Z";
const char __RAL_pl_PL_locale_am_pm_indicator[] = "AM\0PM\0";

const __CODE __RAL_locale_data_t __RAL_pl_PL_locale = 
{
  __RAL_data_utf8_comma,   // decimal_point
  __RAL_pl_PL_locale_thousands_sep,   // thousands_sep
  __RAL_pl_PL_locale_grouping,   // grouping
  __RAL_pl_PL_locale_int_currency_symbol,   // int_currency_symbol
  __RAL_pl_PL_locale_currency_symbol,   // currency_symbol
  __RAL_data_utf8_comma,   // mon_decimal_point
  __RAL_pl_PL_locale_mon_thousands_sep,   // mon_thousands_sep
  __RAL_pl_PL_locale_mon_grouping,   // mon_grouping
  __RAL_data_empty_string,   // positive_sign
  __RAL_data_utf8_minus,   // negative_sign
  2,	// int_frac_digits
  2,	// frac_digits
  0,	// p_cs_precedes
  1,	// p_sep_by_space
  0,	// n_cs_precedes
  1,	// n_sep_by_space
  1,	// p_sign_posn
  1,	// n_sign_posn
  0,	// int_p_cs_precedes
  0,	// int_n_cs_precedes
  1,	// int_p_sep_by_space
  1,	// int_n_sep_by_space
  1,	// int_p_sign_posn
  1,	// int_n_sign_posn
  __RAL_pl_PL_locale_day_names,
  __RAL_pl_PL_locale_abbrev_day_names,
  __RAL_pl_PL_locale_month_names,
  __RAL_pl_PL_locale_abbrev_month_names,
  __RAL_pl_PL_locale_am_pm_indicator,
  __RAL_pl_PL_locale_time_fmt,
  __RAL_pl_PL_locale_date_fmt,
  __RAL_pl_PL_locale_date_time_fmt,
};

