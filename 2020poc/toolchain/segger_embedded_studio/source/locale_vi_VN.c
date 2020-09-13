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

const char __RAL_vi_VN_locale_day_names[] = 
{
  "Ch\341\273\247 nh\341\272\255t\0"
  "Th\341\273\251 hai\0"
  "Th\341\273\251 ba\0"
  "Th\341\273\251 t\306\260\0"
  "Th\341\273\251 n\304\203m\0"
  "Th\341\273\251 s\303\241u\0"
  "Th\341\273\251 b\341\272\243y\0"
};

const char __RAL_vi_VN_locale_abbrev_day_names[] = 
{
  "CN\0"
  "Th 2\0"
  "Th 3\0"
  "Th 4\0"
  "Th 5\0"
  "Th 6\0"
  "Th 7\0"
};

const char __RAL_vi_VN_locale_month_names[] = 
{
  "th\303\241ng m\341\273\231t\0"
  "th\303\241ng hai\0"
  "th\303\241ng ba\0"
  "th\303\241ng t\306\260\0"
  "th\303\241ng n\304\203m\0"
  "th\303\241ng s\303\241u\0"
  "th\303\241ng b\341\272\243y\0"
  "th\303\241ng t\303\241m\0"
  "th\303\241ng ch\303\255n\0"
  "th\303\241ng m\306\260\341\273\235i\0"
  "th\303\241ng m\306\260\341\273\235i m\341\273\231t\0"
  "th\303\241ng m\306\260\341\273\235i hai\0"
};

const char __RAL_vi_VN_locale_abbrev_month_names[] = 
{
  "thg 1\0"
  "thg 2\0"
  "thg 3\0"
  "thg 4\0"
  "thg 5\0"
  "thg 6\0"
  "thg 7\0"
  "thg 8\0"
  "thg 9\0"
  "thg 10\0"
  "thg 11\0"
  "thg 12\0"
};

const char __RAL_vi_VN_locale_grouping[] = "3";
const char __RAL_vi_VN_locale_int_currency_symbol[] = "VND";
const char __RAL_vi_VN_locale_currency_symbol[] = "\342\202\253";
const char __RAL_vi_VN_locale_mon_grouping[] = "3";
const char __RAL_vi_VN_locale_time_fmt[] = "%H:%M:%S";
const char __RAL_vi_VN_locale_date_fmt[] = "%d/%m/%Y";
const char __RAL_vi_VN_locale_date_time_fmt[] = "%H:%M:%S %Z Ng\303\240y %d th\303\241ng %m n\304\203m %Y";
const char __RAL_vi_VN_locale_am_pm_indicator[] = "SA\0CH\0";

const __CODE __RAL_locale_data_t __RAL_vi_VN_locale = 
{
  __RAL_data_utf8_comma,   // decimal_point
  __RAL_data_utf8_period,   // thousands_sep
  __RAL_vi_VN_locale_grouping,   // grouping
  __RAL_vi_VN_locale_int_currency_symbol,   // int_currency_symbol
  __RAL_vi_VN_locale_currency_symbol,   // currency_symbol
  __RAL_data_utf8_comma,   // mon_decimal_point
  __RAL_data_utf8_period,   // mon_thousands_sep
  __RAL_vi_VN_locale_mon_grouping,   // mon_grouping
  __RAL_data_empty_string,   // positive_sign
  __RAL_data_utf8_minus,   // negative_sign
  0,	// int_frac_digits
  0,	// frac_digits
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
  __RAL_vi_VN_locale_day_names,
  __RAL_vi_VN_locale_abbrev_day_names,
  __RAL_vi_VN_locale_month_names,
  __RAL_vi_VN_locale_abbrev_month_names,
  __RAL_vi_VN_locale_am_pm_indicator,
  __RAL_vi_VN_locale_time_fmt,
  __RAL_vi_VN_locale_date_fmt,
  __RAL_vi_VN_locale_date_time_fmt,
};

