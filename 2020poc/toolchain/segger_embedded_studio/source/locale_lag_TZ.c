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

const char __RAL_lag_TZ_locale_day_names[] = 
{
  "Jumap\303\255iri\0"
  "Jumat\303\241tu\0"
  "Juma\303\255ne\0"
  "Jumat\303\241ano\0"
  "Alam\303\255isi\0"
  "Ijum\303\241a\0"
  "Jumam\303\263osi\0"
};

const char __RAL_lag_TZ_locale_abbrev_day_names[] = 
{
  "P\303\255ili\0"
  "T\303\241atu\0"
  "\303\215ne\0"
  "T\303\241ano\0"
  "Alh\0"
  "Ijm\0"
  "M\303\263osi\0"
};

const char __RAL_lag_TZ_locale_month_names[] = 
{
  "K\312\211f\303\272ngat\311\250\0"
  "K\312\211naan\311\250\0"
  "K\312\211keenda\0"
  "Kwiikumi\0"
  "Kwiinyamb\303\241la\0"
  "Kwiidwaata\0"
  "K\312\211m\312\211\312\211nch\311\250\0"
  "K\312\211v\311\250\311\250r\311\250\0"
  "K\312\211saat\312\211\0"
  "Kwiinyi\0"
  "K\312\211saano\0"
  "K\312\211sasat\312\211\0"
};

const char __RAL_lag_TZ_locale_abbrev_month_names[] = 
{
  "F\303\272ngat\311\250\0"
  "Naan\311\250\0"
  "Keenda\0"
  "Ik\303\272mi\0"
  "Inyambala\0"
  "Idwaata\0"
  "M\312\211\312\211nch\311\250\0"
  "V\311\250\311\250r\311\250\0"
  "Saat\312\211\0"
  "Inyi\0"
  "Saano\0"
  "Sasat\312\211\0"
};

const char __RAL_lag_TZ_locale_grouping[] = "3";
const char __RAL_lag_TZ_locale_int_currency_symbol[] = "TZS";
const char __RAL_lag_TZ_locale_currency_symbol[] = "TSh";
const char __RAL_lag_TZ_locale_mon_grouping[] = "3";
const char __RAL_lag_TZ_locale_time_fmt[] = "%I:%M:%S %p";
const char __RAL_lag_TZ_locale_date_fmt[] = "%d/%m/%Y";
const char __RAL_lag_TZ_locale_date_time_fmt[] = "%e %B %Y %I:%M:%S %p %Z";
const char __RAL_lag_TZ_locale_am_pm_indicator[] = "TOO\0MUU\0";

const __CODE __RAL_locale_data_t __RAL_lag_TZ_locale = 
{
  __RAL_data_utf8_period,   // decimal_point
  __RAL_data_utf8_comma,   // thousands_sep
  __RAL_lag_TZ_locale_grouping,   // grouping
  __RAL_lag_TZ_locale_int_currency_symbol,   // int_currency_symbol
  __RAL_lag_TZ_locale_currency_symbol,   // currency_symbol
  __RAL_data_utf8_period,   // mon_decimal_point
  __RAL_data_utf8_comma,   // mon_thousands_sep
  __RAL_lag_TZ_locale_mon_grouping,   // mon_grouping
  __RAL_data_empty_string,   // positive_sign
  __RAL_data_utf8_minus,   // negative_sign
  0,	// int_frac_digits
  0,	// frac_digits
  1,	// p_cs_precedes
  1,	// p_sep_by_space
  1,	// n_cs_precedes
  1,	// n_sep_by_space
  1,	// p_sign_posn
  1,	// n_sign_posn
  1,	// int_p_cs_precedes
  1,	// int_n_cs_precedes
  1,	// int_p_sep_by_space
  1,	// int_n_sep_by_space
  1,	// int_p_sign_posn
  1,	// int_n_sign_posn
  __RAL_lag_TZ_locale_day_names,
  __RAL_lag_TZ_locale_abbrev_day_names,
  __RAL_lag_TZ_locale_month_names,
  __RAL_lag_TZ_locale_abbrev_month_names,
  __RAL_lag_TZ_locale_am_pm_indicator,
  __RAL_lag_TZ_locale_time_fmt,
  __RAL_lag_TZ_locale_date_fmt,
  __RAL_lag_TZ_locale_date_time_fmt,
};

