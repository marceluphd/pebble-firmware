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

const char __RAL_nd_ZW_locale_day_names[] = 
{
  "Sonto\0"
  "Mvulo\0"
  "Sibili\0"
  "Sithathu\0"
  "Sine\0"
  "Sihlanu\0"
  "Mgqibelo\0"
};

const char __RAL_nd_ZW_locale_abbrev_day_names[] = 
{
  "Son\0"
  "Mvu\0"
  "Sib\0"
  "Sit\0"
  "Sin\0"
  "Sih\0"
  "Mgq\0"
};

const char __RAL_nd_ZW_locale_month_names[] = 
{
  "Zibandlela\0"
  "Nhlolanja\0"
  "Mbimbitho\0"
  "Mabasa\0"
  "Nkwenkwezi\0"
  "Nhlangula\0"
  "Ntulikazi\0"
  "Ncwabakazi\0"
  "Mpandula\0"
  "Mfumfu\0"
  "Lwezi\0"
  "Mpalakazi\0"
};

const char __RAL_nd_ZW_locale_abbrev_month_names[] = 
{
  "Zib\0"
  "Nhl\0"
  "Mbi\0"
  "Mab\0"
  "Nkw\0"
  "Nhl\0"
  "Ntu\0"
  "Ncw\0"
  "Mpa\0"
  "Mfu\0"
  "Lwe\0"
  "Mpa\0"
};

const char __RAL_nd_ZW_locale_grouping[] = "3";
const char __RAL_nd_ZW_locale_int_currency_symbol[] = "USD";
const char __RAL_nd_ZW_locale_currency_symbol[] = "US$";
const char __RAL_nd_ZW_locale_mon_grouping[] = "3";
const char __RAL_nd_ZW_locale_time_fmt[] = "%I:%M:%S %p";
const char __RAL_nd_ZW_locale_date_fmt[] = "%d/%m/%Y";
const char __RAL_nd_ZW_locale_date_time_fmt[] = "%e %B %Y %I:%M:%S %p %Z";
const char __RAL_nd_ZW_locale_am_pm_indicator[] = "AM\0PM\0";

const __CODE __RAL_locale_data_t __RAL_nd_ZW_locale = 
{
  __RAL_data_utf8_period,   // decimal_point
  __RAL_data_utf8_comma,   // thousands_sep
  __RAL_nd_ZW_locale_grouping,   // grouping
  __RAL_nd_ZW_locale_int_currency_symbol,   // int_currency_symbol
  __RAL_nd_ZW_locale_currency_symbol,   // currency_symbol
  __RAL_data_utf8_period,   // mon_decimal_point
  __RAL_data_utf8_comma,   // mon_thousands_sep
  __RAL_nd_ZW_locale_mon_grouping,   // mon_grouping
  __RAL_data_empty_string,   // positive_sign
  __RAL_data_empty_string,   // negative_sign
  2,	// int_frac_digits
  2,	// frac_digits
  1,	// p_cs_precedes
  0,	// p_sep_by_space
  1,	// n_cs_precedes
  0,	// n_sep_by_space
  1,	// p_sign_posn
  0,	// n_sign_posn
  1,	// int_p_cs_precedes
  1,	// int_n_cs_precedes
  0,	// int_p_sep_by_space
  0,	// int_n_sep_by_space
  1,	// int_p_sign_posn
  0,	// int_n_sign_posn
  __RAL_nd_ZW_locale_day_names,
  __RAL_nd_ZW_locale_abbrev_day_names,
  __RAL_nd_ZW_locale_month_names,
  __RAL_nd_ZW_locale_abbrev_month_names,
  __RAL_nd_ZW_locale_am_pm_indicator,
  __RAL_nd_ZW_locale_time_fmt,
  __RAL_nd_ZW_locale_date_fmt,
  __RAL_nd_ZW_locale_date_time_fmt,
};

