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

const char __RAL_kw_GB_locale_day_names[] = 
{
  "De Sul\0"
  "De Lun\0"
  "De Merth\0"
  "De Merher\0"
  "De Yow\0"
  "De Gwener\0"
  "De Sadorn\0"
};

const char __RAL_kw_GB_locale_abbrev_day_names[] = 
{
  "Sul\0"
  "Lun\0"
  "Mth\0"
  "Mhr\0"
  "Yow\0"
  "Gwe\0"
  "Sad\0"
};

const char __RAL_kw_GB_locale_month_names[] = 
{
  "Mys Genver\0"
  "Mys Whevrel\0"
  "Mys Merth\0"
  "Mys Ebrel\0"
  "Mys Me\0"
  "Mys Efan\0"
  "Mys Gortheren\0"
  "Mye Est\0"
  "Mys Gwyngala\0"
  "Mys Hedra\0"
  "Mys Du\0"
  "Mys Kevardhu\0"
};

const char __RAL_kw_GB_locale_abbrev_month_names[] = 
{
  "Gen\0"
  "Whe\0"
  "Mer\0"
  "Ebr\0"
  "Me\0"
  "Efn\0"
  "Gor\0"
  "Est\0"
  "Gwn\0"
  "Hed\0"
  "Du\0"
  "Kev\0"
};

const char __RAL_kw_GB_locale_grouping[] = "3";
const char __RAL_kw_GB_locale_int_currency_symbol[] = "GBP";
const char __RAL_kw_GB_locale_currency_symbol[] = "\302\243";
const char __RAL_kw_GB_locale_mon_grouping[] = "3";
const char __RAL_kw_GB_locale_time_fmt[] = "%H:%M:%S";
const char __RAL_kw_GB_locale_date_fmt[] = "%d/%m/%Y";
const char __RAL_kw_GB_locale_date_time_fmt[] = "%e %B %Y %H:%M:%S %Z";
const char __RAL_kw_GB_locale_am_pm_indicator[] = "a.m.\0p.m.\0";

const __CODE __RAL_locale_data_t __RAL_kw_GB_locale = 
{
  __RAL_data_utf8_period,   // decimal_point
  __RAL_data_utf8_comma,   // thousands_sep
  __RAL_kw_GB_locale_grouping,   // grouping
  __RAL_kw_GB_locale_int_currency_symbol,   // int_currency_symbol
  __RAL_kw_GB_locale_currency_symbol,   // currency_symbol
  __RAL_data_utf8_period,   // mon_decimal_point
  __RAL_data_utf8_comma,   // mon_thousands_sep
  __RAL_kw_GB_locale_mon_grouping,   // mon_grouping
  __RAL_data_empty_string,   // positive_sign
  __RAL_data_utf8_minus,   // negative_sign
  2,	// int_frac_digits
  2,	// frac_digits
  1,	// p_cs_precedes
  0,	// p_sep_by_space
  1,	// n_cs_precedes
  0,	// n_sep_by_space
  1,	// p_sign_posn
  1,	// n_sign_posn
  1,	// int_p_cs_precedes
  1,	// int_n_cs_precedes
  0,	// int_p_sep_by_space
  0,	// int_n_sep_by_space
  1,	// int_p_sign_posn
  1,	// int_n_sign_posn
  __RAL_kw_GB_locale_day_names,
  __RAL_kw_GB_locale_abbrev_day_names,
  __RAL_kw_GB_locale_month_names,
  __RAL_kw_GB_locale_abbrev_month_names,
  __RAL_kw_GB_locale_am_pm_indicator,
  __RAL_kw_GB_locale_time_fmt,
  __RAL_kw_GB_locale_date_fmt,
  __RAL_kw_GB_locale_date_time_fmt,
};

