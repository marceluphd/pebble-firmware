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

const char __RAL_kam_KE_locale_day_names[] = 
{
  "Wa kyumwa\0"
  "Wa kwamb\304\251l\304\251lya\0"
  "Wa kel\304\251\0"
  "Wa katat\305\251\0"
  "Wa kana\0"
  "Wa katano\0"
  "Wa thanthat\305\251\0"
};

const char __RAL_kam_KE_locale_abbrev_day_names[] = 
{
  "Wky\0"
  "Wkw\0"
  "Wkl\0"
  "Wt\305\251\0"
  "Wkn\0"
  "Wtn\0"
  "Wth\0"
};

const char __RAL_kam_KE_locale_month_names[] = 
{
  "Mwai wa mbee\0"
  "Mwai wa kel\304\251\0"
  "Mwai wa katat\305\251\0"
  "Mwai wa kana\0"
  "Mwai wa katano\0"
  "Mwai wa thanthat\305\251\0"
  "Mwai wa muonza\0"
  "Mwai wa nyaanya\0"
  "Mwai wa kenda\0"
  "Mwai wa \304\251kumi\0"
  "Mwai wa \304\251kumi na \304\251mwe\0"
  "Mwai wa \304\251kumi na il\304\251\0"
};

const char __RAL_kam_KE_locale_abbrev_month_names[] = 
{
  "Mbe\0"
  "Kel\0"
  "Kt\305\251\0"
  "Kan\0"
  "Ktn\0"
  "Tha\0"
  "Moo\0"
  "Nya\0"
  "Knd\0"
  "\304\250ku\0"
  "\304\250km\0"
  "\304\250kl\0"
};

const char __RAL_kam_KE_locale_grouping[] = "3";
const char __RAL_kam_KE_locale_int_currency_symbol[] = "KES";
const char __RAL_kam_KE_locale_currency_symbol[] = "Ksh";
const char __RAL_kam_KE_locale_mon_grouping[] = "3";
const char __RAL_kam_KE_locale_time_fmt[] = "%I:%M:%S %p";
const char __RAL_kam_KE_locale_date_fmt[] = "%d/%m/%Y";
const char __RAL_kam_KE_locale_date_time_fmt[] = "%e %B %Y %I:%M:%S %p %Z";
const char __RAL_kam_KE_locale_am_pm_indicator[] = "\304\250yakwakya\0\304\250yaw\304\251oo\0";

const __CODE __RAL_locale_data_t __RAL_kam_KE_locale = 
{
  __RAL_data_utf8_period,   // decimal_point
  __RAL_data_utf8_comma,   // thousands_sep
  __RAL_kam_KE_locale_grouping,   // grouping
  __RAL_kam_KE_locale_int_currency_symbol,   // int_currency_symbol
  __RAL_kam_KE_locale_currency_symbol,   // currency_symbol
  __RAL_data_utf8_period,   // mon_decimal_point
  __RAL_data_utf8_comma,   // mon_thousands_sep
  __RAL_kam_KE_locale_mon_grouping,   // mon_grouping
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
  __RAL_kam_KE_locale_day_names,
  __RAL_kam_KE_locale_abbrev_day_names,
  __RAL_kam_KE_locale_month_names,
  __RAL_kam_KE_locale_abbrev_month_names,
  __RAL_kam_KE_locale_am_pm_indicator,
  __RAL_kam_KE_locale_time_fmt,
  __RAL_kam_KE_locale_date_fmt,
  __RAL_kam_KE_locale_date_time_fmt,
};

