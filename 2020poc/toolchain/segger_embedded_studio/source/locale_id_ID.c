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

const char __RAL_id_ID_locale_day_names[] = 
{
  "Minggu\0"
  "Senin\0"
  "Selasa\0"
  "Rabu\0"
  "Kamis\0"
  "Jumat\0"
  "Sabtu\0"
};

const char __RAL_id_ID_locale_abbrev_day_names[] = 
{
  "Min\0"
  "Sen\0"
  "Sel\0"
  "Rab\0"
  "Kam\0"
  "Jum\0"
  "Sab\0"
};

const char __RAL_id_ID_locale_month_names[] = 
{
  "Januari\0"
  "Februari\0"
  "Maret\0"
  "April\0"
  "Mei\0"
  "Juni\0"
  "Juli\0"
  "Agustus\0"
  "September\0"
  "Oktober\0"
  "November\0"
  "Desember\0"
};

const char __RAL_id_ID_locale_abbrev_month_names[] = 
{
  "Jan\0"
  "Feb\0"
  "Mar\0"
  "Apr\0"
  "Mei\0"
  "Jun\0"
  "Jul\0"
  "Agt\0"
  "Sep\0"
  "Okt\0"
  "Nov\0"
  "Des\0"
};

const char __RAL_id_ID_locale_grouping[] = "3";
const char __RAL_id_ID_locale_int_currency_symbol[] = "IDR";
const char __RAL_id_ID_locale_currency_symbol[] = "Rp";
const char __RAL_id_ID_locale_mon_grouping[] = "3";
const char __RAL_id_ID_locale_time_fmt[] = "%H:%M:%S";
const char __RAL_id_ID_locale_date_fmt[] = "%d/%m/%y";
const char __RAL_id_ID_locale_date_time_fmt[] = "%e %B %Y %H:%M:%S %Z";
const char __RAL_id_ID_locale_am_pm_indicator[] = "pagi\0malam\0";

const __CODE __RAL_locale_data_t __RAL_id_ID_locale = 
{
  __RAL_data_utf8_comma,   // decimal_point
  __RAL_data_utf8_period,   // thousands_sep
  __RAL_id_ID_locale_grouping,   // grouping
  __RAL_id_ID_locale_int_currency_symbol,   // int_currency_symbol
  __RAL_id_ID_locale_currency_symbol,   // currency_symbol
  __RAL_data_utf8_comma,   // mon_decimal_point
  __RAL_data_utf8_period,   // mon_thousands_sep
  __RAL_id_ID_locale_mon_grouping,   // mon_grouping
  __RAL_data_empty_string,   // positive_sign
  __RAL_data_utf8_minus,   // negative_sign
  0,	// int_frac_digits
  0,	// frac_digits
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
  __RAL_id_ID_locale_day_names,
  __RAL_id_ID_locale_abbrev_day_names,
  __RAL_id_ID_locale_month_names,
  __RAL_id_ID_locale_abbrev_month_names,
  __RAL_id_ID_locale_am_pm_indicator,
  __RAL_id_ID_locale_time_fmt,
  __RAL_id_ID_locale_date_fmt,
  __RAL_id_ID_locale_date_time_fmt,
};

