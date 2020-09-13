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

#ifndef SUPPORT_WIDTH_PRECISION
#define SUPPORT_WIDTH_PRECISION 1
#endif

// By default, don't support wide char.
#ifndef SUPPORT_WCHAR
#define SUPPORT_WCHAR 0
#endif

#include <wchar.h>
#include <wctype.h>
#include <limits.h>

#if defined(WIN32)

#include <math.h>
#include <string.h>
#include <float.h>

#ifndef signbit
// No 'signbit' function.
static signbit(double f)
{
  union {
    double d;
    long long s;
  } u;

  u.d = f;
  return u.s < 0;
}
#endif

#ifdef WIN32
#define uintptr_t unsigned
#else
#include <stdint.h>
#endif

#define __MEMSPACE
#define __CODE

typedef unsigned __int64 uint64_t;
typedef __int64 int64_t;

#if _MSC_VER < 1900
#define INT8_C(x) (x)
#define UINT8(x) (x##U)
#define INT16_C(x) (x)
#define UINT16_C(x) (x##U)
#define INT32_C(x) (x##L)
#define UINT32_C(x) (x##UL)
#define INT64_C(x) ((int64_t)(x##L))
#define UINT64_C(x) ((uint64_t)(x##UL))
#endif

#define SUPPORT_LONG_LONG 1
#define SUPPORT_FLOAT 1
#include <stdarg.h>
#define __RAL_pow10(n) pow(10, n)
#define isCHAR(flag) ((flag) & FORMAT_CHAR)
#define isSHORT(flag) ((flag) & FORMAT_SHORT)
#define isLONG(flag) ((flag) & FORMAT_LONG)
#define isLONGLONG(flag) ((flag) & FORMAT_LONG_LONG)
#define isLONGDOUBLE(flag) ((flag) & FORMAT_LONGDOUBLE)

#ifndef isinf
#define isinf(x) (!_finite(x) && !_isnan(x))
#endif
#ifndef isnan
#define isnan(x) _isnan(x)
#endif

#include "__vfprintf_helpers.c"

#elif defined(UNIX)

#if defined(SOLARIS)
#include <inttypes.h>
#include <ieeefp.h>
int isinf(double x) { return !finite(x) && x==x; }
#else
#include <stdint.h>
#endif

#include <string.h>
#include <stdarg.h>
#include <math.h>

#define __MEMSPACE
#define __CODE

#define SUPPORT_LONG_LONG 1
#define SUPPORT_FLOAT 1

#define __RAL_pow10(n) pow(10, n)
#define isSHORT(flag) ((flag) & FORMAT_SHORT)
#define isLONG(flag) ((flag) & FORMAT_LONG)
#define isLONGLONG(flag) ((flag) & FORMAT_LONG_LONG)
#define isLONGDOUBLE(flag) ((flag) & FORMAT_LONGDOUBLE)

#include "__vfprintf_helpers.c"

#else

#if defined(__SHORT_DOUBLES)
float __RAL_pow10f(int);
#else
double __RAL_pow10(int);
#endif
#include "stdlib.h"
#include "stdarg.h"
#include "stdint.h"
#include "../libm_fast/math.h"
#include "string.h"
#define isSHORT(flag) (sizeof(short) < sizeof(int) && ((flag) & FORMAT_SHORT))
#define isLONG(flag) (SUPPORT_LONG && sizeof(long) > sizeof(int) && ((flag) & FORMAT_LONG))
#define isLONGLONG(flag) (SUPPORT_LONG_LONG && sizeof(uint64_t) >= sizeof(int) && ((flag) & FORMAT_LONG_LONG))
#define isLONGDOUBLE(flag) (sizeof(double) != sizeof(long double) && ((flag) & FORMAT_LONGDOUBLE))

#endif

extern __CODE const char __RAL_hex_uc[16];
extern __CODE const char __RAL_hex_lc[16];

// If we support doubles, then we also support long longs,
// and if we supports floats, then we also support longs.
#if defined(SUPPORT_FLOAT)
  #if defined(__SHORT_DOUBLES)
    #define SUPPORT_LONG 1
  #else
    #define SUPPORT_LONG_LONG 1
  #endif
#endif

#if SUPPORT_LONG_LONG

  // All long longs are 64 bits
  #define SUPPORT_LONG 1
  #define SUPPORT_INT 1
  #undef uvalue_t
  #define uvalue_t uint64_t
  #undef value_t
  #define value_t int64_t

#elif SUPPORT_LONG

  // All long longs are 32 bits
  #define SUPPORT_INT 1
  #define SUPPORT_LONG_LONG 0
  #undef uvalue_t
  #define uvalue_t unsigned long
  #undef value_t
  #define value_t long

#elif SUPPORT_INT

  // Ints are either 16 or 32 bits.
  #define SUPPORT_LONG 0
  #define SUPPORT_LONG_LONG 0
  #undef uvalue_t
  #define uvalue_t unsigned int
  #undef value_t
  #define value_t int

#else
#error "Need to define one of SUPPORT_INT, SUPPORT_LONG, or SUPPORT_LONG_LONG to 1"
#endif

#include "__vfprintf.h"

#define isFORMAT_E(flags)  flags & FORMAT

#if SUPPORT_FLOAT
#ifndef IPOW10
#define IPOW10
// 10^i
static __CODE const uint64_t ipow10[] = {
  UINT64_C(1),
  UINT64_C(10),
  UINT64_C(100),
  UINT64_C(1000),
  UINT64_C(10000),
  UINT64_C(100000),
  UINT64_C(1000000),
  UINT64_C(10000000),
  UINT64_C(100000000),
  UINT64_C(1000000000),
  UINT64_C(10000000000),
  UINT64_C(100000000000),
  UINT64_C(1000000000000),
  UINT64_C(10000000000000),
  UINT64_C(100000000000000),
  UINT64_C(1000000000000000),
  UINT64_C(10000000000000000),
  UINT64_C(100000000000000000),
  UINT64_C(1000000000000000000),
  UINT64_C(10000000000000000000)
};
#endif
#endif

#ifdef __SHORT_DOUBLES
// Maximum number of leading decimal digits in a float.
#define MAX_FLOAT_DIGITS 9
#else
// Maximum number of leading decimal digits in a double.
#define MAX_FLOAT_DIGITS 16
#endif

#ifndef __VFPRINTF
#define __VFPRINTF __vfprintf
#endif

void __RAL_compute_wide_metrics(const wchar_t *wstr,
                                unsigned max_bytes,
                                int *n_bytes);

void __VFPRINTF_PRINT_WIDE_STRING(__printf_t *p, const wchar_t *wstr, unsigned max_bytes);

#if SUPPORT_WCHAR
#ifndef RAL_COMPUTE_WIDE_METRICS
#define RAL_COMPUTE_WIDE_METRICS
void
__RAL_compute_wide_metrics(const wchar_t *wstr, unsigned max_bytes, int *n_bytes)
{
  mbstate_t state;
  char buf[MB_LEN_MAX];

  // Compute number of wide characters we can fir into max_bytes.
  __RAL_init_mbstate(&state);
  *n_bytes = 0;
  while (*wstr)
    {
      // Convert each character, one at a time.
      int n = wctomb(buf, *wstr++);
      if (*n_bytes + n > max_bytes)
        break;
      *n_bytes += n;
    }
}
#endif

#ifndef RAL_PRINT_WIDE_STRING
#define RAL_PRINT_WIDE_STRING
void
__VFPRINTF_PRINT_WIDE_STRING(__printf_t *p, const wchar_t *wstr, unsigned max_bytes)
{
  mbstate_t state;
  char buf[MB_LEN_MAX];
  unsigned nbytes;
  int i, n;

  // Compute number of wide characters we can fir into max_bytes.
  __RAL_init_mbstate(&state);
  nbytes = 0;
  while (*wstr)
    {
      // Convert each character, one at a time.
      n = wctomb(buf, *wstr++);
      if (nbytes + n > max_bytes)
        break;
      for (i = 0; i < n; ++i)
        __VFPRINTF_PUTC(p, buf[i]);
      nbytes += n;
    }
}
#endif
#endif

int
__VFPRINTF(__printf_t *p, __MEMSPACE const __VFPRINTF_CHAR *fmt, ARGTYPE args)
{
  __VFPRINTF_CHAR ch;
  int exp, len;
#if SUPPORT_FLOAT
#if __SHORT_DOUBLES
  uint32_t u;
#else
  uint64_t u;
#endif
#endif
#if SUPPORT_WCHAR
  wchar_t wc[2];
#endif

  // Number of characters that we must buffer for the largest octal value.
  char buff[(8*sizeof(value_t)+2)/3];

  union
    {
      uvalue_t u;
      value_t i;
      const char *str;
      const wchar_t *wstr;
#if defined(SUPPORT_FLOAT)
#if defined(__SHORT_DOUBLES)
      float r;
#else
      double r;
#endif
#endif
    } v;

  // No characters output yet.
  p->charcount = 0;

  while ((ch = *fmt++) != 0)
    {
      if (ch != '%')
        __VFPRINTF_PUTC(p, ch);
      else
        {
#if SUPPORT_WIDTH_PRECISION
          int width = 0, precision = 0;
#endif
          int flags = 0;
          unsigned prefix = 0;

          // Gather flags.
          for (;;)
            {
              switch (ch = *fmt++)
                {
                case ' ':
                  flags |= FORMAT_SPACE;
                  continue;

                case '#':
                  flags |= FORMAT_ALTERNATIVE;
                  continue;

                case '\'':
                  flags |= FORMAT_TICK;
                  continue;

                case '^':
                  flags |= FORMAT_ENGINEERING;
                  continue;

                case '+':
                  flags |= FORMAT_SIGNED;
                  continue;

#if SUPPORT_WIDTH_PRECISION
                case '-':
                  flags |= FORMAT_LEFT_JUSTIFY;
                  continue;

                case '0':
                  flags |= FORMAT_ZERO_FILL;
                  continue;
#endif
                }
              break;
            }

          // Find width specification.
#if SUPPORT_WIDTH_PRECISION
          if (ch == '*')
            {
              width = ARGTYPE_getIntArg(args);
              if (width < 0)
                {
                  width = -width;
                  flags |= FORMAT_LEFT_JUSTIFY;
                }
              ch = *fmt++;
            }
          else
            {
              while ('0' <= ch && ch <= '9')
                {
                  width = width*10 + (ch-'0');
                  ch = *fmt++;
                }
            }
          if (width < 0)
            width = 0;

          // Deal with precision specification.
          if (ch == '.')
            {
              ch = *fmt++;
              if (ch == '*')
                {
                  precision = ARGTYPE_getIntArg(args);
                  ch = *fmt++;
                }
              else
                while ('0' <= ch && ch <= '9')
                  {
                    precision = precision*10 + (ch-'0');
                    ch = *fmt++;
                  }
              if (precision >= 0)
                flags |= FORMAT_HAVE_PRECISION;
            }
#endif

          // Deal with 'z' and 't' specifiers which, for the architectures
          // we support, are the same size as a pointer and integer.  'z' is
          // a size_t and 't' is a ptrdiff_t.
          if (ch == 't' || ch == 'z')
            {
              ch = *fmt++;
            }
          // Only support 'j' when we have long-long support as intmax_t
          // is long long.
          else if (SUPPORT_LONG_LONG && ch == 'j')
            {
              flags |= FORMAT_LONG_LONG;
              ch = *fmt++;
            }
          // Deal with long specifiers.  For wide chars, we need to support
          // %lc and %ls but don't need to support %ld or %li, for instance.
          else if ((SUPPORT_LONG || SUPPORT_WCHAR) && ch == 'l')
            {
              ch = *fmt++;
              if (SUPPORT_LONG_LONG && ch == 'l')
                {
                  flags |= FORMAT_LONG_LONG;
                  ch = *fmt++;
                }
              else
                flags |= FORMAT_LONG;
            }
          // Uses ISO/IEC 8988:1999(E) semantics where 'L' is long double.
          else if (SUPPORT_LONG && ch == 'L')
            {
              flags |= FORMAT_LONG;
            }
          // Deal with short specifiers.
          else if (ch == 'h')
            {
              if ((ch = *fmt++) == 'h')
                {
                  flags |= FORMAT_CHAR;
                  ch = *fmt++;
                }
              else
                flags |= FORMAT_SHORT;
            }

          switch (ch)
            {
            default:
#if SUPPORT_WIDTH_PRECISION
              if (__VFPRINTF_USER_FORMAT_EXTENDER)
                __VFPRINTF_USER_FORMAT_EXTENDER(ch, p, flags, width, precision, &args);
#else
              if (__VFPRINTF_USER_FORMAT_EXTENDER)
                __VFPRINTF_USER_FORMAT_EXTENDER(ch, p, flags, -1, -1, &args);
#endif
              continue;

            // End of format string and no format character at end.
            case 0:
              // Do what C99 expects and return a -ve value.
              return -1;

            case '%':
              __VFPRINTF_PUTC(p, ch);
              continue;

            case 'c':
#if SUPPORT_WCHAR
              if (flags & FORMAT_LONG)
                {
                  // wint_t argument -- it can be considered long.
                  wc[0] = ARGTYPE_getLongArg(args);
                  wc[1] = 0;
                  flags &= ~FORMAT_HAVE_PRECISION;
                  v.wstr = wc;
                  goto sformat;
                }
              else
                {
                  ch = ARGTYPE_getIntArg(args);
                }
#else
              ch = ARGTYPE_getIntArg(args);
              // FALLTHROUGH
#endif

              // Default handling of unrecognized characters is to print them.
#if SUPPORT_WIDTH_PRECISION
              --width;
              __VFPRINTF_PRE_PADDING(flags, width, p);
#endif
              __VFPRINTF_PUTC(p, ch);
postpad:
#if SUPPORT_WIDTH_PRECISION
              if (flags & FORMAT_LEFT_JUSTIFY)
                __VFPRINTF_PRINT_PADDING(' ', width, p);
#endif
              continue;

            case 'n':
              if (flags & FORMAT_CHAR)
                {
                  ARGTYPE_setCharPtrArg(args, p->charcount);
                }
              else if (SUPPORT_LONG_LONG && isLONGLONG(flags))
                {
                  ARGTYPE_setLongLongPtrArg(args, p->charcount);
                }
              else if (SUPPORT_LONG && isLONG(flags))
                {
                  ARGTYPE_setLongPtrArg(args, p->charcount);
                }
              else
                {
                  ARGTYPE_setIntPtrArg(args, p->charcount);
                }
              continue;

            case 's':
              // Grab argument.
              v.str = ARGTYPE_getStringArg(args);

#if SUPPORT_WCHAR
sformat:
              if (v.str && (flags & FORMAT_LONG))
                {
#if SUPPORT_WIDTH_PRECISION
                  // Compute wide metrics.
                  if (width == 0 && (flags & FORMAT_HAVE_PRECISION) == 0)
                    {
                      // Just %ls, whole string up to trailing zero goes
                      // out.
                      __RAL_compute_wide_metrics(v.wstr, INT_MAX, &len);
                    }
                  else if (flags & FORMAT_HAVE_PRECISION)
                    {
                      // No width specified, but precision is specified.
                      __RAL_compute_wide_metrics(v.wstr, precision, &len);
                    }
                  else
                    {
                      // Precision is not specified but width is specified.
                      __RAL_compute_wide_metrics(v.wstr, width, &len);
                    }

                  // Pre and post padding...
                  width -= len;
                  __VFPRINTF_PRE_PADDING(flags, width, p);
                  __VFPRINTF_PRINT_WIDE_STRING(p, v.wstr, len);
#else
                  __VFPRINTF_PRINT_WIDE_STRING(p, v.wstr, INT_MAX);
#endif
                  goto postpad;
                }
#endif

// Format a narrow string...
fmtstr:
#if SUPPORT_WIDTH_PRECISION
              // This is what glibc does, so do it too.
              if (v.str == 0)
                v.str = "(null)";

              // We do not zero-fill strings.
              flags &= ~FORMAT_ZERO_FILL;

              // Subtle differences when a precision is specified.
              if (flags & FORMAT_HAVE_PRECISION)
                {
                  // Precision specified; string does not necessarily
                  // need to be terminated with a null.
                  len = strnlen(v.str, precision);
                }
              else
                {
                  // No precision specified, string is as long as it is...
                  len = strlen(v.str);
                }

              width -= len;
              __VFPRINTF_PRE_PADDING(flags, width, p);
              while (len--)
                __VFPRINTF_PUTC(p, *v.str++);
              goto postpad;
#else
              while (*v.str)
                __VFPRINTF_PUTC(p, *v.str++);
              continue;
#endif

            case 'p':
              v.u = (uvalue_t)(uintptr_t)ARGTYPE_getPtrArg(args);
              if (flags & FORMAT_ALTERNATIVE)
                prefix = '#';
              flags |= FORMAT_HAVE_PRECISION;
#if SUPPORT_WIDTH_PRECISION
              precision = sizeof(void *) * 2;  // Assumes sizeof() delivers bytes...
#endif
              break;

            case 'X':
              flags |= FORMAT_CAPITALS;
            case 'x':
              if (flags & FORMAT_ALTERNATIVE)
                prefix = ch == 'x' ? ('0'*0x100+'x') : ('0'*0x100+'X'); // "0x" : "0X"
              if (SUPPORT_WIDTH_PRECISION && (flags & FORMAT_HAVE_PRECISION))
                flags &= ~FORMAT_ZERO_FILL;
              goto fmtint;

            case 'o':
              if (flags & FORMAT_ALTERNATIVE)
                prefix = '0';
              if (SUPPORT_WIDTH_PRECISION && (flags & FORMAT_HAVE_PRECISION))
                flags &= ~FORMAT_ZERO_FILL;
              goto fmtint;

            case 'u':
              if (SUPPORT_WIDTH_PRECISION && (flags & FORMAT_HAVE_PRECISION))
                flags &= ~FORMAT_ZERO_FILL;
              goto fmtint;

            case 'i':
            case 'd':
              flags |= FORMAT_INPUT_SIGNED;
fmtint:

              if (flags & FORMAT_INPUT_SIGNED)
                {
                  if (SUPPORT_LONG_LONG && isLONGLONG(flags))
                    v.i = ARGTYPE_getLongLongArg(args);
                  else if (SUPPORT_LONG && isLONG(flags))
                    v.i = ARGTYPE_getLongArg(args);
                  else
                    v.i = ARGTYPE_getIntArg(args);

                  if (isSHORT(flags))
                    v.i = (short)v.i;
                  else if (flags & FORMAT_CHAR)
                    v.i = (char)v.i;

                  if (v.i < 0)
                    {
                      v.u = 0U-v.u;
                      prefix = '-';
                    }
                  else
                    {
                      if (flags & FORMAT_SIGNED)
                        prefix = '+';
                      else if (flags & FORMAT_SPACE)
                        prefix = ' ';
                    }
                }
              else
                {
                  if (SUPPORT_LONG_LONG && isLONGLONG(flags))
                    v.u = ARGTYPE_getLongLongArg(args);
                  else if (SUPPORT_LONG && isLONG(flags))
                    v.u = ARGTYPE_getUnsignedLongArg(args);
                  else
                    v.u = ARGTYPE_getUnsignedArg(args);

                  if (isSHORT(flags))
                    v.u = (unsigned short)v.u;
                  else if (flags & FORMAT_CHAR)
                    v.u = (unsigned char)v.u;
                }

              if (SUPPORT_WIDTH_PRECISION && (flags & FORMAT_HAVE_PRECISION))
                flags &= ~FORMAT_ZERO_FILL;
              break;

#if defined(SUPPORT_FLOAT)
#if 0
            case 'A':
              flags |= FORMAT_CAPITALS;
            case 'a':
              r = ARGTYPE_getDoubleArg(args);
              prefix = 0;
              v = 0;
              break;
#endif

            case 'E':
              flags |= FORMAT_CAPITALS;
            case 'e':
              flags |= FORMAT_FLOAT_E;
              goto fmtreal;

            case 'F':
              flags |= FORMAT_CAPITALS;
            case 'f':
              flags |= FORMAT_FLOAT_F;
              goto fmtreal;

            case 'G':
              flags |= FORMAT_CAPITALS;
            case 'g':
              flags |= FORMAT_FLOAT_G;
fmtreal:
              // Hate this inline, but for small micros, need to
              // conserve stack space.  Yerch.
#ifdef __SHORT_DOUBLES
              v.r = (float)ARGTYPE_getDoubleArg(args);
#else
              v.r = ARGTYPE_getDoubleArg(args);
#endif

              // Supply default precision.
              if ((flags & FORMAT_HAVE_PRECISION) == 0)
                precision = 6;

              // Precision is defined as a minimum of 1 for %g with specified precision=0.
              if (precision == 0 &&
                  (flags & FORMAT_FLOAT_G) == FORMAT_FLOAT_G)
                precision = 1;

              if (isinf(v.r))
                {
                  if (v.r < 0)
                    {
                      v.str = flags & FORMAT_CAPITALS ? "-INF" : "-inf";
                    }
                  else
                    {
                      v.str = flags & FORMAT_CAPITALS ? "+INF" : "+inf";
                      if ((flags & FORMAT_SIGNED) == 0)
                        ++v.str;
                    }
                  flags &= ~FORMAT_HAVE_PRECISION;
                  goto fmtstr;
                }
              if (isnan(v.r))
                {
                  v.str = flags & FORMAT_CAPITALS ? "NAN" : "nan";
                  flags &= ~FORMAT_HAVE_PRECISION;
                  goto fmtstr;
                }

              if (signbit(v.r))
                {
                  flags |= FORMAT_NEGATIVE;
                  v.r = -v.r;
                }

              // Get base-2 exponent.
#ifdef __SHORT_DOUBLES
              frexpf(v.r, &exp);
#else
              frexp(v.r, &exp);
#endif

              // Convert to base-10 exponent.  log10(2) is approximately
              // 77/256, but that means we need to use 32-bit arithmetic
              // to compute 0x3ff*77 without overflow, so use the cruder
              // value 3/10.
              exp = exp * 3 / 10;

              // Adjust to get true log10(v); correct for overestimation.
              if (v.r)
                {
#ifdef __SHORT_DOUBLES
                  while (v.r > __RAL_pow10f(exp+1))
#else
                  while (v.r > __RAL_pow10(exp+1))
#endif
                    ++exp;
#ifdef __SHORT_DOUBLES
                  while (v.r < __RAL_pow10f(exp))
#else
                  while (v.r < __RAL_pow10(exp))
#endif
                    --exp;
                }

              // If our value is out of range or requires exponential format, normalise it.
              // In this case, 1 <= r < 10.
              if ((flags & FORMAT_FLOAT_G) == FORMAT_FLOAT_E ||
                  ((flags & FORMAT_FLOAT_G) == FORMAT_FLOAT_G && !(precision > exp && exp >= -4)))
                {
                  // 'E' conversion.
#ifdef __SHORT_DOUBLES
                  v.r *= __RAL_pow10f(-exp);
#else
                  // Can't represent 1e-308.
                  if (exp == 308)
                    {
                      v.r /= 1.e308;
                    }
                  else
                    v.r *= __RAL_pow10(-exp);
#endif
                  if (v.r)
                    {
                      if (isinf(v.r))
                        {
                          if (v.r < 0)
                            v.str = flags & FORMAT_CAPITALS ? "-INF" : "-inf";
                          else
                            {
                              v.str = flags & FORMAT_CAPITALS ? "+INF" : "+inf";
                              if ((flags & FORMAT_SIGNED) == 0)
                                ++v.str;
                            }
                          flags &= ~FORMAT_HAVE_PRECISION;
                          goto fmtstr;
                        }
#ifdef __SHORT_DOUBLES
                      while (v.r >= 10.0f)
                        ++exp, v.r /= 10.0f;  // don't use "r *= 0.1" as 0.1 introduces errors
                      while (v.r < 1.0f)
                        --exp, v.r *= 10.0f;
#else
                      while (v.r >= 10.0)
                        ++exp, v.r /= 10.0;  // don't use "r *= 0.1" as 0.1 introduces errors
                      while (v.r < 1.0)
                        --exp, v.r *= 10.0;
#endif
                    }

                  // E format
                  if (precision && (flags & FORMAT_FLOAT_G) == FORMAT_FLOAT_G)
                    --precision;

                  // Round value.
#ifdef __SHORT_DOUBLES
                  v.r += __RAL_pow10f(-precision) * 0.5F;
                  if (v.r >= 10.0f)
                    ++exp, v.r /= 10.0f;  // don't use "r *= 0.1" as 0.1 introduces errors
#else
                  v.r += __RAL_pow10(-precision) * 0.5F;
                  if (v.r >= 10.0)
                    ++exp, v.r /= 10.0;  // don't use "r *= 0.1" as 0.1 introduces errors
#endif

                  // G format adjustment; trim trailing zeroes.
                  if (precision && (flags & FORMAT_FLOAT_F) && (flags & FORMAT_ALTERNATIVE) == 0)
                    {
                      if (precision > MAX_FLOAT_DIGITS)
                        precision = MAX_FLOAT_DIGITS;
#ifdef __SHORT_DOUBLES
                      u = (uint32_t)(v.r * __RAL_pow10f(precision));
#else
                      u = (uint64_t)(v.r * __RAL_pow10(precision));
#endif
                      if (u)
                        while (precision && (u % 10 == 0))
                          --precision, u /= 10;
                      else
                        precision = 0;
                    }
                  flags &= ~FORMAT_FLOAT_F;

                  // Figure out number of significant digits.
                  width -= (5 + precision);    // 5 chars minimum, dE+ee

                  // Decimal point required?
                  if (precision || (flags & FORMAT_ALTERNATIVE))
                    --width;

                  // Engineering notation?
                  if (flags & FORMAT_ENGINEERING)
                    {
                      switch (exp % 3)
                        {
                        case          0: len = 1;                                   break;
                        case -2: case 1: len = 2; v.r *= 10;  exp -= 1; width -= 1; break;
                        case -1: case 2: len = 3; v.r *= 100; exp -= 2; width -= 2; break;
                        }
                      if (precision < 0)
                        precision = 0;
                    }
                  else
                    len = 1;

                  // Leading sign or blank?
                  if (flags & FORMAT_NEGATIVE)
                    --width;

                  // Three digit exponent?
                  if (abs(exp) >= 100)
                    --width;
#ifdef __SHORT_DOUBLES
                  u = (uint32_t)v.r;
#else
                  u = (uint64_t)v.r;
#endif
#ifdef WIN32  // lame compiler
                  v.r -= (double)(int64_t)u;
#else
#ifdef __SHORT_DOUBLES
                  v.r -= (float)u;
#else
                  v.r -= (double)u;
#endif
#endif
                }
              else
                {
                  // 'F' conversion for '%f' or '%g' format.
                  if (flags & FORMAT_FLOAT_E)
                    {
                      // 'g' format, precision > 0.
                      precision -= exp+1;
                      if (precision > MAX_FLOAT_DIGITS)
                        precision = MAX_FLOAT_DIGITS;

                      if (exp >= 15)
                        {
                          // Don't you admire users' boundless optimism?
                          precision = 0;
                        }
                      else if ((flags & FORMAT_ALTERNATIVE) == 0)
                        {
                          // How many zeroes do we have to trim?
                          while (precision)
                            {
#ifdef __SHORT_DOUBLES
                              float d = floorf(v.r * __RAL_pow10f(precision) + 0.5f);
                              if (fmodf(d, 10) == 0)
#else
                              double d = floor(v.r * __RAL_pow10(precision) + 0.5);
                              if (fmod(d, 10) == 0)
#endif
                                --precision;
                              else
                                break;
                            }
                        }
                    }

                  // Remove 'G' format specified to go to 'F' format.
                  flags &= ~FORMAT_FLOAT_E;

                  // Round value to appropriate precision.
                  if (exp-MAX_FLOAT_DIGITS >= -precision)
#ifdef __SHORT_DOUBLES
                    v.r += ldexpf(__RAL_pow10f(exp-MAX_FLOAT_DIGITS), -1);
#else
                    v.r += ldexp(__RAL_pow10(exp-MAX_FLOAT_DIGITS), -1);
#endif
                  else
#ifdef __SHORT_DOUBLES
                    v.r += ldexpf(__RAL_pow10f(-precision), -1);
#else
                    v.r += ldexp(__RAL_pow10(-precision), -1);
#endif
#ifdef __SHORT_DOUBLES
                  if (v.r >= __RAL_pow10f(exp+1))
#else
                  if (v.r >= __RAL_pow10(exp+1))
#endif
                    ++exp;

                  // Compute width of 'F' conversion
                  if (exp >= 0)
                    {
                      if (exp > MAX_FLOAT_DIGITS)
                        {
                          exp -= MAX_FLOAT_DIGITS;
#ifdef __SHORT_DOUBLES
                          u = (uint32_t)(v.r * __RAL_pow10f(-exp));
#else
                          u = (uint64_t)(v.r * __RAL_pow10(-exp));
#endif
                          v.r = 0.0;
                        }
                      else
                        {
#ifdef __SHORT_DOUBLES
                          u = (uint32_t)v.r;
#else
                          u = (uint64_t)v.r;
#endif
#ifdef WIN32  // lame compiler
                          v.r -= (double)(int64_t)u;
#else
#ifdef __SHORT_DOUBLES
                          v.r -= (float)u;
#else
                          v.r -= (double)u;
#endif
#endif
                          exp = 0;
                        }
                    }
                  else
                    {
                      u = 0;
                      exp = 0;
                    }

                  // Compute number of leading digits before decimal point.
                  len = 1;
                  while (u >= ipow10[len])
                    ++len;

                  // Reduce width specification by that.
                  width -= precision + len + exp;

                  // Decimal point required?
                  if (precision || (flags & FORMAT_ALTERNATIVE))
                    --width;

                  // Sign or blanker?
                  if (flags & FORMAT_NEGATIVE)
                    --width;
                }

              // Must print something...
              if (width < 0)
                width = 0;

              // Pad left
              if ((flags & (FORMAT_LEFT_JUSTIFY | FORMAT_ZERO_FILL)) == 0)
                while (width)
                  --width, __VFPRINTF_PUTC(p, ' ');

              // Sign
              if (flags & FORMAT_SIGNED)
                __VFPRINTF_PUTC(p, flags & FORMAT_SPACE ? '-' : '+');
              else if (flags & FORMAT_SPACE)
                __VFPRINTF_PUTC(p, ' ');

              if ((flags & FORMAT_LEFT_JUSTIFY) == 0)
                while (width)
                  --width, __VFPRINTF_PUTC(p, '0');

              // Print digits before decimal point.
              do
                {
                  --len;
                  ch = '0';
                  while (u >= ipow10[len])
                    ++ch, u -= ipow10[len];
                  __VFPRINTF_PUTC(p, ch);
                }
              while (len);

              // Additional power-of-ten digits past significance, sigh.
              if (flags & FORMAT_FLOAT_F)
                while (exp > 0)
                  --exp, __VFPRINTF_PUTC(p, '0');

              if (precision || flags & FORMAT_ALTERNATIVE)
                {
                  __VFPRINTF_PUTC(p, '.');
                  if (precision > 0)
                    {
                      if (precision > MAX_FLOAT_DIGITS)
                        len = MAX_FLOAT_DIGITS;
                      else
                        len = precision;
                    }

                  // Shift decimal part so that it's an integer.
                  precision -= len;
                  if (flags & FORMAT_FLOAT_F)
#ifdef __SHORT_DOUBLES
                    u = (uint32_t)(v.r * __RAL_pow10f(len-exp));
#else
                    u = (uint64_t)(v.r * __RAL_pow10(len-exp));
#endif
                  else
#ifdef __SHORT_DOUBLES
                    u = (uint32_t)(v.r * __RAL_pow10f(len));
#else
                    u = (uint64_t)(v.r * __RAL_pow10(len));
#endif

                  // Print decimal digits.
                  while (len)
                    {
                      --len;
                      ch = '0';
                      while (u >= ipow10[len])
                        ++ch, u -= ipow10[len];
                      __VFPRINTF_PUTC(p, ch);
                    }

                  // Print trailing decimal digits past reasonable precision.
                  while (precision)
                    --precision, __VFPRINTF_PUTC(p, '0');
                }

              if (flags & FORMAT_FLOAT_E)
                {
                  // Deal with exponent...
                  __VFPRINTF_PUTC(p, flags & FORMAT_CAPITALS ? 'E' : 'e');
                  if (exp < 0)
                    __VFPRINTF_PUTC(p, '-'), exp = -exp;
                  else
                    __VFPRINTF_PUTC(p, '+');

                  if (exp >= 100)
                    {
                      __VFPRINTF_PUTC(p, (exp / 100 + '0'));
                      exp %= 100;
                    }
                  __VFPRINTF_PUTC(p, exp / 10 + '0');
                  __VFPRINTF_PUTC(p, exp % 10 + '0');
                }

              // Pad right
              while (width)
                --width, __VFPRINTF_PUTC(p, ' ');
              continue;
#endif
            }

          len = 0;

#if SUPPORT_WIDTH_PRECISION
          if ((flags & FORMAT_HAVE_PRECISION) == 0)
            precision = 1;
#endif

          switch (ch)
            {
            case 'p':
            case 'X':
            case 'x':
#if SUPPORT_WIDTH_PRECISION == 0
              if (v.u == 0)
                buff[len++] = '0';
#endif
              while (v.u)
                {
                  if (flags & FORMAT_CAPITALS)
                    buff[len] = __RAL_hex_uc[v.u & 0xf];
                  else
                    buff[len] = __RAL_hex_lc[v.u & 0xf];
                  ++len;
                  v.u >>= 4;
                }
              break;
            case 'o':
#if SUPPORT_WIDTH_PRECISION == 0
              if (v.u == 0)
                buff[len++] = '0';
#endif
              while (v.u)
                {
                  buff[len] = '0' + ((unsigned)v.u & 07);
                  ++len;
                  v.u >>= 3;
                }
              break;
            case 'u':
            case 'i':
            case 'd':
#if SUPPORT_WIDTH_PRECISION == 0
              if (v.u == 0)
                buff[len++] = '0';
#endif
              while (v.u)
                {
                  if (flags & FORMAT_TICK)
                    if ((len & 3) == 3)
                      {
                        buff[len] = ',';
                        ++len;
                      }
                  buff[len] = '0' + (unsigned)(v.u % 10U);
                  ++len;
                  v.u /= 10U;
                }
              break;
            }

#if SUPPORT_WIDTH_PRECISION
          if ((precision -= len) < 0)
            precision = 0;

          // Compute padding.
          width -= precision;
          width -= len;
          if (prefix >= 0x100)
            --width;
          if (prefix)
            --width;
#if 0
          if ((flags & FORMAT_ZERO_FILL) == 0)
            {
              __VFPRINTF_PRE_PADDING(flags, width, p);
              width = 0;
            }
#endif
#endif
          if (prefix >= 0x100)
            __VFPRINTF_PUTC(p, prefix >> 8);
          if (prefix)
            __VFPRINTF_PUTC(p, prefix);
#if SUPPORT_WIDTH_PRECISION
          __VFPRINTF_PRE_PADDING(flags, width, p);
          __VFPRINTF_PRINT_PADDING('0', precision, p);
#endif
          while (--len >= 0)
            __VFPRINTF_PUTC(p, buff[len]);

#if SUPPORT_WIDTH_PRECISION
          if (flags & FORMAT_LEFT_JUSTIFY)
            __VFPRINTF_PRINT_PADDING(' ', width, p);
#endif
        }
    }

  // Null terminate string if we haven't overflowed.
  if (p->string && p->charcount < p->maxchars)
    p->string[p->charcount] = 0;

  return p->charcount;
}

