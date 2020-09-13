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

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <stdarg.h>
#include "__vfprintf.h"

#if defined(WIN32)

#define __MEMSPACE
#define __CODE
typedef unsigned __int64 uint64_t;
typedef __int64 int64_t;
typedef int charcount_t;
int __digit(int ch, int radix);
#define SUPPORT_LONG_LONG 1
#define isinf !_finite
#define __RAL_pow10(n) pow(10, n)
#include <float.h>

#define isSHORT(flag) ((flag) & FLAG_SHORT)
#define isLONG(flag) ((flag) & FLAG_LONG)
#define isLONGLONG(flag) ((flag) & FLAG_LONG_LONG)
#define isLONGDOUBLE(flag) ((flag) & FLAG_LONGDOUBLE)

#elif defined(UNIX)

#define __MEMSPACE
#define __CODE
#define SUPPORT_LONG_LONG 1
#define SUPPORT_FLOAT 1

#if defined(__sparc__)
#include <inttypes.h>
#else
#include <stdint.h>
#endif

typedef int charcount_t;
int __digit(int ch, int radix);
#define __RAL_pow10(n) pow(10, n)

#define isSHORT(flag) ((flag) & FLAG_SHORT)
#define isLONG(flag) ((flag) & FLAG_LONG)
#define isLONGLONG(flag) ((flag) & FLAG_LONG_LONG)
#define isLONGDOUBLE(flag) ((flag) & FLAG_LONGDOUBLE)

#else

#include "stddef.h"
#include "stdint.h"
#include "__libc.h"
#undef charcount_t
#define charcount_t ptrdiff_t

// Macros, with the help of the compiler, ensure that we can
// test for LONG and SHORT properly, but not generate extra code.
// Fabulous.
#define isSHORT(flag) (sizeof(short) != sizeof(int) && ((flag) & FLAG_SHORT))
#define isLONG(flag) (SUPPORT_LONG && sizeof(int) != sizeof(long) && ((flag) & FLAG_LONG))
#define isLONGLONG(flag) (SUPPORT_LONG_LONG && sizeof(int64_t) != sizeof(long) && ((flag) & FLAG_LONG_LONG))
#define isLONGDOUBLE(flag) (sizeof(double) != sizeof(long double) && ((flag) & FLAG_LONGDOUBLE))

#endif

#if SUPPORT_LONG_LONG
#define SUPPORT_LONG 1
#define SUPPORT_INT 1
#undef uvalue_t
#define uvalue_t uint64_t
#undef value_t
#define value_t int64_t
#undef RD_INT
#define RD_INT rd_int_long_long
#elif SUPPORT_LONG
#define SUPPORT_INT 1
#define SUPPORT_LONG_LONG 0
#undef uvalue_t
#define uvalue_t unsigned long
#undef value_t
#define value_t long
#undef RD_INT
#define RD_INT rd_int_long
#elif SUPPORT_INT
#define SUPPORT_LONG 0
#define SUPPORT_LONG_LONG 0
#undef uvalue_t
#define uvalue_t unsigned int
#undef value_t
#define value_t int
#undef RD_INT
#define RD_INT rd_int
#else
#error "Need to define one of SUPPORT_INT, SUPPORT_LONG, or SUPPORT_LONG_LONG to 1"
#endif

// some macros for scanf("%[...]")
#define MAPTYPE int
#define MAPTYPEBITS (sizeof(MAPTYPE) * 8)
#define CHARLIMIT 256

#define FLAG_NOSTORE      (1<<0)
#define FLAG_LONG_LONG    (1<<1)
#define FLAG_LONG         (1<<2)
#define FLAG_SHORT        (1<<3)
#define FLAG_CHAR         (1<<4)
#define FLAG_FIELDGIVEN   (1<<5)
#define FLAG_LONGDOUBLE   (1<<6)
#define FLAG_ALLOWSIGN    (1<<7)
#define FLAG_DOTSEEN      (1<<8)
#define FLAG_NUMOK        (1<<9)
#define FLAG_NUMNEG       (1<<10)
#define FLAG_EXPNEG       (1<<11)

#define countgetc(p) (++charcount, __VFPRINTF_GETC(p))

#define CVTEOF (-1)
#define CVTFAIL (-2)

int __VFPRINTF_GETC(__scanf_t *);

#ifndef __UNGETC
#define __UNGETC
static int
__ungetc(int c, __scanf_t *p)
{
  if (p->is_string)
    {
      if (c != EOF)
        --((__string_scanf_t *)p)->string;
    }
  else
    ((__stream_scanf_t *)p)->ungetc_fn(c);
  return c;
}
#endif

// We can't realistically read more characters than our total address
// space can hold, so use ptrdiff_t as the type needed to hold character
// counts.

#if (!defined(RD_LONG_LONG_DEFINED) && SUPPORT_LONG_LONG) ||\
    (!defined(RD_LONG_DEFINED) && SUPPORT_LONG) ||\
    (!defined(RD_INT_DEFINED) && SUPPORT_INT)
static charcount_t
RD_INT(__scanf_t *p, ARGTYPE *ap, int flag, int radix, int field)
{
  charcount_t charcount = -1;
  uvalue_t n = 0;
  int ch, digit;

  // Skip leading whitespace.
  while ((isspace)(ch = countgetc(p)))
    ;
  if (ch == EOF)
    return CVTEOF;
  flag &= ~(FLAG_NUMOK | FLAG_NUMNEG);
  if (field > 0 && (flag & FLAG_ALLOWSIGN))
    switch (ch)
      {
      case '-':
        flag |= FLAG_NUMNEG;
      case '+':
        ch = countgetc(p);
        --field;
      }
  if (field > 0 && ch == '0')
    {
      flag |= FLAG_NUMOK;
      --field;
      ch = countgetc(p);
      if (field > 0 && (ch == 'x' || ch == 'X') && (radix == 0 || radix == 16))
        {
          flag &= ~FLAG_NUMOK;
          --field;
          ch = countgetc(p);
          radix = 16;
        }
      else if (radix == 0)
        radix = 8;
    }
  if (radix == 0)
    radix = 10;
  while (field > 0 && (digit = __digit(ch, radix)) >= 0)
    {
      flag |= FLAG_NUMOK;
      --field;
      n = n * radix + digit;
      ch = countgetc(p);
    }
  __ungetc(ch, p);
    if ((flag & FLAG_NUMOK) == 0)
    return CVTFAIL;
  if ((flag & FLAG_NOSTORE) == 0)
    {
      ARGTYPE_PtrArg p = ARGTYPE_getPtrArg(*ap);
      if ((flag & (FLAG_ALLOWSIGN | FLAG_NUMNEG)) == (FLAG_ALLOWSIGN | FLAG_NUMNEG))
        n = -(value_t)n;
      if (flag & FLAG_CHAR)
        ARGTYPE_writePtrArgUnsignedChar(*ap, p, (unsigned char)n);
      else if (isSHORT(flag))
        ARGTYPE_writePtrArgUnsignedShort(*ap, p, (unsigned short)n);
      else if (isLONG(flag))
        ARGTYPE_writePtrArgUnsignedLong(*ap, p, (unsigned long)n);
      else if (isLONGLONG(flag))
        ARGTYPE_writePtrArgUnsignedLongLong(*ap, p, (uint64_t)n);
      else
        ARGTYPE_writePtrArgUnsignedInt(*ap, p, (unsigned int)n);
    }
  return charcount;
}
#endif
#if SUPPORT_LONG_LONG
#define RD_LONG_LONG_DEFINED
#elif SUPPORT_LONG
#define RD_LONG_DEFINED
#elif SUPPORT_INT
#define RD_INT_DEFINED
#endif

#ifndef RD_STRING
#define RD_STRING
static charcount_t
rd_string(__scanf_t *p, ARGTYPE *ap, int flag, int field)
{
  charcount_t charcount = -1;
  int ch;
  ARGTYPE_CharPtrArg s = ARGTYPE_PtrArg_NULL;

  // skip leading space.
  while ((isspace)(ch = countgetc(p)))
    ;

  // Fail if EOF and no characters read.
  if (ch == EOF)
    return CVTEOF;

  // If we need to store the value, snag the pointer.
  if ((flag & FLAG_NOSTORE) == 0)
    s = ARGTYPE_getPtrArg(*ap);

  // Snag the string.
  while (field > 0 && ch != EOF && !(isspace)(ch))
    {
      --field;
      if ((flag & FLAG_NOSTORE) == 0)
        {
          ARGTYPE_writePtrArgChar(*ap, s, (char)ch);
          s++;
        }
      ch = countgetc(p);
    }

  // Of course, we always read one too many.
  __ungetc(ch, p);

  // Terminate the string that we read.
  if ((flag & FLAG_NOSTORE) == 0)
    ARGTYPE_writePtrArgChar(*ap, s, 0);
  return charcount;
}
#endif

#ifdef SUPPORT_CHAR_CLASS
#ifndef RD_STRING_MAP
#define RD_STRING_MAP
static charcount_t
rd_string_map(__scanf_t *p, ARGTYPE *ap, int flag, int field, MAPTYPE charmap[])
{
  charcount_t charcount = -1;
  int ch;
  char *s = NULL;

  // If we need to store the value, snag the pointer.
  if ((flag & FLAG_NOSTORE) == 0)
    s = ARGTYPE_getPtrArg(*ap);
  ch = countgetc(p);
  if (ch == EOF)
    return CVTEOF;
  while (field > 0 &&
         (unsigned)ch < CHARLIMIT &&
         (charmap[(unsigned)ch/MAPTYPEBITS] & (1 << ((unsigned)ch % MAPTYPEBITS))))
    {
      --field;
      if ((flag & FLAG_NOSTORE) == 0)
        {
          ARGTYPE_writePtrArgChar(*ap, s, ch);
          s++;
        }
      ch = countgetc(p);
    }
  __ungetc(ch, p);
  if ((flag & FLAG_NOSTORE) == 0)
    ARGTYPE_writePtrArgChar(*ap, s, 0);
  return charcount;
}
#endif
#endif

#if SUPPORT_FLOAT
#ifndef RD_REAL
#define RD_REAL
static charcount_t
rd_real(__scanf_t *p, ARGTYPE *ap, int flag, int field)
{
  charcount_t charcount = -1;
  int ch, x = 0;
  double l = 0.0;

  // Not counted towards field width.
  while ((isspace)(ch = countgetc(p)))
    ;

  if (ch == EOF)
    return CVTEOF;
  flag &= ~(FLAG_NUMOK | FLAG_DOTSEEN | FLAG_NUMNEG);
  if (field > 0)
    switch (ch)
    {
    case '-':
      flag |= FLAG_NUMNEG;
    case '+':
      ch = countgetc(p);
      --field;
    }

  while (field > 0)
    {
      if (ch == '.' && (flag & FLAG_DOTSEEN) == 0)
        {
          flag |= FLAG_DOTSEEN;
          --field;
        }
      else if ((isdigit)(ch))
        {
          flag |= FLAG_NUMOK;
          --field;
          l = l*10 + (ch - '0');
          if (flag & FLAG_DOTSEEN)
            x -= 1;
        }
      else
        break;
      ch = countgetc(p);
    }

  // Unread the 'e' in (e.g.) "+.e" as this isn't valid.
  if (field > 0 && (ch == 'e' || ch == 'E') && (flag & FLAG_NUMOK))
    {
      int x2 = 0;
      flag &= ~(FLAG_NUMOK | FLAG_EXPNEG);
      --field;
      switch (ch = countgetc(p))
        {
        case '-':
          flag |= FLAG_EXPNEG;
          //FALLTHROUGH
        case '+':
          ch = countgetc(p);
          --field;
        }
      while (field > 0 && (isdigit)(ch))
        {
          flag |= FLAG_NUMOK;
          --field;
          x2 = (x2 > 1000) ? 10000 : 10*x2 + (ch - '0');
          ch = countgetc(p);
        }
      if (flag & FLAG_EXPNEG)
        x -= x2;
      else
        x += x2;
    }

  // We always read one too many.
  __ungetc(ch, p);

  // Scale exponent.
  if (x)
    l *= __RAL_pow10(x);

  if (isinf(l))
    errno = ERANGE;
  if (flag & FLAG_NUMNEG)
    l = -l;

  if ((flag & FLAG_NUMOK) == 0)
    return CVTFAIL;
  if (flag & FLAG_LONG)
    {
      if ((flag & FLAG_NOSTORE) == 0)
        {
          ARGTYPE_PtrArg p = ARGTYPE_getPtrArg(*ap);
#ifndef __SES_RISCV
          if (isLONGDOUBLE(flag))
            ARGTYPE_writePtrArgLongDouble(*ap, p, l);
          else
#endif
            ARGTYPE_writePtrArgDouble(*ap, p, l);
        }
    }
  else
    {
      // Narrow double to float.
      float f = (float)l;
      if (isinf(f))
        {
          errno = ERANGE;
        }

      // Treat overflow consistently whether or not stored.
      if ((flag & FLAG_NOSTORE) == 0)
        {
          ARGTYPE_PtrArg p = ARGTYPE_getPtrArg(*ap);
          ARGTYPE_writePtrArgFloat(*ap, p, f);
        }
    }
  return charcount;
}
#endif
#endif

#ifndef __VFSCANF
#define __VFSCANF __vfscanf
#endif

int
__VFSCANF(__scanf_t *p, __MEMSPACE const __VFSCANF_CHAR *fmt, ARGTYPE argv)
{
  int ch;
  int field, flag;
  int cnt = 0;  // number of fields read
  charcount_t charcount = 0;
  charcount_t worked;
  ARGTYPE_CharPtrArg cp;
  for (;;)
    {
      int fch;
      switch (fch = *fmt++)
        {
        case 0:
          // End of format string.
          return cnt;

        default:
          if ((isspace)(fch))
            {
              while ((isspace)(fch = *fmt++))
                ;
              --fmt;
              while ((isspace)(ch = __VFPRINTF_GETC(p)))
                ++charcount;
              __ungetc(ch, p);
              continue;
            }
          else if ((ch = __VFPRINTF_GETC(p)) == fch)
            {
              ++charcount;
              continue;
            }

          // Offending char is left unread.
          __ungetc(ch, p);
          if (ch == EOF && cnt == 0)
            return EOF;

          // Unmatched literal.
          return cnt;

        case '%':
          field = 0, flag = 0;
          if (*fmt == '*')
            {
              ++fmt;
              flag |= FLAG_NOSTORE;
            }
          while ((isdigit)(fch = *fmt++))
            {
              field = field*10 + (fch - '0');
              flag |= FLAG_FIELDGIVEN;
            }
          if ((flag & FLAG_FIELDGIVEN) == 0)
            field = INT_MAX;
          if (SUPPORT_LONG && fch == 'l')
            {
              fch = *fmt++;
              if (SUPPORT_LONG_LONG && fch == 'l')
                {
                  flag |= FLAG_LONG_LONG;
                  fch = *fmt++;
                }
              else
                flag |= FLAG_LONG;
            }
          else if (fch == 'L')
            {
              fch = *fmt++;
              flag |= FLAG_LONG | FLAG_LONGDOUBLE;
            }
          else if (fch == 'h')
            {
              fch = *fmt++;
              if (fch == 'h')
                {
                  flag |= FLAG_CHAR;
                  fch = *fmt++;
                }
              else
                flag |= FLAG_SHORT;
            }

          switch (fch)
            {
            default:
              // Illegal conversion
              return cnt;

            case '%':
              ch = __VFPRINTF_GETC(p);
              if (ch == '%')
                {
                  ++charcount;
                  continue;
                }
              __ungetc(ch, p);
              return (ch == EOF && cnt == 0) ? EOF : cnt;

            case 'c':
              if ((flag & FLAG_FIELDGIVEN) == 0)
                field = 1;
              cp = ARGTYPE_PtrArg_NULL;
              if ((flag & FLAG_NOSTORE) == 0)
                cp = ARGTYPE_getPtrArg(argv);
              // If no characters match, fail.
              if (field == 0)
                return cnt;
              for (; field > 0; --field)
                {
                  if ((ch = countgetc(p)) == EOF)
                    return cnt == 0 ? EOF : cnt;
                  if ((flag & FLAG_NOSTORE) == 0)
                    {
                      ARGTYPE_writePtrArgChar(argv, cp, (char)ch);
                      cp++;
                    }
                }
              if ((flag & FLAG_NOSTORE) == 0)
                ++cnt;
              continue;

            case 'd':
              worked = RD_INT(p, &argv, flag | FLAG_ALLOWSIGN, 10, field);
              break;

#if SUPPORT_FLOAT
            case 'e':
            case 'E':
            case 'f':
            case 'g':
            case 'G':
              worked = rd_real(p, &argv, flag, field);
              break;
#endif

            case 'i':
              worked = RD_INT(p, &argv, flag | FLAG_ALLOWSIGN, 0, field);
              break;

            case 'n':
              if ((flag & FLAG_NOSTORE) == 0)
                {
                  ARGTYPE_PtrArg p = ARGTYPE_getPtrArg(argv);
                  if (flag & FLAG_CHAR)
                    ARGTYPE_writePtrArgChar(argv, p, (char)charcount);
                  else if (isSHORT(flag))
                    ARGTYPE_writePtrArgShort(argv, p, (short)charcount);
                  else if (isLONG(flag))
                    ARGTYPE_writePtrArgLong(argv, p, (long)charcount);
                  else if (isLONGLONG(flag))
                    ARGTYPE_writePtrArgLongLong(argv, p, (int64_t)charcount);
                  else
                    ARGTYPE_writePtrArgInt(argv, p, (int)charcount);
                }
              continue;

            case 'o':
              worked = RD_INT(p, &argv, flag | FLAG_ALLOWSIGN, 8, field);
              break;

            case 'p':
              worked = RD_INT(p, &argv, flag & ~(FLAG_LONG | FLAG_SHORT | FLAG_CHAR | FLAG_LONG_LONG), 16, field);
              break;

            case 's':
              worked = rd_string(p, &argv, flag, field);
              break;

            case 'u':
              worked = RD_INT(p, &argv, flag | FLAG_ALLOWSIGN, 10, field);
              break;

            case 'x':
            case 'X':
              worked = RD_INT(p, &argv, flag | FLAG_ALLOWSIGN, 16, field);
              break;

#ifdef SUPPORT_CHAR_CLASS
            case '[':
              {
                int negated = 0, i;
                MAPTYPE charmap[CHARLIMIT/MAPTYPEBITS];
                if ((fch = *fmt++) == '^')
                  {
                    negated = 1;
                    fch = *fmt++;
                  }
                memset(charmap, 0, sizeof(charmap));
                do
                  {
                    if (fch == 0)
                      return cnt;  // %[... unterminated
                    if ((unsigned)fch < CHARLIMIT)
                      charmap[fch/MAPTYPEBITS] |= 1 << (fch%MAPTYPEBITS);
                  }
                while ((fch = *fmt++) != ']');

                if (negated)
                  for (i = 0; i < CHARLIMIT/MAPTYPEBITS; ++i)
                    charmap[i] = ~charmap[i];
                worked = rd_string_map(p, &argv, flag, field, charmap);
              }
              break;
#endif
            }

          if (worked < 0)
            {
              // conversion failed...
              return worked == CVTEOF && cnt == 0 ? EOF : cnt;
            }

          if ((flag & FLAG_NOSTORE) == 0)
            ++cnt;
          charcount += worked;
          continue;
        }
    }
}

