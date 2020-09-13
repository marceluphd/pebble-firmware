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

#include <ctype.h>

int
(isalpha)(int c)
{
  return ('A' <= (unsigned)c && (unsigned)c <= 'Z') ||
	 ('a' <= (unsigned)c && (unsigned)c <= 'z');
}

int
(isupper)(int c)
{
  return 'A' <= (unsigned)c && (unsigned)c <= 'Z';
}

int
(islower)(int c)
{
  return 'a' <= (unsigned)c && (unsigned)c <= 'z';
}

int
(isdigit)(int c)
{
  return '0' <= (unsigned)c && (unsigned)c <= '9';
}

int
(isxdigit)(int c)
{
  return ('0' <= (unsigned)c && (unsigned)c <= '9') ||
	 ('a' <= (unsigned)c && (unsigned)c <= 'f') ||
	 ('A' <= (unsigned)c && (unsigned)c <= 'F');
}

int
(isspace)(int c)
{
  return (0x09 <= (unsigned)c && (unsigned)c <= 0x0d) || ((unsigned)c == ' ');
}

int
(ispunct)(int c)
{
  return ('!' <= (unsigned)c && (unsigned)c <= '/') ||
         (':' <= (unsigned)c && (unsigned)c <= '@') ||
	 ('[' <= (unsigned)c && (unsigned)c <= '`') ||
	 ('{' <= (unsigned)c && (unsigned)c <= '~');
}

int
(isalnum)(int c)
{
  return ('A' <= (unsigned)c && (unsigned)c <= 'Z') ||
	 ('a' <= (unsigned)c && (unsigned)c <= 'z') ||
	 ('0' <= (unsigned)c && (unsigned)c <= '9');
}

int
(isprint)(int c)
{
  return (unsigned)c >= 0x20 && c != 0x7F;
}

int
(isgraph)(int c)
{
  return (unsigned)c > 0x20 && c != 0x7F;
}

int
(iscntrl)(int c)
{
  return (unsigned)c < 0x20 || c == 0x7F;
}

int
(tolower)(int c)
{
  return (unsigned char)'A' <= (unsigned char)c && (unsigned char)c <= (unsigned char)'Z' ? c + ('a' - 'A') : c;
}

int
(toupper)(int c)
{
  return (unsigned char)'a' <= (unsigned char)c && (unsigned char)c <= (unsigned char)'z' ? c + ('A' - 'a') : c;
}

int
(isblank)(int c)
{
  return c == ' ' || c == '\t';
}
