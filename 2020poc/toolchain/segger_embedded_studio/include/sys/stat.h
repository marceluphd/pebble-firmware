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

#ifndef _STAT_H
#define _STAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>

struct stat 
{
  mode_t st_mode;
  off_t	st_size;
};

#define	_IFMT  0170000
#define	_IFDIR 0040000
#define	_IFCHR 0020000
#define	_IFBLK 0060000
#define	_IFREG 0100000
#define	_IFLNK 0120000
#define	_IFSOCK	0140000
#define	_IFIFO 0010000

#define	S_IFMT  _IFMT
#define	S_IFDIR	_IFDIR
#define	S_IFCHR	_IFCHR
#define	S_IFBLK	_IFBLK
#define	S_IFREG	_IFREG
#define	S_IFLNK	_IFLNK
#define	S_IFSOCK _IFSOCK
#define	S_IFIFO _IFIFO

#define	S_IRWXU (S_IRUSR | S_IWUSR | S_IXUSR)
#define	S_IRUSR	0000400
#define	S_IWUSR	0000200
#define	S_IXUSR 0000100
#define	S_IRWXG	(S_IRGRP | S_IWGRP | S_IXGRP)
#define	S_IRGRP	0000040
#define	S_IWGRP	0000020
#define	S_IXGRP 0000010
#define	S_IRWXO	(S_IROTH | S_IWOTH | S_IXOTH)
#define	S_IROTH	0000004
#define	S_IWOTH	0000002
#define	S_IXOTH 0000001

#define	S_IREAD	S_IRUSR
#define	S_IWRITE S_IWUSR

int fstat(int fildes, struct stat *buf);

#ifdef __cplusplus
}
#endif

#endif
