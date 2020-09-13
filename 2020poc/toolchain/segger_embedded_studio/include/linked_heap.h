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

#ifndef LINKED_HEAP_H
#define LINKED_HEAP_H

#include "stdlib.h"

typedef struct LINKED_HEAP_BLOCK_tag
{
  struct LINKED_HEAP_BLOCK_tag *next;
  unsigned size;
} LINKED_HEAP_BLOCK_t;

typedef struct
{
  long total_capacity;     // total size of the heap
  long alloc_requests;     // number of times alloc called
  long alloc_failures;     // how many times alloc has failed
  long free_requests;      // number of times free called with non-zero argument
  long free_failures;      // how many times free has been called with a bad argument or corrupt heap
  long used_bytes;         // how many bytes are used in the heap (a multiple of the allocation unit)
  long free_bytes;         // how many bytes are free in the heap
  long min_free_bytes;     // lowest number of free bytes bytes in the heap
  long fragmentation;      // number of non-contiguous free fragments (i.e. length of free list)
  long max_fragmentation;  // maximum length of free list ever recorded
  long chunk_size;         // Everything is allocated as a multiple of this size
} LINKED_HEAP_STATS_t;

typedef struct
{
  LINKED_HEAP_STATS_t stats;

  // Private fields, do not assume anything about these, nor change them...
  LINKED_HEAP_BLOCK_t *head;
  size_t size;
  size_t chunk_size;
} LINKED_HEAP_t;

int linked_heap_init(LINKED_HEAP_t *h, void *base, size_t size, size_t chunk_size);
void *linked_heap_alloc(LINKED_HEAP_t *h, size_t size);
void linked_heap_free(LINKED_HEAP_t *h, void *addr);

#endif
