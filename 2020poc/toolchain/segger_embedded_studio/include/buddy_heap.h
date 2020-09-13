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

#ifndef BUDDY_HEAP_H
#define BUDDY_HEAP_H

#include <stdlib.h>

typedef struct buddy_link_tag
{
  struct buddy_link_tag *next;
  struct buddy_link_tag *prev;
} __BUDDY_LINKS_t;

typedef struct {
  long total_capacity;     // total size of the heap
  long alloc_requests;     // number of times alloc called
  long alloc_failures;     // how many times alloc has failed
  long free_requests;      // number of times free called with non-zero argument
  long free_failures;      // how many times free has been called with a bad argument or corrupt heap
  long used_bytes;         // how many bytes are used in the heap (a multiple of the allocation unit)
  long free_bytes;         // how many bytes are free in the heap
  long min_free_bytes;     // lowest number of free bytes bytes in the heap
  long blocks_split;       // how many times blocks have needed to be split.
  long blocks_coalesed;    // how many times blocks have been coalesced.
} BUDDY_HEAP_STATS_t;

typedef struct
{
  char *mem;
  unsigned max_k;     // 'm' in Fundamental Algorithms
  unsigned min_k;     // minimum block size returned
  unsigned min_size;  // 1 << min_k
  unsigned size;
  BUDDY_HEAP_STATS_t stats;
  __BUDDY_LINKS_t free_head[sizeof(void *)*8];  // 4 entries unused, could do something here...
} BUDDY_HEAP_t;


void *buddy_alloc(BUDDY_HEAP_t *h, size_t size);
void buddy_free(BUDDY_HEAP_t *h, void *data);
int buddy_heap_init(BUDDY_HEAP_t *h, void *mem, size_t size);

// For the future...
//
//typedef struct
//{
//  BUDDY_HEAP_t heap;
//  CTL_MUTEX_t mutex;
//  CTL_EVENT_SET_t events;
//} CTL_BUDDY_HEAP_t;
//
//int ctl_buddy_heap_init(CTL_BUDDY_HEAP_t *h, void *mem, size_t size);
//void *ctl_buddy_alloc(CTL_BUDDY_HEAP_t *h, size_t size, CTL_TIMEOUT_t timeout, CTL_);
//void *ctl_buddy_alloc_with_timeout(CTL_BUDDY_HEAP_t *h, size_t size, CTL_TIMEOUT_t timeout, CTL_TIME_t t);
//void ctl_buddy_free(CTL_BUDDY_HEAP_t *h, void *data);

#endif
