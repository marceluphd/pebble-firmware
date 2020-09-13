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

#include "stdlib.h"
#include "string.h"
#include "__libc.h"

#if defined(__CROSSWORKS_ARM) || defined(__SES_ARM) || defined(__SES_RISCV)

void *calloc(size_t nobj, size_t size)__attribute__((weak));
void *malloc(size_t size)__attribute__((weak));
void free(void *addr)__attribute__((weak));
void *realloc(void *p, size_t size)__attribute__((weak));

#endif

typedef struct heap_tag
{
  struct heap_tag *next;
  unsigned size;
} heap_t;

extern heap_t __heap_start__;
static heap_t *heap = &__heap_start__;

#define HEAP_CHUNK_SIZE  16   /* Must be a power of two. */
#define ADDADR(x,n)  ((void *)((unsigned char *)(x)+(n)))

#if defined(__CROSSWORKS_ARM) || defined(__SES_ARM) || defined(__SES_RISCV)
extern char __heap_end__[];
static size_t
raw_heap_size(void)
{
  return __heap_end__ - (char *)&__heap_start__;
}
static int
in_heap(void *addr)
{  
  return addr >= (void*)&__heap_start__ && addr <= (void*)__heap_end__;
}
#endif

void *
calloc(size_t nobj, size_t size)
{
  size_t arrsize = nobj * size;
  void *m = malloc(arrsize);
  if (m)
    memset(m, 0, arrsize);
  return m;
}

void *
malloc(size_t size)
{          
  heap_t *p, *q, *best;
  unsigned best_size;    

#if defined(__CROSSWORKS_ARM) || defined(__SES_ARM) || defined(__SES_RISCV)
  // Protect against small heaps requested by the user/IDE.
  if (raw_heap_size() <= HEAP_CHUNK_SIZE)
    return 0;
#endif

  __heap_lock();

  /* Round up to a multiple of HEAP_CHUNK_SIZE and account for size word. */
  size = (size + sizeof(unsigned) + HEAP_CHUNK_SIZE-1) & (unsigned)(-HEAP_CHUNK_SIZE);

  /* Find first block that could satisfy request. */
  p = heap;
  q = 0;

  /* No best-fit block. */
  best_size = ~0U;
  best = 0;

  /* Search for best-fit block. */
  while (p)
    {

      /* Check whether block is correct size. */
      if (p->size == size)
	{
	  /* Exact fit - remove block from the free list. */
	  if (q)
	    q->next = p->next;
	  else
	    heap = p->next;

	  /* Short-cut return. */
	  *((unsigned *)p) = size;
          __heap_unlock();
	  return ((char *)p) + sizeof(unsigned);
	}

      /* Check whether block can be split. */
      else if (p->size > size && p->size < best_size)
	{
	  best = p;
	  best_size = p->size;
	}

      /* Consider next block. */
      q = p;
      p = p->next;
    }

  /* Block must be split; take free memory from top of block. */
  if (best)
    {
      best->size -= size;
      p = ADDADR(best, best_size - size);
      *((unsigned *)p) = size;  
      //memset((char*)p+sizeof(unsigned), 0xAE, size-2);
      __heap_unlock();
      return ((char *)p) + sizeof(unsigned);
    }
  else
    {
      __heap_unlock();
      return 0;
    }
}

void
free(void *addr)
{
  heap_t *p, *rover, *prev;
  unsigned size;  

  if (addr == 0)
    return;

#if defined(__CROSSWORKS_ARM) || defined(__SES_ARM) || defined(__SES_RISCV)
  if (!in_heap(addr))
    return;
#endif

  __heap_lock();

  /* Adjust backwards to actual memory block origin. */
  addr = ((char *)addr) - sizeof(unsigned);

  /* Recover size */
  size = *((unsigned *)addr);
  
  if (addr)
    {
      //memset(addr, 0xFE, size);  

      /* Deallocated onto non-existent heap? */
      if (heap == 0)
	{
	  /* Yes, add this floating block to the heap. */
	  heap = addr;
	  heap->next = 0;
	  heap->size = size;
	}
      else
	{
	  /* Search for block adjacent to p, but with lower address. */
	  p = addr; rover = heap; prev = 0;
	  while (rover && rover < (heap_t *)addr)
	    {
	      prev = rover; rover = rover->next;
	    }

	  /* Check for double-dispose of a pointer. */
	  /* For efficiency, this could be removed. */
    #if 0
	  if ((rover = addr) ||
	      prev & (DIFADR(addr, ADDADR(prev, prev->size)) < 0))
	    HALT(DOUBLEDISPOSE);
	  END;
    #endif

	  /* Return block to free list */
	  p->next = rover;
	  p->size = size;
	  if (prev)
	    prev->next = p;
	  else
	    heap = p;

	  /* See if can join with block to right. */
	  if (rover && (ADDADR(p, size) == rover))
	    {
	      p->size += rover->size;
	      p->next = rover->next;
	    }

	  /* See if can join with block to left. */
	  if (prev && ADDADR(prev, prev->size) == p)
	    {
	      prev->size += p->size;
	      prev->next = p->next;
	    }
	}
    }
  __heap_unlock();
}

void *
realloc(void *p, size_t size)
{
  /* Allocate area of requested size. */
  void *pp = malloc(size);
  
  /* If no memory, keep existing block and indicate failure to reallocate. */
  if (pp == 0)
    return pp;

  /* Copy existing block if we were given one. */
  if (p)
    {
      /* Recover size */
      size_t existing_size = ((unsigned *)p)[-1];

      /* Limit size to copy. */
      if (existing_size < size)
        size = existing_size;

      /* Make copy of data. */
      memcpy(pp, p, size);

      /* And we're done with the original block. */
      free(p);
    }

  /* Return newly allocated block. */
  return pp;
}
