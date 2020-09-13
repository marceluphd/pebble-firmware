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

function allocated(start, end)
{
  while (start < end)
    {
      var s = TargetInterface.peekWord(start);
      WScript.Echo(s.toString() + " bytes allocated at 0x" + (start+4).toString(16) + "\n");
      start += s;
    }
}

function heapwalk()
{   
  var heap_start = Debug.evaluate("__heap_start__");
  var heap_end = Debug.evaluate("__heap_end__");
  var heap_size = Debug.evaluate("__heap_size__");
  var heap = Debug.evaluate("heap");
  if (heap == 0)
    {
      WScript.Echo("heap is full\n");
      allocated(heap_start, heap_end);      
    }
  else if (heap == heap_start && TargetInterface.peekWord(heap_start+4)==heap_size)
    {
      WScript.Echo("heap is empty "+heap_size+ " bytes free\n");
    }
  else  
    {
      var freelist = [];
      var h = heap;
      while (h)
        {
          var n = TargetInterface.peekWord(h);
          var s = TargetInterface.peekWord(h+4);
          WScript.Echo(s.toString()+ " bytes free at 0x"+h.toString(16)+"\n");
          freelist.push(h);
          freelist.push(h+s);
          h = n;
        }
      freelist.sort(function(a, b){return a-b});
      s = heap_start;
      for (i=0;i<freelist.length;i+=2)
        {
          if (s != freelist[i])
            allocated(s, freelist[i]);
          s = freelist[i+1];
        }
      allocated(s, heap_end);
    }
} 

