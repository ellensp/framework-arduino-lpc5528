/*
BSD 3-Clause License

Copyright (c) 2021-2022 WPI Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _MACROS_H_
#define _MACROS_H_

#pragma once

////////////////////////////////////////////////////////////////////////////////
// HANDY MACROS
////////////////////////////////////////////////////////////////////////////////

#define VALUE_BETWEEN(v,l,h) (((v)>(l)) && ((v)<(h)))
#define VALUE_WITHIN(v,l,h) (((v)>=(l)) && ((v)<=(h)))
#define output_pgm_message(wa,fp,mp,el) wa = &mp, fp((char *)pgm_read_pointer(wa), el)
#define output_if_between(v,l,h,wa,fp,mp,el) if (VALUE_BETWEEN(v,l,h)) output_pgm_message(wa,fp,mp[v-(l+1)],el);

#define SWAP(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)))
#ifndef __BYTE_GRABBING_DEFINED__
#define __BYTE_GRABBING_DEFINED__ 1
#ifdef BROKEN_OPTIMIZER_LITTLE_ENDIAN
// Note: Use this if your compiler generates horrible assembler!
#define BGRAB0(__usi__)  (((uint8_t *)&(__usi__))[0])
#define BGRAB1(__usi__)  (((uint8_t *)&(__usi__))[1])
#define BGRAB2(__usi__)  (((uint8_t *)&(__usi__))[2])
#define BGRAB3(__usi__)  (((uint8_t *)&(__usi__))[3])
#define BGRAB4(__usi__)  (((uint8_t *)&(__usi__))[4])
#define BGRAB5(__usi__)  (((uint8_t *)&(__usi__))[5])
#define BGRAB6(__usi__)  (((uint8_t *)&(__usi__))[6])
#define BGRAB7(__usi__)  (((uint8_t *)&(__usi__))[7])
#else
// Note: The cast alone to uint8_t is actually enough.
// GCC throws out the "& 0xFF", and the size is no different.
// Some compilers need it.
#define BGRAB0(__usi__)  ((uint8_t)((__usi__) & 0xFF ))
#define BGRAB1(__usi__)  ((uint8_t)(((__usi__) >> 8) & 0xFF))
#define BGRAB2(__usi__)  ((uint8_t)(((__usi__) >> 16) & 0xFF))
#define BGRAB3(__usi__)  ((uint8_t)(((__usi__) >> 24) & 0xFF))
#define BGRAB4(__usi__)  ((uint8_t)(((__usi__) >> 32) & 0xFF))
#define BGRAB5(__usi__)  ((uint8_t)(((__usi__) >> 40) & 0xFF))
#define BGRAB6(__usi__)  ((uint8_t)(((__usi__) >> 48) & 0xFF))
#define BGRAB7(__usi__)  ((uint8_t)(((__usi__) >> 56) & 0xFF))
#endif
#define BOVER1(__usi__)  ((uint16_t)(__usi__) << 8)
#define BOVER2(__usi__)  ((uint32_t)(__usi__) << 16)
#define BOVER3(__usi__)  ((uint32_t)(__usi__) << 24)
#define BOVER4(__usi__)  ((uint64_t)(__usi__) << 32)
#define BOVER5(__usi__)  ((uint64_t)(__usi__) << 40)
#define BOVER6(__usi__)  ((uint64_t)(__usi__) << 48)
#define BOVER7(__usi__)  ((uint64_t)(__usi__) << 56)

// These are the smallest and fastest ways I have found so far in pure C/C++.
#define BMAKE16(__usc1__,__usc0__) ((uint16_t)((uint16_t)(__usc0__) | (uint16_t)BOVER1(__usc1__)))
#define BMAKE32(__usc3__,__usc2__,__usc1__,__usc0__) ((uint32_t)((uint32_t)(__usc0__) | (uint32_t)BOVER1(__usc1__) | (uint32_t)BOVER2(__usc2__) | (uint32_t)BOVER3(__usc3__)))
#define BMAKE64(__usc7__,__usc6__,__usc5__,__usc4__,__usc3__,__usc2__,__usc1__,__usc0__) ((uint64_t)((uint64_t)__usc0__ | (uint64_t)BOVER1(__usc1__) | (uint64_t)BOVER2(__usc2__) | (uint64_t)BOVER3(__usc3__) | (uint64_t)BOVER4(__usc4__) | (uint64_t)BOVER5(__usc5__) | (uint64_t)BOVER6(__usc6__) | (uint64_t)BOVER1(__usc7__)))
#endif

/*
 * Debug macros: Strings are stored in progmem (flash) instead of RAM.
 */
#define USBTRACE(s) (Notify(PSTR(s), 0x80))
#define USBTRACE1(s,l) (Notify(PSTR(s), l))
#define USBTRACE2(s,r) (Notify(PSTR(s), 0x80), D_PrintHex((r), 0x80), Notify(PSTR("\r\n"), 0x80))
#define USBTRACE3(s,r,l) (Notify(PSTR(s), l), D_PrintHex((r), l), Notify(PSTR("\r\n"), l))

#endif //_MACROS_H_
