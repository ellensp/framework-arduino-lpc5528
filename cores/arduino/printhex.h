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

#ifndef _PRINTHEX_H_
#define _PRINTHEX_H_

#pragma once

#include <Print.h>
#define DEBUG_USB_HOST

void E_Notifyc(char c, int lvl);

template <class T>
void PrintHex(T val, int lvl) {
  int num_nybbles = sizeof (T) * 2;
  do {
    char v = 48 + (((val >> (num_nybbles - 1) * 4)) & 0x0F);
    if (v > 57) v += 7;
    E_Notifyc(v, lvl);
  } while (--num_nybbles);
}

template <class T>
void PrintBin(T val, int lvl) {
  for (T mask = (((T)1) << ((sizeof (T) << 3) - 1)); mask; mask >>= 1)
    E_Notifyc(val & mask ? '1' : '0', lvl);
}

// template <class T>
// void SerialPrintHex(T val) {
//   int num_nybbles = sizeof (T) * 2;
//   do {
//     char v = 48 + (((val >> (num_nybbles - 1) * 4)) & 0x0F);
//     if (v > 57) v += 7;
//     USB_HOST_SERIAL.print(v);
//   } while (--num_nybbles);
// }

template <class T>
void PrintHex2(Print *prn, T val) {
  T mask = (((T)1) << (((sizeof (T) << 1) - 1) << 2));
  while (mask > 1) {
    if (val < mask) prn->print("0");
    mask >>= 4;
  }
  prn->print((T)val, HEX);
}

template <class T> void D_PrintHex(T val __attribute__((unused)), int lvl __attribute__((unused))) {
  #ifdef DEBUG_USB_HOST
    PrintHex<T > (val, lvl);
  #endif
}

template <class T>
void D_PrintBin(T val, int lvl) {
  #ifdef DEBUG_USB_HOST
    PrintBin<T > (val, lvl);
  #endif
}

#endif // _PRINTABLE_H_