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

#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#pragma once

#include <stdint.h>
#include <printhex.h>

#define PSTR(s)   (s)

#define DEBUG_USB_HOST

extern int UsbDEBUGlvl;

void E_Notify(char const * msg, int lvl);
void E_Notify(uint8_t b, int lvl);
void E_NotifyStr(char const * msg, int lvl);
void E_Notifyc(char c, int lvl);

#ifdef DEBUG_USB_HOST
  #define Notify E_Notify
  #define NotifyStr E_NotifyStr
  #define Notifyc E_Notifyc
  void NotifyFailGetDevDescr(uint8_t reason);
  void NotifyFailSetDevTblEntry(uint8_t reason);
  void NotifyFailGetConfDescr(uint8_t reason);
  void NotifyFailSetConfDescr(uint8_t reason);
  void NotifyFailGetDevDescr();
  void NotifyFailSetDevTblEntry();
  void NotifyFailGetConfDescr();
  void NotifyFailSetConfDescr();
  void NotifyFailUnknownDevice(uint16_t VID, uint16_t PID);
  void NotifyFail(uint8_t rcode);
#else
  #define Notify(...) ((void)0)
  #define NotifyStr(...) ((void)0)
  #define Notifyc(...) ((void)0)
  #define NotifyFailGetDevDescr(...) ((void)0)
  #define NotifyFailSetDevTblEntry(...) ((void)0)
  #define NotifyFailGetConfDescr(...) ((void)0)
  #define NotifyFailGetDevDescr(...) ((void)0)
  #define NotifyFailSetDevTblEntry(...) ((void)0)
  #define NotifyFailGetConfDescr(...) ((void)0)
  #define NotifyFailSetConfDescr(...) ((void)0)
  #define NotifyFailUnknownDevice(...) ((void)0)
  #define NotifyFail(...) ((void)0)
#endif

template <class ERROR_TYPE>
void ErrorMessage(uint8_t level, char const * msg, ERROR_TYPE rcode = 0) {
  #ifdef DEBUG_USB_HOST
    Notify(msg, level);
    Notify(PSTR(": "), level);
    D_PrintHex<ERROR_TYPE > (rcode, level);
    Notify(PSTR("\r\n"), level);
  #endif
}

template <class ERROR_TYPE>
void ErrorMessage(char const * msg __attribute__((unused)), ERROR_TYPE rcode __attribute__((unused)) = 0) {
  #ifdef DEBUG_USB_HOST
    Notify(msg, 0x80);
    Notify(PSTR(": "), 0x80);
    D_PrintHex<ERROR_TYPE > (rcode, 0x80);
    Notify(PSTR("\r\n"), 0x80);
  #endif
}

#endif //_MESSAGE_H_