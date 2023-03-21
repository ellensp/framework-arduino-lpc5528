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

#include "message.h"
#include "HardwareSerial.h"
#include "printhex.h"

#define pgm_read_byte(addr) (*reinterpret_cast<const uint8_t*>(addr))

// 0x80 is the default (i.e. trace) to turn off set this global to something lower.
// this allows for 126 other debugging levels.
// TO-DO: Allow assignment to a different serial port by software
int UsbDEBUGlvl = 0x80;

#define message_serial Serial3
void E_Notifyc(char c, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  message_serial.print(c);
}

void E_Notify(char const * msg, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  if (!msg) return;
  while (const char c = pgm_read_byte(msg++)) E_Notifyc(c, lvl);
  // Serial.print(msg);
}

void E_NotifyStr(char const * msg, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  if (!msg) return;
  while (const char c = *msg++) E_Notifyc(c, lvl);
  // Serial.print(msg);
}

void E_Notify(uint8_t b, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  message_serial.print(b, DEC);
}

void E_Notify(double d, int lvl) {
  if (UsbDEBUGlvl < lvl) return;
  message_serial.print(d);
}

#ifdef DEBUG_USB_HOST

  void NotifyFailGetDevDescr() {
    Notify(PSTR("\r\ngetDevDescr "), 0x80);
  }

  void NotifyFailSetDevTblEntry() {
    Notify(PSTR("\r\nsetDevTblEn "), 0x80);
  }

  void NotifyFailGetConfDescr() {
    Notify(PSTR("\r\ngetConf "), 0x80);
  }

  void NotifyFailSetConfDescr() {
    Notify(PSTR("\r\nsetConf "), 0x80);
  }

  void NotifyFailGetDevDescr(uint8_t reason) {
    NotifyFailGetDevDescr();
    NotifyFail(reason);
  }

  void NotifyFailSetDevTblEntry(uint8_t reason) {
    NotifyFailSetDevTblEntry();
    NotifyFail(reason);

  }

  void NotifyFailGetConfDescr(uint8_t reason) {
    NotifyFailGetConfDescr();
    NotifyFail(reason);
  }

  void NotifyFailSetConfDescr(uint8_t reason) {
    NotifyFailSetConfDescr();
    NotifyFail(reason);
  }

  void NotifyFailUnknownDevice(uint16_t VID, uint16_t PID) {
    Notify(PSTR("\r\nUnknown Device Connected - VID: "), 0x80);
    D_PrintHex<uint16_t > (VID, 0x80);
    Notify(PSTR(" PID: "), 0x80);
    D_PrintHex<uint16_t > (PID, 0x80);
  }

  void NotifyFail(uint8_t rcode) {
    D_PrintHex<uint8_t > (rcode, 0x80);
    Notify(PSTR("\r\n"), 0x80);
  }

#endif // DEBUG_USB_HOST

