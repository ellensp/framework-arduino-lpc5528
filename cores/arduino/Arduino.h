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

#ifndef _ARDUINO_H_
#define _ARDUINO_H_

#include "pinmapping.h"
#include "wiring_digital.h"
#include "wiring_time.h"
#include "CDCSerial.h"
#include "HardwareSerial.h"
#include "masstorage1.h"
#include "Wire.h"
#include "SPI.h"
#include "adc.h"
#include "pwm.h"
#include "timers.h"
#include <binary.h>
#include <const_functions.h>
// #include "I2SOut.h"


enum{
    INPUT = 0x00,
    OUTPUT,
    INPUT_PULLUP
};

#define HIGH            0x01
#define LOW             0x00

enum BitOrder {
  LSBFIRST = 0,
  MSBFIRST = 1
};

typedef uint8_t byte;
typedef int16_t pin_t;

#define constrain(value, arg_min, arg_max) ((value) < (arg_min) ? (arg_min) :((value) > (arg_max) ? (arg_max) : (value)))

void cli(void); // Disable
void sei(void); // Enable

int32_t random(int32_t);
int32_t random(int32_t, int32_t);
void randomSeed(uint32_t);

char *dtostrf (double __val, signed char __width, unsigned char __prec, char *__s);
void tone(const pin_t _pin, const uint32_t frequency, const uint32_t duration = 0);
void noTone(const pin_t _pin);

extern void setup(void) ;
extern void loop(void) ;

#endif // _ARDUINO_H_
