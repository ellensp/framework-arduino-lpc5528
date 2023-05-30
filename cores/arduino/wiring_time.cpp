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

#include "wiring_time.h"

volatile uint64_t ms_count = 0;

extern void SysTick_Callback(void);

extern void SYSTICK_Callback();

extern "C"{
  void SysTick_Handler(void) {
    SYSTICK_Callback();
    ms_count++;
  }
}

uint64_t millis(void) {
  return ms_count;
}

uint64_t micros(void) {
  return ((ms_count * 1000) + ( (SysTick->LOAD - SysTick->VAL)/ (CLOCK_GetFreq(kCLOCK_CoreSysClk)/1000000) ) );  
}

void delay(uint64_t ms) {
  uint64_t ms_begin = millis();
  while(millis() - ms_begin <= ms) {
  }
}

void delayMicroseconds(uint64_t us) {
  uint64_t ms_begin = micros();
  while(micros() - ms_begin <= us) {
  }
}

void systick_init(void) {
  if (SysTick_Config((CLOCK_GetFreq(kCLOCK_CoreSysClk)) / 1000U)) { // 1ms
      while (1){
      }
  }
}

