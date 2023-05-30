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

#ifndef _PWM_H_
#define _PWM_H_

#include "fsl_sctimer.h"
#include "fsl_iocon.h"
#include "Arduino.h"

#define PwmFrequency 10000
#define SCTOUT_PIN_CONFIG_05 (0x05u|0x0100u|0x0400u)
#define SCTOUT_PIN_CONFIG_04 (0x04u|0x0100u|0x0400u)
#define SCTOUT_PIN_CONFIG_03 (0x03u|0x0100u|0x0400u)
#define SCTOUT_PIN_CONFIG_02 (0x02u|0x0100u|0x0400u)

/*
* setup PWM
* pwm_pin:
* 	the pwm pin
* pwm_duty：
*   the pwm duty
*/
void analogWrite(uint8_t pwm_pin,uint16_t pwm_duty);

/*
* change pwm duty
* pwm_pin:
* 	the pwm pin
* pwm_duty：
*   the pwm duty
*/
void analogWriteDuty(uint8_t pwm_pin,uint16_t pwm_duty);

/*
* change the range of the pwm duty
* bit:
* 	 100~65535
*/
void analogWriteResolution(uint16_t bit);

//------------------------------------------------imitate 1768 library

/*
* pwm init
* frequency:
* 	 not over the clock
*/
void pwm_init(const uint16_t frequency);

/*
* attach pwm to pwm pin,and setup the init duty
* pwm_pin:
* 	 the sct output pin
* value:
*    the pwm duty(default is 0~100)
*/
bool pwm_attach_pin(const uint8_t pwm_pin, const uint32_t value);

/*
* update the duty
* pin:
* 	 the sct output pin
* value:
*    the pwm duty(default is 0~100)
*/
bool pwm_write(const uint8_t pin, const uint32_t value);


/*
* setup the duty in microsecond
* pin:
* 	 the sct output pin
* value:
*    not over the pwm period
*/
bool pwm_write_us(const uint8_t pin, const uint32_t value);

/*
* stop the pwm output and reset the pin to gpio
* pin:
* 	 the sct output pin
*/
bool pwm_detach_pin(const uint8_t pin);

/*
* acquire the pwm period
* pin:
* 	 the sct output pin
* return:
*    the max counter value
*/
uint32_t pwm_get_period(const uint8_t pin);

/*
* change the pwm frequency
* pin:
* 	 the sct output pin
* frequency:
*    the pwm frequency
*/
bool pwm_set_frequency(const uint8_t pin, const uint32_t frequency);

/*
* return the pwm pin's output channel
* pin:
* 	 the sct output pin
* return:
*    the sct output number
*/
uint8_t pwm_outchannel_id(const uint8_t pin);

#endif //_PWM_H_
