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

#ifndef _ADC_H_
#define _ADC_H_

#include "fsl_lpadc.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "fsl_power.h"
#include "Arduino.h"

#define ADC_PIN_CONFIG (0x00u|0x0400u)

/*
* config adc
*/
void adc_config(void);

/*
* setup the adc reference voltage
* reference:
* 		kLPADC_ReferenceVoltageAlt1 --reserved
* 		kLPADC_ReferenceVoltageAlt2 --VDDA
* 		kLPADC_ReferenceVoltageAlt3 --VREFP
*/
void analogReference(uint8_t reference);


/*
* set the sizes(in bits) of the value returned by the analogRead()
* bit:
* 		kLPADC_ConversionResolutionStandard 12bits
* 		kLPADC_ConversionResolutionHigh   16bits
* 		other value are default set to 12bits
*/
void analogReadResolution(lpadc_conversion_resolution_mode_t bitsel);

//------------------------------------------------another version

/*
* init the adc module
* adc_pin:
* 		the adc pin
*/
void adc_init(uint8_t adc_pin);

/*
* read the adc result
* adc_pin:
* 		the adc pin
* return:
*     the adc value
*/
uint32_t analogRead(uint8_t adc_pin);

/*
* return the adcpin's channel
* adc_pin:
* 		the adc pin
* return:
*     the adc channel
*/
uint8_t adc_outtrigger_id(const uint8_t adc_pin);

#endif //_ADC_H_
