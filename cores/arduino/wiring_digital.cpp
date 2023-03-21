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


#include "Arduino.h"

#include "fsl_gpio.h"
#include "fsl_iocon.h"

void pinMode(uint32_t pin, uint32_t mode)
{
	if(!PIN_CHECK_VALID(pin)){
		return;
	}
	
	uint32_t modefunc;
	gpio_pin_config_t pin_config = {
        kGPIO_DigitalOutput,
        0,
    };
	
	switch(mode){
		case INPUT:
		{
			modefunc = 0x100; // no pull-down or  pull-up
			pin_config.pinDirection = kGPIO_DigitalInput;
		}
		break;
		case OUTPUT:
		{
			modefunc = 0x100; // no pull-down or  pull-up
		}
		break;
		case INPUT_PULLUP:
		{
			modefunc = 0x120; //  pull-up
			pin_config.pinDirection = kGPIO_DigitalInput;
		}
		break;
		default:
			return;
	}
	/* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);
	IOCON_PinMuxSet(IOCON, get_gpio_port(pin), get_gpio_pin(pin), modefunc);
	/* Init GPIO */
	GPIO_PinInit(GPIO, get_gpio_port(pin), get_gpio_pin(pin), &pin_config);
}

void digitalWrite(uint32_t pin, uint32_t val)
{
	if(!PIN_CHECK_VALID(pin)){
		return;
	}
	GPIO_PinWrite(GPIO, get_gpio_port(pin),get_gpio_pin(pin), val);
}

int digitalRead(uint32_t pin)
{
	if(!PIN_CHECK_VALID(pin)){
		return 0;
	}
	return GPIO_PinRead(GPIO,get_gpio_port(pin),get_gpio_pin(pin));
}
