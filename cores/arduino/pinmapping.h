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

#ifndef _PIN_MAPPING_H_
#define _PIN_MAPPING_H_

#ifdef __cplusplus
extern "C" {
#endif

#pragma once

#define P_NC -1
#define P0_00 0x00
#define P0_01 0x01
#define P0_02 0x02
#define P0_03 0x03
#define P0_04 0x04
#define P0_05 0x05
#define P0_06 0x06
#define P0_07 0x07
#define P0_08 0x08
#define P0_09 0x09
#define P0_10 0x0A
#define P0_11 0x0B
#define P0_12 0x0C
#define P0_13 0x0D
#define P0_14 0x0E
#define P0_15 0x0F
#define P0_16 0x10
#define P0_17 0x11
#define P0_18 0x12
#define P0_19 0x13
#define P0_20 0x14
#define P0_21 0x15
#define P0_22 0x16
#define P0_23 0x17
#define P0_24 0x18
#define P0_25 0x19
#define P0_26 0x1A
#define P0_27 0x1B
#define P0_28 0x1C
#define P0_29 0x1D
#define P0_30 0x1E
#define P0_31 0x1F

#define P1_00 0x20
#define P1_01 0x21
#define P1_02 0x22
#define P1_03 0x23
#define P1_04 0x24
#define P1_05 0x25
#define P1_06 0x26
#define P1_07 0x27
#define P1_08 0x28
#define P1_09 0x29
#define P1_10 0x2A
#define P1_11 0x2B
#define P1_12 0x2C
#define P1_13 0x2D
#define P1_14 0x2E
#define P1_15 0x2F
#define P1_16 0x30
#define P1_17 0x31
#define P1_18 0x32
#define P1_19 0x33
#define P1_20 0x34
#define P1_21 0x35
#define P1_22 0x36
#define P1_23 0x37
#define P1_24 0x38
#define P1_25 0x39
#define P1_26 0x3A
#define P1_27 0x3B
#define P1_28 0x3C
#define P1_29 0x3D
#define P1_30 0x3E
#define P1_31 0x3F

#define A0  P0_16
#define A1  P0_23
#define A2  P0_00
#define A3  P1_31
#define A4  P0_13
#define A5  P0_14

#define D0  P1_24
#define D1  P0_27
#define D2	P0_15
#define D3	P1_06
#define D4  P1_07
#define D5	P1_04
#define D6	P1_10
#define D7	P1_09
#define D8	P1_08
#define D9	P1_05
#define D10	P1_01
#define D11	P0_26
#define D12	P1_03
#define D13	P1_02
#define D14	P1_21
#define D15	P1_20

#define LED_RED   P1_06
#define LED_GREEN P1_07
#define LED_BLUE  P1_04

#define USER_SW   P1_09
#define WAKE_SW   P1_18

#define UART_TX   D1
#define UART_RX	  D0

#define IIC_SCL   D15
#define IIC_SDA	  D14

#define SPI_CS    D10
#define SPI_CLK   D13
#define SPI_MOSI  D11
#define SPI_MISO  D12

#define MAX_PIN_NUM  0x40

#define PIN_CHECK_VALID(pin) ((pin >=  MAX_PIN_NUM) ? 0 : 1)
#define PIN_HAS_ADC(pin) ((pin == P0_23 || pin == P0_10 || pin == P0_15 || \
      pin == P0_31 || pin == P1_08 || pin == P0_16 || pin == P1_00 || pin == P1_09) ? 1 : 0)

#ifdef __cplusplus
}
#endif

#endif //_PIN_MAPPING_H_