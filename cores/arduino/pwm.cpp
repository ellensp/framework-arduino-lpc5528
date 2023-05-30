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

#include "pwm.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t ResulotionBit = 100;
uint8_t  sct_init_flag = 1;


/*******************************************************************************
 * PWM Functions
 ******************************************************************************/

void analogWriteResolution(uint16_t bit)
{
	SCTIMER_StopTimer(SCT0, (uint32_t)kSCTIMER_Counter_U);
	if(bit>=100&&bit<=65535)
	{
   		ResulotionBit = bit;
	}
	SCTIMER_StartTimer(SCT0, (uint32_t)kSCTIMER_Counter_U);
}

void analogWrite(uint8_t pwm_pin,uint16_t pwm_duty)
{
    uint32_t freq = 0;
	uint32_t pwm_freq=0;
	uint32_t pwm_period = 0;
	uint32_t pwm_pulse = 0;
	sctimer_config_t sctimerInfo;

	SCTIMER_GetDefaultConfig(&sctimerInfo);
	sctimerInfo.enableCounterUnify = false;
	freq = CLOCK_GetFreq(kCLOCK_BusClk);
	pwm_freq = PwmFrequency;

	pwm_period = freq / pwm_freq -1;
	pwm_pulse = pwm_period * pwm_duty / ResulotionBit;
    SCTIMER_StopTimer(SCT0, (uint32_t)kSCTIMER_Counter_L);

	if(sct_init_flag == 1)
	{
		/* Initialize SCTimer module */
		SCTIMER_Init(SCT0, &sctimerInfo);
		SCT0->CONFIG &= ~(1 << 0);      // two 16-bit timers,
		SCT0->CONFIG |= ( 1 << 17);     // auto limit
		SCT0->MATCHREL[0] = pwm_period; // match 0
		SCT0->EV[0].STATE = 0XFFFFFFFF; // event 0 happens in all states
		SCT0->EV[0].CTRL = (1 << 12);   // match 0 condition only
		sct_init_flag= 2;
	}
    CLOCK_EnableClock(kCLOCK_Iocon);
	switch(pwm_pin)
	{
		//SCT_OUT0
		case 0x02: //PIO0_2
		case 0x11: //PIO0_17
		case 0x24: //PIO1_4
		case 0x37: //PIO1_23
        if(pwm_pin==0x02) IOCON_PinMuxSet(IOCON, 0U, 2U,  SCTOUT_PIN_CONFIG_03);
				if(pwm_pin==0x11) IOCON_PinMuxSet(IOCON, 0U, 17U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x24) IOCON_PinMuxSet(IOCON, 1U, 4U,  SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x37) IOCON_PinMuxSet(IOCON, 1U, 23U, SCTOUT_PIN_CONFIG_02);
				SCT0->MATCHREL[1] = pwm_pulse;// match 1 used for duty cycle
				SCT0->EV[1].STATE = 0XFFFFFFFF; // event 1 happens in all states
				SCT0->EV[1].CTRL = (0x1 << 0) | (1 << 12); // match 1 condition only
				SCT0->OUT[0].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT0
				SCT0->OUT[0].CLR = ( 1 << 1); // event 1 will clear SCTx_OUT0
		break;

		//SCT_OUT1
		case 0x03: //PIO0_3
		case 0x12: //PIO0_18
		case 0x28: //PIO1_8
		case 0x38: //PIO1_24
			  if(pwm_pin==0x03) IOCON_PinMuxSet(IOCON, 0U, 3U,  SCTOUT_PIN_CONFIG_03);
				if(pwm_pin==0x12) IOCON_PinMuxSet(IOCON, 0U, 18U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x28) IOCON_PinMuxSet(IOCON, 1U, 8U,  SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x38) IOCON_PinMuxSet(IOCON, 1U, 24U, SCTOUT_PIN_CONFIG_02);
				SCT0->MATCHREL[2] = pwm_pulse;// match 2 used for duty cycle
				SCT0->EV[2].STATE = 0XFFFFFFFF; // event 2 happens in all states
				SCT0->EV[2].CTRL = (0x2 << 0) | (1 << 12); // match 2 condition only
				SCT0->OUT[1].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT1
				SCT0->OUT[1].CLR = ( 1 << 2); // event 2 will clear SCTx_OUT1
		break;

		//SCT_OUT2
		case 0x0A: //P0_10
        case 0x0F: //P0_15
		case 0x13: //P0_19
		case 0x29: //P1_09
		case 0x39: //P1_25
			  if(pwm_pin==0x0A) IOCON_PinMuxSet(IOCON, 0U, 10U, SCTOUT_PIN_CONFIG_05);
				if(pwm_pin==0x0F) IOCON_PinMuxSet(IOCON, 0U, 15U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x13) IOCON_PinMuxSet(IOCON, 0U, 19U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x29) IOCON_PinMuxSet(IOCON, 1U, 9U,  SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x39) IOCON_PinMuxSet(IOCON, 1U, 25U, SCTOUT_PIN_CONFIG_02);
				SCT0->MATCHREL[3] = pwm_pulse;// match 3 used for duty cycle
				SCT0->EV[3].STATE = 0XFFFFFFFF; // event 3 happens in all states
				SCT0->EV[3].CTRL = (0x3 << 0) | (1 << 12); // match 3 condition only
				SCT0->OUT[2].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT2
				SCT0->OUT[2].CLR = ( 1 << 3); // event 3 will clear SCTx_OUT2
		break;

		//SCT_OUT3
		case 0x16: //P0_22
		case 0x1F: //P0_31
		case 0x2A: //P1_10
		case 0x3A: //P1_26
			    if(pwm_pin==0x16) IOCON_PinMuxSet(IOCON, 0U, 22U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x1F) IOCON_PinMuxSet(IOCON, 0U, 31U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x2A) IOCON_PinMuxSet(IOCON, 1U, 10U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x3A) IOCON_PinMuxSet(IOCON, 1U, 26U, SCTOUT_PIN_CONFIG_02);
				SCT0->MATCHREL[4] = pwm_pulse;// match 4 used for duty cycle
				SCT0->EV[4].STATE = 0XFFFFFFFF; // event 4 happens in all states
				SCT0->EV[4].CTRL = (0x4 << 0) | (1 << 12); // match 4 condition only
				SCT0->OUT[3].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT3
				SCT0->OUT[3].CLR = ( 1 << 4); // event 4 will clear SCTx_OUT3
		break;

		//SCT_OUT4
		case 0x17: //P0_23
		case 0x23: //P1_03
		case 0x31: //P1_17
			    if(pwm_pin==0x17) IOCON_PinMuxSet(IOCON, 0U, 23U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x23) IOCON_PinMuxSet(IOCON, 1U, 3U,  SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x31) IOCON_PinMuxSet(IOCON, 1U, 17U, SCTOUT_PIN_CONFIG_04);
				SCT0->MATCHREL[5] = pwm_pulse;// match 5 used for duty cycle
				SCT0->EV[5].STATE = 0XFFFFFFFF; // event 5 happens in all states
				SCT0->EV[5].CTRL = (0x5 << 0) | (1 << 12); // match 5 condition only
				SCT0->OUT[4].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT4
				SCT0->OUT[4].CLR = ( 1 << 5); // event 5 will clear SCTx_OUT4
		break;

		//SCT_OUT5
		case 0x1A: //P0_26
		case 0x32: //P1_18
			    if(pwm_pin==0x1A) IOCON_PinMuxSet(IOCON, 0U, 26U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x32) IOCON_PinMuxSet(IOCON, 1U, 18U, SCTOUT_PIN_CONFIG_04);
				SCT0->MATCHREL[6] = pwm_pulse;// match 6 used for duty cycle
				SCT0->EV[6].STATE = 0XFFFFFFFF; // event 6 happens in all states
				SCT0->EV[6].CTRL = (0x6 << 0) | (1 << 12); // match 6 condition only
				SCT0->OUT[5].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT5
				SCT0->OUT[5].CLR = ( 1 << 6); // event 6 will clear SCTx_OUT5
		break;

		//SCT_OUT6
		case 0x1B: //P0_27
		case 0x3F: //P1_31
			    if(pwm_pin==0x1B) IOCON_PinMuxSet(IOCON, 0U, 27U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x3F) IOCON_PinMuxSet(IOCON, 1U, 31U, SCTOUT_PIN_CONFIG_04);
				SCT0->MATCHREL[7] = pwm_pulse;// match 7 used for duty cycle
				SCT0->EV[7].STATE = 0XFFFFFFFF; // event 7 happens in all states
				SCT0->EV[7].CTRL = (0x7 << 0) | (1 << 12); // match 7 condition only
				SCT0->OUT[6].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT6
				SCT0->OUT[6].CLR = ( 1 << 7); // event 7 will clear SCTx_OUT6
		break;

        //SCT_OUT7
		case 0x1C: //P0_28
		case 0x33: //P1_19
			    if(pwm_pin==0x1C) IOCON_PinMuxSet(IOCON, 0U, 28U, SCTOUT_PIN_CONFIG_04);
				if(pwm_pin==0x33) IOCON_PinMuxSet(IOCON, 1U, 19U, SCTOUT_PIN_CONFIG_02);
				SCT0->MATCHREL[8] = pwm_pulse;// match 5 used for duty cycle
				SCT0->EV[8].STATE = 0XFFFFFFFF; // event 5 happens in all states
				SCT0->EV[8].CTRL = (0x8 << 0) | (1 << 12); // match 8 condition only
				SCT0->OUT[7].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT4
				SCT0->OUT[7].CLR = ( 1 << 8); // event 5 will clear SCTx_OUT4
		break;

		//SCT_OUT8
		case 0x1D: //P0_29
			    IOCON_PinMuxSet(IOCON, 0U, 29U, SCTOUT_PIN_CONFIG_04);
				SCT0->MATCHREL[9] = pwm_pulse;// match 6 used for duty cycle
				SCT0->EV[9].STATE = 0XFFFFFFFF; // event 6 happens in all states
				SCT0->EV[9].CTRL = (0x9 << 0) | (1 << 12); // match 6 condition only
				SCT0->OUT[8].SET = ( 1 << 0); 	// event 0 will set SCTx_OUT5
				SCT0->OUT[8].CLR = ( 1 << 9); // event 6 will clear SCTx_OUT5
		break;

		//SCT_OUT9
	    case 0x1E: //P0_30
			    IOCON_PinMuxSet(IOCON, 0U, 30U, SCTOUT_PIN_CONFIG_04);
				SCT0->MATCHREL[10] = pwm_pulse;             // match 7 used for duty cycle
				SCT0->EV[10].STATE = 0XFFFFFFFF;            // event 7 happens in all states
				SCT0->EV[10].CTRL = (0xA << 0) | (1 << 12); // match 7 condition only
				SCT0->OUT[9].SET = ( 1 << 0); 	            // event 0 will set SCTx_OUT6
				SCT0->OUT[9].CLR = ( 1 << 10);              // event 7 will clear SCTx_OUT6
		break;

		default :
		break;
	}
	CLOCK_DisableClock(kCLOCK_Iocon);
	SCTIMER_StartTimer(SCT0, (uint32_t)kSCTIMER_Counter_L);
}


void analogWriteDuty(uint8_t pwm_pin,uint16_t pwm_duty) {
	uint32_t pwm_pulse = 0;
	SCT0->CONFIG |= (1 << 7);

	pwm_pulse = SCT0->MATCHREL[0] * pwm_duty / ResulotionBit;
	SCT0->MATCHREL[(pwm_pin + 1)] = pwm_pulse;

	SCT0->CONFIG &= ~(1 << 7);
}

//********************************************
void pwm_init(const uint16_t frequency) {
    uint32_t Clockfreq = 0;
	// uint32_t pwm_period = 0;
	sctimer_config_t sctimerInfo;

	SCTIMER_GetDefaultConfig(&sctimerInfo);
	// sctimerInfo.enableCounterUnify = false;
	Clockfreq = CLOCK_GetFreq(kCLOCK_BusClk);

	// pwm_period = (Clockfreq / frequency) -1;
    // SCTIMER_StopTimer(SCT0, (uint32_t)kSCTIMER_Counter_L);
	// SCTIMER_StopTimer(SCT0, (uint32_t)kSCTIMER_Counter_U);

	/* Initialize SCTimer module */
	SCTIMER_Init(SCT0, &sctimerInfo);
	SCT0->CONFIG |= (1 << 0);       // 32-bit timers,
	// SCT0->CONFIG &= ~(1 << 0);       // two 16-bit timers,
	SCT0->CONFIG |= ( 1 << 17);      // auto limit
	// SCT0->MATCHREL[0] = pwm_period;  // match 0
	SCT0->MATCHREL[0] = (Clockfreq / frequency) -1;  // match 0
	SCT0->EV[0].STATE = 0XFFFFFFFF;  // event 0 happens in all states
	SCT0->EV[0].CTRL = (1 << 12);    // match 0 condition only
}


bool pwm_attach_pin(const uint8_t pwm_pin, const uint32_t value) {
	    uint32_t pwm_pulse = 0;
		pwm_pulse = SCT0->MATCHREL[0] * value / ResulotionBit;
	    CLOCK_EnableClock(kCLOCK_Iocon);
		switch(pwm_pin)
		{
			//SCT_OUT0
			case 0x02: //PIO0_2
			case 0x11: //PIO0_17
			case 0x24: //PIO1_4
			case 0x37: //PIO1_23
					if(pwm_pin==0x02) IOCON_PinMuxSet(IOCON, 0U, 2U,  SCTOUT_PIN_CONFIG_03);
					if(pwm_pin==0x11) IOCON_PinMuxSet(IOCON, 0U, 17U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x24) IOCON_PinMuxSet(IOCON, 1U, 4U,  SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x37) IOCON_PinMuxSet(IOCON, 1U, 23U, SCTOUT_PIN_CONFIG_02);
					SCT0->MATCHREL[1] = SCT0->MATCHREL[0] * value / ResulotionBit;
					// SCT0->MATCHREL[1] = pwm_pulse;             // match 1 used for duty cycle
					SCT0->EV[1].STATE = 0XFFFFFFFF;            // event 1 happens in all states
					SCT0->EV[1].CTRL = (0x1 << 0) | (1 << 12); // match 1 condition only
					SCT0->OUT[0].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT0
					SCT0->OUT[0].CLR = ( 1 << 1);              // event 1 will clear SCTx_OUT0
			break;

			//SCT_OUT1
			case 0x03: //PIO0_3
			case 0x12: //PIO0_18
			case 0x28: //PIO1_8
			case 0x38: //PIO1_24
					if(pwm_pin==0x03) IOCON_PinMuxSet(IOCON, 0U, 3U,  SCTOUT_PIN_CONFIG_03);
					if(pwm_pin==0x12) IOCON_PinMuxSet(IOCON, 0U, 18U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x28) IOCON_PinMuxSet(IOCON, 1U, 8U,  SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x38) IOCON_PinMuxSet(IOCON, 1U, 24U, SCTOUT_PIN_CONFIG_02);
					SCT0->MATCHREL[2] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 2 used for duty cycle
					SCT0->EV[2].STATE = 0XFFFFFFFF;            // event 2 happens in all states
					SCT0->EV[2].CTRL = (0x2 << 0) | (1 << 12); // match 2 condition only
					SCT0->OUT[1].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT1
					SCT0->OUT[1].CLR = ( 1 << 2);              // event 2 will clear SCTx_OUT1
			break;

			//SCT_OUT2
			case 0x0A: //P0_10
			case 0x0F: //P0_15
			case 0x13: //P0_19
			case 0x29: //P1_09
			case 0x39: //P1_25
					if(pwm_pin==0x0A) IOCON_PinMuxSet(IOCON, 0U, 10U, SCTOUT_PIN_CONFIG_05);
					if(pwm_pin==0x0F) IOCON_PinMuxSet(IOCON, 0U, 15U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x13) IOCON_PinMuxSet(IOCON, 0U, 19U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x29) IOCON_PinMuxSet(IOCON, 1U, 9U,  SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x39) IOCON_PinMuxSet(IOCON, 1U, 25U, SCTOUT_PIN_CONFIG_02);
					SCT0->MATCHREL[3] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 3 used for duty cycle
					SCT0->EV[3].STATE = 0XFFFFFFFF;            // event 3 happens in all states
					SCT0->EV[3].CTRL = (0x3 << 0) | (1 << 12); // match 3 condition only
					SCT0->OUT[2].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT2
					SCT0->OUT[2].CLR = ( 1 << 3);              // event 3 will clear SCTx_OUT2
			break;

			//SCT_OUT3
			case 0x16: //P0_22
			case 0x1F: //P0_31
			case 0x2A: //P1_10
			case 0x3A: //P1_26
					if(pwm_pin==0x16) IOCON_PinMuxSet(IOCON, 0U, 22U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x1F) IOCON_PinMuxSet(IOCON, 0U, 31U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x2A) IOCON_PinMuxSet(IOCON, 1U, 10U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x3A) IOCON_PinMuxSet(IOCON, 1U, 26U, SCTOUT_PIN_CONFIG_02);
					SCT0->MATCHREL[4] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 4 used for duty cycle
					SCT0->EV[4].STATE = 0XFFFFFFFF;            // event 4 happens in all states
					SCT0->EV[4].CTRL = (0x4 << 0) | (1 << 12); // match 4 condition only
					SCT0->OUT[3].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT3
					SCT0->OUT[3].CLR = ( 1 << 4);              // event 4 will clear SCTx_OUT3
			break;

			//SCT_OUT4
			case 0x17: //P0_23
			case 0x23: //P1_03
			case 0x31: //P1_17
					if(pwm_pin==0x17) IOCON_PinMuxSet(IOCON, 0U, 23U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x23) IOCON_PinMuxSet(IOCON, 1U, 3U,  SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x31) IOCON_PinMuxSet(IOCON, 1U, 17U, SCTOUT_PIN_CONFIG_04);
					SCT0->MATCHREL[5] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 5 used for duty cycle
					SCT0->EV[5].STATE = 0XFFFFFFFF;            // event 5 happens in all states
					SCT0->EV[5].CTRL = (0x5 << 0) | (1 << 12); // match 5 condition only
					SCT0->OUT[4].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT4
					SCT0->OUT[4].CLR = ( 1 << 5);              // event 5 will clear SCTx_OUT4
			break;

			//SCT_OUT5
			case 0x1A: //P0_26
			case 0x32: //P1_18
					if(pwm_pin==0x1A) IOCON_PinMuxSet(IOCON, 0U, 26U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x32) IOCON_PinMuxSet(IOCON, 1U, 18U, SCTOUT_PIN_CONFIG_04);
					SCT0->MATCHREL[6] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 6 used for duty cycle
					SCT0->EV[6].STATE = 0XFFFFFFFF;            // event 6 happens in all states
					SCT0->EV[6].CTRL = (0x6 << 0) | (1 << 12); // match 6 condition only
					SCT0->OUT[5].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT5
					SCT0->OUT[5].CLR = ( 1 << 6);              // event 6 will clear SCTx_OUT5
			break;

			//SCT_OUT6
			case 0x1B: //P0_27
			case 0x3F: //P1_31
					if(pwm_pin==0x1B) IOCON_PinMuxSet(IOCON, 0U, 27U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x3F) IOCON_PinMuxSet(IOCON, 1U, 31U, SCTOUT_PIN_CONFIG_04);
					SCT0->MATCHREL[7] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 7 used for duty cycle
					SCT0->EV[7].STATE = 0XFFFFFFFF;            // event 7 happens in all states
					SCT0->EV[7].CTRL = (0x7 << 0) | (1 << 12); // match 7 condition only
					SCT0->OUT[6].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT6
					SCT0->OUT[6].CLR = ( 1 << 7);              // event 7 will clear SCTx_OUT6
			break;

			//SCT_OUT7
			case 0x1C: //P0_28
			case 0x33: //P1_19
					if(pwm_pin==0x1C) IOCON_PinMuxSet(IOCON, 0U, 28U, SCTOUT_PIN_CONFIG_04);
					if(pwm_pin==0x33) IOCON_PinMuxSet(IOCON, 1U, 19U, SCTOUT_PIN_CONFIG_02);
					SCT0->MATCHREL[8] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 5 used for duty cycle
					SCT0->EV[8].STATE = 0XFFFFFFFF;            // event 5 happens in all states
					SCT0->EV[8].CTRL = (0x8 << 0) | (1 << 12); // match 8 condition only
					SCT0->OUT[7].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT4
					SCT0->OUT[7].CLR = ( 1 << 8);              // event 5 will clear SCTx_OUT4
			break;

			//SCT_OUT8
			case 0x1D: //P0_29
					IOCON_PinMuxSet(IOCON, 0U, 29U, SCTOUT_PIN_CONFIG_04);
					SCT0->MATCHREL[9] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 6 used for duty cycle
					SCT0->EV[9].STATE = 0XFFFFFFFF;            // event 6 happens in all states
					SCT0->EV[9].CTRL = (0x9 << 0) | (1 << 12); // match 6 condition only
					SCT0->OUT[8].SET = ( 1 << 0); 	           // event 0 will set SCTx_OUT5
					SCT0->OUT[8].CLR = ( 1 << 9);              // event 6 will clear SCTx_OUT5
			break;

			//SCT_OUT9
			case 0x1E: //P0_30
					IOCON_PinMuxSet(IOCON, 0U, 30U, SCTOUT_PIN_CONFIG_04);
					SCT0->MATCHREL[10] = SCT0->MATCHREL[0] * value / ResulotionBit;             // match 7 used for duty cycle
					SCT0->EV[10].STATE = 0XFFFFFFFF;            // event 7 happens in all states
					SCT0->EV[10].CTRL = (0xA << 0) | (1 << 12); // match 7 condition only
					SCT0->OUT[9].SET = ( 1 << 0); 	            // event 0 will set SCTx_OUT6
					SCT0->OUT[9].CLR = ( 1 << 10);              // event 7 will clear SCTx_OUT6
			break;

			default :
			break;
		}
		CLOCK_DisableClock(kCLOCK_Iocon);
		SCTIMER_StartTimer(SCT0, (uint32_t)kSCTIMER_Counter_U);
		// SCTIMER_StartTimer(SCT0, (uint32_t)kSCTIMER_Counter_L);
		return true;
}

bool pwm_detach_pin(const uint8_t pin) {
        SCTIMER_StopTimer(SCT0, (uint32_t)kSCTIMER_Counter_U);
		pinMode(pin,OUTPUT);
		digitalWrite(pin,LOW);
		return true;
}

uint32_t pwm_get_period(const uint8_t pin) {
	return (SCT0->MATCHREL[0]+1);
}

bool pwm_set_frequency(const uint8_t pin, const uint32_t frequency) {
	return 0;
}

bool pwm_write(const uint8_t pin, const uint32_t value) {
	uint32_t pwm_pulse = 0;
	uint8_t pwmid=0;

	SCT0->CONFIG |= (1 << 7);
	pwm_pulse = SCT0->MATCHREL[0] * ((float)(value) / (float)(ResulotionBit));
    pwmid = pwm_outchannel_id(pin);
	if(pwmid>=0&&pwmid<=9) SCT0->MATCHREL[pwmid+1] = pwm_pulse;
    // if(pwmid>=0&&pwmid<=9) SCT0->MATCHREL[pwmid+1] = SCT0->MATCHREL[0] * value / ResulotionBit;
	SCT0->CONFIG &= ~(1 << 7);
	return 1;
}

bool pwm_write_us(const uint8_t pin, const uint32_t value) {
	uint32_t pwm_pulse = 0;
	uint8_t pwmid=0;

    SCT0->CONFIG |= (1 << 7);
	pwm_pulse = CLOCK_GetFreq(kCLOCK_BusClk) / 1000000 * value;
    pwmid = pwm_outchannel_id(pin);
	if(pwmid>=0&&pwmid<=9) SCT0->MATCHREL[pwmid+1] = pwm_pulse;

	SCT0->CONFIG &= ~(1 << 7);
	return 1;
}

uint8_t pwm_outchannel_id(const uint8_t pin) {
	switch(pin)
		{
			//SCT_OUT0
			case 0x02: //PIO0_2
			case 0x11: //PIO0_17
			case 0x24: //PIO1_4
			case 0x37: //PIO1_23
					return 0;
			break;

			//SCT_OUT1
			case 0x03: //PIO0_3
			case 0x12: //PIO0_18
			case 0x28: //PIO1_8
			case 0x38: //PIO1_24
					return 1;
			break;

			//SCT_OUT2
			case 0x0A: //P0_10
			case 0x0F: //P0_15
			case 0x13: //P0_19
			case 0x29: //P1_09
			case 0x39: //P1_25
					return 2;
			break;

			//SCT_OUT3
			case 0x16: //P0_22
			case 0x1F: //P0_31
			case 0x2A: //P1_10
			case 0x3A: //P1_26
					return 3;
			break;

			//SCT_OUT4
			case 0x17: //P0_23
			case 0x23: //P1_03
			case 0x31: //P1_17
					return 4;
			break;

			//SCT_OUT5
			case 0x1A: //P0_26
			case 0x32: //P1_18
					return 5;
			break;

			//SCT_OUT6
			case 0x1B: //P0_27
			case 0x3F: //P1_31
					return 6;
			break;

			//SCT_OUT7
			case 0x1C: //P0_28
			case 0x33: //P1_19
					return 7;
			break;

			//SCT_OUT8
			case 0x1D: //P0_29
					return 8;
			break;

			//SCT_OUT9
			case 0x1E: //P0_30
					return 9;
			break;

			default :
			 		return 100;
			break;
		}
}
