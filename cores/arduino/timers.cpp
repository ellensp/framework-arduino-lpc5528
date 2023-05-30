#include "timers.h"

static void (*pFunc_Ctimer[3]) (void * para) = {NULL,NULL,NULL};

void timer_init(void) {
  ctimer_config_t config;
  CTIMER_GetDefaultConfig(&config);

  CLOCK_AttachClk(kPLL0_to_CTIMER0);
  CLOCK_AttachClk(kPLL0_to_CTIMER1);
  CLOCK_AttachClk(kPLL0_to_CTIMER2);
//   CLOCK_AttachClk(kPLL0_to_CTIMER4);
  CTIMER_Init(CTIMER0, &config);
  CTIMER_Init(CTIMER1, &config);
  CTIMER_Init(CTIMER2, &config);
//   CTIMER_Init(CTIMER4, &config);
}

// interval = 1 ,means 1us
void timer_start(uint8_t timer_num,uint32_t interval) {
  ctimer_match_config_t matchConfig0;
  ctimer_match_config_t matchConfig1;
  ctimer_match_config_t matchConfig2;
//   ctimer_match_config_t matchConfig4;

	switch (timer_num) {

		case 0:/* Configuration 0 */
			matchConfig0.enableCounterReset = true;
			matchConfig0.enableCounterStop  = false;
			matchConfig0.matchValue         = interval*(CLOCK_GetCTimerClkFreq(0U) / 1000000);
			matchConfig0.outControl         = kCTIMER_Output_NoAction;
			matchConfig0.outPinInitState    = true;
			matchConfig0.enableInterrupt    = true;
			CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &matchConfig0);
		CTIMER0->MCR |= (1 << 24); // TIMER0 Channel0 Reload enable
		CTIMER0->MSR[0] |= matchConfig0.matchValue; // TIMER0 Channel0 reload value
		NVIC_SetPriority(CTIMER0_IRQn,2);
			CTIMER_StartTimer(CTIMER0);
			break;

		case 1:/* Configuration 1 */
			matchConfig1.enableCounterReset = true;
			matchConfig1.enableCounterStop  = false;
			matchConfig1.matchValue         = interval*(CLOCK_GetCTimerClkFreq(1U) / 1000000);
			matchConfig1.outControl         = kCTIMER_Output_NoAction;
			matchConfig1.outPinInitState    = true;
			matchConfig1.enableInterrupt    = true;
			CTIMER_SetupMatch(CTIMER1, kCTIMER_Match_0, &matchConfig1);
		CTIMER1->MCR |= (1 << 24); // TIMER1 Channel0 Reload enable
		CTIMER1->MSR[0] |= matchConfig1.matchValue; // TIMER1 Channel0 reload value
		NVIC_SetPriority(CTIMER1_IRQn,2);
			CTIMER_StartTimer(CTIMER1);
			break;

		case 2:/* Configuration 2 */
			matchConfig2.enableCounterReset = true;
			matchConfig2.enableCounterStop  = false;
			matchConfig2.matchValue         = interval*(CLOCK_GetCTimerClkFreq(2U) / 1000000);
			matchConfig2.outControl         = kCTIMER_Output_NoAction;
			matchConfig2.outPinInitState    = true;
			matchConfig2.enableInterrupt    = true;
			CTIMER_SetupMatch(CTIMER2, kCTIMER_Match_0, &matchConfig2);
		CTIMER2->MCR |= (1 << 24); // TIMER2 Channel0 Reload enable
		CTIMER2->MSR[0] |= matchConfig2.matchValue; // TIMER2 Channel0 reload value
		NVIC_SetPriority(CTIMER2_IRQn,2);
			CTIMER_StartTimer(CTIMER2);
			break;
			// case 4:
			//     matchConfig4.enableCounterReset = true;
			//     matchConfig4.enableCounterStop  = false;
			//     matchConfig4.matchValue         = interval*(CLOCK_GetCTimerClkFreq(4U) / 1000000);//give a default value 
			//     matchConfig4.outControl         = kCTIMER_Output_NoAction;
			//     matchConfig4.outPinInitState    = true;
			//     matchConfig4.enableInterrupt    = true; // disable interrupt first
			//     CTIMER_SetupMatch(CTIMER4, kCTIMER_Match_0, &matchConfig4);
			//     CTIMER4->MCR |= (1 << 24); // TIMER4 Channel0 Reload enable
			//     CTIMER4->MSR[0] = matchConfig4.matchValue; // TIMER4 Channel0 reload value
			//     NVIC_SetPriority(CTIMER4_IRQn, 2);
			//     CTIMER_StartTimer(CTIMER4);
			//     break;
		default: break;
	}
}


void timer_stop(uint8_t timer_num) {
	switch(timer_num)
	{
		case 0 : CTIMER_StopTimer(CTIMER0);
				 break;
		case 1 : CTIMER_StopTimer(CTIMER1);
				 break;
		case 2 : CTIMER_StopTimer(CTIMER2);
				 break;
        // case 4 : CTIMER_StopTimer(CTIMER4);
		// 		 break;
		default : break;
	}
}

void timer_update_period(uint8_t timer_num,uint32_t interval) {
	uint32_t match_val = 0;
	switch(timer_num)
	{
		case 0 : match_val = interval*(CLOCK_GetCTimerClkFreq(0U) / 1000000);
				 CTIMER0->MSR[0] = match_val;
				 break;
		case 1 : match_val = interval*(CLOCK_GetCTimerClkFreq(1U) / 1000000);
				 CTIMER1->MSR[0] = match_val;
				 break;
		case 2 : match_val = interval*(CLOCK_GetCTimerClkFreq(2U) / 1000000);
				 CTIMER2->MSR[0] = match_val;
				 break;
        // case 4 : match_val = interval*(CLOCK_GetCTimerClkFreq(4U) / 1000000);
		// 		 CTIMER4->MSR[0] = match_val;
		// 		 break;
		default : break;
	}
}

void timer_isr_callback_register(uint8_t time_num, p_handler timer_isr_user) {
	pFunc_Ctimer[time_num] = timer_isr_user ;
}

#if 0

extern "C"{
	void CTIMER0_IRQHandler(void) {
		uint32_t int_stat = 0;
		/* Get Interrupt status flags */
		int_stat = CTIMER_GetStatusFlags(CTIMER0);
		/* Clear the status flags that were set */
		CTIMER_ClearStatusFlags(CTIMER0, int_stat);
		//USER Code
		if(pFunc_Ctimer[0] != NULL)
		{
			pFunc_Ctimer[0](0);
		}

	}
	void CTIMER1_IRQHandler(void) {
		uint32_t int_stat = 0;
		/* Get Interrupt status flags */
		int_stat = CTIMER_GetStatusFlags(CTIMER1);
		/* Clear the status flags that were set */
		CTIMER_ClearStatusFlags(CTIMER1, int_stat);
		//USER Code
		if(pFunc_Ctimer[1] != NULL)
		{
			pFunc_Ctimer[1](0);
		}
	}

	void CTIMER2_IRQHandler(void) {
		uint32_t int_stat = 0;
		/* Get Interrupt status flags */
		int_stat = CTIMER_GetStatusFlags(CTIMER2);
		/* Clear the status flags that were set */
		CTIMER_ClearStatusFlags(CTIMER2, int_stat);
		//USER Code
		if(pFunc_Ctimer[2] != NULL)
		{
			pFunc_Ctimer[2](0);
		}
	}
}

#endif
