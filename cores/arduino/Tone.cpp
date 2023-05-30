
#include <pinmapping.h>
#include "Arduino.h"

pin_t tone_pin = P_NC;
volatile int32_t toggles = 0;
bool initialised_tone_timer = false;

void tone(const pin_t _pin, const uint32_t frequency, const uint32_t duration) {
  if ((tone_pin != P_NC && tone_pin != _pin) || _pin == P_NC || frequency == 0) return;
  // NVIC_DisableIRQ(TIMER2_IRQn);
  tone_pin = _pin;
  toggles = duration ? 2 * frequency * duration / 1000 : -1;

  if (!initialised_tone_timer) {
    // timer_init();
    initialised_tone_timer = true;
  }
  timer_start(2,frequency*5);
}

void noTone(const pin_t _pin) {
  pinMode(P1_25,OUTPUT);
  digitalWrite(_pin,LOW);
  tone_pin = P_NC;
  timer_stop(2);
}

extern "C" [[gnu::optimize("O3")]] void CTIMER2_IRQHandler(void) {
  uint32_t int_stat = 0;
  static uint8_t BuzzPinVal=0;
	/* Get Interrupt status flags */
	int_stat = CTIMER_GetStatusFlags(CTIMER2);
	/* Clear the status flags that were set */
	CTIMER_ClearStatusFlags(CTIMER2, int_stat);

  if (toggles != 0) {
    if(toggles > 0) {
        toggles--;
    }

    if(BuzzPinVal==0) {
      pinMode(P1_25,OUTPUT);
      digitalWrite(P1_25,HIGH);
      // digitalWrite(P1_25,!(digitalRead(P1_25)));
      BuzzPinVal=1;
      // Serial3.printf("pin HIGH\r\n");
    }
    else {
      pinMode(P1_25,OUTPUT);
      digitalWrite(P1_25,LOW);
      BuzzPinVal=0;
    }
  }
  else {
    noTone(tone_pin);
  }
}
