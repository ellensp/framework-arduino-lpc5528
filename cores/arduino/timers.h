/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2013 by Paul Stoffregen <paul@pjrc.com> (delayMicroseconds)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _TIMERS_H_
#define _TIMERS_H_

#ifndef __cplusplus
extern "C" {
#endif

#include "Arduino.h"

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "fsl_clock.h"
#include "fsl_ctimer.h"
#include <stdint.h>

typedef void (*p_handler)(void * para);

// Timers API
void timer_init(void);
//interval = 1 means 1us
void timer_start(uint8_t timer_num,uint32_t interval);
void timer_stop(uint8_t timer_num);
void timer_update_period(uint8_t timer_num,uint32_t interval);
// register callback function
void timer_isr_callback_register(uint8_t time_no, p_handler timer_isr_user);

#ifndef __cplusplus
extern "C" {
#endif

#endif
