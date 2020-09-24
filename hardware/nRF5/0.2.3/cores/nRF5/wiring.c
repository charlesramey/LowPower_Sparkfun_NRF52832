/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.

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

#include <nrf.h>

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

void init( void ) {
/**   Rev 3 moved millis() to RTC1 with lp_timer
  NVIC_SetPriority(RTC2_IRQn, 15);
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_EnableIRQ(RTC2_IRQn);
**/
  #if defined(USE_LFXO)
    NRF_CLOCK->LFCLKSRC = (uint32_t)((CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
  #elif defined(USE_LFSYNT)
    NRF_CLOCK->LFCLKSRC = (uint32_t)((CLOCK_LFCLKSRC_SRC_Synth << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
  #else //USE_LFRC
    NRF_CLOCK->LFCLKSRC = (uint32_t)((CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
  #endif
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

/**   Rev 3 moved millis() to RTC1 with lp_timer
  NRF_RTC2->PRESCALER = 0;
  NRF_RTC2->INTENSET = RTC_INTENSET_OVRFLW_Msk;
  NRF_RTC2->EVTENSET = RTC_EVTEN_OVRFLW_Msk;
  NRF_RTC2->TASKS_START = 1;
 **/
 
 /**  No Reset Pin.  This define did not work for me. Needs extra linker script like NFC-GPIO pins?
  Reset pin code is in system_nrf52.c and needs the define -DCONFIG_GPIO_AS_PINRESET added to the 
   xxx.menu.softdevice.s132.build.extra_flags  in boards.txt for the particular board
  **/
}

#ifdef __cplusplus
}
#endif
