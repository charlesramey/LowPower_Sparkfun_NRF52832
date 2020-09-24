/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

#ifndef _WIRING_DIGITAL_
#define _WIRING_DIGITAL_

#ifdef __cplusplus
 extern "C" {
#endif

// #include "WVariant.h"

/**
 * \brief Configures the specified pin to behave either as an input or an output. See the description of digital pins for details.
 * For nRF52 extra modes are available.  
 * Standard drive is typically 2mA (min 1mA) '0' sink (low) or '1' source (high). High drive (VDD > 2.7V) is typically 10mA low, 9mA high (min 6mA)
 *
 * OUTPUT_S0S1  Standard '0', standard '1'  same as OUTPUT
 * OUTPUT_H0S1  High drive '0', standard '1'
 * OUTPUT_S0H1  Standard '0', high drive '1'
 * OUTPUT_H0H1  High drive '0', high 'drive '1''
 * OUTPUT_D0S1  Disconnect '0' standard '1' (normally used for wired-or connections) 
 * OUTPUT_D0H1  Disconnect '0', high drive '1' (normally used for wired-or connections) 
 * OUTPUT_S0D1  Standard '0'. disconnect '1' (normally used for wired-and connections) 
 * OUTPUT_H0D1  High drive '0', disconnect '1' (normally used for wired-and connections) 
 * NOTE P0.27 should be only low (standard) drive, low frequency
 *
 * \param ulPin The number of the pin whose mode you wish to set
 * \param ulMode Can be INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN
 * or OUTPUT_S0S1, OUTPUT_H0S1, OUTPUT_S0H1, OUTPUT_H1H1, OUTPUT_D0S1, OUTPUT_D0H1, OUTPUT_S0D1, OUTPUT_H0D1
 */
extern void pinMode( uint32_t dwPin, uint32_t dwMode ) ;

/**
 * \brief Write a HIGH or a LOW value to a digital pin.
 *
 * If the pin has been configured as an OUTPUT with pinMode(), its voltage will be set to the
 * corresponding value: 5V (or 3.3V on 3.3V boards) for HIGH, 0V (ground) for LOW.
 *
 * If the pin is configured as an INPUT, writing a HIGH value with digitalWrite() will enable an internal
 * 20K pullup resistor (see the tutorial on digital pins). Writing LOW will disable the pullup. The pullup
 * resistor is enough to light an LED dimly, so if LEDs appear to work, but very dimly, this is a likely
 * cause. The remedy is to set the pin to an output with the pinMode() function.
 *
 * \note Digital pin PIN_LED is harder to use as a digital input than the other digital pins because it has an LED
 * and resistor attached to it that's soldered to the board on most boards. If you enable its internal 20k pull-up
 * resistor, it will hang at around 1.7 V instead of the expected 5V because the onboard LED and series resistor
 * pull the voltage level down, meaning it always returns LOW. If you must use pin PIN_LED as a digital input, use an
 * external pull down resistor.
 *
 * \param dwPin the pin number
 * \param dwVal HIGH or LOW
 */
extern void digitalWrite( uint32_t dwPin, uint32_t dwVal ) ;

/**
 * \brief Reads the value from a specified digital pin, either HIGH or LOW.
 *
 * \param ulPin The number of the digital pin you want to read (int)
 *
 * \return HIGH or LOW
 */
extern int digitalRead( uint32_t ulPin ) ;

#ifdef __cplusplus
}
#endif

#endif /* _WIRING_DIGITAL_ */
