/* GPIO helper - utility-functions for direct GPIO-Access
 *
 * Copyright (C) 2016, F. Boesing
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy.
 * Please support PJRC's efforts to develop open source software by purchasing
 * Teensy or other genuine PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef GPIO_helper_h_
#define GPIO_helper_h_
#include <core_pins.h>

#if defined(KINETISK) //TODO: KINETISL (Teensy LC)

/*
#define PORTD_DFER		(*(volatile uint32_t *)0x4004C0C0) // Digital Filter Enable Register
#define PORTD_DFCR		(*(volatile uint32_t *)0x4004C0C4) // Digital Filter Clock Register
#define PORTD_DFWR		(*(volatile uint32_t *)0x4004C0C8) // Digital Filter Width Register
*/

struct sGPIO {
  uint32_t PDOR; //Port Data Output Register
  uint32_t PSOR; //Port Set Output Register
  uint32_t PCOR; //Port Clear Output Register
  uint32_t PTOR; //Port Toggle Output Register
  uint32_t PDIR; //Port Data Input Register
  uint32_t PDDR; //Port Data Direction Register
};

static volatile struct sGPIO * const GPIO_A = (struct sGPIO *)0x400FF000;
static volatile struct sGPIO * const GPIO_B = (struct sGPIO *)0x400FF040;
static volatile struct sGPIO * const GPIO_C = (struct sGPIO *)0x400FF080;
static volatile struct sGPIO * const GPIO_D = (struct sGPIO *)0x400FF0C0;
static volatile struct sGPIO * const GPIO_E = (struct sGPIO *)0x400FF0D0;

#endif

#if defined(KINETISL) || defined(KINETISK)
static const uint32_t pin_to_bitmask(const int pin) __attribute__((const));
static const uint32_t pin_to_bitmask(const int pin)
{
  switch(pin) {
	  default : return 0;
	  case 0 : return CORE_PIN0_BITMASK; break;
	  case 1 : return CORE_PIN1_BITMASK; break;
	  case 2 : return CORE_PIN2_BITMASK; break;
	  case 3 : return CORE_PIN3_BITMASK; break;
	  case 4 : return CORE_PIN4_BITMASK; break;
	  case 5 : return CORE_PIN5_BITMASK; break;
	  case 6 : return CORE_PIN6_BITMASK; break;
	  case 7 : return CORE_PIN7_BITMASK; break;
	  case 8 : return CORE_PIN8_BITMASK; break;
	  case 9 : return CORE_PIN9_BITMASK; break;
	  case 10: return CORE_PIN10_BITMASK; break;
	  case 11: return CORE_PIN11_BITMASK; break;
	  case 12: return CORE_PIN12_BITMASK; break;
	  case 13: return CORE_PIN13_BITMASK; break;
	  case 14: return CORE_PIN14_BITMASK; break;
	  case 15: return CORE_PIN15_BITMASK; break;
	  case 16: return CORE_PIN16_BITMASK; break;
	  case 17: return CORE_PIN17_BITMASK; break;
	  case 18: return CORE_PIN18_BITMASK; break;
	  case 19: return CORE_PIN18_BITMASK; break;
	  case 20: return CORE_PIN20_BITMASK; break;
	  case 21: return CORE_PIN21_BITMASK; break;
	  case 22: return CORE_PIN22_BITMASK; break;
	  case 23: return CORE_PIN23_BITMASK; break;
	  case 24: return CORE_PIN24_BITMASK; break;
	  case 25: return CORE_PIN25_BITMASK; break;
	  case 26: return CORE_PIN26_BITMASK; break;
#if defined(KINETISK)
	  case 27: return CORE_PIN27_BITMASK; break;
	  case 28: return CORE_PIN28_BITMASK; break;
	  case 29: return CORE_PIN29_BITMASK; break;
	  case 30: return CORE_PIN30_BITMASK; break;
	  case 31: return CORE_PIN31_BITMASK; break;
	  case 32: return CORE_PIN32_BITMASK; break;
	  case 33: return CORE_PIN33_BITMASK; break;
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	  case 34: return CORE_PIN34_BITMASK; break;
	  case 35: return CORE_PIN35_BITMASK; break;
	  case 36: return CORE_PIN36_BITMASK; break;
	  case 37: return CORE_PIN37_BITMASK; break;
	  case 38: return CORE_PIN38_BITMASK; break;
	  case 39: return CORE_PIN39_BITMASK; break;
#endif
#endif
  }
}
#endif

#endif