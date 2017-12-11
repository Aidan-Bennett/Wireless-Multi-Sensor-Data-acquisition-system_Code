/*

  Arduino Library for simplifying quick enabling and disabling 
  of preconfigured external interrupts (preconfigured with 
  AttachInterrupt()), and uses elements from the WInterrupts.c
  file by Arduino LLC.
  Written by Aidan Bennett-Reilly for BRANZ NZ - 13/12/2016
  
  ---
  
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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


#ifndef _samd21_Sleep_function_h

#define _samd21_Sleep_function_h


#if defined(__SAMD21G18A__) || defined(__SAMD21J18A__) || defined(D__SAMD21J18A__)



#ifndef _samd21_h
	#include <Arduino.h>
#endif

#define MAX_INTERRUPT_PINS 10

class samd21_Sleep_function {
public:
	samd21_Sleep_function();

	bool begin(uint32_t pin);
	bool addExternalInterrupt(uint32_t pin);
	void standby();
	void idle();
	void restoreRegAfterIdle( );
	void interruptDisable();
	void interruptEnable();
	void pauseWakeupEvent( );
	

private:
	uint32_t _externalIntPin[MAX_INTERRUPT_PINS];
	uint8_t _NumExternalPins;
	
	uint32_t _AHBsettings;
	uint32_t _APBAsettings;
	uint32_t _APBBsettings;
	uint32_t _APBCsettings;
	uint32_t _NVMCTRL_CTRLB;
	uint32_t _SYSCTRL_VREG;
	uint32_t _GENCTRL0_BACKUP;
	
	bool _initialised;
	
	uint32_t pinToExternalInterrupt(uint32_t pin);
	void printbits( uint32_t myLong) ;
	
};



#else	
	#error This library only supports boards with an SAMD21G18A processor for now. Modify standbyStart() for compatability with your microcontroller.

#endif

#endif

