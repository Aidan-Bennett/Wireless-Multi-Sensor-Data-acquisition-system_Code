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

#include "samd21_Sleep_function.h"


samd21_Sleep_function::samd21_Sleep_function()
{
	_initialised = false;
}

/**************************************************************************/
/*!
    @brief  Initialises library with a interrupt pin to use

	
	@params[in] _externalIntPin
				The array holding the pins to use as external interrupts
    @params[in] _NumExternalPins
                The number of pins to use as external interrupts
				
	@params[in] pin
                The digital pin value to set the interrupts on.	
				
	@returns    True if initialising was successful and pin is valid.
*/
/**************************************************************************/
bool samd21_Sleep_function::begin(uint32_t pin) {
	uint32_t tempPin =  pinToExternalInterrupt(pin);
	//Serial.println(tempPin);
	if(tempPin != 512){
			
		_NumExternalPins = 0;
		_externalIntPin[_NumExternalPins] = tempPin;
		//_NumExternalPins++;
		_initialised = true;
		return _initialised;
	}
	else{
		_initialised = false;
		return _initialised;
	}
	
}


/**************************************************************************/
/*!
    @brief  Adds a Pin for the corresponding external interrupt to be 
			enabled/disabled by the library

    @params[in] _externalIntPin
				The array holding the pins to use as external interrupts
    @params[in] _NumExternalPins
                The number of pins to use as external interrupts
				
	@params[in] pin
                The digital pin value to set the interrupts on.		

    @returns    True if adding pin was successful.
*/
/**************************************************************************/
bool samd21_Sleep_function::addExternalInterrupt(uint32_t pin) {
	uint32_t tempPin =  pinToExternalInterrupt(pin);
	
	if( _initialised && tempPin != 512 && (_NumExternalPins +1 < MAX_INTERRUPT_PINS ) ){  //-1 for array offset
		_NumExternalPins++;
		_externalIntPin[_NumExternalPins] = tempPin;
		
		return true;
	}
	else{
		return false; // no more pins supported by the library, or library not initialised or an invalid pin was given
	}
	
}



/**************************************************************************/
/*!
    @brief  Puts the device into Standby sleep mode

*/
/**************************************************************************/
// 
void samd21_Sleep_function::standby( ) {
	uint32_t temp;
	if(!_initialised){
		return;
	}
	
	//_AHBsettings = PM->AHBMASK.reg;
	//_APBAsettings = PM->APBAMASK.reg;
	//_APBBsettings = PM->APBBMASK.reg;
	//_APBCsettings = PM->APBCMASK.reg;
	_NVMCTRL_CTRLB = REG_NVMCTRL_CTRLB;
	_GENCTRL0_BACKUP = REG_GCLK_GENCTRL;
	_SYSCTRL_VREG = REG_SYSCTRL_VREG; // enable regulator to operate in idle mode
	
	
	//PM->AHBMASK.reg &= 0xFFFFFFF1; // Disable HPB1, HPB2 and DSU clocks
	
	//PM->APBBMASK.reg &= 0xFFFFFFE5; //turn DSU, PORT and DMAC clocks off, but leave PAC1 on
	//PM->APBCMASK.reg &= 0xFFE00001; // turn off all peripheral devices clocks except for PAC2
	
	//PM->APBAMASK.reg &= 0xFFFFFFCD; //turn Off WDT, RTC, PM, but leave PAC0 enabled
	
	//PM->APBCMASK.reg &= 0xFFE02802; // turn off all peripheral devices clocks apart from TC3, TC5 and EVSYS
	REG_NVMCTRL_CTRLB |= NVMCTRL_CTRLB_SLEEPPRM_DISABLED ; // fix for hangs while in sleep mode - see SAMD21 Errata
	REG_SYSCTRL_VREG |= SYSCTRL_VREG_RUNSTDBY;	// allow regulator to operate in standby

	
	interruptDisable();
	interruptEnable(); // enable interrupts for waking up
	
	PM-> SLEEP.bit.IDLE = 2; // set lowest idle mode
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; 	 // set Standby mode to use the lowest sleep state
	
	
	
	//delay(1);
	//asm("wfi");				// enter sleep mode & wait for Interrupt
	__WFI();
	//interruptDisable();
	//Restore clock settings
	
	//REG_GCLK_GENCTRL = _GENCTRL0_BACKUP;
	//while (GCLK->STATUS.bit.SYNCBUSY);
	//PM->APBAMASK.reg = _APBAsettings; //turn EIC and PAC0 back on
	//delay(1);
	//PM->AHBMASK.reg =  _AHBsettings; // Enable HPB1, HPB2 and DSU clocks
	//delay(1);
	
	//PM->APBBMASK.reg = _APBBsettings; // PAC1, DSU, PORT and DMAC clocks Back on
	//PM->APBCMASK.reg = _APBCsettings; // turn Sercom 4 & 5, EVSYS & PAC2 clocks on (rest can stay off till needed).
	REG_NVMCTRL_CTRLB = _NVMCTRL_CTRLB ;
	REG_SYSCTRL_VREG = _SYSCTRL_VREG;
}


/**************************************************************************/
/*!
    @brief  Puts the device into Standby sleep mode

*/
/**************************************************************************/
// reads a Page from FRAM and returns a byte
void samd21_Sleep_function::idle( ) {
	

	if(!_initialised){
		return;
	}
	//digitalWrite(13, HIGH);
	
	_AHBsettings = PM->AHBMASK.reg;
	_APBAsettings = PM->APBAMASK.reg;
	_APBBsettings = PM->APBBMASK.reg;
	_APBCsettings = PM->APBCMASK.reg;
	_NVMCTRL_CTRLB = REG_NVMCTRL_CTRLB;
	_SYSCTRL_VREG = REG_SYSCTRL_VREG; // enable regulator to operate in idle mode
	
	//interruptDisable(); // clear flag before sleeping
	//interruptEnable(); // enable interrupts for waking up
	
	//PM->AHBMASK.reg &= 0xFFFFFFF1; // Disable HPB1, HPB2 and DSU clocks
	//PM->APBAMASK.reg &= 0xFFFFFF8E; //turn EIC and PAC0 Off (and also WDT & RTC if not already disabled)
	//PM->APBBMASK.reg &= 0xFFFFFFE4; //turn PAC1, DSU, PORT and DMAC clocks off
	PM->APBCMASK.reg &= 0xFFc02803; // turn off all peripheral devices clocks apart from TC3, TC5, EVSYS and PAC2
			 
	
	PM-> SLEEP.reg |= PM_SLEEP_IDLE_CPU; // set idle mode to cpu idle only
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; 	 // set sleep to use idle mode instead of standby mode
	REG_SYSCTRL_VREG |= SYSCTRL_VREG_RUNSTDBY;	// allow regulator to operate in standby
	REG_NVMCTRL_CTRLB |= NVMCTRL_CTRLB_SLEEPPRM_DISABLED ;	// fix for hangs while in sleep mode - see SAMD21 Errata
	
	interruptDisable();
	interruptEnable();
	
	//asm("nop");
	//asm("wfi");				// enter sleep mode & wait for Interrupt
	__WFI();
	//digitalWrite(13, LOW);
	//interruptDisable();
	//Restore clock settings
	//PM->AHBMASK.reg =  _AHBsettings; // Enable HPB1, HPB2 and DSU clocks
	//PM->APBAMASK.reg = _APBAsettings; //turn EIC and PAC0 back on
	//PM->APBBMASK.reg = _APBBsettings; // PAC1, DSU, PORT and DMAC clocks Back on
	
	
}

/**************************************************************************/
/*!
    @brief  Restores the regisers after idle is used

*/
/**************************************************************************/
// reads a Page from FRAM and returns a byte
void samd21_Sleep_function::restoreRegAfterIdle( ) {
	PM->AHBMASK.reg =  _AHBsettings; // Enable HPB1, HPB2 and DSU clocks
	PM->APBAMASK.reg = _APBAsettings; //turn EIC and PAC0 back on
	PM->APBBMASK.reg = _APBBsettings; // PAC1, DSU, PORT and DMAC clocks Back on
	PM->APBCMASK.reg = _APBCsettings; // turn Sercom 4 & 5, EVSYS & PAC2 clocks on (rest can stay off till needed).
	REG_NVMCTRL_CTRLB = _NVMCTRL_CTRLB ;
	REG_SYSCTRL_VREG = _SYSCTRL_VREG;

}

/**************************************************************************/
/*!
    @brief  Disables External interrupts on Pins, and prevents device waking
			from the pins

    @params[in] _externalIntPin
				The array holding the pins to use as external interrupts
    @params[in] _NumExternalPins
                The number of pins to use as external interrupts

*/
/**************************************************************************/

void samd21_Sleep_function::interruptDisable( ) {
	if(!_initialised){
		return;
	}
	
	 EIC->CTRL.bit.ENABLE = 1;
	 while (EIC->STATUS.bit.SYNCBUSY == 1) { }
	
	for(uint8_t i = 0; i <= _NumExternalPins; i++){

		EIC->INTENCLR.reg =  EIC_INTENCLR_EXTINT(1 << _externalIntPin[i]); 	//disable external interrput pin
		EIC->INTFLAG.reg  =  EIC_INTFLAG_EXTINT(1 << _externalIntPin[i]); 	//reset Interrupt flag - re-enable if stability is an issue
		
		//EIC->WAKEUP.reg  &= ~EIC_WAKEUP_WAKEUPEN(1 << _externalIntPin[i]); 	// disable waking from pin
		//EIC->EVCTRL.reg  &= ~EIC_EVCTRL_EXTINTEO(1 << _externalIntPin[i]); 		// enable event interrupt output
	}

}




/**************************************************************************/
/*!
    @brief  Disables External interrupts on Pins, and prevents device waking
			from the pins

    @params[in] _externalIntPin
				The array holding the pins to use as external interrupts
    @params[in] _NumExternalPins
                The number of pins to use as external interrupts

*/
/**************************************************************************/
void samd21_Sleep_function::pauseWakeupEvent( ) {
	if(!_initialised){
		return;
	}
	EIC->CTRL.bit.ENABLE = 1; // just to force register syncronisation
	while (EIC->STATUS.bit.SYNCBUSY == 1) { }
	for(uint8_t i = 0; i <= _NumExternalPins; i++){
		EIC->EVCTRL.reg &=  ~EIC_EVCTRL_EXTINTEO(1 << _externalIntPin[i]); 	//disable external interrput pin
	}

}





/**************************************************************************/
/*!
    @brief  Enables External interrupts on Pins, and allows device waking
			from the pins

    @params[in] _externalIntPin
				The array holding the pins to use as external interrupts
    @params[in] _NumExternalPins
                The number of pins to use as external interrupts

*/
/**************************************************************************/
// reads a Page from FRAM and returns a byte
void samd21_Sleep_function::interruptEnable( ) {
	if(!_initialised){
		return;
	}
	
	EIC->CTRL.bit.ENABLE = 1; // just to force register syncronisation
	while (EIC->STATUS.bit.SYNCBUSY == 1) { }
	
	for(uint8_t i = 0; i <= _NumExternalPins; i++){
		//while (EIC->STATUS.bit.SYNCBUSY == 1) { }
		EIC->INTFLAG.reg  =  EIC_INTFLAG_EXTINT(1 << _externalIntPin[i]); 	//reset Interrupt flag
		
		EIC->INTENSET.reg |=  EIC_INTENSET_EXTINT(1 << _externalIntPin[i]); 	// Enable interrupts pin
		
		EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN(1 << _externalIntPin[i]); //enables wakeup on pakage pin
		
		
		
		//EIC->EVCTRL.reg |=  EIC_EVCTRL_EXTINTEO(1 << _externalIntPin[i]);
	}
	//printbits( EIC->WAKEUP.reg); 
	//printbits( EIC->INTENSET.reg); 

}



/**************************************************************************/
/*!
    @brief  Finds the External Interrupt pin value from the digital pin

	@params[in] pin
                The digital pin value to set the interrupts on.	

    @returns    The 32-bit value for the external interrupt	corresponding 
				to the provided digital pin.
*/
/**************************************************************************/
 uint32_t samd21_Sleep_function::pinToExternalInterrupt(uint32_t pin){
	 
	 /* this code is taken from the Adafruit SAMD21 WInterrupts.c to
		check if the provided digital pin maps to an external interrupt
		value, and returns the value.
		
	 */
	#if ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606
		EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
	#else
		EExt_Interrupts in = digitalPinToInterrupt(pin);
	#endif
	if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI)
		return 512;
	//EIC_INTENCLR_EXTINT(1 << in);
	else
		return in;
	
} 



void samd21_Sleep_function::printbits( uint32_t myLong) { //uint8_t bitSize, byte myByte = 0, uint16_t myInt = 0,
  uint8_t count = 0;
  for (uint32_t mask = 0x80000000; mask; mask >>= 1) {
    if (count == 8 || count == 16 || count == 24) {
      Serial.print(' ');
    }

    if (mask  & myLong)
      Serial.print('1');
    else
      Serial.print('0');
    count++;
  }

  Serial.println();
}