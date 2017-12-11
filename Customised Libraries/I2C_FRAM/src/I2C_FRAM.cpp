/**************************************************************************/
/*!
    @file     I2C_FRAM.cpp
    @author   KTOWN (Adafruit Industries) & Aidan Bennett-Reilly (BRANZ NZ)
    @license  BSD (see license.txt)

	Arduino Library for Fujitsu Semiconductor MB85RC256V 256Kb FRAM module.
	Written by Aidan Bennett-Reilly for BRANZ NZ - 13/12/2016
	
	This library modifies the Adafruit I2C FRAM library to include address 
	limit checks, writing/reading floats and uint16_t to FRAM and sequential
	reading from FRAM, for a Fujitsu Semiconductor MB85RC256V 256Kb FRAM 
	module, used in a custom version of a Open Source Building Science Sensors 
	(OSBSS - http://www.osbss.com/).
	

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v0.1 - First alpha
	
	

*/
/**************************************************************************/

#include "I2C_FRAM.h"




#define _addrLimit 32768




I2C_FRAM::I2C_FRAM()
{
	_initialised = false;
}

bool I2C_FRAM::begin(uint8_t address) {
  _address = address;
  Wire.begin();
  
  /* Make sure we're actually connected */
  uint16_t manufID, prodID;
  getDeviceID(&manufID, &prodID);
  if (manufID != 0x00A)
  {
    //Serial.print("Unexpected Manufacturer ID: 0x");
    //Serial.println(manufID, HEX);
    return false;
  }
  if (prodID != 0x510)
  {
    //Serial.print("Unexpected Product ID: 0x");
    //Serial.println(prodID, HEX);
    return false;
  }

  /* Everything seems to be properly initialised and connected */
  _initialised = true;

  return true;

}






/**************************************************************************/
/*!
    @brief  Reads multiple 8 bit value from the specified FRAM address

    @params[in] _address
                The I2C address of the FRAM memory chip (1010+A2+A1+A0)
    @params[in] addr
                The 16-bit address to read from in FRAM memory

    @returns    The 8-bit value retrieved at addr
*/
/**************************************************************************/
//write a byte to an addresss in FRAM.
// Supports writing 'numBytes' number of bytes to FRAM if pointing to an 
// array (or raw writing of structs). Returns true if an address overflow occurs.
// NOTE: Arduino libraries may impose a 32 byte limit on transactions so this 
// should be taken into consideration when using this fucntion
bool I2C_FRAM::readBlock(uint16_t addr, uint8_t * bytes, uint8_t numBytes){
	uint8_t i =0;
	if( addr <= _addrLimit){
		// begin transmision and send address to start writing to
		Wire.beginTransmission(_address);
		Wire.write((addr >> 8) & 0xFF);
		Wire.write(addr & 0xFF);

		Wire.endTransmission(false);

		//delay(1);

		Wire.requestFrom(_address, numBytes);
		while(Wire.available()){
			bytes[i] = Wire.read();
			i++;
		}
		
	}
	if( numBytes == i && ((addr +numBytes) > _addrLimit) ){
		return true;
	}
	return false;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value from the specified FRAM address

    @params[in] _address
                The I2C address of the FRAM memory chip (1010+A2+A1+A0)
    @params[in] addr
                The 16-bit address to read from in FRAM memory

    @returns    The 8-bit value retrieved at addr
*/
/**************************************************************************/
// reads a Page from FRAM and returns a byte
uint8_t I2C_FRAM::readByte(uint16_t addr) {

	//uint8_t buffer =0;
	
	// should not try to read a page higher than maximum number of pages
	//if ( addr <= _addrLimit){

		// Send two address bytes to read from
		Wire.beginTransmission(_address);
		Wire.write((addr >> 8) & 0xFF);
		Wire.write(addr & 0xFF);

		Wire.endTransmission(false);

		//delay(1);

		Wire.requestFrom(_address, (uint8_t)1);
		return Wire.read();
	//}
	

	//return buffer; 
}





/**************************************************************************/
/*!
    @brief  Reads an 8 bit value from the next contiguous FRAM address.
			Allows overflow from last address to first address of FRAM

    @params[in] _address
                The I2C address of the FRAM memory chip (1010+A2+A1+A0)

    @returns    The 8-bit value retrieved at the next address from the last
				accessed address 
*/
/**************************************************************************/
uint8_t I2C_FRAM::readByte() {
	uint8_t buffer;
	Wire.requestFrom(_address, (uint8_t)1);
	buffer = Wire.read();
	return buffer;
}



/**************************************************************************/
/*!
    @brief  A wrapper for simple writing of single bytes to FRAM Memory
			
    @params[in] addr
                The 16-bit address to write to in FRAM memory		
	@params[in] byte
                The byte(s) to write to FRAM memory		

*/
/**************************************************************************/
void I2C_FRAM::writeByte(uint16_t addr, uint8_t value){
	
	writeBlock(addr, &value,1);
}





/**************************************************************************/
/*!
    @brief  Writes a series of 8 bit values starting at the specified FRAM 
			address.
			Only writes to the address if it is lower or equal to 
			_addrLimit which is the max block count of the MB85RC256V module.
			
    @params[in] _address
                The I2C address of the FRAM memory chip (1010+A2+A1+A0)
    @params[in] addr
                The 16-bit address to write to in FRAM memory		
	@params[out] byte
                The byte(s) to write to FRAM memory		
	@params[in] numBytes
                The number of bytes to write to FRAM memory	
					
	@returns    True if the memory address rolls over, data could be 
				potentially lost.
*/
/**************************************************************************/
//write a byte to an addresss in FRAM.
// Supports writing 'numBytes' number of bytes to FRAM if pointing to an 
// array (or raw writing of structs). Returns true if an address overflow occurs.
// NOTE: Arduino libraries may impose a 32 byte limit on transactions so this 
// should be taken into consideration when using this fucntion
bool I2C_FRAM::writeBlock(uint16_t addr, uint8_t bytes[], uint8_t numBytes){
	
	if( addr <= _addrLimit){
		// begin transmision and send address to start writing to
		Wire.beginTransmission(_address);
		Wire.write((addr >> 8) & 0xFF);
		Wire.write(addr & 0xFF);
		
		Wire.write(bytes, numBytes);
		//for(int i =0; i < numBytes; i++){
		//	Wire.write(bytes[i]);
			
		//}
		Wire.endTransmission();
		return true;
	}
	//if( numBytes != 1 && ((addr +numBytes) > _addrLimit) ){
	//	return true;
	//}
	return false;
}


/**************************************************************************/
/*!
    @brief  Writes a float value to the specified FRAM address

    @params[in] addr
                The 16-bit address to read from in FRAM memory
    @params[in] value
                The value to write to FRAM memory

*/
/**************************************************************************/
bool I2C_FRAM::writeFloat(uint16_t addr, float value) {
	uint8_t data[4];
	memcpy(data, &value, 4);
	return writeBlock(addr, data, 4);
}
/* void I2C_FRAM::writeFloat(uint16_t addr, float value) {
  uint8_t *tempPointer = (uint8_t*) &value;
  
  for (uint8_t i = 0; i < 4; i++) {
    writeByte(addr + i, tempPointer[i] );
	
  }

} */





/**************************************************************************/
/*!
    @brief  Writes a Double prescision float value to the specified FRAM address

    @params[in] addr
                The 16-bit address to read from in FRAM memory
    @params[in] value
                The value to write to FRAM memory

*/
/**************************************************************************/
bool I2C_FRAM::writeDouble(uint16_t addr, double value) {
	uint8_t data[8];
	memcpy(data, &value, 8);
	return writeBlock(addr, data, 8);
}
/* void I2C_FRAM::writeFloat(uint16_t addr, float value) {
  uint8_t *tempPointer = (uint8_t*) &value;
  
  for (uint8_t i = 0; i < 4; i++) {
    writeByte(addr + i, tempPointer[i] );
	
  }

} */


/**************************************************************************/
/*!
    @brief  Writes a uint16_t value to the specified FRAM address

    @params[in] addr
                The 16-bit address to read from in FRAM memory
    @params[in] value
                The value to write to FRAM memory

*/
/**************************************************************************/
bool I2C_FRAM::writeUInt16(uint16_t addr, uint16_t value) {
	uint8_t data[2];
	memcpy(data, &value, 2);
	return writeBlock(addr, data, 2);
}
// Method to write a 2 byte integer to the Fram given an Address
/* void FRAM::writeInt16(uint16_t addr, uint16_t value) {
  uint8_t *tempPointer = (uint8_t*) &value;
  for (uint8_t i = 0; i < 2; i++) {
    writeByte(addr + i, tempPointer[i] );
  }
} */

/**************************************************************************/
/*!
    @brief  Writes a uint16_t value to the specified FRAM address

    @params[in] addr
                The 16-bit address to read from in FRAM memory
    @params[in] value
                The value to write to FRAM memory

*/
/**************************************************************************/
bool I2C_FRAM::writeUInt32(uint16_t addr, uint32_t value) {
	uint8_t data[4];
	memcpy(data, &value, 4);
	return writeBlock(addr, data, 4);
}


/**************************************************************************/
/*!
    @brief  Reads a float value from the specified FRAM address

    @params[in] addr
                The 16-bit address to read from in FRAM memory

    @returns    The float value retrieved at addr
*/
/**************************************************************************/
// Method to read a 4 byte float from the Fram given an Address
float I2C_FRAM::readFloat(uint16_t addr){
	uint8_t temp[4];
	float value;
	readBlock(addr, temp, 4);
	//for (uint8_t i = 0; i < 4; i++) {
	//temp[0] = readByte(addr);
	//temp[1] = readByte(addr + 1);
	//temp[2] = readByte(addr + 2);
	//temp[3] = readByte(addr + 3);

	//}

	memcpy(&value, temp, 4);
	return value;
	//return *( (float *)temp );
}


/**************************************************************************/
/*!
    @brief  Reads a Double precision float value from the specified FRAM address

    @params[in] addr
                The 16-bit address to read from in FRAM memory

    @returns    The Double value retrieved at addr
*/
/**************************************************************************/
// Method to read a 4 byte float from the Fram given an Address
double I2C_FRAM::readDouble(uint16_t addr){
	uint8_t temp[8];
	float value;
	readBlock(addr, temp, 8);
	//for (uint8_t i = 0; i < 4; i++) {
	//temp[0] = readByte(addr);
	//temp[1] = readByte(addr + 1);
	//temp[2] = readByte(addr + 2);
	//temp[3] = readByte(addr + 3);

	//}

	memcpy(&value, temp, 8);
	return value;
	//return *( (float *)temp );
}

/**************************************************************************/
/*!
    @brief  Reads a uint16_t value from the specified FRAM address

    @params[in] addr
                The 16-bit address to read from in FRAM memory

    @returns    The uint16_t value retrieved at addr
*/
/**************************************************************************/
// Method to read a 2 byte integer from the Fram given an Address
uint16_t I2C_FRAM::readUInt16(uint16_t addr){
	uint8_t temp[2];
	uint16_t value;
	readBlock(addr, temp, 2);
	//temp[0] = readByte(addr);
	//temp[1] = readByte(addr+1);

	memcpy(&value, temp, 2);
	return value;
}



/**************************************************************************/
/*!
    @brief  Reads a uint32_t value from the specified FRAM address

    @params[in] addr
                The 16-bit address to read from in FRAM memory

    @returns    The uint16_t value retrieved at addr
*/
/**************************************************************************/
// Method to read a 4 byte integer from the Fram given an Address
uint32_t I2C_FRAM::readUInt32(uint16_t addr){
	uint8_t temp[4];
	uint32_t value;
	readBlock(addr, temp, 4);
	//temp[0] = readByte(addr);
	//temp[1] = readByte(addr+1);
	//temp[2] = readByte(addr+2);
	//temp[3] = readByte(addr+3);
	
	memcpy(&value, temp, 4);
	return value;
	//return copyByteArrayToInt32(temp);
	//return *( (uint32_t *)temp );
  
}


	
/**************************************************************************/
/*!
    @brief  Initiates a software reset sequence or command retry on the FRAM 
			module

*/
/**************************************************************************/
// function not finished/tested yet
void I2C_FRAM::sleep(){
	
	
/* 	Wire.beginTransmission(0xF8);
		Wire.write((addr >> 8) & 0xFF);
		Wire.write(_address);
		
		
		
	if ( !sercom->startTransmissionWIRE( 0xF8, WIRE_READ_FLAG ) )
	  {
		sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		return 2 ;  // Address error
	  }

	  // Send all buffer
	  
	// send address data
	if ( !sercom->sendDataMasterWIRE( _address << 1 ) )
	{
	  sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
	  return 3 ;  // Nack or error
	}
	  
	if ( !sercom->startTransmissionWIRE( 0x86, WIRE_READ_FLAG ) )
	{
		sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
		return 2 ;  // Address error
	}
	  
	if (stopBit)
	{
		sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
	}    */
	
}


/**************************************************************************/
/*!
    @brief  Initiates a software reset sequence or command retry on the FRAM 
			module

*/
/**************************************************************************/
// function not finished/tested yet
void I2C_FRAM::wake(){
	/* for(uint8_t i =0; i < 9; i++){
		Wire.beginTransmission(_address);
		Wire.write(0x01);
	} */
    // Wire.endTransmission();
	
}


/**************************************************************************/
/*!
    @brief  Initiates a software reset sequence or command retry on the FRAM 
			module

*/
/**************************************************************************/
// function not finished/tested yet
void I2C_FRAM::recover(){
	for(uint8_t i =0; i < 9; i++){
		Wire.beginTransmission(_address);
		Wire.write(0x01);
	}
    // Wire.endTransmission();
	
}


/**************************************************************************/
/*!
    @brief  Reads the Manufacturer ID and the Product ID frm the IC

    @params[out]  manufacturerID
                  The 12-bit manufacturer ID (Fujitsu = 0x00A)
    @params[out]  productID
                  The memory density (bytes 11..8) and proprietary
                  Product ID fields (bytes 7..0). Should be 0x510 for
                  the MB85RC256V.
*/
/**************************************************************************/
void I2C_FRAM::getDeviceID(uint16_t *manufacturerID, uint16_t *productID)
{
  uint8_t a[3] = { 0, 0, 0 };
  uint8_t results;
  
  Wire.beginTransmission(MB85RC_SLAVE_ID >> 1);
  Wire.write(_address << 1);
  results = Wire.endTransmission(false);

  Wire.requestFrom(MB85RC_SLAVE_ID >> 1, 3);
  a[0] = Wire.read();
  a[1] = Wire.read();
  a[2] = Wire.read();

  /* Shift values to separate manuf and prod IDs */
  /* See p.10 of http://www.fujitsu.com/downloads/MICRO/fsa/pdf/products/memory/fram/MB85RC256V-DS501-00017-3v0-E.pdf */
  *manufacturerID = (a[0] << 4) + (a[1]  >> 4);
  *productID = ((a[1] & 0x0F) << 8) + a[2];
}


