/*********************************************************************
This is an Arduino library for our Monochrome SHARP Memory Displays

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1393

These displays use SPI to communicate, 3 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include "Adafruit_SharpMemSPI1.h"


#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif
#ifndef _swap_uint16_t
#define _swap_uint16_t(a, b) { uint16_t t = a; a = b; b = t; }
#endif

/**************************************************************************
    Sharp Memory Display Connector
    -----------------------------------------------------------------------
    Pin   Function        Notes
    ===   ==============  ===============================
      1   VIN             3.3-5.0V (into LDO supply)
      2   3V3             3.3V out
      3   GND
      4   SCLK            Serial Clock
      5   MOSI            Serial Data Input
      6   CS              Serial Chip Select
      9   EXTMODE         COM Inversion Select (Low = SW clock/serial)
      7   EXTCOMIN        External COM Inversion Signal
      8   DISP            Display On(High)/Off(Low)

 **************************************************************************/

#define SHARPMEM_BIT_WRITECMD   (0x80)
#define SHARPMEM_BIT_VCOM       (0x40)
#define SHARPMEM_BIT_CLEAR      (0x20)
#define TOGGLE_VCOM             do { _sharpmem_vcom = _sharpmem_vcom ? 0x00 : SHARPMEM_BIT_VCOM; } while(0);


byte sharpmem_buffer[(SHARPMEM_LCDWIDTH * SHARPMEM_LCDHEIGHT) / 8];
SPISettings SharpSPISettings = SPISettings(SPI_CLOCK, LSBFIRST, SPI_MODE0);

/* ************* */
/* CONSTRUCTORS  */
/* ************* */

Adafruit_SharpMem::Adafruit_SharpMem( uint8_t ss) :



Adafruit_GFX(SHARPMEM_LCDWIDTH, SHARPMEM_LCDHEIGHT) {
  //_clk = clk;
  //_mosi = mosi;
  _ss = ss;
  //_led_pin = 13;

  // Set pin state before direction to make sure they start this way (no glitching)
  digitalWrite(_ss, LOW);  
  //digitalWrite(_clk, LOW);  
  //digitalWrite(_mosi, HIGH);  
  
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, LOW);
  //pinMode(_clk, OUTPUT);
  //pinMode(_mosi, OUTPUT);

  
  digitalWrite(41, HIGH); 
  
  //clkport     = portOutputRegister(digitalPinToPort(_clk));
  //clkpinmask  = digitalPinToBitMask(_clk);
  //dataport    = portOutputRegister(digitalPinToPort(_mosi));
  //datapinmask = digitalPinToBitMask(_mosi);
  
  
  // Set the vcom bit to a defined state
  _sharpmem_vcom = SHARPMEM_BIT_VCOM;

}

void Adafruit_SharpMem::begin() {
  setRotation(0); //2
  SPI1.begin();
  digitalWrite(41, LOW);
}

/* *************** */
/* PRIVATE METHODS */
/* *************** */

 



// /**************************************************************************/
// /*!
    // @brief  Sends a single byte in pseudo-SPI.
// */
// /**************************************************************************/
// void Adafruit_SharpMem::sendbyte(uint8_t data) 
// {
  // uint8_t i = 0;

  // // LCD expects LSB first
  // for (i=0; i<8; i++) 
  // { 
    // // Make sure clock starts low
    // //digitalWrite(_clk, LOW);
    // *clkport &= ~clkpinmask;
    // if (data & 0x80) 
      // //digitalWrite(_mosi, HIGH);
      // *dataport |=  datapinmask;
    // else 
      // //digitalWrite(_mosi, LOW);
      // *dataport &= ~datapinmask;

    // // Clock is active high
    // //digitalWrite(_clk, HIGH);
    // *clkport |=  clkpinmask;
    // data <<= 1; 
  // }
  // // Make sure clock ends low
  // //digitalWrite(_clk, LOW);
  // *clkport &= ~clkpinmask;
// }



// void Adafruit_SharpMem::sendbyteLSB(uint8_t data) 
// {
  // uint8_t i = 0;

  // // LCD expects LSB first
  // for (i=0; i<8; i++) 
  // { 
    // // Make sure clock starts low
    // //digitalWrite(_clk, LOW);
    // *clkport &= ~clkpinmask;
    // if (data & 0x01) 
      // //digitalWrite(_mosi, HIGH);
      // *dataport |=  datapinmask;
    // else 
      // //digitalWrite(_mosi, LOW);
      // *dataport &= ~datapinmask;
    // // Clock is active high
    // //digitalWrite(_clk, HIGH);
    // *clkport |=  clkpinmask;
    // data >>= 1; 
  // }
  // // Make sure clock ends low
  // //digitalWrite(_clk, LOW);
  // *clkport &= ~clkpinmask;
// }
/* ************** */
/* PUBLIC METHODS */
/* ************** */

// 1<<n is a costly operation on AVR -- table usu. smaller & faster
static const uint8_t PROGMEM
  set[] = {  1,  2,  4,  8,  16,  32,  64,  128 },
  clr[] = { ~1, ~2, ~4, ~8, ~16, ~32, ~64, ~128 };

/**************************************************************************/
/*! 
    @brief Draws a single pixel in image buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)
*/
/**************************************************************************/
void Adafruit_SharpMem::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  if((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  switch(rotation) {
   case 1:
    _swap_int16_t(x, y);
    x = WIDTH  - 1 - x;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    _swap_int16_t(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  if(color) {
    sharpmem_buffer[(y*SHARPMEM_LCDWIDTH + x) / 8] |=
      pgm_read_byte(&set[x & 7]);
  } else {
    sharpmem_buffer[(y*SHARPMEM_LCDWIDTH + x) / 8] &=
      pgm_read_byte(&clr[x & 7]);
  }
}

/**************************************************************************/
/*! 
    @brief Gets the value (1 or 0) of the specified pixel from the buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)

    @return     1 if the pixel is enabled, 0 if disabled
*/
/**************************************************************************/
uint8_t Adafruit_SharpMem::getPixel(uint16_t x, uint16_t y)
{
  if((x >= _width) || (y >= _height)) return 0; // <0 test not needed, unsigned

  switch(rotation) {
   case 1:
    _swap_uint16_t(x, y);
    x = WIDTH  - 1 - x;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    _swap_uint16_t(x, y);
    y = HEIGHT - 1 - y;
    break;
  }

  return sharpmem_buffer[(y*SHARPMEM_LCDWIDTH + x) / 8] &
    pgm_read_byte(&set[x & 7]) ? 1 : 0;
}

/**************************************************************************/
/*! 
    @brief Clears the screen
*/
/**************************************************************************/
void Adafruit_SharpMem::clearDisplay() 
{
  memset(sharpmem_buffer, 0xff, (SHARPMEM_LCDWIDTH * SHARPMEM_LCDHEIGHT) / 8);
  // Send the clear screen command rather than doing a HW refresh (quicker)
	
	SPI1.beginTransaction(SharpSPISettings);
	digitalWrite(_ss, HIGH); 
	SPI1.transfer(flipBitOrder(_sharpmem_vcom | SHARPMEM_BIT_CLEAR));
	SPI1.transfer(0x00);
	//#if defined (__SAMD21G18A__)
		//while(SPI_BSY);
		while (SERCOM2->SPI.INTFLAG.bit.TXC  == 0){
			//wait till transfer is complete
		}
	//#endif

	SPI1.endTransaction();
    TOGGLE_VCOM;
	digitalWrite(_ss, LOW);
	SPI1.endTransaction();
}

/**************************************************************************/
/*! 
    @brief Renders the contents of the pixel buffer on the LCD
*/
/**************************************************************************/
void Adafruit_SharpMem::refresh(void) 
{
  uint16_t i, totalbytes, currentline, oldline;  
  totalbytes = (SHARPMEM_LCDWIDTH * SHARPMEM_LCDHEIGHT) / 8;

  // Send the write command
  SPI1.beginTransaction(SharpSPISettings);

  digitalWrite(_ss, HIGH); 
  SPI1.transfer( flipBitOrder( SHARPMEM_BIT_WRITECMD | _sharpmem_vcom));
  TOGGLE_VCOM;

  // Send the address for line 1
  oldline = currentline = 1;
  SPI1.transfer(currentline);

  // Send image buffer
  for (i=0; i<totalbytes; i++)
  {
    SPI1.transfer(sharpmem_buffer[i]);
    currentline = ((i+1)/(SHARPMEM_LCDWIDTH/8)) + 1;
    if(currentline != oldline)
    {
      // Send end of line and address bytes
		SPI1.transfer(0x00);
		
		//#if defined (__SAMD21G18A__)
			//while(SPI_BSY);
			while (SERCOM2->SPI.INTFLAG.bit.TXC  == 0){
				//wait till transfer is complete
			}
		//#endif
	  
		if (currentline <= SHARPMEM_LCDHEIGHT)
		{
			SPI1.transfer(currentline);
		}
		oldline = currentline;
    }
  }

  // Send another trailing 16 bits for the last line
  SPI1.transfer(0x00);
  SPI1.transfer(0x00);
  //#if defined (__SAMD21G18A__)
	  //while(SPI_BSY);
	  while (SERCOM2->SPI.INTFLAG.bit.TXC  == 0){
		//wait till transfer is complete
	  }
  //#endif
  
  //SPI1.transfer(0x00);
  digitalWrite(_ss,LOW); 
  SPI1.endTransaction();
}


uint8_t Adafruit_SharpMem::flipBitOrder(uint8_t data){
	uint8_t i = 0;
	uint8_t dataFlipped = 0;
	//Serial.println(data,HEX);
	//Serial.println("-");
	for (i=0; i<8; i++) 
	{ 
		dataFlipped = dataFlipped << 1;
		dataFlipped |= ((data >> i) & 0x01);
		
		//Serial.print(((data >> i) & 0x01),HEX);
		//Serial.print("  ");
		//Serial.println(dataFlipped,HEX);
		
	}
	//Serial.println("-");
	//Serial.println(dataFlipped,HEX);
  // Make sure clock ends low
  //digitalWrite(_clk, LOW);
  return dataFlipped;
	
}