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

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
 
#include <Adafruit_GFX.h>

#include <SPI.h>
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
	

// LCD Dimensions
#define SHARPMEM_LCDWIDTH       (400)
#define SHARPMEM_LCDHEIGHT      (240) 
#define SPI_CLOCK				(4000000)

class Adafruit_SharpMem : public Adafruit_GFX {
 public:
  Adafruit_SharpMem(uint8_t ss);
  void begin(void);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  uint8_t getPixel(uint16_t x, uint16_t y);
  void clearDisplay();
  void refresh(void);

 private:
  uint8_t _ss, _clk, _mosi;
#ifdef __AVR__
  volatile uint8_t *dataport, *clkport;
  uint8_t _sharpmem_vcom, datapinmask, clkpinmask;
#endif
#if defined(__SAM3X8E__) || defined(ARDUINO_ARCH_SAMD)
    volatile RwReg *dataport, *clkport;
    uint32_t _sharpmem_vcom, datapinmask, clkpinmask;
#endif

  //void sendbyte(uint8_t data);
  //void sendbyteLSB(uint8_t data);
  uint8_t flipBitOrder(uint8_t data);

};
