/*********************************************************************
This is a library for our Monochrome OLEDs based on SSH1106 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution

Re Branch for SSH1106 by Rudi ARS Creative Bekasi 12-12-2020

*********************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
 #define WIRE_WRITE Wire.write
#else
 #include "WProgram.h"
  #define WIRE_WRITE Wire.send
#endif
/*
#ifdef __SAM3X8E__
 typedef volatile RwReg PortReg;
 typedef uint32_t PortMask;
#else
  typedef volatile uint8_t PortReg;
  typedef uint8_t PortMask;
#endif
*/
//typedef volatile RwReg PortReg;
// typedef uint32_t PortMask;
#include <SPI.h>
#include <Adafruit_GFX.h>

#define BLACK 0
#define WHITE 1
#define INVERSE 2

#define SSH1106_I2C_ADDRESS   0x3C	// 011110+SA0+RW - 0x3C or 0x3D
// Address for 128x32 is 0x3C
// Address for 128x64 is 0x3D (default) or 0x3C (if SA0 is grounded)

/*=========================================================================
    SSH1106 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
    Select the appropriate display below to create an appropriately
    sized framebuffer, etc.

    SSH1106_128_64  128x64 pixel display

    SSH1106_128_32  128x32 pixel display

    SSH1106_96_16

    -----------------------------------------------------------------------*/
   #define SSH1106_128_64
//   #define SSH1106_128_32
//   #define SSH1106_96_16
/*=========================================================================*/

#if defined SSH1106_128_64 && defined SSH1106_128_32
  #error "Only one SSH1106 display can be specified at once in SSH1106.h"
#endif
#if !defined SSH1106_128_64 && !defined SSH1106_128_32 && !defined SSH1106_96_16
  #error "At least one SSH1106 display must be specified in SSH1106.h"
#endif

#if defined SSH1106_128_64
  #define SSH1106_LCDWIDTH                  128
  #define SSH1106_LCDHEIGHT                 64
#endif
#if defined SSH1106_128_32
  #define SSH1106_LCDWIDTH                  128
  #define SSH1106_LCDHEIGHT                 32
#endif
#if defined SSH1106_96_16
  #define SSH1106_LCDWIDTH                  96
  #define SSH1106_LCDHEIGHT                 16
#endif

#define SSH1106_SETCONTRAST 0x81
#define SSH1106_DISPLAYALLON_RESUME 0xA4
#define SSH1106_DISPLAYALLON 0xA5
#define SSH1106_NORMALDISPLAY 0xA6
#define SSH1106_INVERTDISPLAY 0xA7
#define SSH1106_DISPLAYOFF 0xAE
#define SSH1106_DISPLAYON 0xAF

#define SSH1106_SETDISPLAYOFFSET 0xD3
#define SSH1106_SETCOMPINS 0xDA

#define SSH1106_SETVCOMDETECT 0xDB

#define SSH1106_SETDISPLAYCLOCKDIV 0xD5
#define SSH1106_SETPRECHARGE 0xD9

#define SSH1106_SETMULTIPLEX 0xA8

#define SSH1106_SETLOWCOLUMN 0x00
#define SSH1106_SETHIGHCOLUMN 0x10

#define SSH1106_SETSTARTLINE 0x40

#define SSH1106_MEMORYMODE 0x20
#define SSH1106_COLUMNADDR 0x21
#define SSH1106_PAGEADDR   0x22

#define SSH1106_COMSCANINC 0xC0
#define SSH1106_COMSCANDEC 0xC8

#define SSH1106_SEGREMAP 0xA0

#define SSH1106_CHARGEPUMP 0x8D

#define SSH1106_EXTERNALVCC 0x1
#define SSH1106_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSH1106_ACTIVATE_SCROLL 0x2F
#define SSH1106_DEACTIVATE_SCROLL 0x2E
#define SSH1106_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSH1106_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSH1106_LEFT_HORIZONTAL_SCROLL 0x27
#define SSH1106_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSH1106_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

class Adafruit_SSH1106 : public Adafruit_GFX {
 public:
  Adafruit_SSH1106(int8_t SID, int8_t SCLK, int8_t DC, int8_t RST, int8_t CS);
  Adafruit_SSH1106(int8_t DC, int8_t RST, int8_t CS);
  Adafruit_SSH1106(int8_t RST);

  void begin(uint8_t switchvcc = SSH1106_SWITCHCAPVCC, uint8_t i2caddr = SSH1106_I2C_ADDRESS, bool reset=true);
  void SSH1106_command(uint8_t c);
  void SSH1106_data(uint8_t c);

  void clearDisplay(void);
  void invertDisplay(uint8_t i);
  void display();

  /*void startscrollright(uint8_t start, uint8_t stop);
  void startscrollleft(uint8_t start, uint8_t stop);

  void startscrolldiagright(uint8_t start, uint8_t stop);
  void startscrolldiagleft(uint8_t start, uint8_t stop);
  void stopscroll(void);*/

  void dim(boolean dim);

  void drawPixel(int16_t x, int16_t y, uint16_t color);

  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

 private:
  int8_t _i2caddr, _vccstate, sid, sclk, dc, rst, cs;
  void fastSPIwrite(uint8_t c);

  boolean hwSPI;
volatile uint32 *mosiport, *clkport, *csport, *dcport;
   uint32_t  mosipinmask, clkpinmask, cspinmask, dcpinmask;

  inline void drawFastVLineInternal(int16_t x, int16_t y, int16_t h, uint16_t color) __attribute__((always_inline));
  inline void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) __attribute__((always_inline));

};
