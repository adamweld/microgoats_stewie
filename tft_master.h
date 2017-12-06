/* Code rewritten from Adafruit Arduino library for the TFT
 *  by Syed Tahmid Mahbub
 * The TFT itself is Adafruit product 1480
 * Included below is the text header from the original Adafruit library
 *  followed by the code
 */

/***************************************************
  This is an Arduino Library for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "plib.h"
//#include <Adafruit_GFX.h>

#define _dc         LATBbits.LATB0
#define TRIS_dc     TRISBbits.TRISB0
#define _dc_high()  {LATBSET = 1;}
#define _dc_low()   {LATBCLR = 1;}

#define _cs         LATBbits.LATB1
#define TRIS_cs     TRISBbits.TRISB1
#define _cs_high()  {LATBSET = 2;}
#define _cs_low()   {LATBCLR = 2;}

#define _rst        LATBbits.LATB2
#define TRIS_rst    TRISBbits.TRISB2
#define _rst_high() {LATBSET = 4;}
#define _rst_low()  {LATBCLR = 4;}

#define ILI9340_TFTWIDTH  240
#define ILI9340_TFTHEIGHT 320

#define ILI9340_NOP     0x00
#define ILI9340_SWRESET 0x01
#define ILI9340_RDDID   0x04
#define ILI9340_RDDST   0x09

#define ILI9340_SLPIN   0x10
#define ILI9340_SLPOUT  0x11
#define ILI9340_PTLON   0x12
#define ILI9340_NORON   0x13

#define ILI9340_RDMODE  0x0A
#define ILI9340_RDMADCTL  0x0B
#define ILI9340_RDPIXFMT  0x0C
#define ILI9340_RDIMGFMT  0x0A
#define ILI9340_RDSELFDIAG  0x0F

#define ILI9340_INVOFF  0x20
#define ILI9340_INVON   0x21
#define ILI9340_GAMMASET 0x26
#define ILI9340_DISPOFF 0x28
#define ILI9340_DISPON  0x29

#define ILI9340_CASET   0x2A
#define ILI9340_PASET   0x2B
#define ILI9340_RAMWR   0x2C
#define ILI9340_RAMRD   0x2E

#define ILI9340_PTLAR   0x30
#define ILI9340_MADCTL  0x36


#define ILI9340_MADCTL_MY  0x80
#define ILI9340_MADCTL_MX  0x40
#define ILI9340_MADCTL_MV  0x20
#define ILI9340_MADCTL_ML  0x10
#define ILI9340_MADCTL_RGB 0x00
#define ILI9340_MADCTL_BGR 0x08
#define ILI9340_MADCTL_MH  0x04

#define ILI9340_PIXFMT  0x3A

#define ILI9340_FRMCTR1 0xB1
#define ILI9340_FRMCTR2 0xB2
#define ILI9340_FRMCTR3 0xB3
#define ILI9340_INVCTR  0xB4
#define ILI9340_DFUNCTR 0xB6

#define ILI9340_PWCTR1  0xC0
#define ILI9340_PWCTR2  0xC1
#define ILI9340_PWCTR3  0xC2
#define ILI9340_PWCTR4  0xC3
#define ILI9340_PWCTR5  0xC4
#define ILI9340_VMCTR1  0xC5
#define ILI9340_VMCTR2  0xC7

#define ILI9340_RDID1   0xDA
#define ILI9340_RDID2   0xDB
#define ILI9340_RDID3   0xDC
#define ILI9340_RDID4   0xDD

#define ILI9340_GMCTRP1 0xE0
#define ILI9340_GMCTRN1 0xE1
/*
#define ILI9340_PWCTR6  0xFC

*/

// Color definitions
#define	ILI9340_BLACK   0x0000
#define	ILI9340_BLUE    0x001F
#define	ILI9340_RED     0xF800
#define	ILI9340_GREEN   0x07E0
#define ILI9340_CYAN    0x07FF
#define ILI9340_MAGENTA 0xF81F
#define ILI9340_YELLOW  0xFFE0
#define ILI9340_WHITE   0xFFFF

#define PBCLK 40000000 // peripheral bus clock
#define SPI_freq    20000000

#define tabspace 4 // number of spaces for a tab

#define dTime_ms PBCLK/2000
#define dTime_us PBCLK/2000000

void tft_init_hw(void);
void tft_spiwrite(unsigned char c);
void tft_spiwrite8(unsigned char c);
void tft_spiwrite16(unsigned short c);
void tft_writecommand(unsigned char c);
void tft_writecommand16(unsigned short c);
void tft_writedata(unsigned char c);
void tft_writedata16(unsigned short c);
void tft_commandList(unsigned char *addr);
void tft_begin(void);
void tft_setAddrWindow(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1);
void tft_pushColor(unsigned short color);
void tft_drawPixel(short x, short y, unsigned short color);
void tft_drawFastVLine(short x, short y, short h, unsigned short color);
void tft_drawFastHLine(short x, short y, short w, unsigned short color);
void tft_fillScreen(unsigned short color);
void tft_fillRect(short x, short y, short w, short h, unsigned short color);
unsigned short tft_Color565(unsigned char r, unsigned char g, unsigned char b);
void tft_setRotation(unsigned char m);
unsigned char tft_spiread(void);
unsigned char tft_readdata(void);
unsigned char tft_readcommand8(unsigned char c);

unsigned short _width, _height;

void delay_ms(unsigned long);
void delay_us(unsigned long);