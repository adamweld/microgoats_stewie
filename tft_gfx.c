/* Code rewritten from Adafruit Arduino library for the TFT
 *  by Syed Tahmid Mahbub
 * The TFT itself is Adafruit product 1480
 * Included below is the text header from the original Adafruit library
 *  followed by the code
 */

/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "tft_gfx.h"
#include "glcdfont.c"
#include "tft_master.h"

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

void tft_drawCircle(short x0, short y0, short r, unsigned short color) {
/* Draw a circle outline with center (x0,y0) and radius r, with given color
 * Parameters:
 *      x0: x-coordinate of center of circle. The top-left of the screen
 *          has x-coordinate 0 and increases to the right
 *      y0: y-coordinate of center of circle. The top-left of the screen
 *          has y-coordinate 0 and increases to the bottom
 *      r:  radius of circle
 *      color: 16-bit color value for the circle. Note that the circle
 *          isn't filled. So, this is the color of the outline of the circle
 * Returns: Nothing
 */
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  tft_drawPixel(x0  , y0+r, color);
  tft_drawPixel(x0  , y0-r, color);
  tft_drawPixel(x0+r, y0  , color);
  tft_drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    tft_drawPixel(x0 + x, y0 + y, color);
    tft_drawPixel(x0 - x, y0 + y, color);
    tft_drawPixel(x0 + x, y0 - y, color);
    tft_drawPixel(x0 - x, y0 - y, color);
    tft_drawPixel(x0 + y, y0 + x, color);
    tft_drawPixel(x0 - y, y0 + x, color);
    tft_drawPixel(x0 + y, y0 - x, color);
    tft_drawPixel(x0 - y, y0 - x, color);
  }
}

void tft_drawCircleHelper( short x0, short y0,
               short r, unsigned char cornername, unsigned short color) {
// Helper function for drawing circles and circular objects
  short f     = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x     = 0;
  short y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      tft_drawPixel(x0 + x, y0 + y, color);
      tft_drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      tft_drawPixel(x0 + x, y0 - y, color);
      tft_drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      tft_drawPixel(x0 - y, y0 + x, color);
      tft_drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      tft_drawPixel(x0 - y, y0 - x, color);
      tft_drawPixel(x0 - x, y0 - y, color);
    }
  }
}

void tft_fillCircle(short x0, short y0, short r, unsigned short color) {
/* Draw a filled circle with center (x0,y0) and radius r, with given color
 * Parameters:
 *      x0: x-coordinate of center of circle. The top-left of the screen
 *          has x-coordinate 0 and increases to the right
 *      y0: y-coordinate of center of circle. The top-left of the screen
 *          has y-coordinate 0 and increases to the bottom
 *      r:  radius of circle
 *      color: 16-bit color value for the circle
 * Returns: Nothing
 */
  tft_drawFastVLine(x0, y0-r, 2*r+1, color);
  tft_fillCircleHelper(x0, y0, r, 3, 0, color);
}

void tft_fillCircleHelper(short x0, short y0, short r,
    unsigned char cornername, short delta, unsigned short color) {
// Helper function for drawing filled circles
  short f     = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x     = 0;
  short y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      tft_drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      tft_drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      tft_drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      tft_drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}

// Bresenham's algorithm - thx wikpedia
void tft_drawLine(short x0, short y0,
			    short x1, short y1,
			    unsigned short color) {
/* Draw a straight line from (x0,y0) to (x1,y1) with given color
 * Parameters:
 *      x0: x-coordinate of starting point of line. The x-coordinate of
 *          the top-left of the screen is 0. It increases to the right.
 *      y0: y-coordinate of starting point of line. The y-coordinate of
 *          the top-left of the screen is 0. It increases to the bottom.
 *      x1: x-coordinate of ending point of line. The x-coordinate of
 *          the top-left of the screen is 0. It increases to the right.
 *      y1: y-coordinate of ending point of line. The y-coordinate of
 *          the top-left of the screen is 0. It increases to the bottom.
 *      color: 16-bit color value for line
 */
  short steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  short dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  short err = dx / 2;
  short ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      tft_drawPixel(y0, x0, color);
    } else {
      tft_drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Draw a rectangle
void tft_drawRect(short x, short y, short w, short h, unsigned short color) {
/* Draw a rectangle outline with top left vertex (x,y), width w
 * and height h at given color
 * Parameters:
 *      x:  x-coordinate of top-left vertex. The x-coordinate of
 *          the top-left of the screen is 0. It increases to the right.
 *      y:  y-coordinate of top-left vertex. The y-coordinate of
 *          the top-left of the screen is 0. It increases to the bottom.
 *      w:  width of the rectangle
 *      h:  height of the rectangle
 *      color:  16-bit color of the rectangle outline
 * Returns: Nothing
 */
  tft_drawFastHLine(x, y, w, color);
  tft_drawFastHLine(x, y+h-1, w, color);
  tft_drawFastVLine(x, y, h, color);
  tft_drawFastVLine(x+w-1, y, h, color);
}

// Draw a rounded rectangle
void tft_drawRoundRect(short x, short y, short w, short h,
        short r, unsigned short color) {
/* Draw a rounded rectangle outline with top left vertex (x,y), width w,
 * height h and radius of curvature r at given color
 * Parameters:
 *      x:  x-coordinate of top-left vertex. The x-coordinate of
 *          the top-left of the screen is 0. It increases to the right.
 *      y:  y-coordinate of top-left vertex. The y-coordinate of
 *          the top-left of the screen is 0. It increases to the bottom.
 *      w:  width of the rectangle
 *      h:  height of the rectangle
 *      color:  16-bit color of the rectangle outline
 * Returns: Nothing
 */
  // smarter version
  tft_drawFastHLine(x+r  , y    , w-2*r, color); // Top
  tft_drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  tft_drawFastVLine(x    , y+r  , h-2*r, color); // Left
  tft_drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  tft_drawCircleHelper(x+r    , y+r    , r, 1, color);
  tft_drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  tft_drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  tft_drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// Fill a rounded rectangle
void tft_fillRoundRect(short x, short y, short w,
				 short h, short r, unsigned short color) {
  // smarter version
  tft_fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  tft_fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  tft_fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

// Draw a triangle
void tft_drawTriangle(short x0, short y0,
				short x1, short y1,
				short x2, short y2, unsigned short color) {
/* Draw a triangle outline with vertices (x0,y0),(x1,y1),(x2,y2) with given color
 * Parameters:
 *      x0: x-coordinate of one of the 3 vertices
 *      y0: y-coordinate of one of the 3 vertices
 *      x1: x-coordinate of one of the 3 vertices
 *      y1: y-coordinate of one of the 3 vertices
 *      x2: x-coordinate of one of the 3 vertices
 *      y2: y-coordinate of one of the 3 vertices
 *      color: 16-bit color value for outline
 * Returns: Nothing
 */
  tft_drawLine(x0, y0, x1, y1, color);
  tft_drawLine(x1, y1, x2, y2, color);
  tft_drawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
void tft_fillTriangle ( short x0, short y0,
				  short x1, short y1,
				  short x2, short y2, unsigned short color) {
/* Draw a filled triangle with vertices (x0,y0),(x1,y1),(x2,y2) with given color
 * Parameters:
 *      x0: x-coordinate of one of the 3 vertices
 *      y0: y-coordinate of one of the 3 vertices
 *      x1: x-coordinate of one of the 3 vertices
 *      y1: y-coordinate of one of the 3 vertices
 *      x2: x-coordinate of one of the 3 vertices
 *      y2: y-coordinate of one of the 3 vertices
 *      color: 16-bit color value
 * Returns: Nothing
 */
  short a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    tft_drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  short
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    tft_drawFastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    tft_drawFastHLine(a, y, b-a+1, color);
  }
}

void tft_drawBitmap(short x, short y,
			      const unsigned char *bitmap, short w, short h,
			      unsigned short color) {

  short i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
	tft_drawPixel(x+i, y+j, color);
      }
    }
  }
}

void tft_write(unsigned char c){
  if (c == '\n') {
    cursor_y += textsize*8;
    cursor_x  = 0;
  } else if (c == '\r') {
    // skip em
  } else if (c == '\t'){
      int new_x = cursor_x + tabspace;
      if (new_x < _width){
          cursor_x = new_x;
      }
  } else {
    tft_drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
    cursor_x += textsize*6;
    if (wrap && (cursor_x > (_width - textsize*6))) {
      cursor_y += textsize*8;
      cursor_x = 0;
    }
  }
}

inline void tft_writeString(char* str){
/* Print text onto screen
 * Call tft_setCursor(), tft_setTextColor(), tft_setTextSize()
 *  as necessary before printing
 */
    while (*str){
        tft_write(*str++);
    }
}

// Draw a character
void tft_drawChar(short x, short y, unsigned char c, unsigned short color, unsigned short bg, unsigned char size) {
    char i, j;
  if((x >= _width)            || // Clip right
     (y >= _height)           || // Clip bottom
     ((x + 6 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;

  for (i=0; i<6; i++ ) {
    unsigned char line;
    if (i == 5)
      line = 0x0;
    else
      line = pgm_read_byte(font+(c*5)+i);
    for ( j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) // default size
          tft_drawPixel(x+i, y+j, color);
        else {  // big size
          tft_fillRect(x+(i*size), y+(j*size), size, size, color);
        }
      } else if (bg != color) {
        if (size == 1) // default size
          tft_drawPixel(x+i, y+j, bg);
        else {  // big size
          tft_fillRect(x+i*size, y+j*size, size, size, bg);
        }
      }
      line >>= 1;
    }
  }
}

inline void tft_setCursor(short x, short y) {
/* Set cursor for text to be printed
 * Parameters:
 *      x = x-coordinate of top-left of text starting
 *      y = y-coordinate of top-left of text starting
 * Returns: Nothing
 */
  cursor_x = x;
  cursor_y = y;
}

inline void tft_setTextSize(unsigned char s) {
/*Set size of text to be displayed
 * Parameters:
 *      s = text size (1 being smallest)
 * Returns: nothing
 */
  textsize = (s > 0) ? s : 1;
}

inline void tft_setTextColor(unsigned short c) {
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

inline void tft_setTextColor2(unsigned short c, unsigned short b) {
/* Set color of text to be displayed
 * Parameters:
 *      c = 16-bit color of text
 *      b = 16-bit color of text background
 */
  textcolor   = c;
  textbgcolor = b;
}

inline void tft_setTextWrap(char w) {
  wrap = w;
}

inline unsigned char tft_getRotation(void) {
/* Returns current roation of screen
 *          0 = no rotation (0 degree rotation)
 *          1 = rotate 90 degree clockwise
 *          2 = rotate 180 degree
 *          3 = rotate 90 degree anticlockwise
 */
  return rotation;
}

void tft_gfx_setRotation(unsigned char x) {
/* Set display rotation in 90 degree steps
 * Parameters:
 *      x: dictate direction of rotation
 *          0 = no rotation (0 degree rotation)
 *          1 = rotate 90 degree clockwise
 *          2 = rotate 180 degree
 *          3 = rotate 90 degree anticlockwise
 * Returns: Nothing
 */
  rotation = (x & 3);
  switch(rotation) {
   case 0:
   case 2:
    _width  = ILI9340_TFTWIDTH;
    _height = ILI9340_TFTHEIGHT;
    break;
   case 1:
   case 3:
    _width  = ILI9340_TFTHEIGHT;;
    _height = ILI9340_TFTWIDTH;
    break;
  }
}

// Return the size of the display (per current rotation)
inline short tft_width(void) {
  return _width;
}

inline short tft_height(void) {
  return _height;
}