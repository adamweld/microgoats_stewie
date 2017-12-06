#include "plib.h"

#define swap(a, b) { short t = a; a = b; b = t; }

unsigned short cursor_y, cursor_x, textsize, textcolor, textbgcolor, wrap, rotation;

void tft_drawLine(short x0, short y0, short x1, short y1, unsigned short color);
void tft_drawRect(short x, short y, short w, short h, unsigned short color);

void tft_drawCircle(short x0, short y0, short r, unsigned short color);
void tft_drawCircleHelper(short x0, short y0, short r, unsigned char cornername,
      unsigned short color);
void tft_fillCircle(short x0, short y0, short r, unsigned short color);
void tft_fillCircleHelper(short x0, short y0, short r, unsigned char cornername,
      short delta, unsigned short color);
void tft_drawTriangle(short x0, short y0, short x1, short y1,
      short x2, short y2, unsigned short color);
void tft_fillTriangle(short x0, short y0, short x1, short y1,
      short x2, short y2, unsigned short color);
void tft_drawRoundRect(short x0, short y0, short w, short h,
      short radius, unsigned short color);
void tft_fillRoundRect(short x0, short y0, short w, short h, short radius, unsigned short color);
void tft_drawBitmap(short x, short y, const unsigned char *bitmap, short w, short h, unsigned short color);
void tft_drawChar(short x, short y, unsigned char c, unsigned short color, unsigned short bg, unsigned char size);
void tft_setCursor(short x, short y);
void tft_setTextColor(unsigned short c);
void tft_setTextColor2(unsigned short c, unsigned short bg);
void tft_setTextSize(unsigned char s);
void tft_setTextWrap(char w);
void tft_gfx_setRotation(unsigned char r);
void tft_write(unsigned char c);
void tft_writeString(char* str);    // This is the function to use to write a string