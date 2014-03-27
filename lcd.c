/*
$Id:$

PCD8544 LCD library!

Copyright (C) 2010 Limor Fried, Adafruit Industries

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <string.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_timer.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>
#include <inc/hw_ssi.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/rom.h>
#include <driverlib/ssi.h>

#include "config.h"

#include "lcd.h"
#include "glcdfont.h"
#include "tasks.h"

#ifdef ENABLE_LCD

#define LCD_RST_PORT	GPIO_PORTB_BASE
#define LCD_RST_PIN		(1 << 2)

#define LCD_DC_PORT		GPIO_PORTF_BASE
#define LCD_DC_PIN		(1 << 4)

static uint8_t cursor_x, cursor_y, textsize, textcolor, contrast;

static void my_setpixel(uint8_t x, uint8_t y, uint8_t color);
static void updateBoundingBox(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax);
static void setContrast(char val);
static void write(uint8_t c);

static void command(uint8_t c);
static void data(uint8_t c);

uint8_t is_reversed = 0;

// the memory buffer for the LCD
uint8_t pcd8544_buffer[LCDWIDTH * LCDHEIGHT / 8] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xF8, 0xF8, 0x18, 0x18, 0x18, 0x38, 0xF0, 0xF0, 0xC0,   // 0x0010 (16) pixels
		0x00, 0x00, 0xC0, 0xF0, 0xF0, 0x38, 0x18, 0x18, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x0020 (32) pixels
		0x00, 0x00, 0xF8, 0xF8, 0xF8, 0xC0, 0xE0, 0x70, 0x38, 0x18, 0x08, 0x00, 0x00, 0x80, 0xC0, 0x70,   // 0x0030 (48) pixels
		0x38, 0xF8, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF8, 0x18, 0x18, 0x18, 0xF8, 0xF0, 0xE0,   // 0x0040 (64) pixels
		0x00, 0xF8, 0xF8, 0xF8, 0x00, 0x00, 0xF8, 0xF8, 0xF8, 0x00, 0x00, 0xF8, 0xF8, 0xF8, 0x00, 0x00,   // 0x0050 (80) pixels
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x18, 0x18, 0x18,   // 0x0060 (96) pixels
		0x1C, 0x0F, 0x0F, 0x03, 0x00, 0x00, 0x03, 0x0F, 0x0F, 0x1C, 0x18, 0x18, 0x18, 0x18, 0x0C, 0x00,   // 0x0070 (112) pixels
		0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x03, 0x07, 0x07, 0x0E, 0x1C, 0x18, 0x10,   // 0x0080 (128) pixels
		0x00, 0x07, 0x07, 0x06, 0x06, 0x1F, 0x1F, 0x1F, 0x06, 0x00, 0x00, 0x07, 0x0F, 0x1F, 0x18, 0x18,   // 0x0090 (144) pixels
		0x18, 0x1F, 0x0F, 0x07, 0x00, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x1F,   // 0x00A0 (160) pixels
		0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x00B0 (176) pixels
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x00C0 (192) pixels
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x00D0 (208) pixels
		0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x40, 0x80, 0xFC, 0x80, 0x40, 0x20, 0x10, 0x00, 0x00, 0x00,   // 0x00E0 (224) pixels
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x00F0 (240) pixels
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x0100 (256) pixels
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,   // 0x0110 (272) pixels
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,   // 0x0120 (288) pixels
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x11, 0x09, 0x05, 0x03, 0x7F, 0x03, 0x05, 0x09,   // 0x0130 (304) pixels
		0x11, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x0140 (320) pixels
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x0150 (336) pixels
		0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xC0, 0x40, 0x40, 0x40, 0x80, 0x00, 0x00, 0x40,   // 0x0160 (352) pixels
		0x40, 0xC0, 0x40, 0x40, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00,   // 0x0170 (368) pixels
		0x00, 0x00, 0x20, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00,   // 0x0180 (384) pixels
		0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x80,   // 0x0190 (400) pixels
		0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0xE0,   // 0x01A0 (416) pixels
		0x00, 0x00, 0x80, 0x00, 0x00, 0x18, 0x07, 0x04, 0x05, 0x0E, 0x10, 0x00, 0x1F, 0x02, 0x02, 0x0E,   // 0x01B0 (432) pixels
		0x11, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x1D, 0x12, 0x12, 0x1F, 0x10, 0x00, 0x43,   // 0x01C0 (448) pixels
		0x6C, 0x1C, 0x03, 0x00, 0x00, 0x10, 0x10, 0x1F, 0x10, 0x10, 0x00, 0x0F, 0x10, 0x10, 0x10, 0x0F,   // 0x01D0 (464) pixels
		0x00, 0x00, 0x1F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F, 0x10, 0x10, 0x10,   // 0x01E0 (480) pixels
		0x09, 0x00, 0x0F, 0x10, 0x10, 0x10, 0x0F, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x1F, 0x10,   // 0x01F0 (496) pixels
		0x10, 0x1F, 0x00, 0x1F, 0x02, 0x0D, 0x10, 0x00,
};

void lcd_init(void)
{ 
	cursor_x = cursor_y = 0;
	textsize = 1;
	textcolor = BLACK;
	contrast = 55;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
	                       SSI_MODE_MASTER, 1000000, 8);

	SSIEnable(SSI0_BASE);

	GPIOPinTypeGPIOOutput(LCD_RST_PORT, LCD_RST_PIN);
	GPIOPinTypeGPIOOutput(LCD_DC_PORT, LCD_DC_PIN);

	// Reset the LCD
	GPIOPinWrite(LCD_RST_PORT, LCD_RST_PIN, 0);
	__delay_us(1000);
	GPIOPinWrite(LCD_RST_PORT, LCD_RST_PIN, LCD_RST_PIN);
	__delay_us(1000);

	command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION ); // get into the EXTENDED mode!
	command(PCD8544_SETBIAS | 0x4); // LCD bias select (4 is optimal?)
  
	command( PCD8544_SETVOP | contrast); // experimentally determined
	command(PCD8544_FUNCTIONSET);  // normal mode
	command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL); // set display to Normal

	while(SSIBusy(SSI0_BASE));

	setContrast(contrast);
	updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
	lcd_display();
	task_enable(TASK_UPDATE_LCD, 0);
}

void lcd_setCursor(uint8_t x, uint8_t y) {
    cursor_x = x;
    cursor_y = y;
}

void lcd_drawstring(char *c) {
    uint8_t i;
    char *ptr = c;
    for (i=0; i<strlen(c); i++) {
        write(*ptr++);
    }
}

static void drawIntegerInBase(unsigned long n, unsigned long base) {
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long i = 0;

  if (n == 0) {
	write('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--) {
	write((buf[i - 1] < 10)?'0' + buf[i - 1]:'A' + buf[i - 1] - 10);
  }
}

void lcd_drawint(long n) {
  if (n < 0) {
    write('-');
    n = -n;
  }

  drawIntegerInBase(n, 10);
}

void lcd_drawfloat(double n) {

	  if (n < 0) {
	    write('-');
	    n = -n;
	  }
	  n += 0.5/1000; // Add rounding factor

	  long integer_part;
	  integer_part = (int)n;
	  drawIntegerInBase(integer_part,10);

	  write('.');

	  n -= integer_part;
	  int decimals = 3;
	  uint8_t decimal_part;
	  while(decimals-- > 0) {
	    n *= 10;
	    decimal_part = (int) n;
	    write('0'+decimal_part);
	    n -= decimal_part;
	  }
}


void  lcd_drawchar(uint8_t x, uint8_t y, char c) {
  uint8_t i, j;
  if (y >= LCDHEIGHT) return;
  if ((x+5) >= LCDWIDTH) return;

  for (i=0; i<5; i++ ) {
    uint8_t d = font[(c*5)+i];
    for (j=0; j<8; j++) {
      if (d & (1 << j)) my_setpixel(x+i, y+j, textcolor);
      else my_setpixel(x+i, y+j, !textcolor);
    }
  }
  for (j=0; j<8; j++) my_setpixel(x+5, y+j, !textcolor);
  updateBoundingBox(x, y, x+5, y + 8);
}

static void write(uint8_t c) {
  if (c == '\n') {
    cursor_y += textsize*8;
    cursor_x = 0;
  } else if (c == '\r') {
    // skip em
  } else {
    lcd_drawchar(cursor_x, cursor_y, c);
    cursor_x += textsize*6;
    if (cursor_x >= (LCDWIDTH-5)) {
      cursor_x = 0;
      cursor_y+=8;
    }
    if (cursor_y >= LCDHEIGHT) 
      cursor_y = 0;
  }
}

static void my_setpixel(uint8_t x, uint8_t y, uint8_t color) {
  if ((x >= LCDWIDTH) || (y >= LCDHEIGHT)) return;

  // x is which column
  if (color) pcd8544_buffer[x+(y/8)*LCDWIDTH] |= (1 << (y%8));
  else pcd8544_buffer[x+ (y/8)*LCDWIDTH] &= ~(1 << (y%8));
}

static void command(uint8_t c) {
	GPIOPinWrite(LCD_DC_PORT, LCD_DC_PIN, 0);
	SSIDataPut(SSI0_BASE, c);
	while(SSIBusy(SSI0_BASE));
}

static void data(uint8_t c) {
	GPIOPinWrite(LCD_DC_PORT, LCD_DC_PIN, LCD_DC_PIN);
	SSIDataPut(SSI0_BASE, c);
	while(SSIBusy(SSI0_BASE));
}

static void setContrast(char val) {
  if (val > 0x7f) val = 0x7f; // char can't be > 0x7f

  command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );
  command( PCD8544_SETVOP | val); 
  command(PCD8544_FUNCTIONSET);  
  
  while(SSIBusy(SSI0_BASE));
}

// reduces how much is refreshed, which speeds it up!
// originally derived from Steve Evans/JCW's mod but cleaned up and optimized
//#define LCD_ENABLE_PARTIAL_UPDATE

#ifdef LCD_ENABLE_PARTIAL_UPDATE
static uint8_t xUpdateMin, xUpdateMax, yUpdateMin, yUpdateMax;
#endif

static void updateBoundingBox(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax) {
#ifdef LCD_ENABLE_PARTIAL_UPDATE
  if (xmin < xUpdateMin) xUpdateMin = xmin;
  if (xmax > xUpdateMax) xUpdateMax = xmax;
  if (ymin < yUpdateMin) yUpdateMin = ymin;
  if (ymax > yUpdateMax) yUpdateMax = ymax;
#endif
}

void lcd_display(void) {
  uint8_t col, maxcol, p;

  for(p = 0; p < 6; p++) {
#ifdef LCD_ENABLE_PARTIAL_UPDATE
    // check if this page is part of update
    if ( yUpdateMin >= ((p+1)*8) ) {
      continue;   // nope, skip it!
    }
    if (yUpdateMax < p*8) {
      break;
    }
#endif

    command(PCD8544_SETYADDR | p);

#ifdef LCD_ENABLE_PARTIAL_UPDATE
    col = xUpdateMin;
    maxcol = xUpdateMax;
#else
    // start at the beginning of the row
    col = 0;
    maxcol = LCDWIDTH-1;
#endif

    command(PCD8544_SETXADDR | col);

    for(; col <= maxcol; col++) {
      //uart_putw_dec(col);
      //uart_putchar(' ');
      data(pcd8544_buffer[(LCDWIDTH*p)+col]);
    }
  }

  command(PCD8544_SETYADDR );  // no idea why this is necessary but it is to finish the last byte?
#ifdef LCD_ENABLE_PARTIAL_UPDATE
  xUpdateMin = LCDWIDTH - 1;
  xUpdateMax = 0;
  yUpdateMin = LCDHEIGHT-1;
  yUpdateMax = 0;
#endif

  while(SSIBusy(SSI0_BASE));
}

// clear everything
void lcd_clear(void) {
  memset(pcd8544_buffer, 0, LCDWIDTH*LCDHEIGHT/8);
  updateBoundingBox(0, 0, LCDWIDTH-1, LCDHEIGHT-1);
  cursor_y = cursor_x = 0;
}

#endif //ENABLE_LCD
