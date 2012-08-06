/* From Arduino LiquidCrystal library, modifications for R2C2
   Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       
*/

/* Copyright (c) 2010 David A. Mellis */
/*********************************************************************
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301 USA 
*********************************************************************/

#ifndef LiquidCrystal_h
#define LiquidCrystal_h

#include <inttypes.h>
#include <stdint.h>

//#include "Print.h"

// commands
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_CURSORSHIFT     0x10
#define LCD_FUNCTIONSET     0x20
#define LCD_SETCGRAMADDR    0x40
#define LCD_SETDDRAMADDR    0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON     0x04
#define LCD_DISPLAYOFF    0x00
#define LCD_CURSORON      0x02
#define LCD_CURSOROFF     0x00
#define LCD_BLINKON       0x01
#define LCD_BLINKOFF      0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE   0x08
#define LCD_CURSORMOVE    0x00
#define LCD_MOVERIGHT     0x04
#define LCD_MOVELEFT      0x00

// flags for function set
#define LCD_8BITMODE      0x10
#define LCD_4BITMODE      0x00
#define LCD_2LINE         0x08
#define LCD_1LINE         0x00
#define LCD_5x10DOTS      0x04
#define LCD_5x8DOTS       0x00

// public
void  LiquidCrystal_8bit(uint8_t rs, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
		uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);

void LiquidCrystal_8bit_rw(uint8_t rs, uint8_t rw, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
		uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);

void   LiquidCrystal_4bit_rw(uint8_t rs, uint8_t rw, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);

void   LiquidCrystal_4bit(uint8_t rs, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);

  static void init(uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
	    uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
	    uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
    
  void begin(uint8_t cols, uint8_t rows); //, uint8_t charsize = LCD_5x8DOTS);

  void clear();
  void home();

  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void leftToRight();
  void rightToLeft();
  void autoscroll();
  void noAutoscroll();

  void createChar(uint8_t location, uint8_t charmap[]);
  void setCursor(uint8_t row, uint8_t col); 
  /*virtual*/ void write(uint8_t value);
  void command(uint8_t value);

  void print(const char *str);

//private:
  void send(uint8_t value, uint8_t mode);
  void write4bits(uint8_t value);
  void write8bits(uint8_t value);
  void pulseEnable();

typedef struct {
} tLiquidCrystal;

#endif
