/*
 *  Copyright (c) 2003 by Matt Cross <matt@dragonflyhollow.org>
 *
 *  This file is part of the firemarshalbill package.
 *
 *  Firemarshalbill is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Firemarshalbill is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Firemarshalbill; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "lcd.h"

int lcd_init(int rows, int columns) {
  int i;

  lcd_write_reg(0,0x30);
  for(i=0;i<100000;i++) ; 
  lcd_write_reg(0,0x30);
  for(i=0;i<10000;i++) ; 
  lcd_write_reg(0,0x30);
  lcd_write_reg(1,0x38);
  lcd_write_reg(1,0x08);
  lcd_write_reg(1,0x01);
  lcd_write_reg(1,0x06);
  lcd_write_reg(1,0x0c);

  return 0;
}

int lcd_string(int row, char *string) {
  int i;

  lcd_write_reg(1,0x80 + (row%2)*40);
  for(i=0;string[i] != '\0';i++) lcd_write_data(string[i]);
  lcd_write_reg(1,0x80 + (row%2)*40+39);

  return 0;
}

int lcd_write_reg(int wait, char val) {
  if(wait != 0) while(((*(volatile unsigned char*)LCD_REG) & 0x80) == 0x80) ;
  *(volatile unsigned char*)LCD_REG = val & 0xff;

  return 0;
}

int lcd_write_data(char val) {
  while(((*(volatile unsigned char*)LCD_REG) & 0x80) == 0x80) ;
  *(volatile unsigned char*)LCD_DATA = val & 0xff;

  return 0;
}

