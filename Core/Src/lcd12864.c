/*
 * lcd12864.c
 *
 *  Created on: Jan 26, 2021
 *      Author: VHEMaster
 */

#include "lcd12864.h"
#include "RREFont.h"
#include "cmsis_os.h"
#include <stdlib.h>

uint8_t lcd_buffer[8][128];

static inline void lcd_data(uint8_t value)
{
  uint32_t bsrr = value | ((value ^ 0xFF) << 16);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
  GPIOA->BSRR = bsrr;
  DelayUs(3);
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
  DelayUs(3);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  taskEXIT_CRITICAL();
  DelayUs(1);
}

static inline void lcd_command(uint8_t value)
{
  uint32_t bsrr = value | ((value ^ 0xFF) << 16);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
  GPIOA->BSRR = bsrr;
  DelayUs(3);
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
  DelayUs(3);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  taskEXIT_CRITICAL();
  DelayUs(1);
}

inline void lcd_update(void)
{

  for(int j = 0; j < 8; j++)
  {
    HAL_GPIO_WritePin(LCD_CS1_GPIO_Port, LCD_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS2_GPIO_Port, LCD_CS2_Pin, GPIO_PIN_SET);

    lcd_command(0x40);
    lcd_command(0xB8 + j);

    HAL_GPIO_WritePin(LCD_CS1_GPIO_Port, LCD_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS2_GPIO_Port, LCD_CS2_Pin, GPIO_PIN_RESET);
    for(int i = 0; i < 64; i++) {
      lcd_data(lcd_buffer[j][i]);
    }
    HAL_GPIO_WritePin(LCD_CS1_GPIO_Port, LCD_CS1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_CS2_GPIO_Port, LCD_CS2_Pin, GPIO_PIN_SET);
    for(int i = 64; i < 128; i++) {
      lcd_data(lcd_buffer[j][i]);
    }
  }
}

inline void lcd_reset(void)
{
  HAL_GPIO_WritePin(LCD_CS1_GPIO_Port, LCD_CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CS2_GPIO_Port, LCD_CS2_Pin, GPIO_PIN_SET);
  lcd_command(0x3F); //Display ON
  lcd_command(0x40); //Set Y address = 0
  lcd_command(0xB8); //Set X page = 0
  lcd_command(0xC0); //Set Z scrolling = 0
  HAL_GPIO_WritePin(LCD_CS1_GPIO_Port, LCD_CS1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_CS2_GPIO_Port, LCD_CS2_Pin, GPIO_PIN_RESET);
}

inline void lcd_clear(void)
{
  for(int i = 0; i < 8; i++)
    for(int j = 0; j < 128; j++)
      lcd_buffer[i][j] = 0;
}

inline void lcd_drawpoint(uint16_t x, uint16_t y)
{
  if(x < 128 && y < 64)
    lcd_buffer[y / 8][x] |= 1 << (y % 8);
}

inline void lcd_clearpoint(uint16_t x, uint16_t y)
{
  if(x < 128 && y < 64)
    lcd_buffer[y / 8][x] &= ~(1 << (y % 8));
}


void lcd_rect_solid(int x, int y, int w, int h, int c)
{
  if(c)
  {
    for(int i = y, ii = 0; ii < h; ii++, i++)
    {
      for(int j = x, jj = 0; jj < w; jj++, j++)
      {
          lcd_drawpoint(j,i);
      }
    }
  }
  else
  {
    for(int i = y, ii = 0; ii < h; ii++, i++)
    {
      for(int j = x, jj = 0; jj < w; jj++, j++)
      {
          lcd_clearpoint(j,i);
      }
    }
  }
}


void lcd_rect(int x, int y, int w, int h, int c)
{
  if(w > 0 && h > 0)
  {
    if(c)
    {
      for(int i = y, ii = 0; ii < h; ii++, i++)
      {
        lcd_drawpoint(x,i);
        lcd_drawpoint(x+w-1,i);
      }
      for(int j = x, jj = 0; jj < w; jj++, j++)
      {
        lcd_drawpoint(j,y);
        lcd_drawpoint(j,y+h-1);
      }
    }
    else
    {
      for(int i = y, ii = 0; ii < h; ii++, i++)
      {
        lcd_clearpoint(x,i);
        lcd_clearpoint(x+w-1,i);
      }
      for(int j = x, jj = 0; jj < w; jj++, j++)
      {
        lcd_clearpoint(j,y);
        lcd_clearpoint(j,y+h-1);
      }
    }
  }
}


void lcd_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
  if(x1>=x0 && y1>=y0)
  {
    if(x1-x0 >= y1-y0)
    {
      int16_t deltax = abs(x1 - x0);
      int16_t deltay = abs(y1 - y0);
      int16_t error = 0;
      int16_t deltaerr = deltay;
      int16_t y = y0,x;
      for(x=x0;x<=x1;x++)
      {
          { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltax)
          {
              y = y + 1;
              error = error - deltax;
          }
      }
    }
    else
    {
      int16_t deltax = abs(x1 - x0);
      int16_t deltay = abs(y1 - y0);
      int16_t error = 0;
      int16_t deltaerr = deltax;
      int16_t x = x0,y;
      for(y=y0;y<=y1;y++)
      {
          { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltay)
          {
              x = x + 1;
              error = error - deltay;
          }
      }
    }
  }
  else if(x0>=x1 && y0>=y1)
  {
    if(x0-x1 >= y0-y1)
    {
      int16_t deltax = abs(x0 - x1);
      int16_t deltay = abs(y0 - y1);
      int16_t error = 0;
      int16_t deltaerr = deltay;
      int16_t y = y1,x;
      for(x=x1;x<=x0;x++)
      {
          { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltax)
          {
              y = y + 1;
              error = error - deltax;
          }
      }
    }
    else
    {
      int16_t deltax = abs(x0 - x1);
      int16_t deltay = abs(y0 - y1);
      int16_t error = 0;
      int16_t deltaerr = deltax;
      int16_t x = x1,y;
      for(y=y1;y<=y0;y++)
      {
          { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltay)
          {
              x = x + 1;
              error = error - deltay;
          }
      }
    }
  }
  else if(x0>=x1 && y1>=y0)
  {
    if(x0-x1 >= y1-y0)
    {
      int16_t deltax = abs(x0 - x1);
      int16_t deltay = abs(y1 - y0);
      int16_t error = 0;
      int16_t deltaerr = deltay;
      int16_t y = y0,x;
      for(x=x0;x>=x1;x--)
      {
          { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltax)
          {
              y = y + 1;
              error = error - deltax;
          }
      }
    }
    else
    {
      int16_t deltax = abs(x0 - x1);
      int16_t deltay = abs(y1 - y0);
      int16_t error = 0;
      int16_t deltaerr = deltax;
      int16_t x = x1,y;
      for(y=y1;y>=y0;y--)
      {
          { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltay)
          {
              x = x + 1;
              error = error - deltay;
          }
      }
    }
  }
  else if(x1>=x0 && y0>=y1)
  {
    if(x1-x0 >= y0-y1)
    {
      int16_t deltax = abs(x1 - x0);
      int16_t deltay = abs(y0 - y1);
      int16_t error = 0;
      int16_t deltaerr = deltay;
      int16_t y = y1,x;
      for(x=x1;x>=x0;x--)
      {
          { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltax)
          {
              y = y + 1;
              error = error - deltax;
          }
      }
    }
    else
    {
      int16_t deltax = abs(x1 - x0);
      int16_t deltay = abs(y0 - y1);
      int16_t error = 0;
      int16_t deltaerr = deltax;
      int16_t x = x0,y;
      for(y=y0;y>=y1;y--)
      {
          { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltay)
          {
              x = x + 1;
              error = error - deltay;
          }
      }
    }
  }

}

void lcd_line_limited(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t lim_x0, int16_t lim_y0, int16_t lim_x1, int16_t lim_y1, uint16_t color)
{
  if(x1>=x0 && y1>=y0)
  {
    if(x1-x0 >= y1-y0)
    {
      int16_t deltax = abs(x1 - x0);
      int16_t deltay = abs(y1 - y0);
      int16_t error = 0;
      int16_t deltaerr = deltay;
      int16_t y = y0,x;
      for(x=x0;x<=x1;x++)
      {
          if(x >= lim_x0 && x <= lim_x1 && y >= lim_y0 && y <= lim_y1) { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltax)
          {
              y = y + 1;
              error = error - deltax;
          }
      }
    }
    else
    {
      int16_t deltax = abs(x1 - x0);
      int16_t deltay = abs(y1 - y0);
      int16_t error = 0;
      int16_t deltaerr = deltax;
      int16_t x = x0,y;
      for(y=y0;y<=y1;y++)
      {
          if(x >= lim_x0 && x <= lim_x1 && y >= lim_y0 && y <= lim_y1) { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltay)
          {
              x = x + 1;
              error = error - deltay;
          }
      }
    }
  }
  else if(x0>=x1 && y0>=y1)
  {
    if(x0-x1 >= y0-y1)
    {
      int16_t deltax = abs(x0 - x1);
      int16_t deltay = abs(y0 - y1);
      int16_t error = 0;
      int16_t deltaerr = deltay;
      int16_t y = y1,x;
      for(x=x1;x<=x0;x++)
      {
          if(x >= lim_x0 && x <= lim_x1 && y >= lim_y0 && y <= lim_y1) { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltax)
          {
              y = y + 1;
              error = error - deltax;
          }
      }
    }
    else
    {
      int16_t deltax = abs(x0 - x1);
      int16_t deltay = abs(y0 - y1);
      int16_t error = 0;
      int16_t deltaerr = deltax;
      int16_t x = x1,y;
      for(y=y1;y<=y0;y++)
      {
          if(x >= lim_x0 && x <= lim_x1 && y >= lim_y0 && y <= lim_y1) { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltay)
          {
              x = x + 1;
              error = error - deltay;
          }
      }
    }
  }
  else if(x0>=x1 && y1>=y0)
  {
    if(x0-x1 >= y1-y0)
    {
      int16_t deltax = abs(x0 - x1);
      int16_t deltay = abs(y1 - y0);
      int16_t error = 0;
      int16_t deltaerr = deltay;
      int16_t y = y0,x;
      for(x=x0;x>=x1;x--)
      {
          if(x >= lim_x0 && x <= lim_x1 && y >= lim_y0 && y <= lim_y1) { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltax)
          {
              y = y + 1;
              error = error - deltax;
          }
      }
    }
    else
    {
      int16_t deltax = abs(x0 - x1);
      int16_t deltay = abs(y1 - y0);
      int16_t error = 0;
      int16_t deltaerr = deltax;
      int16_t x = x1,y;
      for(y=y1;y>=y0;y--)
      {
          if(x >= lim_x0 && x <= lim_x1 && y >= lim_y0 && y <= lim_y1) { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltay)
          {
              x = x + 1;
              error = error - deltay;
          }
      }
    }
  }
  else if(x1>=x0 && y0>=y1)
  {
    if(x1-x0 >= y0-y1)
    {
      int16_t deltax = abs(x1 - x0);
      int16_t deltay = abs(y0 - y1);
      int16_t error = 0;
      int16_t deltaerr = deltay;
      int16_t y = y1,x;
      for(x=x1;x>=x0;x--)
      {
          if(x >= lim_x0 && x <= lim_x1 && y >= lim_y0 && y <= lim_y1) { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltax)
          {
              y = y + 1;
              error = error - deltax;
          }
      }
    }
    else
    {
      int16_t deltax = abs(x1 - x0);
      int16_t deltay = abs(y0 - y1);
      int16_t error = 0;
      int16_t deltaerr = deltax;
      int16_t x = x0,y;
      for(y=y0;y>=y1;y--)
      {
          if(x >= lim_x0 && x <= lim_x1 && y >= lim_y0 && y <= lim_y1) { if(color) lcd_drawpoint(x,y); else lcd_clearpoint(x,y); }
          error = error+deltaerr;
          if(2*error >= deltay)
          {
              x = x + 1;
              error = error - deltay;
          }
      }
    }
  }
}

void lcd_circle5x5(int16_t x, int16_t y)
{
  lcd_drawpoint(x-2,y-1);
  lcd_drawpoint(x-2,y);
  lcd_drawpoint(x-2,y+1);
  lcd_drawpoint(x+2,y+1);
  lcd_drawpoint(x+2,y);
  lcd_drawpoint(x+2,y-1);
  lcd_drawpoint(x-1,y-2);
  lcd_drawpoint(x,y-2);
  lcd_drawpoint(x+1,y-2);
  lcd_drawpoint(x-1,y+2);
  lcd_drawpoint(x,y+2);
  lcd_drawpoint(x+1,y+2);
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      lcd_clearpoint(x+i-1,y+j-1);
}

void lcd_circle5x5fill(int16_t x, int16_t y)
{
  lcd_drawpoint(x-2,y-1);
  lcd_drawpoint(x-2,y);
  lcd_drawpoint(x-2,y+1);
  lcd_drawpoint(x+2,y+1);
  lcd_drawpoint(x+2,y);
  lcd_drawpoint(x+2,y-1);
  lcd_drawpoint(x-1,y-2);
  lcd_drawpoint(x,y-2);
  lcd_drawpoint(x+1,y-2);
  lcd_drawpoint(x-1,y+2);
  lcd_drawpoint(x,y+2);
  lcd_drawpoint(x+1,y+2);
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      lcd_drawpoint(x+i-1,y+j-1);
}

void showFont(char *name)
{
  TIM2->CCR3 = 255;
  lcd_clear();
  font_setCR(0);
  font_printStr(0,font_getHeight()*0,name);
  font_printStr(0,font_getHeight()*1,"0123456789:;.-+*()!?/");
  font_printStr(0,font_getHeight()*2,"ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  font_printStr(0,font_getHeight()*3,"abcdefghijklmnopqrstuvwxyz");
  lcd_update();
  DelayMs(3000);
  TIM2->CCR3 = 0;
}

void lcd_init(void)
{
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  DelayUs(500);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  DelayMs(2);

  lcd_reset();
  lcd_clear();
  lcd_update();

  font_init(lcd_rect_solid, 128, 64);
  font_setCR(0);

  //font_setFont(&rre_arialb_16); showFont("[rre_arialb_16]"); lcd_clear();
}


