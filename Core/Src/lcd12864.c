/*
 * lcd12864.c
 *
 *  Created on: Jan 26, 2021
 *      Author: VHEMaster
 */

#include "lcd12864.h"
#include "RREFont.h"
#include "cmsis_os.h"

uint8_t lcd_buffer[8][128];

static inline void lcd_data(uint8_t value)
{
  uint32_t bsrr = value | ((value ^ 0xFF) << 16);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
  GPIOA->BSRR = bsrr;
  DelayNs(1300);
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
  DelayNs(1300);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  taskEXIT_CRITICAL();
}

static inline void lcd_command(uint8_t value)
{
  uint32_t bsrr = value | ((value ^ 0xFF) << 16);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
  GPIOA->BSRR = bsrr;
  DelayNs(1300);
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
  DelayNs(1300);
  HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);
  taskEXIT_CRITICAL();
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
    for(int i = 0; i < 64; i++)
    {
      lcd_data(lcd_buffer[j][i]);
    }
    HAL_GPIO_WritePin(LCD_CS1_GPIO_Port, LCD_CS1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_CS2_GPIO_Port, LCD_CS2_Pin, GPIO_PIN_SET);
    for(int i = 64; i < 128; i++)
    {
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

inline void lcd_drawpoint(uint8_t x, uint8_t y)
{
  if(x < 128 && y < 64)
    lcd_buffer[y / 8][x] |= 1 << (y % 8);
}

inline void lcd_clearpoint(uint8_t x, uint8_t y)
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


  //font_setFont(&rre_ubuntu_32); showFont("[rre_ubuntu_32]");

}
