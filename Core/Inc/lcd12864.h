/*
 * lcd12864.h
 *
 *  Created on: Jan 26, 2021
 *      Author: VHEMaster
 */

#ifndef INC_LCD12864_H_
#define INC_LCD12864_H_

#include "main.h"
#include "cmsis_os.h"
#include "delay.h"

extern void lcd_init(void);
extern void lcd_update(void);
extern void lcd_reset(void);
extern void lcd_clear(void);
extern void lcd_drawpoint(uint8_t x, uint8_t y);
extern void lcd_clearpoint(uint8_t x, uint8_t y);
extern void lcd_rect_solid(int x, int y, int w, int h, int c);
extern void lcd_rect(int x, int y, int w, int h, int c);
extern void lcd_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
extern void lcd_line_limited(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t lim_x0, int16_t lim_y0, int16_t lim_x1, int16_t lim_y1, uint16_t color);
extern void lcd_circle5x5(int16_t x, int16_t y);
#endif /* INC_LCD12864_H_ */
