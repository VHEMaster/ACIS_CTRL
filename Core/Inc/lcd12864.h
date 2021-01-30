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

#endif /* INC_LCD12864_H_ */
