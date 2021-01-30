/*
 * controls.h
 *
 *  Created on: Jan 26, 2021
 *      Author: VHEMaster
 */

#ifndef INC_CONTROLS_H_
#define INC_CONTROLS_H_

#include "main.h"
#include "cmsis_os.h"
#include "delay.h"

extern volatile uint8_t BUT_LEFT_PRESS;
extern volatile uint8_t BUT_RIGHT_PRESS;
extern volatile uint8_t BUT_UP_PRESS;
extern volatile uint8_t BUT_DOWN_PRESS;
extern volatile uint8_t BUT_ENTER_PRESS;
extern volatile uint8_t BUT_CANCEL_PRESS;

extern volatile uint32_t BUT_LEFT_TIME;
extern volatile uint32_t BUT_RIGHT_TIME;
extern volatile uint32_t BUT_UP_TIME;
extern volatile uint32_t BUT_DOWN_TIME;
extern volatile uint32_t BUT_ENTER_TIME;
extern volatile uint32_t BUT_CANCEL_TIME;

extern volatile uint8_t BUT_LEFT;
extern volatile uint8_t BUT_RIGHT;
extern volatile uint8_t BUT_UP;
extern volatile uint8_t BUT_DOWN;
extern volatile uint8_t BUT_ENTER;
extern volatile uint8_t BUT_CANCEL;

extern volatile void controls_irq(void);

#endif /* INC_CONTROLS_H_ */
