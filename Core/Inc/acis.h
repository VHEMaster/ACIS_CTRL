/*
 * acis.h
 *
 *  Created on: Jan 26, 2021
 *      Author: VHEMaster
 */

#ifndef INC_ACIS_H_
#define INC_ACIS_H_

#include "main.h"
#include "cmsis_os.h"
#include "delay.h"
#include "xCommand.h"

extern void acis_main_task(void * argument);
extern void acis_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length);

#endif /* INC_ACIS_H_ */
