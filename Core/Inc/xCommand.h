/*
 * xCommand.h
 *
 *  Created on: Dec 6, 2020
 *      Author: denys.prokhorov
 */


#ifndef XCOMMAND_H_
#define XCOMMAND_H_

#include "cmsis_os.h"
#include "xProFIFO.h"
#include "FreeRTOS.h"

#define HEADER_ACK_BIT (1<<7)
#define HEADER_ALARM_BITS ((1<<7)|(1<<6))
#define HEADER_MASK_BITS (0xFF ^ HEADER_ALARM_BITS)
#define TASK_SLEEP  { osDelay(1); } // Task must give it's time to another process or just skip some time
#define MAX_PACK_LEN (384)

typedef enum {
    etrNone,
    etrPC,
    etrACIS,
    etrCTRL,
    etrCount
} eTransChannels;

extern sProFIFO fifoAcisRx;
extern sProFIFO fifoAcisTx;
extern sProFIFO fifoPcRx;
extern sProFIFO fifoPcTx;
extern void initFIFOs(void);
extern void xGetter(void * arg);
extern uint8_t xSender(eTransChannels xChaDest, uint8_t* xMsgPtr, uint32_t xMsgLen);

#endif /* XCOMMAND_H_ */
