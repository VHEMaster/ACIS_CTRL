#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#define DelayTimer TIM5
#define __DELAY_TIM_CLK_ENABLE() __TIM5_CLK_ENABLE()

#define Delay_Tick (DelayTimer->CNT)
#define COUNTERS 50
extern volatile uint32_t DelStart[COUNTERS];

extern void DelayInit(void);
extern void DelayNs(uint32_t micros);
extern void DelayUs(uint32_t micros);
extern void DelayMs(uint32_t millis);
extern uint32_t DelayDiff(uint32_t a, uint32_t b);
#define DelayStartCount(counter) (DelStart[counter]=Delay_Tick)
extern uint32_t DelayStopCount(uint32_t counter);
