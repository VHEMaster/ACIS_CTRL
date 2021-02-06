#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "delay.h"

static TIM_HandleTypeDef htim_delay;

volatile uint32_t DelStart[COUNTERS];
static volatile uint32_t prescaller = 0;

void DelayInit(void)
{
  __DELAY_TIM_CLK_ENABLE();
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim_delay.Instance = DelayTimer;
    prescaller = (HAL_RCC_GetPCLK1Freq() * 2 / 1000000);
    htim_delay.Init.Prescaler = 0;
    htim_delay.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_delay.Init.Period = DelayMask;
    htim_delay.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim_delay);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim_delay, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim_delay, &sMasterConfig);

    HAL_TIM_Base_Start(&htim_delay);

}
 
inline void DelayNs(uint32_t val)
{
  uint32_t tickstart = Delay_Tick * 1000;
  while(DelayDiff(Delay_Tick * 1000, tickstart) < val) {}
}

inline void DelayUs(uint32_t val)
{
  uint32_t tickstart = Delay_Tick;
  while(DelayDiff(Delay_Tick, tickstart) < val) {}
}
 
inline void DelayMs(uint32_t val)
{
  DelayUs(val * 1000);
}

inline uint32_t DelayDiff(uint32_t a, uint32_t b)
{
	if(a >= b)
		return (a - b) / prescaller;
	return ((DelayMask - b) + a) / prescaller;
}

inline uint32_t DelayStopCount(uint32_t counter)
{
	return DelayDiff(Delay_Tick, DelStart[counter]);
	
}

