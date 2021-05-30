#include "timer.h"
#include "mpr121.h"



TIM_HandleTypeDef *_mytim3_init;

void my_timer_init(TIM_HandleTypeDef * htim)
{
    _mytim3_init = htim;
}

void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(_mytim3_init);
	mpr121_display_touch();
}


