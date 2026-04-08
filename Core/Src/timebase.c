#include "timebase.h"
#include "stm32f4xx_hal.h"



static const float us_to_s = 1.0e-6f;
extern TIM_HandleTypeDef htim2;



void timebase_init(void){

	HAL_TIM_Base_Start(&htim2);

}

uint32_t timebase_now_us(void){

	return __HAL_TIM_GET_COUNTER(&htim2);

}



// Attention : avant le premier appel, initialiser last_us dans pid_init()
// avec par exemple : pid->last_us = timebase_now_us();
float timebase_dt_s(uint32_t *last_us){   // calcule dt depuis last_us et met à jour last_us

	if(!last_us){ return 0.0f; }


	uint32_t now = timebase_now_us();
	float dt = (now - *last_us)*us_to_s;
	*last_us = now;

	return dt;

}


void timebase_delay_us(uint32_t us){

	uint32_t start = timebase_now_us();

	while( (timebase_now_us() - start) < us );
}


