
#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef PWM_Init(TIM_HandleTypeDef *timer);

void PWM_SetDuty(
    uint8_t left_percent,
    uint8_t right_percent);

void PWM_Stop(void);

#endif /* INC_PWM_H_ */
