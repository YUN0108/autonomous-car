#include "pwm.h"

#include <stddef.h>

#define PWM_LEFT_CHANNEL     TIM_CHANNEL_2
#define PWM_RIGHT_CHANNEL    TIM_CHANNEL_3

static TIM_HandleTypeDef *s_pwm_timer = NULL;

HAL_StatusTypeDef PWM_Init(TIM_HandleTypeDef *timer)
{
    if (timer == NULL)
    {
        return HAL_ERROR;
    }

    s_pwm_timer = timer;


    __HAL_TIM_SET_COMPARE(
        s_pwm_timer,
        PWM_LEFT_CHANNEL,
        0U);

    __HAL_TIM_SET_COMPARE(
        s_pwm_timer,
        PWM_RIGHT_CHANNEL,
        0U);


    if (HAL_TIM_PWM_Start(
            s_pwm_timer,
            PWM_LEFT_CHANNEL) != HAL_OK)
    {
        s_pwm_timer = NULL;
        return HAL_ERROR;
    }


    if (HAL_TIM_PWM_Start(
            s_pwm_timer,
            PWM_RIGHT_CHANNEL) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(
            s_pwm_timer,
            PWM_LEFT_CHANNEL);

        s_pwm_timer = NULL;
        return HAL_ERROR;
    }

    PWM_Stop();

    return HAL_OK;
}

void PWM_SetDuty(
    uint8_t left_percent,
    uint8_t right_percent)
{
    uint32_t period;
    uint32_t left_compare;
    uint32_t right_compare;

    if (s_pwm_timer == NULL)
    {
        return;
    }

    if (left_percent > 100U)
    {
        left_percent = 100U;
    }

    if (right_percent > 100U)
    {
        right_percent = 100U;
    }


    period =
        __HAL_TIM_GET_AUTORELOAD(s_pwm_timer) + 1U;

    left_compare =
        (period * (uint32_t)left_percent) / 100U;

    right_compare =
        (period * (uint32_t)right_percent) / 100U;

    __HAL_TIM_SET_COMPARE(
        s_pwm_timer,
        PWM_LEFT_CHANNEL,
        left_compare);

    __HAL_TIM_SET_COMPARE(
        s_pwm_timer,
        PWM_RIGHT_CHANNEL,
        right_compare);
}

void PWM_Stop(void)
{
    PWM_SetDuty(0U, 0U);
}
