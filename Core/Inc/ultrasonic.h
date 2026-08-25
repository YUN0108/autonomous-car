#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "main.h"
#include "tim.h"

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
    ULTRASONIC_LEFT = 0,
    ULTRASONIC_CENTER,
    ULTRASONIC_RIGHT,
    ULTRASONIC_SENSOR_COUNT

} UltrasonicSensor_t;


typedef struct
{
    uint16_t distance_cm;
    bool has_measurement;
    bool timeout;
    uint32_t age_ms;

} UltrasonicReading_t;


HAL_StatusTypeDef Ultrasonic_Init(
        TIM_HandleTypeDef *htim_us);

void Ultrasonic_Task(void);

void Ultrasonic_EXTI_Callback(
        uint16_t gpio_pin);

void Ultrasonic_GetReading(
        UltrasonicSensor_t sensor,
        UltrasonicReading_t *reading);

/* Compatibility helpers */
uint16_t Ultrasonic_GetDistanceCm(
        UltrasonicSensor_t sensor);

bool Ultrasonic_HasMeasurement(
        UltrasonicSensor_t sensor);

bool Ultrasonic_IsTimeout(
        UltrasonicSensor_t sensor);

uint32_t Ultrasonic_GetMeasurementAgeMs(
        UltrasonicSensor_t sensor);


#endif /* INC_ULTRASONIC_H_ */
