#include "ultrasonic.h"

#include "main.h"

#include <stddef.h>


#define ULTRASONIC_INTER_TRIGGER_MS      60U
#define ULTRASONIC_ECHO_TIMEOUT_MS       40U

#define ULTRASONIC_MIN_DISTANCE_CM        2U
#define ULTRASONIC_MAX_DISTANCE_CM      400U


typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;

    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;

} UltrasonicPinMap_t;


typedef enum
{
    ULTRASONIC_STATE_IDLE = 0,
    ULTRASONIC_STATE_WAIT_RISE,
    ULTRASONIC_STATE_WAIT_FALL

} UltrasonicState_t;


typedef enum
{
    ULTRASONIC_NEXT_SIDE_RIGHT = 0,
    ULTRASONIC_NEXT_SIDE_LEFT

} UltrasonicNextSide_t;


static const UltrasonicPinMap_t
s_pin_map[ULTRASONIC_SENSOR_COUNT] =
{
    {
        US_LEFT_TRIG_GPIO_Port,
        US_LEFT_TRIG_Pin,
        US_LEFT_ECHO_GPIO_Port,
        US_LEFT_ECHO_Pin
    },
    {
        US_CENTER_TRIG_GPIO_Port,
        US_CENTER_TRIG_Pin,
        US_CENTER_ECHO_GPIO_Port,
        US_CENTER_ECHO_Pin
    },
    {
        US_RIGHT_TRIG_GPIO_Port,
        US_RIGHT_TRIG_Pin,
        US_RIGHT_ECHO_GPIO_Port,
        US_RIGHT_ECHO_Pin
    }
};


static TIM_HandleTypeDef *s_htim_us = NULL;

static volatile UltrasonicState_t s_state =
        ULTRASONIC_STATE_IDLE;

static volatile uint8_t s_active_sensor =
        (uint8_t)ULTRASONIC_CENTER;

static volatile uint16_t s_echo_rise_us = 0U;

static volatile uint16_t
s_distance_cm[ULTRASONIC_SENSOR_COUNT] =
{
    ULTRASONIC_MAX_DISTANCE_CM,
    ULTRASONIC_MAX_DISTANCE_CM,
    ULTRASONIC_MAX_DISTANCE_CM
};

static volatile uint32_t
s_update_tick_ms[ULTRASONIC_SENSOR_COUNT] =
{
    0U,
    0U,
    0U
};

static volatile bool
s_has_measurement[ULTRASONIC_SENSOR_COUNT] =
{
    false,
    false,
    false
};

static volatile bool
s_timeout[ULTRASONIC_SENSOR_COUNT] =
{
    false,
    false,
    false
};

static UltrasonicNextSide_t s_next_side =
        ULTRASONIC_NEXT_SIDE_RIGHT;

static uint32_t s_trigger_tick_ms = 0U;
static uint32_t s_next_trigger_tick_ms = 0U;


static uint16_t Ultrasonic_TimerUs(void);
static void Ultrasonic_DelayUs(uint16_t delay_us);
static void Ultrasonic_SendTrigger(uint8_t sensor);

static void Ultrasonic_CompleteMeasurement(
        uint16_t distance_cm,
        bool timeout);

static void Ultrasonic_AdvanceSensor(void);

static int32_t Ultrasonic_FindSensorByEchoPin(
        uint16_t gpio_pin);

static bool Ultrasonic_TimeReached(
        uint32_t now,
        uint32_t target);


HAL_StatusTypeDef Ultrasonic_Init(
        TIM_HandleTypeDef *htim_us)
{
    HAL_StatusTypeDef status;
    uint32_t i;

    if (htim_us == NULL)
    {
        return HAL_ERROR;
    }

    s_htim_us = htim_us;

    status = HAL_TIM_Base_Start(s_htim_us);

    if (status != HAL_OK)
    {
        return status;
    }

    for (i = 0U;
         i < (uint32_t)ULTRASONIC_SENSOR_COUNT;
         i++)
    {
        HAL_GPIO_WritePin(
            s_pin_map[i].trig_port,
            s_pin_map[i].trig_pin,
            GPIO_PIN_RESET);

        s_distance_cm[i] = ULTRASONIC_MAX_DISTANCE_CM;
        s_update_tick_ms[i] = 0U;
        s_has_measurement[i] = false;
        s_timeout[i] = false;
    }

    s_state = ULTRASONIC_STATE_IDLE;
    s_active_sensor = (uint8_t)ULTRASONIC_CENTER;
    s_echo_rise_us = 0U;

    s_next_side = ULTRASONIC_NEXT_SIDE_RIGHT;

    s_trigger_tick_ms = HAL_GetTick();
    s_next_trigger_tick_ms = HAL_GetTick() + 100U;

    return HAL_OK;
}


void Ultrasonic_Task(void)
{
    uint32_t now;

    if (s_htim_us == NULL)
    {
        return;
    }

    now = HAL_GetTick();

    if (s_state == ULTRASONIC_STATE_IDLE)
    {
        if (Ultrasonic_TimeReached(
                now,
                s_next_trigger_tick_ms))
        {
            s_trigger_tick_ms = now;

            s_next_trigger_tick_ms =
                now + ULTRASONIC_INTER_TRIGGER_MS;

            s_state = ULTRASONIC_STATE_WAIT_RISE;

            Ultrasonic_SendTrigger(s_active_sensor);
        }

        return;
    }

    if ((uint32_t)(now - s_trigger_tick_ms) >=
        ULTRASONIC_ECHO_TIMEOUT_MS)
    {
        Ultrasonic_CompleteMeasurement(
            ULTRASONIC_MAX_DISTANCE_CM,
            true);
    }
}


void Ultrasonic_EXTI_Callback(
        uint16_t gpio_pin)
{
    int32_t sensor;

    uint16_t now_us;
    uint16_t pulse_us;

    uint32_t distance_cm;

    GPIO_PinState echo_level;

    if ((s_htim_us == NULL) ||
        (s_active_sensor >=
         (uint8_t)ULTRASONIC_SENSOR_COUNT))
    {
        return;
    }

    sensor = Ultrasonic_FindSensorByEchoPin(gpio_pin);

    if ((sensor < 0) ||
        ((uint8_t)sensor != s_active_sensor))
    {
        return;
    }

    echo_level = HAL_GPIO_ReadPin(
        s_pin_map[s_active_sensor].echo_port,
        s_pin_map[s_active_sensor].echo_pin);

    now_us = Ultrasonic_TimerUs();

    if ((s_state == ULTRASONIC_STATE_WAIT_RISE) &&
        (echo_level == GPIO_PIN_SET))
    {
        s_echo_rise_us = now_us;
        s_state = ULTRASONIC_STATE_WAIT_FALL;
        return;
    }

    if ((s_state == ULTRASONIC_STATE_WAIT_FALL) &&
        (echo_level == GPIO_PIN_RESET))
    {
        pulse_us = (uint16_t)(
            now_us - s_echo_rise_us);

        distance_cm =
            ((uint32_t)pulse_us + 29U) / 58U;

        if (distance_cm < ULTRASONIC_MIN_DISTANCE_CM)
        {
            distance_cm = ULTRASONIC_MIN_DISTANCE_CM;
        }
        else if (distance_cm > ULTRASONIC_MAX_DISTANCE_CM)
        {
            distance_cm = ULTRASONIC_MAX_DISTANCE_CM;
        }

        Ultrasonic_CompleteMeasurement(
            (uint16_t)distance_cm,
            false);
    }
}


void Ultrasonic_GetReading(
        UltrasonicSensor_t sensor,
        UltrasonicReading_t *reading)
{
    if (reading == NULL)
    {
        return;
    }

    if ((uint32_t)sensor >=
        (uint32_t)ULTRASONIC_SENSOR_COUNT)
    {
        reading->distance_cm = ULTRASONIC_MAX_DISTANCE_CM;
        reading->has_measurement = false;
        reading->timeout = false;
        reading->age_ms = UINT32_MAX;
        return;
    }

    reading->distance_cm = s_distance_cm[sensor];
    reading->has_measurement = s_has_measurement[sensor];
    reading->timeout = s_timeout[sensor];

    if (reading->has_measurement)
    {
        reading->age_ms =
            HAL_GetTick() - s_update_tick_ms[sensor];
    }
    else
    {
        reading->age_ms = UINT32_MAX;
    }
}


uint16_t Ultrasonic_GetDistanceCm(
        UltrasonicSensor_t sensor)
{
    if ((uint32_t)sensor >=
        (uint32_t)ULTRASONIC_SENSOR_COUNT)
    {
        return ULTRASONIC_MAX_DISTANCE_CM;
    }

    return s_distance_cm[sensor];
}


bool Ultrasonic_HasMeasurement(
        UltrasonicSensor_t sensor)
{
    if ((uint32_t)sensor >=
        (uint32_t)ULTRASONIC_SENSOR_COUNT)
    {
        return false;
    }

    return s_has_measurement[sensor];
}


bool Ultrasonic_IsTimeout(
        UltrasonicSensor_t sensor)
{
    if ((uint32_t)sensor >=
        (uint32_t)ULTRASONIC_SENSOR_COUNT)
    {
        return false;
    }

    return s_timeout[sensor];
}


uint32_t Ultrasonic_GetMeasurementAgeMs(
        UltrasonicSensor_t sensor)
{
    if (((uint32_t)sensor >=
         (uint32_t)ULTRASONIC_SENSOR_COUNT) ||
        (!s_has_measurement[sensor]))
    {
        return UINT32_MAX;
    }

    return HAL_GetTick() - s_update_tick_ms[sensor];
}


static uint16_t Ultrasonic_TimerUs(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_us);
}


static void Ultrasonic_DelayUs(
        uint16_t delay_us)
{
    uint16_t start_us;

    start_us = Ultrasonic_TimerUs();

    while ((uint16_t)(
            Ultrasonic_TimerUs() - start_us) < delay_us)
    {
        /* busy wait for a few microseconds */
    }
}


static void Ultrasonic_SendTrigger(
        uint8_t sensor)
{
    const UltrasonicPinMap_t *pin;

    if (sensor >= (uint8_t)ULTRASONIC_SENSOR_COUNT)
    {
        return;
    }

    pin = &s_pin_map[sensor];

    __HAL_GPIO_EXTI_CLEAR_IT(pin->echo_pin);

    HAL_GPIO_WritePin(
        pin->trig_port,
        pin->trig_pin,
        GPIO_PIN_RESET);

    Ultrasonic_DelayUs(2U);

    HAL_GPIO_WritePin(
        pin->trig_port,
        pin->trig_pin,
        GPIO_PIN_SET);

    Ultrasonic_DelayUs(10U);

    HAL_GPIO_WritePin(
        pin->trig_port,
        pin->trig_pin,
        GPIO_PIN_RESET);
}


static void Ultrasonic_CompleteMeasurement(
        uint16_t distance_cm,
        bool timeout)
{
    uint8_t sensor;

    sensor = s_active_sensor;

    if (sensor < (uint8_t)ULTRASONIC_SENSOR_COUNT)
    {
        s_distance_cm[sensor] = distance_cm;
        s_update_tick_ms[sensor] = HAL_GetTick();
        s_has_measurement[sensor] = true;
        s_timeout[sensor] = timeout;
    }

    s_state = ULTRASONIC_STATE_IDLE;

    Ultrasonic_AdvanceSensor();
}


static void Ultrasonic_AdvanceSensor(void)
{
    if (s_active_sensor == (uint8_t)ULTRASONIC_CENTER)
    {
        if (s_next_side == ULTRASONIC_NEXT_SIDE_RIGHT)
        {
            s_active_sensor = (uint8_t)ULTRASONIC_RIGHT;
            s_next_side = ULTRASONIC_NEXT_SIDE_LEFT;
        }
        else
        {
            s_active_sensor = (uint8_t)ULTRASONIC_LEFT;
            s_next_side = ULTRASONIC_NEXT_SIDE_RIGHT;
        }
    }
    else
    {
        s_active_sensor = (uint8_t)ULTRASONIC_CENTER;
    }
}


static int32_t Ultrasonic_FindSensorByEchoPin(
        uint16_t gpio_pin)
{
    uint32_t i;

    for (i = 0U;
         i < (uint32_t)ULTRASONIC_SENSOR_COUNT;
         i++)
    {
        if (s_pin_map[i].echo_pin == gpio_pin)
        {
            return (int32_t)i;
        }
    }

    return -1;
}


static bool Ultrasonic_TimeReached(
        uint32_t now,
        uint32_t target)
{
    return ((int32_t)(now - target) >= 0);
}
