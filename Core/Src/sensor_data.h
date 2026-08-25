#ifndef INC_SENSOR_DATA_H_
#define INC_SENSOR_DATA_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t left_cm;
    uint16_t center_cm;
    uint16_t right_cm;

    bool left_valid;
    bool center_valid;
    bool right_valid;

    bool left_timeout;
    bool center_timeout;
    bool right_timeout;

    uint32_t left_age_ms;
    uint32_t center_age_ms;
    uint32_t right_age_ms;

} SensorData_t;

#endif /* INC_SENSOR_DATA_H_ */
