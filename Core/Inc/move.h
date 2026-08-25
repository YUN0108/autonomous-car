#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
    MOVE_DIRECTION_STOP = 0,
    MOVE_DIRECTION_FORWARD,
    MOVE_DIRECTION_BACKWARD,
    MOVE_DIRECTION_LEFT,
    MOVE_DIRECTION_RIGHT,
    MOVE_DIRECTION_PIVOT_LEFT,
    MOVE_DIRECTION_PIVOT_RIGHT

} MoveDirection_t;


/*
 * Initialize motor movement module.
 * PWM_Init() must already have been called.
 */
void Move_Init(void);


/*
 * Non-blocking direction-change processing.
 *
 * IMPORTANT:
 * Call this periodically from ControlTask.
 * Recommended period: 1~5 ms.
 */
void Move_Task(void);


/*
 * Returns true while waiting for the motor direction
 * protection dead-time to finish.
 */
bool Move_IsBusy(void);


/* Manual speed */
void Move_SetSpeed(uint8_t speed_percent);
uint8_t Move_GetSpeed(void);


/*
 * Returns the direction that is currently applied to the motor.
 * During direction-change dead-time, STOP is returned.
 */
MoveDirection_t Move_GetDirection(void);


/* Manual movement */
void Move_Forward(void);
void Move_Backward(void);
void Move_Left(void);
void Move_Right(void);
void Move_Stop(void);


/* AUTO movement */
void Move_ForwardPWM(
        uint8_t left_percent,
        uint8_t right_percent);

void Move_PivotLeft(uint8_t speed_percent);
void Move_PivotRight(uint8_t speed_percent);


#endif /* INC_MOVE_H_ */
