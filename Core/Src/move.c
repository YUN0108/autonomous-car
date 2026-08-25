#include "move.h"

#include "main.h"
#include "pwm.h"

#define MOVE_DEFAULT_SPEED               75U
#define MOVE_DIRECTION_DELAY_MS          30U


/* Manual left/right steering */
#define MOVE_TURN_DELTA_PERCENT           5U
#define MOVE_TURN_MIN_INNER_SPEED        55U
#define MOVE_FORWARD_LEFT_TRIM            4U


typedef enum
{
    MOVE_PWM_MODE_MANUAL = 0,
    MOVE_PWM_MODE_CUSTOM

} MovePwmMode_t;


static uint8_t s_speed_percent = MOVE_DEFAULT_SPEED;

static MoveDirection_t s_direction =
    MOVE_DIRECTION_STOP;


static MovePwmMode_t s_pwm_mode =
    MOVE_PWM_MODE_MANUAL;


static bool s_direction_change_pending = false;

static uint32_t s_direction_change_tick_ms = 0U;

static MoveDirection_t s_pending_direction =
    MOVE_DIRECTION_STOP;

static MovePwmMode_t s_pending_pwm_mode =
    MOVE_PWM_MODE_MANUAL;

static uint8_t s_pending_left_pwm = 0U;
static uint8_t s_pending_right_pwm = 0U;

static void Move_RequestManual(
        MoveDirection_t direction);

static void Move_RequestCustom(
        MoveDirection_t direction,
        uint8_t left_pwm,
        uint8_t right_pwm);

static void Move_Request(
        MoveDirection_t direction,
        MovePwmMode_t pwm_mode,
        uint8_t left_pwm,
        uint8_t right_pwm);

static void Move_ApplyRequest(
        MoveDirection_t direction,
        MovePwmMode_t pwm_mode,
        uint8_t left_pwm,
        uint8_t right_pwm);

static void Move_ApplyDirectionPins(
        MoveDirection_t direction);

static bool Move_RequiresDeadTime(
        MoveDirection_t current_direction,
        MoveDirection_t new_direction);

static int8_t Move_GetLeftPolarity(
        MoveDirection_t direction);

static int8_t Move_GetRightPolarity(
        MoveDirection_t direction);

static void Move_SetAllDirectionPinsLow(void);

static void Move_ApplyManualPWM(void);

static uint8_t Move_GetTurnInnerSpeed(void);

static uint8_t Move_GetForwardLeftSpeed(void);


void Move_Init(void)
{
    PWM_Stop();
    Move_SetAllDirectionPinsLow();

    s_speed_percent = MOVE_DEFAULT_SPEED;

    s_direction = MOVE_DIRECTION_STOP;
    s_pwm_mode = MOVE_PWM_MODE_MANUAL;

    s_direction_change_pending = false;
    s_direction_change_tick_ms = 0U;

    s_pending_direction = MOVE_DIRECTION_STOP;
    s_pending_pwm_mode = MOVE_PWM_MODE_MANUAL;

    s_pending_left_pwm = 0U;
    s_pending_right_pwm = 0U;
}

void Move_Task(void)
{
    uint32_t now;

    if (!s_direction_change_pending)
    {
        return;
    }

    now = HAL_GetTick();

    if ((uint32_t)(
            now - s_direction_change_tick_ms) <
        MOVE_DIRECTION_DELAY_MS)
    {
        return;
    }

    s_direction_change_pending = false;

    Move_ApplyRequest(
        s_pending_direction,
        s_pending_pwm_mode,
        s_pending_left_pwm,
        s_pending_right_pwm);
}


bool Move_IsBusy(void)
{
    return s_direction_change_pending;
}


void Move_SetSpeed(uint8_t speed_percent)
{
    if (speed_percent > 100U)
    {
        speed_percent = 100U;
    }

    s_speed_percent = speed_percent;


    if ((!s_direction_change_pending) &&
        (s_direction != MOVE_DIRECTION_STOP) &&
        (s_pwm_mode == MOVE_PWM_MODE_MANUAL))
    {
        Move_ApplyManualPWM();
    }
}


uint8_t Move_GetSpeed(void)
{
    return s_speed_percent;
}


MoveDirection_t Move_GetDirection(void)
{
    return s_direction;
}


void Move_Forward(void)
{
    Move_RequestManual(
        MOVE_DIRECTION_FORWARD);
}


void Move_Backward(void)
{
    Move_RequestManual(
        MOVE_DIRECTION_BACKWARD);
}


void Move_Left(void)
{
    Move_RequestManual(
        MOVE_DIRECTION_LEFT);
}


void Move_Right(void)
{
    Move_RequestManual(
        MOVE_DIRECTION_RIGHT);
}


void Move_Stop(void)
{
    s_direction_change_pending = false;

    s_pending_direction = MOVE_DIRECTION_STOP;

    PWM_Stop();
    Move_SetAllDirectionPinsLow();

    s_direction = MOVE_DIRECTION_STOP;
    s_pwm_mode = MOVE_PWM_MODE_MANUAL;
}



void Move_ForwardPWM(
        uint8_t left_percent,
        uint8_t right_percent)
{
    Move_RequestCustom(
        MOVE_DIRECTION_FORWARD,
        left_percent,
        right_percent);
}


void Move_PivotLeft(uint8_t speed_percent)
{
    Move_RequestCustom(
        MOVE_DIRECTION_PIVOT_LEFT,
        speed_percent,
        speed_percent);
}


void Move_PivotRight(uint8_t speed_percent)
{
    Move_RequestCustom(
        MOVE_DIRECTION_PIVOT_RIGHT,
        speed_percent,
        speed_percent);
}



static void Move_RequestManual(
        MoveDirection_t direction)
{
    Move_Request(
        direction,
        MOVE_PWM_MODE_MANUAL,
        0U,
        0U);
}


static void Move_RequestCustom(
        MoveDirection_t direction,
        uint8_t left_pwm,
        uint8_t right_pwm)
{
    Move_Request(
        direction,
        MOVE_PWM_MODE_CUSTOM,
        left_pwm,
        right_pwm);
}


static void Move_Request(
        MoveDirection_t direction,
        MovePwmMode_t pwm_mode,
        uint8_t left_pwm,
        uint8_t right_pwm)
{

    if (direction == MOVE_DIRECTION_STOP)
    {
        Move_Stop();
        return;
    }


    if (s_direction_change_pending)
    {
        s_pending_direction = direction;
        s_pending_pwm_mode = pwm_mode;

        s_pending_left_pwm = left_pwm;
        s_pending_right_pwm = right_pwm;

        return;
    }


    if (Move_RequiresDeadTime(
            s_direction,
            direction))
    {

        PWM_Stop();
        Move_SetAllDirectionPinsLow();

        s_direction = MOVE_DIRECTION_STOP;

        s_pending_direction = direction;
        s_pending_pwm_mode = pwm_mode;

        s_pending_left_pwm = left_pwm;
        s_pending_right_pwm = right_pwm;

        s_direction_change_tick_ms =
            HAL_GetTick();

        s_direction_change_pending = true;

        return;
    }


    Move_ApplyRequest(
        direction,
        pwm_mode,
        left_pwm,
        right_pwm);
}



static void Move_ApplyRequest(
        MoveDirection_t direction,
        MovePwmMode_t pwm_mode,
        uint8_t left_pwm,
        uint8_t right_pwm)
{
    Move_ApplyDirectionPins(direction);

    s_direction = direction;
    s_pwm_mode = pwm_mode;


    if (pwm_mode == MOVE_PWM_MODE_MANUAL)
    {
        Move_ApplyManualPWM();
    }
    else
    {
        PWM_SetDuty(
            left_pwm,
            right_pwm);
    }
}



static void Move_ApplyDirectionPins(
        MoveDirection_t direction)
{
    switch (direction)
    {
        case MOVE_DIRECTION_FORWARD:
        case MOVE_DIRECTION_LEFT:
        case MOVE_DIRECTION_RIGHT:


            HAL_GPIO_WritePin(
                MOTOR_L_IN1_GPIO_Port,
                MOTOR_L_IN1_Pin,
                GPIO_PIN_SET);

            HAL_GPIO_WritePin(
                MOTOR_L_IN2_GPIO_Port,
                MOTOR_L_IN2_Pin,
                GPIO_PIN_RESET);

            HAL_GPIO_WritePin(
                MOTOR_R_IN1_GPIO_Port,
                MOTOR_R_IN1_Pin,
                GPIO_PIN_SET);

            HAL_GPIO_WritePin(
                MOTOR_R_IN2_GPIO_Port,
                MOTOR_R_IN2_Pin,
                GPIO_PIN_RESET);

            break;


        case MOVE_DIRECTION_BACKWARD:

            HAL_GPIO_WritePin(
                MOTOR_L_IN1_GPIO_Port,
                MOTOR_L_IN1_Pin,
                GPIO_PIN_RESET);

            HAL_GPIO_WritePin(
                MOTOR_L_IN2_GPIO_Port,
                MOTOR_L_IN2_Pin,
                GPIO_PIN_SET);

            HAL_GPIO_WritePin(
                MOTOR_R_IN1_GPIO_Port,
                MOTOR_R_IN1_Pin,
                GPIO_PIN_RESET);

            HAL_GPIO_WritePin(
                MOTOR_R_IN2_GPIO_Port,
                MOTOR_R_IN2_Pin,
                GPIO_PIN_SET);

            break;

        case MOVE_DIRECTION_PIVOT_LEFT:

            HAL_GPIO_WritePin(
                MOTOR_L_IN1_GPIO_Port,
                MOTOR_L_IN1_Pin,
                GPIO_PIN_RESET);

            HAL_GPIO_WritePin(
                MOTOR_L_IN2_GPIO_Port,
                MOTOR_L_IN2_Pin,
                GPIO_PIN_SET);

            HAL_GPIO_WritePin(
                MOTOR_R_IN1_GPIO_Port,
                MOTOR_R_IN1_Pin,
                GPIO_PIN_SET);

            HAL_GPIO_WritePin(
                MOTOR_R_IN2_GPIO_Port,
                MOTOR_R_IN2_Pin,
                GPIO_PIN_RESET);

            break;


        case MOVE_DIRECTION_PIVOT_RIGHT:

            HAL_GPIO_WritePin(
                MOTOR_L_IN1_GPIO_Port,
                MOTOR_L_IN1_Pin,
                GPIO_PIN_SET);

            HAL_GPIO_WritePin(
                MOTOR_L_IN2_GPIO_Port,
                MOTOR_L_IN2_Pin,
                GPIO_PIN_RESET);

            HAL_GPIO_WritePin(
                MOTOR_R_IN1_GPIO_Port,
                MOTOR_R_IN1_Pin,
                GPIO_PIN_RESET);

            HAL_GPIO_WritePin(
                MOTOR_R_IN2_GPIO_Port,
                MOTOR_R_IN2_Pin,
                GPIO_PIN_SET);

            break;


        case MOVE_DIRECTION_STOP:
        default:

            PWM_Stop();
            Move_SetAllDirectionPinsLow();

            break;
    }
}



static bool Move_RequiresDeadTime(
        MoveDirection_t current_direction,
        MoveDirection_t new_direction)
{
    int8_t current_left;
    int8_t current_right;

    int8_t new_left;
    int8_t new_right;


    if ((current_direction == MOVE_DIRECTION_STOP) ||
        (new_direction == MOVE_DIRECTION_STOP))
    {
        return false;
    }


    current_left =
        Move_GetLeftPolarity(current_direction);

    current_right =
        Move_GetRightPolarity(current_direction);

    new_left =
        Move_GetLeftPolarity(new_direction);

    new_right =
        Move_GetRightPolarity(new_direction);


    if ((current_left != 0) &&
        (new_left != 0) &&
        (current_left != new_left))
    {
        return true;
    }

    if ((current_right != 0) &&
        (new_right != 0) &&
        (current_right != new_right))
    {
        return true;
    }

    return false;
}


static int8_t Move_GetLeftPolarity(
        MoveDirection_t direction)
{
    switch (direction)
    {
        case MOVE_DIRECTION_FORWARD:
        case MOVE_DIRECTION_LEFT:
        case MOVE_DIRECTION_RIGHT:
        case MOVE_DIRECTION_PIVOT_RIGHT:

            return 1;


        case MOVE_DIRECTION_BACKWARD:
        case MOVE_DIRECTION_PIVOT_LEFT:

            return -1;


        case MOVE_DIRECTION_STOP:
        default:

            return 0;
    }
}


static int8_t Move_GetRightPolarity(
        MoveDirection_t direction)
{
    switch (direction)
    {
        case MOVE_DIRECTION_FORWARD:
        case MOVE_DIRECTION_LEFT:
        case MOVE_DIRECTION_RIGHT:
        case MOVE_DIRECTION_PIVOT_LEFT:

            return 1;


        case MOVE_DIRECTION_BACKWARD:
        case MOVE_DIRECTION_PIVOT_RIGHT:

            return -1;


        case MOVE_DIRECTION_STOP:
        default:

            return 0;
    }
}



static void Move_ApplyManualPWM(void)
{
    uint8_t inner_speed;

    inner_speed = Move_GetTurnInnerSpeed();


    switch (s_direction)
    {
        case MOVE_DIRECTION_FORWARD:

            PWM_SetDuty(
                Move_GetForwardLeftSpeed(),
                s_speed_percent);

            break;


        case MOVE_DIRECTION_BACKWARD:

            PWM_SetDuty(
                s_speed_percent,
                s_speed_percent);

            break;


        case MOVE_DIRECTION_LEFT:

            PWM_SetDuty(
                inner_speed,
                s_speed_percent);

            break;


        case MOVE_DIRECTION_RIGHT:

            PWM_SetDuty(
                s_speed_percent,
                inner_speed);

            break;


        case MOVE_DIRECTION_PIVOT_LEFT:
        case MOVE_DIRECTION_PIVOT_RIGHT:

            PWM_SetDuty(
                s_speed_percent,
                s_speed_percent);

            break;


        case MOVE_DIRECTION_STOP:
        default:

            PWM_Stop();

            break;
    }
}



static uint8_t Move_GetTurnInnerSpeed(void)
{
    uint8_t inner_speed;


    if (s_speed_percent <=
        MOVE_TURN_MIN_INNER_SPEED)
    {
        return s_speed_percent;
    }


    if (s_speed_percent >
        MOVE_TURN_DELTA_PERCENT)
    {
        inner_speed =
            s_speed_percent -
            MOVE_TURN_DELTA_PERCENT;
    }
    else
    {
        inner_speed =
            s_speed_percent;
    }


    if (inner_speed <
        MOVE_TURN_MIN_INNER_SPEED)
    {
        inner_speed =
            MOVE_TURN_MIN_INNER_SPEED;
    }


    if (inner_speed >
        s_speed_percent)
    {
        inner_speed =
            s_speed_percent;
    }


    return inner_speed;
}


static uint8_t Move_GetForwardLeftSpeed(void)
{
    if (s_speed_percent >
        MOVE_FORWARD_LEFT_TRIM)
    {
        return (uint8_t)(
            s_speed_percent -
            MOVE_FORWARD_LEFT_TRIM);
    }

    return 0U;
}



static void Move_SetAllDirectionPinsLow(void)
{
    HAL_GPIO_WritePin(
        MOTOR_L_IN1_GPIO_Port,
        MOTOR_L_IN1_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        MOTOR_L_IN2_GPIO_Port,
        MOTOR_L_IN2_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        MOTOR_R_IN1_GPIO_Port,
        MOTOR_R_IN1_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        MOTOR_R_IN2_GPIO_Port,
        MOTOR_R_IN2_Pin,
        GPIO_PIN_RESET);
}
