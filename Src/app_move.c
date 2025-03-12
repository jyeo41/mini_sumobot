#include "app_move.h"
#include "trace.h"
#include "assert_handler.h"
#include "tb6612fng.h"
#include "ir_receiver.h"
#include <stdbool.h>

static bool move_initialized = false;

typedef struct {
    uint8_t motor_left;
    uint8_t motor_right;
}move_motor_speeds_t;

static const move_motor_speeds_t move_maneuvers[][5] = {
    [MOVE_DIRECTION_FORWARD] = {   
        [MOVE_SPEED_STOP] = {0, 0},
        [MOVE_SPEED_SLOW] = {25, 25},
        [MOVE_SPEED_MEDIUM] = {50, 50},
        [MOVE_SPEED_FAST] = {75, 75},
        [MOVE_SPEED_MAX] = {100, 100}
    },
    [MOVE_DIRECTION_REVERSE] = {
        [MOVE_SPEED_STOP] = {0, 0},
        [MOVE_SPEED_SLOW] = {25, 25},
        [MOVE_SPEED_MEDIUM] = {50, 50},
        [MOVE_SPEED_FAST] = {75, 75},
        [MOVE_SPEED_MAX] = {100, 100}
    },
    [MOVE_DIRECTION_ROTATE_LEFT] = {
        [MOVE_SPEED_STOP] = {0, 0},
        [MOVE_SPEED_SLOW] = {20, 20},
        [MOVE_SPEED_MEDIUM] = {40, 40},
        [MOVE_SPEED_FAST] = {60, 60},
        [MOVE_SPEED_MAX] = {100, 100}
    },
    [MOVE_DIRECTION_ROTATE_RIGHT] = {
        [MOVE_SPEED_STOP] = {0, 0},
        [MOVE_SPEED_SLOW] = {20, 20},
        [MOVE_SPEED_MEDIUM] = {40, 40},
        [MOVE_SPEED_FAST] = {60, 60},
        [MOVE_SPEED_MAX] = {100, 100}
    }
};

void move_initialize(void)
{
    ASSERT(!move_initialized);
    tb6612fng_initialize();
    move_initialized = true;
}

void move_test(void)
{
    ir_receiver_cmd_e ir_cmd = IR_RECEIVER_CMD_NONE;

    /* The setting of the motors should maintain until the application changes the direction or speed.
     * Need "static" keyword to achieve this. */
    static move_direction_e direction = MOVE_DIRECTION_FORWARD;
    static move_speed_e speed = MOVE_SPEED_STOP;

    ir_cmd = ir_receiver_get_cmd();

    switch(ir_cmd) {
        case IR_RECEIVER_CMD_0:
            speed = MOVE_SPEED_STOP;
            break;
        case IR_RECEIVER_CMD_1:
            speed = MOVE_SPEED_SLOW;
            break;
        case IR_RECEIVER_CMD_2:
            speed = MOVE_SPEED_MEDIUM;
            break;
        case IR_RECEIVER_CMD_3:
            speed = MOVE_SPEED_FAST;
            break;
        case IR_RECEIVER_CMD_4:
            speed = MOVE_SPEED_MAX;
            break;
        case IR_RECEIVER_CMD_5:
        case IR_RECEIVER_CMD_6:
        case IR_RECEIVER_CMD_7:
        case IR_RECEIVER_CMD_8:
        case IR_RECEIVER_CMD_9:
        case IR_RECEIVER_CMD_UP:
            direction = MOVE_DIRECTION_FORWARD;
            break;
        case IR_RECEIVER_CMD_DOWN:
            direction = MOVE_DIRECTION_REVERSE;
            break;
        case IR_RECEIVER_CMD_LEFT:
            direction = MOVE_DIRECTION_ROTATE_LEFT;
            break;
        case IR_RECEIVER_CMD_RIGHT:
            direction = MOVE_DIRECTION_ROTATE_RIGHT;
            break;
        case IR_RECEIVER_CMD_VOL_DECREASE:
        case IR_RECEIVER_CMD_VOL_INCREASE:
        case IR_RECEIVER_CMD_PLAY_PAUSE:
        case IR_RECEIVER_CMD_SETUP:
        case IR_RECEIVER_CMD_STOP_OR_MODE:
        case IR_RECEIVER_CMD_ENTER_OR_SAVE:
        case IR_RECEIVER_CMD_UNDO:
        case IR_RECEIVER_CMD_NONE:
        default:
            break;
    }
    move_set(direction, speed);
}

void move_set(move_direction_e direction, move_speed_e speed)
{
    uint8_t motor_speed_left = move_maneuvers[direction][speed].motor_left;
    uint8_t motor_speed_right = move_maneuvers[direction][speed].motor_right;

    tb6612fng_mode_e motor_mode_left;
    tb6612fng_mode_e motor_mode_right;

    switch(direction) {
        case MOVE_DIRECTION_FORWARD:
            motor_mode_left = TB6612FNG_MODE_FORWARD;
            motor_mode_right = TB6612FNG_MODE_FORWARD;
            break;
        case MOVE_DIRECTION_REVERSE:
            motor_mode_left = TB6612FNG_MODE_REVERSE;
            motor_mode_right = TB6612FNG_MODE_REVERSE;
            break;
        case MOVE_DIRECTION_ROTATE_LEFT:
            motor_mode_left = TB6612FNG_MODE_REVERSE;
            motor_mode_right = TB6612FNG_MODE_FORWARD;
            break;
        case MOVE_DIRECTION_ROTATE_RIGHT:
            motor_mode_left = TB6612FNG_MODE_FORWARD;
            motor_mode_right = TB6612FNG_MODE_REVERSE;
            break;
        default:
            break;
    }

    tb6612fng_mode_set(TB6612FNG_MOTOR_LEFT, motor_mode_left);
    tb6612fng_mode_set(TB6612FNG_MOTOR_RIGHT, motor_mode_right);
    tb6612fng_dutycycle_set(TB6612FNG_MOTOR_LEFT, motor_speed_left);
    tb6612fng_dutycycle_set(TB6612FNG_MOTOR_RIGHT, motor_speed_right);
}
