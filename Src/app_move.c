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
                                }
};

// cppcheck-suppress unusedFunction
void move_initialize(void)
{
    ASSERT(!move_initialized);
    tb6612fng_initialize();
    move_initialized = true;
}

// cppcheck-suppress unusedFunction
void move_test(void)
{

}

// cppcheck-suppress unusedFunction
void move_set(move_direction_e direction, move_speed_e speed)
{
    switch (direction) {
        case MOVE_DIRECTION_FORWARD:
            tb6612fng_mode_set(TB6612FNG_MOTOR_LEFT, TB6612FNG_MODE_FORWARD);
            tb6612fng_mode_set(TB6612FNG_MOTOR_RIGHT, TB6612FNG_MODE_FORWARD);
            tb6612fng_dutycycle_set(TB6612FNG_MOTOR_LEFT, move_maneuvers[direction][speed].motor_left);
            tb6612fng_dutycycle_set(TB6612FNG_MOTOR_RIGHT, move_maneuvers[direction][speed].motor_right);
            break;
        case MOVE_DIRECTION_REVERSE:
            break;
        case MOVE_DIRECTION_ROTATE_LEFT:
            break;
        case MOVE_DIRECTION_ROTATE_RIGHT:
            break;
    }
}
