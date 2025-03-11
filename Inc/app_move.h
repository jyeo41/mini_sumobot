#ifndef APP_MOVE_H_
#define APP_MOVE_H_

/* Enum for the different kind of movements the motors should be able to do. */
typedef enum {
    MOVE_DIRECTION_FORWARD,
    MOVE_DIRECTION_REVERSE,
    MOVE_DIRECTION_ROTATE_LEFT,
    MOVE_DIRECTION_ROTATE_RIGHT
}move_direction_e;

/* Enum for the different speed thresholds of the motors. */
typedef enum {
    MOVE_SPEED_STOP,
    MOVE_SPEED_SLOW,
    MOVE_SPEED_MEDIUM,
    MOVE_SPEED_FAST,
    MOVE_SPEED_MAX
}move_speed_e;

/* Initialize function. */
void move_initialize(void);
/* Function to set the movement type and speed of the motors. */
void move_set(move_direction_e direction, move_speed_e speed);
/* Test function. */
void move_test(void);

#endif /* APP_MOVE_H_ */
