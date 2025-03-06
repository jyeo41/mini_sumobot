#ifndef TB6612FNG_H_
#define TB6612FNG_H_

#include <stdint.h>

/* Motor directions*/
typedef enum {
    TB6612FNG_DIRECTION_STOP,
    TB6612FNG_DIRECTION_FORWARD,
    TB6612FNG_DIRECTION_REVERSE
}tb6612fng_direction_e;

/* The orientation of the motors will be opposite of each other 
 * which means I need to be able to control them independently between them
 * when setting the direction/speed
 */
typedef enum {
    TB6612FNG_MOTOR_LEFT,
    TB6612FNG_MOTOR_RIGHT
}tb6612fng_motor_e;

void tb6612fng_initialize(void);
void tb6612fng_test(void);
void tb6612fng_direction(tb6612fng_motor_e motor, tb6612fng_direction_e direction);
void tb6612fng_speed(tb6612fng_motor_e motor, uint8_t duty_cycle);

#endif /* TB6612FNG_H_ */
