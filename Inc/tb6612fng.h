#ifndef TB6612FNG_H_
#define TB6612FNG_H_

#include <stdint.h>

/* Motor direction mode. Clockwise/counterclockwise. */
typedef enum {
    TB6612FNG_MODE_STOP,
    TB6612FNG_MODE_FORWARD,
    TB6612FNG_MODE_REVERSE
}tb6612fng_mode_e;

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
void tb6612fng_mode_set(tb6612fng_motor_e motor, tb6612fng_mode_e mode);
void tb6612fng_dutycycle_set(tb6612fng_motor_e motor, uint8_t duty_cycle);

#endif /* TB6612FNG_H_ */
