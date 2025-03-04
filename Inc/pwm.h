#ifndef PWM_H_
#define PWM_H_
#include <stdint.h>

typedef enum {
    PWM_TB6612FNG_MOTOR_LEFT,
    PWM_TB6612FNG_MOTOR_RIGHT,
}pwm_e;

void pwm_initialize(void);
void pwm_duty_cycle_set(pwm_e pwm, uint8_t duty_cycle);

#endif  /* PWM_H_ */
