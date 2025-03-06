#ifndef PWM_H_
#define PWM_H_
#include <stdint.h>

typedef enum {
    PWM_CHANNEL1,
    PWM_CHANNEL2,
}pwm_e;

void pwm_initialize(void);
void pwm_duty_cycle_set(pwm_e pwm, uint8_t duty_cycle);
void pwm_test(void);

#endif  /* PWM_H_ */
