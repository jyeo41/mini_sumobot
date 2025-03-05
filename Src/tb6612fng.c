#include <stdbool.h>
#include "tb6612fng.h"
#include "pwm.h"
#include "assert_handler.h"
#include "trace.h"
#include "gpio.h"
#include "systick.h"

static bool tb6612fng_initialized = false;

void tb6612fng_initialize(void)
{
    ASSERT(!tb6612fng_initialized);
    pwm_initialize();

    /* Configure the motor driver control pins. */
    gpio_configure_pin(TB6612FNG_MOTOR_LEFT_IN1, GPIO_MODE_OUTPUT, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED, 
                       GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM);
    ASSERT(gpio_config_compare(TB6612FNG_MOTOR_LEFT_IN1, GPIOC, 8, GPIO_MODE_OUTPUT,
                               GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM));

    gpio_configure_pin(TB6612FNG_MOTOR_LEFT_IN2, GPIO_MODE_OUTPUT, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED, 
                       GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM);
    ASSERT(gpio_config_compare(TB6612FNG_MOTOR_LEFT_IN2, GPIOC, 9, GPIO_MODE_OUTPUT,
                               GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM));
    tb6612fng_initialized = true;
}

void tb6612fng_test(void)
{
    const uint8_t speeds[] = {100, 75, 50, 25};
    const tb6612fng_direction_e directions[] = {
        TB6612FNG_DIRECTION_FORWARD,
        TB6612FNG_DIRECTION_REVERSE,
        TB6612FNG_DIRECTION_FORWARD,
        TB6612FNG_DIRECTION_REVERSE,
    };
    const uint8_t length = sizeof(speeds)/sizeof(speeds[0]);

    for (uint8_t i = 0; i < length; i++) {
        TRACE("TB6612FNG direction: %d and speed: %d\n", directions[i], speeds[i]);
        tb6612fng_direction(TB6612FNG_MOTOR_LEFT, directions[i]);
        tb6612fng_speed(TB6612FNG_MOTOR_LEFT, speeds[i]);
        systick_delay_ms(2000);

        
        tb6612fng_direction(TB6612FNG_MOTOR_LEFT, TB6612FNG_DIRECTION_STOP);
        tb6612fng_speed(TB6612FNG_MOTOR_LEFT, 0);
        systick_delay_ms(1000);
    }
}

void tb6612fng_direction(tb6612fng_motor_e motor, tb6612fng_direction_e direction)
{
    /* Datasheet for the breakboard gives the truth table for the following directions */
    if (motor == TB6612FNG_MOTOR_LEFT) {
        switch (direction) {
            case TB6612FNG_DIRECTION_FORWARD:
                gpio_data_output_set(TB6612FNG_MOTOR_LEFT_IN1);
                gpio_data_output_clear(TB6612FNG_MOTOR_LEFT_IN2);
                break;
            case TB6612FNG_DIRECTION_REVERSE:
                gpio_data_output_set(TB6612FNG_MOTOR_LEFT_IN2);
                gpio_data_output_clear(TB6612FNG_MOTOR_LEFT_IN1);
                break;
            case TB6612FNG_DIRECTION_STOP:
                gpio_data_output_clear(TB6612FNG_MOTOR_LEFT_IN1);
                gpio_data_output_clear(TB6612FNG_MOTOR_LEFT_IN2);
                break;
        }
    } else {
        switch (direction) {
            case TB6612FNG_DIRECTION_FORWARD:
                gpio_data_output_set(TB6612FNG_MOTOR_RIGHT_IN1);
                gpio_data_output_clear(TB6612FNG_MOTOR_RIGHT_IN2);
                break;
            case TB6612FNG_DIRECTION_REVERSE:
                gpio_data_output_set(TB6612FNG_MOTOR_RIGHT_IN2);
                gpio_data_output_clear(TB6612FNG_MOTOR_RIGHT_IN1);
                break;
            case TB6612FNG_DIRECTION_STOP:
                gpio_data_output_clear(TB6612FNG_MOTOR_RIGHT_IN1);
                gpio_data_output_clear(TB6612FNG_MOTOR_RIGHT_IN2);
                break;
        }
    }
}

void tb6612fng_speed(tb6612fng_motor_e motor, uint8_t duty_cycle)
{
    pwm_e pwm_channel;

    switch (motor) {
        case TB6612FNG_MOTOR_LEFT:
            pwm_channel = PWM_CHANNEL1;
            break;
        case TB6612FNG_MOTOR_RIGHT:
            pwm_channel = PWM_CHANNEL2;
            break;
    }
    pwm_duty_cycle_set(pwm_channel, duty_cycle);
}
