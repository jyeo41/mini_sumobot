#include <stm32f4xx.h>
#include <stdbool.h>
#include "pwm.h"
#include "assert_handler.h"
#include "trace.h"
#include "gpio.h"
#include "systick.h"

/* 16000 prescalers results in a 1kHz frequency for the counter. */
#define TIM3_PRESCALER      16000
#define ARR_PERIOD          100
#define DEFAULT_DUTY_CYCLE  0

typedef struct {
    /* CCRx register is where you set the duty cycle for a given PWM channel. */
    volatile uint32_t *const ccr;
}pwm_channel_t;

static pwm_channel_t pwm_channels[] = {
    [PWM_TB6612FNG_MOTOR_LEFT] = {.ccr = &TIM3->CCR1},
    [PWM_TB6612FNG_MOTOR_RIGHT] = {.ccr = &TIM3->CCR2},
};

static bool pwm_initialized = false;

/* Using Timer3 Channels 1 and 2.
 * GPIO Port C pins 6 and 7. */
void pwm_initialize(void)
{
    ASSERT(!pwm_initialized);
    
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Set PC6 and PC7 to Alt Function Mode for TIM3 (AF2) */
    gpio_configure_pin(PWM_LEFT_TIM3_CHANNEL1, GPIO_MODE_ALTERNATE, GPIO_AF2, GPIO_RESISTOR_DISABLED, 
                       GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM);
    ASSERT(gpio_config_compare(PWM_LEFT_TIM3_CHANNEL1, GPIOC, 6, GPIO_MODE_ALTERNATE,
                               GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM));
    gpio_configure_pin(PWM_RIGHT_TIM3_CHANNEL2, GPIO_MODE_ALTERNATE, GPIO_AF2, GPIO_RESISTOR_DISABLED, 
                       GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM);
    ASSERT(gpio_config_compare(PWM_RIGHT_TIM3_CHANNEL2, GPIOC, 7, GPIO_MODE_ALTERNATE,
                               GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM));

    /* Configure the output pin:
     *      1. Select the output mode by writing CCS bits in CCMRx register.
     *      2. Select the polarity by writing the CCxP bit in CCER register.
     * Select the PWM mode (PWM1 or PWM2) by writing OCxM bits in CCMRx register.
     * Program the period and the duty cycle in ARR and CCRx register.
     * Set the preload bit in CCMRx register and the ARPE bit in CR1 register
     * Select the counting mode (edge aligned as up or down counting or center aligned)
     * Enable the capture compare
     * Enable the counter. */

    /* Set channels 1 and 2 to output mode. */
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S);
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S);

    /* Set channel 1 and 2 to ACTIVE HIGH output. This means duty cycle is active from the start of the counter
     * until it hits the CCR value, then its low until it hits the ARR value. */
    TIM3->CCER &= ~(TIM_CCER_CC1P);
    TIM3->CCER &= ~(TIM_CCER_CC2P);

    /* Select PWM mode 1 (binary 110). */
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);

    /* Set the prescaler, period, and duty cycle. */
    TIM3->PSC = TIM3_PRESCALER - 1;
    TIM3->ARR = ARR_PERIOD - 1;
    TIM3->CCR1 = DEFAULT_DUTY_CYCLE;
    TIM3->CCR2 = 25;

    /* Set the preload bit and ARPE bit. */
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM3->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM3->CR1 |= TIM_CR1_ARPE;

    /* Set to edge aligned, up counting mode. */
    TIM3->CR1 &= ~(TIM_CR1_CMS);
    TIM3->CR1 &= ~(TIM_CR1_DIR);

    /* Enable capture/compare and the counter. */
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CCER |= TIM_CCER_CC2E;
    TIM3->CR1 |= TIM_CR1_CEN;
    
    pwm_initialized = true;
}

void pwm_test(void)
{
    uint8_t i, j;

    for (i = 0; i <= 100; i += 25) {
        TRACE("TB6612FNG Left Motor: Duty Cycle %u\n", i);
        pwm_duty_cycle_set(PWM_TB6612FNG_MOTOR_LEFT, i);
        systick_delay_ms(1500);
    }
    TRACE("\n");

    for (j = 0; j <= 100; j += 25) {
        TRACE("TB6612FNG Right Motor: Duty Cycle %u\n", j);
        pwm_duty_cycle_set(PWM_TB6612FNG_MOTOR_RIGHT, j);
        systick_delay_ms(1500);
    }
    TRACE("\n");
}

void pwm_duty_cycle_set(pwm_e pwm, uint8_t duty_cycle)
{
    ASSERT(duty_cycle <= 100);
    *pwm_channels[pwm].ccr = duty_cycle;
}
