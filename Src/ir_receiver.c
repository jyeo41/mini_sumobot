#include "ir_receiver.h"
#include "gpio.h"
#include "assert_handler.h"
#include "led.h"
#include <stdbool.h>

static bool initialized = false;

void ir_receiver_initialize(void)
{
    ASSERT(!initialized);
    
    /* Setting the IR Receiver for pin PB0, GPIO Falling Edge Interrupt trigger */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    gpio_mode_set(IR_RECEIVER, GPIO_MODE_INPUT);
    gpio_interrupt_set(IR_RECEIVER, GPIO_INTERRUPT_TRIGGER_FALLING, 6, 2);

    initialized = true;
}

void gpio_interrupt_set(gpio_pin_names_e pin,
			gpio_interrupt_edge_trigger_e edge_trigger, 
			IRQn_Type IRQn,
			uint32_t priority);
ir_receiver_cmd_e ir_receiver_get_cmd(void);

// cppcheck-suppress unusedFunction
void EXTI0_IRQHandler(void)
{
    led_toggle(LED_ORANGE);
    EXTI->PR |= (1 << 0);
}
