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

    gpio_configure_pin(IR_RECEIVER, GPIO_MODE_INPUT, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);
    gpio_interrupt_set(IR_RECEIVER, GPIO_INTERRUPT_TRIGGER_FALLING, 6, 2);

    initialized = true;
}

ir_receiver_cmd_e ir_receiver_get_cmd(void);

// cppcheck-suppress unusedFunction
void EXTI0_IRQHandler(void)
{
    led_toggle(LED_ORANGE);
    EXTI->PR |= (1 << 0);
}
