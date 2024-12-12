#include "i2c.h"
#include "gpio.h"
#include "assert_handler.h"
#include <stm32f4xx.h>
#include <stdbool.h>

#define APB_CLOCK                               16000000
#define APB_CLOCK_IN_MHZ                        (APB_CLOCK / 1000000)
#define I2C_STANDARD_MODE_SCL                   100000
#define I2C_DEVICE_ADDRESS                      0x29

/* I2C Bus Specs says max TRise time for standard mode is 1000ns or 1us. Convert it to hertz */
#define I2C_STANDARD_MODE_SCL_MAX_TRISE_IN_HZ   1000000


static bool initialized = false;

static uint32_t i2c_ccr_calculate(const uint32_t apb_clock, const uint32_t desired_scl_clock);

#if 0
/* Generate start condition and poll until start flag is set */
static inline void i2c_start_condition(void)
{
    I2C2->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
    while (!(I2C2->SR1 & I2C_SR1_SB));
}

/* Send the device address and a 0 or 1 to signify write/read and poll until ADDR flag is set.
 * Must read SR1 and SR2 to clear the ADDR flag by reading SR1 followed by SR2. */
static void i2c_device_addr_send(uint8_t device_addr, uint8_t write_read)
{
    uint16_t register_read;

    /* RM Section 27.3.3 "Start Condition" Says:
     *  Master waits for a read of the SR1 register followed by a write in the DR register with the slave address. */
    //register_read = I2C2->SR1;
    I2C2->DR |= ((device_addr << 1) | write_read);
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    register_read = I2C2->SR1 | I2C2->SR2;
    (void)register_read;
}

static void i2c_memory_addr_send(const uint8_t memory_addr) 
{
    /* Reference Manual Figure 244 "transfer sequence diagram for master receiver" shows to write the data to DR register first,
     * then wait until the TXE flag is set.
     *
     * Section 27.3.3 I2C */
    #if 0
    do {
        I2C2->DR |= memory_addr[memory_addr_size--];
    } while (!(I2C2->SR1 & I2C_SR1_TXE) && (memory_addr_size > 0));
    #endif
    I2C2->DR |= memory_addr;
    while (!(I2C2->SR1 & I2C_SR1_TXE));
}
#endif

void i2c_read(const uint8_t device_addr, const uint8_t memory_addr, uint8_t* data, uint8_t data_size)
{
    #if 0
    /* Master address phase */
    i2c_start_condition();
    i2c_device_addr_send(device_addr, 0);
    i2c_memory_addr_send(memory_addr);

    /* Master receiver phase */
    i2c_start_condition();
    i2c_device_addr_send(device_addr, 1);

    /* In the master receiver phase, after you send the device address again the ACK bit 
     *  must be cleared if its only 1 byte, or set if its multiple bytes. */
    if (data_size > 1) {
        I2C2->CR1 |= I2C_CR1_ACK;
    } else {
        I2C2->CR1 &= ~(I2C_CR1_ACK);
    }

    while (data_size > 0) {
        if (data_size == 1) {
            I2C2->CR1 &= ~(I2C_CR1_ACK);
            I2C2->CR1 |= I2C_CR1_STOP;
            while (!(I2C2->SR1 & I2C_SR1_RXNE)) {}
            *data++ = I2C2->DR;
        } else {
            while (!(I2C2->SR1 & I2C_SR1_RXNE)) {}
            *data++ = I2C2->DR;
        }
        data_size--;
    }
    #endif
    
    uint16_t register_read;

    /* Most basic implementation */
    /* Send the start condition then wait until its acknowledged. */
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));

    /* Send the device address then wait until its acknowledged */
    I2C2->DR = (device_addr << 1);
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    register_read = I2C2->SR1 | I2C2->SR2;

    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->DR = memory_addr;
    while (!(I2C2->SR1 & I2C_SR1_BTF));


    /* Phase 2 */
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    I2C2->DR = ((device_addr << 1) | 1);
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    register_read = I2C2->SR1 | I2C2->SR2;
    (void)register_read;
    
    for (uint8_t i = 0; i < data_size; i++) {
        if (i + 1 == data_size) {
            I2C2->CR1 &= ~I2C_CR1_ACK;
            I2C2->CR1 |= I2C_CR1_STOP;
        }
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        data[i] = I2C2->DR;
    }
}

#if 0
/* Device address is the slave's address. */
i2c_return_error_e i2c_write(const uint8_t device_addr, const uint8_t* memory_addr, uint8_t memory_addr_size,
				   uint8_t* data, uint8_t data_size)
{
    i2c_start_condition();
    i2c_device_addr_send(device_addr, 0);
    i2c_
    

}
#endif

/* Reference Manual Chapter 27 I2C section 27.3.3 "I2C Master Mode" */
void i2c_initialize(void)
{
    /* 1. Enable I2C and GPIO clock.
     * 2. Configure I2C pins for alternate function.
     *      - Set AF in MODER
     *      - Set Open Drain (requirement for I2C pins) in OTYPER
     *      - Set internal pull ups for both pins
     *      - Set AFx in AFR register, check which one it is in datasheet.
     * 3. Fully reset the I2C peripheral
     * 4. Set the peripheral input clock from APB1 bus
     * 5. Configure SM/FM and set the frequency of the SCL line
     * 6. Configure maximum Trise for SCL line
     * 7. Enable the peripheral
     */
    ASSERT(!initialized);
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    gpio_configure_pin(I2C2_SCL, GPIO_MODE_ALTERNATE, GPIO_AF4, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_OPENDRAIN);
    gpio_configure_pin(I2C2_SDA, GPIO_MODE_ALTERNATE, GPIO_AF4, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_OPENDRAIN);
    ASSERT(gpio_config_compare(I2C2_SCL, GPIOB, 10, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_OPENDRAIN));
    ASSERT(gpio_config_compare(I2C2_SDA, GPIOB, 11, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_OPENDRAIN));

    /* I2C reset */
    I2C2->CR1 |= I2C_CR1_SWRST;
    I2C2->CR1 &= ~(I2C_CR1_SWRST);

    /* Set APB clock frequency in FREQ. Must be between 2MHz and 50 MHz. */
    I2C2->CR2 |= (APB_CLOCK_IN_MHZ << I2C_CR2_FREQ_Pos);

    /* Configure I2C_CCR bit fields.
     *  Clear FS bit for standard mode
     *  Calculate CCR value for standard mode */
    I2C2->CCR &= ~(I2C_CCR_FS);
    I2C2->CCR |= i2c_ccr_calculate(APB_CLOCK, I2C_STANDARD_MODE_SCL);

    /* Configure the Trise register. Formula is ((Trise_max_scl / Tpckl1) + 1).
     * Make sure its a 6 bit field. */
    I2C2->TRISE = (((APB_CLOCK / I2C_STANDARD_MODE_SCL_MAX_TRISE_IN_HZ) + 1) & 0x3F);

    /* Enable the peripheral */
    I2C2->CR1 |= I2C_CR1_PE;

    initialized = true;
}

/* Should handle both cases of Standard and Fast mode, but to keep the function within the scope of my project,
 *  only implement the standard mode calculation. 
 * 
 * Formula for calculating CCR is in section 27.6.8 where pclk1 is the APB1 clock
 *  Thigh = CCR * Tpclk1
 *  Tlow = CCR * Tpclk1
 *  Tscl = Thigh + Tlow     (the high + low part of the signal means its the full SCL) 
 *  Tscl = 2CCR * Tpclk1
 *  CCR = Tscl / (2 * Tpclk1)
 */
static uint32_t i2c_ccr_calculate(const uint32_t apb_clock, const uint32_t desired_scl_clock)
{
    uint32_t ccr_return = 0;
    
    /* This formula is using HZ instead of T so the apb_clock and desired_scl_clock are the inverse of Tscl and Tpclk1 */
    ccr_return = apb_clock / (2 * desired_scl_clock);

    /* Should return a 12 bit value */
    return ccr_return & 0xFFF;
}

