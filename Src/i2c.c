#include "i2c.h"
#include "gpio.h"
#include "assert_handler.h"
#include "trace.h"
#include "systick.h"
#include <stm32f4xx.h>
#include <stdbool.h>

#define APB_CLOCK                               16000000
#define APB_CLOCK_IN_MHZ                        (APB_CLOCK / 1000000)
#define I2C_STANDARD_MODE_SCL                   100000
#define I2C_VL53L0X_DEVICE_ADDRESS                     0x29
#define I2C_VL53L0X_SENSOR_ID                   0xEE
#define I2C_VL53L0X_SENSOR_ID_REGISTER          0xC0
#define I2C_VL53L0X_WRITE_REGISTER              0x01

/* I2C Bus Specs says max TRise time for standard mode is 1000ns or 1us. Convert it to hertz */
#define I2C_STANDARD_MODE_SCL_MAX_TRISE_IN_HZ   1000000


static bool initialized = false;

static uint32_t i2c_ccr_calculate(const uint32_t apb_clock, const uint32_t desired_scl_clock);
static void i2c_start_condition(void);
static void i2c_stop_condition(void);

static void i2c_device_addr_send(uint8_t device_addr);
static void i2c_data_send(uint8_t* data, uint8_t data_size);


void i2c_read(const uint8_t device_addr, const uint8_t memory_addr, uint8_t* data, uint8_t data_size)
{
    uint16_t register_read;

    /* Most basic implementation */
    /* Send the start condition then wait until its acknowledged. */
    i2c_start_condition();

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
    // cppcheck-suppress redundantAssignment
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

/* Device address is the slave's address. */
void i2c_write(const uint8_t device_addr, const uint8_t memory_addr, uint8_t* data, uint8_t data_size)
{
    /* Send the start condition then wait until its acknowledged. */
    i2c_start_condition();
    i2c_device_addr_send(device_addr);

    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->DR = memory_addr;
    while (!(I2C2->SR1 & I2C_SR1_BTF));
    
    i2c_data_send(data, data_size);
    i2c_stop_condition();
}

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

    gpio_configure_pin(I2C2_SCL, GPIO_MODE_ALTERNATE, GPIO_AF4, GPIO_RESISTOR_DISABLED,
                       GPIO_OTYPE_OPENDRAIN, GPIO_SPEED_HIGH);
    gpio_configure_pin(I2C2_SDA, GPIO_MODE_ALTERNATE, GPIO_AF4, GPIO_RESISTOR_DISABLED,
                       GPIO_OTYPE_OPENDRAIN, GPIO_SPEED_HIGH);
    ASSERT(gpio_config_compare(I2C2_SCL, GPIOB, 10, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED,
                               GPIO_OTYPE_OPENDRAIN, GPIO_SPEED_HIGH));
    ASSERT(gpio_config_compare(I2C2_SDA, GPIOB, 11, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED,
                               GPIO_OTYPE_OPENDRAIN, GPIO_SPEED_HIGH));

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

/* Standalone test function to call inside of main, testing to make sure the I2C read and write functions work properly */
void i2c_test_read_write(void)
{
    uint8_t read_i2c[4] = {0, 0, 0, 0};
    /* Test value to write to a register in the sensor. */
    uint32_t write_i2c = 0xFF;

    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_SENSOR_ID_REGISTER, read_i2c, 1);
    if (read_i2c[0] == I2C_VL53L0X_SENSOR_ID) {
            TRACE("Read expected sensor ID (0xEE) from VL53L0X\n");
    } else {
            TRACE("Read UNexpected sensor ID 0x%X from VL53L0X, expected (0xEE)\n", read_i2c[0]);
    }

    /* "0x01" is one of the registers you can write to for the VL53L0X sensor. */
    i2c_write(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&write_i2c, 1);
    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, read_i2c, 1);
    if (read_i2c[0] == write_i2c) {
            TRACE("Wrote expected VL53L0x value 0x%X\n", write_i2c);
    } else {
            TRACE("Wrote unexpected value. Read 0x%X instead of 0x%X\n", read_i2c[0], write_i2c);
    }
    systick_delay_ms(1000);
    
    #if 0
    write_i2c = 0xABCD;
    i2c_write(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&write_i2c, 2);
    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&read_i2c, 2);
    TRACE("Writing 0xABCD, Read 0x%X\n\n", read_i2c);
    #endif
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

static void i2c_start_condition(void)
{
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
}

/* bit 8 of the device_addr will always be 0 since we have to write the address from master. */
static void i2c_device_addr_send(uint8_t device_addr)
{
    uint16_t register_read;
    
    I2C2->DR = (device_addr << 1);
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    register_read = I2C2->SR1 | I2C2->SR2;
    (void)register_read;
}

/* Can send a variable number of bytes, must be at least 1 byte*/
static void i2c_data_send(uint8_t* data, uint8_t data_size)
{
    for (uint8_t i = 0; i < data_size; i++) {
        while (!(I2C2->SR1 & I2C_SR1_TXE));
        I2C2->DR |= data[i];
    }
    /* Once last byte of data has been sent, need to wait for BTF to set. */
    while (!(I2C2->SR1 & I2C_SR1_BTF));

}

static void i2c_stop_condition(void)
{
    I2C2->CR1 |= I2C_CR1_STOP;
}
