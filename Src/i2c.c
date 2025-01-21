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
#define I2C_VL53L0X_DEVICE_ADDRESS              0x29
#define I2C_VL53L0X_SENSOR_ID                   0xEE
#define I2C_VL53L0X_SENSOR_ID_REGISTER          0xC0
#define I2C_VL53L0X_WRITE_REGISTER              0x01
#define TIMEOUT_COUNT                           UINT16_MAX

/* I2C Bus Specs says max TRise time for standard mode is 1000ns or 1us. Convert it to hertz */
#define I2C_STANDARD_MODE_SCL_MAX_TRISE_IN_HZ   1000000

typedef union {
    uint8_t byte;
    uint16_t half_word;
    uint32_t word;
}i2c_data;


static bool initialized = false;

static uint32_t i2c_ccr_calculate(const uint32_t apb_clock, const uint32_t desired_scl_clock);
static i2c_return_error_e i2c_start_condition(void);
static inline void i2c_stop_condition(void);

static i2c_return_error_e i2c_device_addr_send(uint8_t device_addr, uint8_t read_write);
static i2c_return_error_e i2c_device_addr_send_1byte_rx(uint8_t device_addr);
static i2c_return_error_e i2c_data_send(const uint8_t* data, const uint8_t data_size);


i2c_return_error_e i2c_read(const uint8_t device_addr, const uint8_t memory_addr, uint8_t* read_buffer, const uint8_t data_size)
{
    uint8_t remaining_data = data_size;
    i2c_return_error_e error = I2C_RETURN_OK;

    error = i2c_start_condition();
    if (error != I2C_RETURN_OK) {
        TRACE("I2C-Read START ERROR: %u\n", error);
        return error;
    }
    error = i2c_device_addr_send(device_addr, 0);
    if (error != I2C_RETURN_OK) {
        TRACE("I2C-Read INITIAL DEVICE ADDR SEND ERROR: %u\n", error);
        return error;
    }
    error = i2c_data_send(&memory_addr, 1);
    if (error != I2C_RETURN_OK) {
        TRACE("I2C-Read MEMORY ADDR SEND ERROR: %u\n", error);
        return error;
    }
    /* Repeated start condition to signal to read from the previous device and memory address sent. */
    error = i2c_start_condition();
    if (error != I2C_RETURN_OK) {
        TRACE("I2C-Read REPEAT START ERROR: %u\n", error);
        return error;
    }

    /* Special case for reading 1 byte. */
    if (data_size == 1) {
        error = i2c_device_addr_send_1byte_rx(device_addr);
        if (error != I2C_RETURN_OK) {
            TRACE("I2C-Read 1BYTE DEVICE ADDR SEND ERROR: %u\n", error);
            return error;
        }
        /* Wait until the DR has some data to be read inside of it. */
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        read_buffer[data_size - remaining_data] = I2C2->DR;

    /* Case when reading multiple bytes. */
    } else {
        /* 1 indicates read bit. */
        error = i2c_device_addr_send(device_addr, 1);
        if (error != I2C_RETURN_OK) {
            TRACE("I2C-Read MULTIBYTE DEVICE ADDR SEND ERROR: %u\n", error);
            return error;
        }
        while (remaining_data > 2) {
            while (!(I2C2->SR1 & I2C_SR1_RXNE));
            read_buffer[data_size - remaining_data] = I2C2->DR;
            I2C2->CR1 |= I2C_CR1_ACK;
            remaining_data--;
        }
        /* Now receive the second to last byte. 
         * Special case in multi-byte reading where you must send a NACK and STOP condition when reading
         *  second to last byte. */
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        read_buffer[data_size - remaining_data] = I2C2->DR;
        I2C2->CR1 &= ~(I2C_CR1_ACK);
        i2c_stop_condition();
        remaining_data--;

        /* Now read the last byte. */
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        read_buffer[data_size - remaining_data] = I2C2->DR;
    }
    return error;
}

/* Device address is the slave's address.
 * Running into an issue where bus seems to hang if master tries to read immediately after a write because
 * of the stop condition being sent by the write function. I presume this is because the stop condition is
 * being sent when it should be a repeated start.
 *
 * The repeated start flag should be set to true if an i2c_read() follows immediately after to the same slave device. */
i2c_return_error_e i2c_write(const uint8_t device_addr, const uint8_t memory_addr, const uint8_t* data, const uint8_t data_size)
{
    i2c_return_error_e error = I2C_RETURN_OK;

    /* Send the start condition then wait until its acknowledged. */
    error = i2c_start_condition();
    if (error != I2C_RETURN_OK) {
        TRACE("I2C-Write START ERROR: %u\n", error);
        return error;
    }
    error = i2c_device_addr_send(device_addr, 0);
    if (error != I2C_RETURN_OK) {
        TRACE("I2C-Write DEVICE ADDR SEND: %u\n", error);
        return error;
    }
    error = i2c_data_send(&memory_addr, 1);
    if (error != I2C_RETURN_OK) {
        TRACE("I2C-Write MEMORY ADDR SEND ERROR: %u\n", error);
        return error;
    }
    error = i2c_data_send(data, data_size);
    if (error != I2C_RETURN_OK) {
        TRACE("I2C-Write DATA SEND ERROR: %u\n", error);
        return error;
    }

    i2c_stop_condition();
    return error;
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

    gpio_configure_pin(I2C2_SCL, GPIO_MODE_ALTERNATE, GPIO_AF4, GPIO_RESISTOR_DISABLED,
                       GPIO_OTYPE_OPENDRAIN, GPIO_SPEED_LOW);
    gpio_configure_pin(I2C2_SDA, GPIO_MODE_ALTERNATE, GPIO_AF4, GPIO_RESISTOR_DISABLED,
                       GPIO_OTYPE_OPENDRAIN, GPIO_SPEED_LOW);
    ASSERT(gpio_config_compare(I2C2_SCL, GPIOB, 10, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED,
                               GPIO_OTYPE_OPENDRAIN, GPIO_SPEED_LOW));
    ASSERT(gpio_config_compare(I2C2_SDA, GPIOB, 11, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED,
                               GPIO_OTYPE_OPENDRAIN, GPIO_SPEED_LOW));

    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
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
    i2c_data data_read;

    /* Test value to write to a register in the sensor. */
    uint32_t write_i2c = 0xFF;
    uint32_t write_byte = 0xCC;

    TRACE("!!STARTING I2C TESTING!!\n");
    /* Test single byte write and read separately. */
    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_SENSOR_ID_REGISTER, (uint8_t*)&data_read.byte, 1);
    if (data_read.byte == I2C_VL53L0X_SENSOR_ID) {
            TRACE("Read expected sensor ID (0xEE) from VL53L0X\n");
    } else {
            TRACE("Read UNexpected sensor ID 0x%X from VL53L0X, expected (0xEE)\n", data_read.byte);
    }
    i2c_write(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&write_byte, 1);
    TRACE("Wrote value 0x%X to VL53L0X register 0x%X\n", write_byte, I2C_VL53L0X_WRITE_REGISTER);
    
    systick_delay_ms(5);
    /* Test sequential single-byte write and read. */
    i2c_write(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&write_i2c, 1);
    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&data_read.byte, 1);
    if (data_read.byte == write_i2c) {
            TRACE("Wrote expected VL53L0x value 0x%X\n\n", write_i2c);
    } else {
            TRACE("Wrote unexpected value. Read 0x%X instead of 0x%X\n\n", data_read.byte, write_i2c);
    }

    /* Test 2-byte write and read. */
    write_i2c = 0xEEEE;
    
    systick_delay_ms(10);
    TRACE("Testing 2-BYTE READ AND WRITE\n");
    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&data_read.half_word, 2);
    TRACE("Read 2-byte value 0x%X\n", data_read.half_word);
    i2c_write(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&write_i2c, 2);
    TRACE("Wrote 2-byte value 0x%X\n", write_i2c);
    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&data_read.half_word, 2);
    TRACE("Read 2-byte value 0x%X\n\n", data_read.half_word);

    /* Small delay so UART can print in time. */
    systick_delay_ms(5);
    /* Test 4-byte write and read. */
    write_i2c = 0xBBBBBBBB;
    
    TRACE("Testing 4-BYTE READ AND WRITE\n");
    systick_delay_ms(5);
    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&data_read.word, 4);
    TRACE("Read 4-byte value 0x%X\n", data_read.word);
    i2c_write(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&write_i2c, 4);
    TRACE("Wrote 4-byte value 0x%X\n", write_i2c);
    i2c_read(I2C_VL53L0X_DEVICE_ADDRESS, I2C_VL53L0X_WRITE_REGISTER, (uint8_t*)&data_read.word, 4);
    TRACE("Read 4-byte value 0x%X\n\n", data_read.word);
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

static i2c_return_error_e i2c_start_condition(void)
{
    uint16_t timeout_retry = TIMEOUT_COUNT;
    
    /* NOTE: IMPORTANT WORKAROUND:
     * For some odd reason the start condition automatically generates a stop condition randomly when a sequential
     * write -> read happens in the test function. This is purely random, sometimes it does this after 16 succesful
     * test function calls, other times it does this back to back. The workaround is to check if the stop bit is set
     * before calling the start condition every time and if it is, to manually clear it in software and then wait
     * for the hardware to catch up and clear it as well.
     */
    if (I2C2->CR1 & I2C_CR1_STOP) {
        I2C2->CR1 &= ~(I2C_CR1_STOP);
    }
    /* Poll until the hardware clears the STOP bit. */
    while (I2C2->CR1 & I2C_CR1_STOP);

    I2C2->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
    while ((!(I2C2->SR1 & I2C_SR1_SB)) && (--timeout_retry));
    if (timeout_retry == 0) {
        return I2C_RETURN_TIMEOUT_ERROR;
    }
    return (I2C2->SR1 & I2C_SR1_AF) ? I2C_RETURN_ACKNOWLEDGE_FAILURE_ERROR : I2C_RETURN_OK;
}

/* read_write parameter serves to set 0 or 1 for bit 0 dynamically when sending the device address */
static i2c_return_error_e i2c_device_addr_send(uint8_t device_addr, uint8_t read_write)
{
    uint16_t register_read;
    uint16_t timeout_retry = TIMEOUT_COUNT;

    I2C2->DR = ((device_addr << 1) | read_write);
    while (!(I2C2->SR1 & I2C_SR1_ADDR) && (--timeout_retry)) {
        if (I2C2->SR1 & I2C_SR1_BERR) {
            return I2C_RETURN_BUS_ERROR;
        }
    }
    if (timeout_retry == 0) {
        return I2C_RETURN_TIMEOUT_ERROR;
    }
    register_read = I2C2->SR1 | I2C2->SR2;
    (void)register_read;
    return (I2C2->SR1 & I2C_SR1_AF) ? I2C_RETURN_ACKNOWLEDGE_FAILURE_ERROR : I2C_RETURN_OK;
}

/* Special case for master receiving 1 byte of data from slave device.
 * The ACK must be disabled BEFORE the ADDR flag is cleared (reading SR1 and SR2)
 *  and a stop condition must be made after waiting for ADDR flag to set. */
static i2c_return_error_e i2c_device_addr_send_1byte_rx(uint8_t device_addr)
{
    uint16_t register_read;
    uint16_t timeout_retry = TIMEOUT_COUNT;

    /* Shift in 0 to bit 8 to send device address. */
    I2C2->DR = ((device_addr << 1) | 1);
    while (!(I2C2->SR1 & I2C_SR1_ADDR) && (--timeout_retry)) {
        if (I2C2->SR1 & I2C_SR1_BERR) {
            return I2C_RETURN_BUS_ERROR;
        }
    }
    if (timeout_retry == 0) {
        return I2C_RETURN_TIMEOUT_ERROR;
    }
    I2C2->CR1 &= ~(I2C_CR1_ACK);
    register_read = I2C2->SR1 | I2C2->SR2;
    i2c_stop_condition();
    (void)register_read;
    return (I2C2->SR1 & I2C_SR1_AF) ? I2C_RETURN_ACKNOWLEDGE_FAILURE_ERROR : I2C_RETURN_OK;
}

/* Helper function to send data of multiple lengths, usually used to send 1, 2, or 4 bytes of data.
 * Also used to send the memory address of the slave device. */
static i2c_return_error_e i2c_data_send(const uint8_t* data, const uint8_t data_size)
{
    for (uint8_t i = 0; i < data_size; i++) {
        while (!(I2C2->SR1 & I2C_SR1_TXE)) {
            if (I2C2->SR1 & I2C_SR1_BERR) {
                return I2C_RETURN_BUS_ERROR;
            }
        }
        I2C2->DR = data[i];
    }
    /* Once last byte of data has been sent, need to wait for BTF to set. */
    while (!(I2C2->SR1 & I2C_SR1_BTF));
    if (I2C2->SR1 & I2C_SR1_AF) {
        i2c_stop_condition();
        return I2C_RETURN_ACKNOWLEDGE_FAILURE_ERROR;
    }
    return I2C_RETURN_OK;
}

static inline void i2c_stop_condition(void)
{
    I2C2->CR1 |= I2C_CR1_STOP;
    /* Send the stop condition and make sure BUSY flag gets cleared. 
     * 1 means communication going on the bus, 0 means bus is idle. So we must wait
     * while the flag is 1 and hardware should clear it to 0. */
    while (I2C2->SR2 & I2C_SR2_BUSY);
}
