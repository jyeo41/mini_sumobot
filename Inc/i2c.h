#ifndef I2C_H_
#define I2C_H_
#include <stdint.h>
#include <stdbool.h>

typedef enum {
	I2C_RETURN_OK                           = 0,
	I2C_RETURN_BUS_ERROR                    = 1,
	I2C_RETURN_ARBITRATION_LOST_ERROR       = 2,
	I2C_RETURN_ACKNOWLEDGE_FAILURE_ERROR    = 3,
    I2C_RETURN_TIMEOUT_ERROR                = 4,
}i2c_return_error_e;

void i2c_initialize(void);
i2c_return_error_e i2c_write(const uint8_t device_addr, const uint8_t memory_addr, const uint8_t* data, const uint8_t data_size);
i2c_return_error_e i2c_read(const uint8_t device_addr, const uint8_t memory_addr, uint8_t* read_buffer, const uint8_t data_size);
void i2c_test_read_write(void);




#endif /* I2C_H_ */
