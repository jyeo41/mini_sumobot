#ifndef I2C_H_
#define I2C_H_
#include <stdint.h>

typedef enum {
	I2C_RETURN_OK,
	I2C_RETURN_BUS_ERROR,
	I2C_RETURN_ARBITRATION_LOST_ERROR,
	I2C_RETURN_ACKNOWLEDGE_FAILURE_ERROR,
}i2c_return_error_e;

void i2c_initialize(void);
void i2c_write(const uint8_t device_addr, const uint8_t memory_addr, uint8_t* data, uint8_t data_size);
void i2c_read(const uint8_t device_addr, const uint8_t memory_addr, uint8_t* data, uint8_t data_size);
void i2c_test_read_write(void);




#endif /* I2C_H_ */
