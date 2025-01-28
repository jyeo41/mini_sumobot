#ifndef VL53L0X_H_
#define VL53L0X_H_

#include <stdint.h>

#define VL53L0X_OUT_OF_RANGE (8190)

typedef enum {
	VL53L0X_RETURN_OK                           = 0,
	VL53L0X_RETURN_I2C_ERROR                    = 1,
}vl53l0x_return_error_e;

void vl53l0x_initialize(void);
void vl53l0x_test_range(void);
vl53l0x_return_error_e vl53l0x_read_range_singular(uint16_t* range);

#endif	/* VL53L0X_H_ */
