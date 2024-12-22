#ifndef VL53L0X_H_
#define VL53L0X_H_

typedef enum {
	VL53L0X_RETURN_OK                           = 0,
	VL53L0X_RETURN_I2C_ERROR                    = 1,
}vl53l0x_return_error_e;

void vl53l0x_test(void);

#endif	/* VL53L0X_H_ */
