#include "vl53l0x.h"
#include "i2c.h"
#include "trace.h"
#include "assert_handler.h"
#include <stdint.h>

/* These register defines were pulled from Artful Bytes blog post on the VL53L0X sensor.
 * https://www.artfulbytes.com/vl53l0x-post
 */
#define VL53L0X_DEVICE_ADDRESS                                           0x29
#define VL53L0X_IDENTIFICATION_MODEL_ID_REGISTER                         0xC0
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REGISTER                0x89
#define VL53L0X_MSRC_CONFIG_CONTROL_REGISTER                             0x60
#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REGISTER     0x44
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_REGISTER                          0x01
#define VL53L0X_DYNAMIC_SPAD_REF_EN_START_OFFSET_REGISTER                0x4F
#define VL53L0X_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REGISTER             0x4E
#define VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT_REGISTER               0xB6
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_REGISTER                    0x0A
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REGISTER                         0x84
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR_REGISTER                          0x0B
#define VL53L0X_RESULT_INTERRUPT_STATUS_REGISTER                         0x13
#define VL53L0X_SYSRANGE_START_REGISTER                                  0x00
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REGISTER                0xB0
#define VL53L0X_RESULT_RANGE_STATUS_REGISTER                             0x14

typedef struct {
    uint8_t addr;
    uint8_t data;
}vl53l0x_addr_data_pair;


static uint8_t stop_variable = 0;
static bool data_initialized = false;

static vl53l0x_return_error_e vl53l0x_write_addr_data_pairs(const vl53l0x_addr_data_pair addr_data_regs[], uint8_t length);

static vl53l0x_return_error_e data_initialize(void)
{
    ASSERT(!data_initialized);
    vl53l0x_return_error_e return_error = VL53L0X_RETURN_OK;

    /* Set 2v8 mode. Can also set 1v8 mode, but my breakout board uses 2v8. */
    uint8_t vhv_config_scl_sda = 0;
    if (i2c_read(VL53L0X_DEVICE_ADDRESS, VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REGISTER, &vhv_config_scl_sda, 1)
        != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    vhv_config_scl_sda |= 0x01;
    if (i2c_write(VL53L0X_DEVICE_ADDRESS, VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REGISTER, &vhv_config_scl_sda, 1)
        != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    /* Writing 0 to 0x88 sets it to standard mode for I2C. 
     * Writing to this register with a 0 multiple times seems to lock up the slave device. */
    if (i2c_write(VL53L0X_DEVICE_ADDRESS, 0x88, 0x00, 1)
        != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    /* The following 3 writes are setting various registers just like ST's API. */
    const vl53l0x_addr_data_pair addr_data_pair1[] = {
        {0x80, 0x01},
        {0xFF, 0x01},
        {0x00, 0x00}
    };
    
    return_error = vl53l0x_write_addr_data_pairs(addr_data_pair1, (sizeof(addr_data_pair1)/sizeof(addr_data_pair1[0])));
    if (return_error != VL53L0X_RETURN_OK) {
        TRACE("VL53L0X Data Initialize, Addr Data Pair, failed. Error Code: %u\n", return_error);
        return return_error;
    }

    /* Need to retrieve the stop variable for each sensor(?) */
    if (i2c_read(VL53L0X_DEVICE_ADDRESS, 0x91, &stop_variable, 1) != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    const vl53l0x_addr_data_pair addr_data_pair2[] = {
        {0x00, 0x01},
        {0xFF, 0x00},
        {0x80, 0x00},
    };

    return_error = vl53l0x_write_addr_data_pairs(addr_data_pair2, (sizeof(addr_data_pair2)/sizeof(addr_data_pair2[0])));
    if (return_error != VL53L0X_RETURN_OK) {
        TRACE("VL53L0X Data Initialize, Addr Data Pair, failed. Error Code: %u\n", return_error);
        return return_error;
    }
    data_initialized = true;
    return return_error;
}

void vl53l0x_test(void)
{
    data_initialize();
}
static vl53l0x_return_error_e vl53l0x_write_addr_data_pairs(const vl53l0x_addr_data_pair addr_data_regs[], uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        if (i2c_write(VL53L0X_DEVICE_ADDRESS, addr_data_regs[i].addr, &addr_data_regs[i].data, 1)) {
            return VL53L0X_RETURN_I2C_ERROR;
        }
        TRACE("Wrote value 0x%X to register 0x%X\n", addr_data_regs[i].data, addr_data_regs[i].addr);
    }
    return VL53L0X_RETURN_OK;
}
