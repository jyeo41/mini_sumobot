#include "vl53l0x.h"
#include "i2c.h"
#include "trace.h"
#include "assert_handler.h"
#include "gpio.h"
#include "systick.h"
#include <stdint.h>

/* Most of the code for this driver was adapted from Artful Bytes blog post on the VL53L0X sensor.
 * The documentation for the VL53L0X sensor is terrible and it does not even come with a memory mapping
 * of the registers you have to use to configure the sensor. ST just expects you to use their API library
 * without writing your own self drivers, however there is a lot of unnecessary code because the API is
 * very dense since the sensor is extremely configurable and complicated. Artful Bytes did a great dive 
 * on setting up a minimal driver for the sensor and I have adapted it to suit my needs with my own i2c
 * driver implementations.
 *
 * https://www.artfulbytes.com/vl53l0x-post
 */
#define VL53L0X_DEFAULT_ADDRESS                                         0x29
#define VL53L0X_EXPECTED_DEVICE_ID                                      0xEE
#define VL53L0X_IDENTIFICATION_MODEL_ID_REGISTER                        0xC0
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REGISTER               0x89
#define VL53L0X_MSRC_CONFIG_CONTROL_REGISTER                            0x60
#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REGISTER    0x44
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_REGISTER                         0x01
#define VL53L0X_DYNAMIC_SPAD_REF_EN_START_OFFSET_REGISTER               0x4F
#define VL53L0X_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REGISTER            0x4E
#define VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT_REGISTER              0xB6
#define VL53L0X_RESULT_RANGE_STATUS_REGISTER                            0x14
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_REGISTER                   0x0A
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REGISTER                        0x84
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR_REGISTER                         0x0B
#define VL53L0X_RESULT_INTERRUPT_STATUS_REGISTER                        0x13
#define VL53L0X_SYSRANGE_START_REGISTER                                 0x00
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REGISTER               0xB0
#define VL53L0X_RESULT_RANGE_STATUS_REGISTER                            0x14
#define VL53L0X_SLAVE_DEVICE_ADDRESS_REGISTER                           0x8A

#define VL53L0X_RANGE_SEQUENCE_STEP_TCC                                 0x10 /* Target Center Check */
#define VL53L0X_RANGE_SEQUENCE_STEP_MSRC                                0x04 /* Minimum Signal Rate Check */
#define VL53L0X_RANGE_SEQUENCE_STEP_DSS                                 0x28 /* Dynamic SPAD Selection */
#define VL53L0X_RANGE_SEQUENCE_STEP_PRE_RANGE                           0x40
#define VL53L0X_RANGE_SEQUENCE_STEP_FINAL_RANGE                         0x80

typedef enum {
    VL53L0X_CALIBRATION_TYPE_VHV,
    VL53L0X_CALIBRATION_TYPE_PHASE,
}vl53l0x_calibration_type_e;

typedef struct {
    uint8_t addr;
    uint8_t data;
}vl53l0x_addr_data_pair;


typedef struct {
    uint8_t addr;
    gpio_pin_names_e gpio_xshut;
}vl53l0x_addr_config_t;

static const vl53l0x_addr_config_t vl53l0x_addr_config[] = {
    [VL53L0X_INDEX_MIDDLE] = {.addr = 0x30, .gpio_xshut = VL53L0X_XSHUT_MIDDLE},
    [VL53L0X_INDEX_LEFT] = {.addr = 0x31, .gpio_xshut = VL53L0X_XSHUT_LEFT},
    [VL53L0X_INDEX_RIGHT] = {.addr = 0x32, .gpio_xshut = VL53L0X_XSHUT_RIGHT},
};

static bool device_initialized = false;
static uint8_t stop_variable = 0;
static uint8_t current_slave_address = VL53L0X_DEFAULT_ADDRESS;

static vl53l0x_return_error_e vl53l0x_check_id(uint8_t device_address);
static vl53l0x_return_error_e vl53l0x_data_initialize(void);
static vl53l0x_return_error_e vl53l0x_write_addr_data_pairs(const vl53l0x_addr_data_pair addr_data_regs[], uint8_t length);
static vl53l0x_return_error_e vl53l0x_static_initialize(void);
static vl53l0x_return_error_e vl53l0x_load_default_tuning_settings(void);
static vl53l0x_return_error_e vl53l0x_interrupt_enable(void);
static vl53l0x_return_error_e vl53l0x_set_sequence_steps_enabled(uint8_t sequence_step);
static vl53l0x_return_error_e vl53l0x_single_ref_calibration(vl53l0x_calibration_type_e calibration);
static vl53l0x_return_error_e vl53l0x_ref_calibration_initialize(void);
static void vl53l0x_initialize_single(vl53l0x_addr_config_t vl53l0x_single);
static void vl53l0x_set_slave_address(uint8_t address);
static vl53l0x_return_error_e vl53l0x_set_device_addresses(void);
static vl53l0x_return_error_e vl53l0x_set_device_address_single(gpio_pin_names_e xshut_pin, uint8_t address);


/* Top most initialize function that will call all sub functions required. */
vl53l0x_return_error_e vl53l0x_initialize(void)
{
    i2c_initialize();
    ASSERT(!device_initialized);

    vl53l0x_return_error_e return_error = VL53L0X_RETURN_OK;

    /* Call initialize address for each of the sensors */
    return_error = vl53l0x_set_device_addresses();
    
    /* Then call vl53l0x_initialize_single for each of the sensors after */
    vl53l0x_initialize_single(vl53l0x_addr_config[VL53L0X_INDEX_MIDDLE]);
    vl53l0x_initialize_single(vl53l0x_addr_config[VL53L0X_INDEX_LEFT]);
    vl53l0x_initialize_single(vl53l0x_addr_config[VL53L0X_INDEX_RIGHT]);

    device_initialized = true;

    return return_error;
}

/* Function used to initialize a SINGLE vl53l0x sensor. This will be called multiple times
 * for each of the sensor being used. In my project I am only using 3.
 *
 * Three things we must do when we initialize this sensor.
 * First check to make sure the ID returned from the sensor is the correct one.
 * Second, for the minimal driver set up we must do "data initialization".
 * Third, we must do "static initialization". */
static void vl53l0x_initialize_single(vl53l0x_addr_config_t vl53l0x_single)
{
    vl53l0x_set_slave_address(vl53l0x_single.addr);
    TRACE("I2C INITIALIZED SUCCESS!\n");
    vl53l0x_check_id(current_slave_address);
    TRACE("VL53L0X CHECK ID SUCCESS!\n");
    vl53l0x_data_initialize();
    TRACE("VL53L0X DATA INITIALIZE SUCCESS!\n");
    vl53l0x_static_initialize();
    TRACE("VL53L0X STATIC INITIALIZE SUCCESS!\n");
    vl53l0x_ref_calibration_initialize();
    TRACE("VL53L0X REF CALIBRATION INITIALIZE SUCCESS!\n");

}

void vl53l0x_test_range(vl53l0x_index_e vl53l0x_index)
{
    uint16_t range = 0;
    vl53l0x_return_error_e return_error = VL53L0X_RETURN_OK;

    vl53l0x_set_slave_address(vl53l0x_addr_config[vl53l0x_index].addr);
    return_error = vl53l0x_read_range_singular(&range);
    if (return_error != VL53L0X_RETURN_OK) {
        TRACE("SENSOR %u: Range measurement failed (error %u)\n", vl53l0x_index, return_error);
    } else {
        if (range != VL53L0X_OUT_OF_RANGE) {
            TRACE("SENSOR %u: Range %u mm\n", vl53l0x_index, range);
        } else {
            TRACE("SENSOR %u: OUT OF RANGE\n", vl53l0x_index);
        }
    }
}

vl53l0x_return_error_e vl53l0x_read_range_singular(uint16_t* range)
{
    uint8_t data = 0x01;
    vl53l0x_return_error_e return_error = VL53L0X_RETURN_OK;

    const vl53l0x_addr_data_pair sysrange_register_pairs[] = {
        {0x80, 0x01},
        {0xFF, 0x01},
        {0x00, 0x00},
        {0x00, 0x01},
        {0xFF, 0x00},
        {0x80, 0x00},
    };

    return_error = vl53l0x_write_addr_data_pairs(sysrange_register_pairs, 3);
    if (return_error != VL53L0X_RETURN_OK) {
        return return_error;
    }

    if (i2c_write(current_slave_address, 0x91, &stop_variable, 1) != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    return_error = vl53l0x_write_addr_data_pairs(sysrange_register_pairs + 3,
                                                ((sizeof(sysrange_register_pairs)/sizeof(sysrange_register_pairs[0])) - 3));
    if (return_error != VL53L0X_RETURN_OK) {
        return return_error;
    }
    
    if (i2c_write(current_slave_address, VL53L0X_SYSRANGE_START_REGISTER, &data, 1)
        != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    uint8_t sysrange_start = 0;
    i2c_return_error_e i2c_return_result = I2C_RETURN_OK;

    do {
        i2c_return_result = i2c_read(current_slave_address, VL53L0X_SYSRANGE_START_REGISTER, &sysrange_start, 1);
    } while ((i2c_return_result == I2C_RETURN_OK) && (sysrange_start & 0x01));
    if (i2c_return_result != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    uint8_t interrupt_status = 0;

    do {
        i2c_return_result = i2c_read(current_slave_address, VL53L0X_RESULT_INTERRUPT_STATUS_REGISTER, &interrupt_status, 1);
    } while ((i2c_return_result == I2C_RETURN_OK) && ((interrupt_status & 0x07) == 0));
    if (i2c_return_result != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    uint8_t range_buffer[2];
    if (i2c_read(current_slave_address, VL53L0X_RESULT_RANGE_STATUS_REGISTER + 10, range_buffer, 2)
        != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }
    
    /* Combine the bytes to correct the order of endianness. */
    *range = ((uint16_t)range_buffer[0] << 8) | range_buffer[1];

    if (i2c_read(current_slave_address, VL53L0X_SYSTEM_INTERRUPT_CLEAR_REGISTER, &data, 1)
        != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    /* 8190 or 8191 may be returned when obstacle is out of range. */
    if (*range == 8190 || *range == 8191) {
        *range = VL53L0X_OUT_OF_RANGE;
    }
    
    return return_error;
}

static vl53l0x_return_error_e vl53l0x_check_id(uint8_t device_address)
{
    uint8_t device_id = 0;
    if (i2c_read(device_address, VL53L0X_IDENTIFICATION_MODEL_ID_REGISTER, &device_id, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X CHECK ID FAILED: WRONG DEVICE ID OF 0x%X, EXPECTED 0x%X\n", 
              device_id, VL53L0X_EXPECTED_DEVICE_ID);
        return VL53L0X_RETURN_I2C_ERROR;
    }
    return VL53L0X_RETURN_OK;
}

static vl53l0x_return_error_e vl53l0x_data_initialize(void)
{
    vl53l0x_return_error_e return_error = VL53L0X_RETURN_OK;

    /* Set 2v8 mode. Can also set 1v8 mode, but my breakout board uses 2v8. */
    uint8_t vhv_config_scl_sda = 0;
    if (i2c_read(current_slave_address, VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REGISTER, &vhv_config_scl_sda, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X DATA INITIALIZE FAILED: READ VHV CONFIG PAD SCL SDA EXTSUP HV\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    vhv_config_scl_sda |= 0x01;
    if (i2c_write(current_slave_address, VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REGISTER, &vhv_config_scl_sda, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X DATA INITIALIZE FAILED: WRITE VHV CONFIG PAD SCL SDA EXTSUP HV\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    /* Writing 0 to 0x88 sets it to standard mode for I2C. 
     * Writing to this register with a 0 multiple times seems to lock up the slave device. */
    if (i2c_write(current_slave_address, 0x88, 0x00, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X DATA INITIALIZE FAILED: SET STANDARD MODE\n"); 
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
        TRACE("VL53L0X DATA INITIALIZE FAILED: WRITE ADDR DATA PAIR 1\n");
        return return_error;
    }

    /* Need to retrieve the stop variable for each sensor(?) */
    if (i2c_read(current_slave_address, 0x91, &stop_variable, 1) != I2C_RETURN_OK) {
        TRACE("VL53L0X DATA INITIALIZE FAILED: READ STOP VARIABLE\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    const vl53l0x_addr_data_pair addr_data_pair2[] = {
        {0x00, 0x01},
        {0xFF, 0x00},
        {0x80, 0x00},
    };

    return_error = vl53l0x_write_addr_data_pairs(addr_data_pair2, (sizeof(addr_data_pair2)/sizeof(addr_data_pair2[0])));
    if (return_error != VL53L0X_RETURN_OK) {
        TRACE("VL53L0X DATA INITIALIZE FAILED: WRITE ADDR DATA PAIR 2\n");
        return return_error;
    }
    return return_error;
}

static vl53l0x_return_error_e vl53l0x_static_initialize(void)
{
    vl53l0x_load_default_tuning_settings();
    vl53l0x_interrupt_enable();
    vl53l0x_set_sequence_steps_enabled(VL53L0X_RANGE_SEQUENCE_STEP_DSS +
                                       VL53L0X_RANGE_SEQUENCE_STEP_PRE_RANGE +
                                       VL53L0X_RANGE_SEQUENCE_STEP_FINAL_RANGE);

    return VL53L0X_RETURN_OK;
}

/* Unexplained default settings for the sensor from ST's API. */
static vl53l0x_return_error_e vl53l0x_load_default_tuning_settings(void) {
    vl53l0x_return_error_e return_error = VL53L0X_RETURN_OK;

    vl53l0x_addr_data_pair default_tuning_addr_data_pair[] = {
        { 0xFF, 0x01 }, { 0x00, 0x00 }, { 0xFF, 0x00 }, { 0x09, 0x00 }, { 0x10, 0x00 },
        { 0x11, 0x00 }, { 0x24, 0x01 }, { 0x25, 0xFF }, { 0x75, 0x00 }, { 0xFF, 0x01 },
        { 0x4E, 0x2C }, { 0x48, 0x00 }, { 0x30, 0x20 }, { 0xFF, 0x00 }, { 0x30, 0x09 },
        { 0x54, 0x00 }, { 0x31, 0x04 }, { 0x32, 0x03 }, { 0x40, 0x83 }, { 0x46, 0x25 },
        { 0x60, 0x00 }, { 0x27, 0x00 }, { 0x50, 0x06 }, { 0x51, 0x00 }, { 0x52, 0x96 },
        { 0x56, 0x08 }, { 0x57, 0x30 }, { 0x61, 0x00 }, { 0x62, 0x00 }, { 0x64, 0x00 },
        { 0x65, 0x00 }, { 0x66, 0xA0 }, { 0xFF, 0x01 }, { 0x22, 0x32 }, { 0x47, 0x14 },
        { 0x49, 0xFF }, { 0x4A, 0x00 }, { 0xFF, 0x00 }, { 0x7A, 0x0A }, { 0x7B, 0x00 },
        { 0x78, 0x21 }, { 0xFF, 0x01 }, { 0x23, 0x34 }, { 0x42, 0x00 }, { 0x44, 0xFF },
        { 0x45, 0x26 }, { 0x46, 0x05 }, { 0x40, 0x40 }, { 0x0E, 0x06 }, { 0x20, 0x1A },
        { 0x43, 0x40 }, { 0xFF, 0x00 }, { 0x34, 0x03 }, { 0x35, 0x44 }, { 0xFF, 0x01 },
        { 0x31, 0x04 }, { 0x4B, 0x09 }, { 0x4C, 0x05 }, { 0x4D, 0x04 }, { 0xFF, 0x00 },
        { 0x44, 0x00 }, { 0x45, 0x20 }, { 0x47, 0x08 }, { 0x48, 0x28 }, { 0x67, 0x00 },
        { 0x70, 0x04 }, { 0x71, 0x01 }, { 0x72, 0xFE }, { 0x76, 0x00 }, { 0x77, 0x00 },
        { 0xFF, 0x01 }, { 0x0D, 0x01 }, { 0xFF, 0x00 }, { 0x80, 0x01 }, { 0x01, 0xF8 },
        { 0xFF, 0x01 }, { 0x8E, 0x01 }, { 0x00, 0x01 }, { 0xFF, 0x00 }, { 0x80, 0x00 }
    };
    if (vl53l0x_write_addr_data_pairs(default_tuning_addr_data_pair, 
                                      sizeof(default_tuning_addr_data_pair)/sizeof(default_tuning_addr_data_pair[0]))) {
        TRACE("VL53L0X MODULE FAILED: LOAD DEFAULT TUNING SETTINGS\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }
    return return_error;
}

/* Enable interrupt pin. Even though we will be using a polling implementation, the interrupt pin
 * must still be enabled to poll the interrupt register. */
static vl53l0x_return_error_e vl53l0x_interrupt_enable(void)
{
    uint8_t data = 0x04;

    /* Interrupt on new sample ready. */
    if (i2c_write(current_slave_address, VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_REGISTER, &data, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X MODULE FAILED: WRITE TO GPIO_HV_MUX_ACTIVE_HIGH_REGISTER.\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    /* Set to active low because the pin is pulled up by default on the breakout board. */
    uint8_t gpio_hv_mux_active_high = 0;
    if (i2c_read(current_slave_address, VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REGISTER, &gpio_hv_mux_active_high, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X MODULE FAILED: READING FROM GPIO_HV_MUX_ACTIVE_HIGH_REGISTER.\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    gpio_hv_mux_active_high &= ~(0x10);
    if (i2c_write(current_slave_address, VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REGISTER, &gpio_hv_mux_active_high, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X MODULE FAILED: RE-WRITE TO GPIO_HV_MUX_ACTIVE_HIGH_REGISTER.\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    uint8_t clear_interrupt = 0x01;
    if (i2c_write(current_slave_address, VL53L0X_SYSTEM_INTERRUPT_CLEAR_REGISTER, &clear_interrupt, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X MODULE FAILED: WRITE TO SYSTEM_INTERRUPT_CLEAR_REGISTER.\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    return VL53L0X_RETURN_OK;
}

static vl53l0x_return_error_e vl53l0x_set_sequence_steps_enabled(uint8_t sequence_step)
{
    if (i2c_write(current_slave_address, VL53L0X_SYSTEM_SEQUENCE_CONFIG_REGISTER, &sequence_step, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X MODULE FAILED: SETTING SEQUENCE STEPS.\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }
    return VL53L0X_RETURN_OK;
}

static vl53l0x_return_error_e vl53l0x_ref_calibration_initialize(void)
{
    vl53l0x_single_ref_calibration(VL53L0X_CALIBRATION_TYPE_VHV);
    //TRACE("VL53L0X VHV SINGLE REF CALIBRATION SUCCESS!\n");
    vl53l0x_single_ref_calibration(VL53L0X_CALIBRATION_TYPE_PHASE);
    //TRACE("VL53L0X PHASE SINGLE REF CALIBRATION SUCCESS!\n");
    if (vl53l0x_set_sequence_steps_enabled(VL53L0X_RANGE_SEQUENCE_STEP_DSS +
                                           VL53L0X_RANGE_SEQUENCE_STEP_PRE_RANGE +
                                           VL53L0X_RANGE_SEQUENCE_STEP_FINAL_RANGE)
        != VL53L0X_RETURN_OK) {
        TRACE("VL53L0X REF CAL INITIALIZE FAILED: SET SEQUENCE STEPS ENABLED");
    }

    return VL53L0X_RETURN_OK;
}

static vl53l0x_return_error_e vl53l0x_single_ref_calibration(vl53l0x_calibration_type_e calibration)
{
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    //TRACE("VL53L0X Starting single ref calibration...\n");

    switch (calibration) {
        case VL53L0X_CALIBRATION_TYPE_VHV:
            sequence_config = 0x01;
            sysrange_start = 0x01 | 0x40;
            break;
        case VL53L0X_CALIBRATION_TYPE_PHASE:
            sequence_config = 0x02;
            //cppcheck-suppress badBitmaskCheck
            sysrange_start = 0x01 | 0x00;
            break; 
    }

    if (i2c_write(current_slave_address, VL53L0X_SYSTEM_SEQUENCE_CONFIG_REGISTER, &sequence_config, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X SINGLE REF CALIBRATION FAILED: SYS SEQ CONFIG REGISTER\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    if (i2c_write(current_slave_address, VL53L0X_SYSRANGE_START_REGISTER, &sysrange_start, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X SINGLE REF CALIBRATION FAILED: SYSRANGE START REGISTER\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    /* Wait for interrupt. */
    uint8_t interrupt_status = 0;
    i2c_return_error_e i2c_return_result = I2C_RETURN_OK;
    
    do {
        i2c_return_result = i2c_read(current_slave_address, VL53L0X_RESULT_INTERRUPT_STATUS_REGISTER,
                                     &interrupt_status, 1);
    } while ((i2c_return_result == I2C_RETURN_OK) && ((interrupt_status & 0x07) == 0));
    if (i2c_return_result != I2C_RETURN_OK) {
        TRACE("VL53L0X SINGLE REF CALIBRATION FAILED: WAIT FOR INTERRUPT\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }


    if (i2c_write(current_slave_address, VL53L0X_SYSTEM_INTERRUPT_CLEAR_REGISTER, (uint8_t*)0x01, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X SINGLE REF CALIBRATION FAILED: SYSTEM INTERRUPT CLEAR REGISTER\n");
        return VL53L0X_RETURN_I2C_ERROR;
    }

    if (i2c_write(current_slave_address, VL53L0X_SYSRANGE_START_REGISTER, (uint8_t*)0x00, 1)
        != I2C_RETURN_OK) {
        TRACE("VL53L0X SINGLE REF CALIBRATION FAILED: SYSRANGE START REGISTER\n");
    }

    return VL53L0X_RETURN_OK;
}

static vl53l0x_return_error_e vl53l0x_write_addr_data_pairs(const vl53l0x_addr_data_pair addr_data_regs[], uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        if (i2c_write(current_slave_address, addr_data_regs[i].addr, &addr_data_regs[i].data, 1)) {
            return VL53L0X_RETURN_I2C_ERROR;
        }
        //TRACE("Wrote value 0x%X to register 0x%X\n", addr_data_regs[i].data, addr_data_regs[i].addr);
    }
    return VL53L0X_RETURN_OK;
}

static void vl53l0x_set_slave_address(uint8_t address)
{
    current_slave_address = address;
}

static vl53l0x_return_error_e vl53l0x_set_device_addresses(void)
{
    /* 1. Set the gpio pins for each of the xshut pins.
     * 2. Make sure they're all off.
     * 3. Turn them on one by one and then set the addresses. */
    gpio_configure_pin(VL53L0X_XSHUT_MIDDLE, GPIO_MODE_OUTPUT, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED,
                       GPIO_OTYPE_PUSHPULL, GPIO_SPEED_LOW);
    ASSERT(gpio_config_compare(VL53L0X_XSHUT_MIDDLE, GPIOB, 5, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED,
                               GPIO_OTYPE_PUSHPULL, GPIO_SPEED_LOW));
    gpio_configure_pin(VL53L0X_XSHUT_LEFT, GPIO_MODE_OUTPUT, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED,
                       GPIO_OTYPE_PUSHPULL, GPIO_SPEED_LOW);
    ASSERT(gpio_config_compare(VL53L0X_XSHUT_LEFT, GPIOB, 6, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED,
                               GPIO_OTYPE_PUSHPULL, GPIO_SPEED_LOW));
    gpio_configure_pin(VL53L0X_XSHUT_RIGHT, GPIO_MODE_OUTPUT, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED,
                       GPIO_OTYPE_PUSHPULL, GPIO_SPEED_LOW);
    ASSERT(gpio_config_compare(VL53L0X_XSHUT_RIGHT, GPIOB, 7, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED,
                               GPIO_OTYPE_PUSHPULL, GPIO_SPEED_LOW));
    #if 0
    #endif

    /* Set the slave address to the original device address temporarily
     * so we can set the unique addresses. */
    vl53l0x_set_slave_address(VL53L0X_DEFAULT_ADDRESS);

    gpio_data_output_clear(VL53L0X_XSHUT_MIDDLE);
    gpio_data_output_clear(VL53L0X_XSHUT_LEFT);
    gpio_data_output_clear(VL53L0X_XSHUT_RIGHT);
    #if 0
    #endif

    vl53l0x_set_device_address_single(VL53L0X_XSHUT_MIDDLE, vl53l0x_addr_config[VL53L0X_INDEX_MIDDLE].addr);
    vl53l0x_set_device_address_single(VL53L0X_XSHUT_LEFT, vl53l0x_addr_config[VL53L0X_INDEX_LEFT].addr);
    vl53l0x_set_device_address_single(VL53L0X_XSHUT_RIGHT, vl53l0x_addr_config[VL53L0X_INDEX_RIGHT].addr);
    #if 0
    #endif

    return VL53L0X_RETURN_OK;
}

/* Input parameter is the name of the xshutpin we want to turn on and the address of that xshut pin pulled from the
 * vl53l0x_addr_config[] array data structure. */
static vl53l0x_return_error_e vl53l0x_set_device_address_single(gpio_pin_names_e xshut_pin, uint8_t address)
{
    /* Then turn on each pin one by one and then set the slave address. */
    gpio_data_output_set(xshut_pin);

    /* Need some amount of delay before we leave hardware standby. */
    systick_delay_ms(1000);

    uint8_t data = address & 0x7F;

    /* Set the unique i2c addresses for each slave device. */
    if (i2c_write(current_slave_address, VL53L0X_SLAVE_DEVICE_ADDRESS_REGISTER, &data, 1)
        != I2C_RETURN_OK) {
        return VL53L0X_RETURN_I2C_ERROR;
    }

    return VL53L0X_RETURN_OK;
}
