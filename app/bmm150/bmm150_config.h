#ifndef BMM150_BMM150_CONFIG_H
#define BMM150_BMM150_CONFIG_H

#include <stddef.h>
#include <stdint.h>

#define BMM150_CHIP_ID = 0x32U
#define BMM150_OVERFLOW_XY_FLIP -4096
#define BMM150_OVERFLOW_Z_HALL -16384
#define BMM150_LSB_SCALE (1.0F / 16.0F)

typedef float float32_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vec3_int16_t;

typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} vec3_float32_t;

typedef enum {
    BMM150_ERR_OK = 0,
    BMM150_ERR_FAIL = 1 << 0,
    BMM150_ERR_NULL = 1 << 1,
} bmm150_err_t;

typedef enum {
    BMM150_SLAVE_ADDRESS_CSB_GND_SD0_GND = 0x10,
    BMM150_SLAVE_ADDRESS_CSB_GND_SD0_VDD = 0x11,
    BMM150_SLAVE_ADDRESS_CSB_VDD_SD0_GND = 0x12,
    BMM150_SLAVE_ADDRESS_CSB_VDD_SD0_VDD = 0x13,
} bmm150_slave_address_t;

typedef enum {
    BMM150_REG_ADDRESS_CHIP_ID = 0x40,
    BMM150_REG_ADDRESS_DATAX_LSB = 0x42,
    BMM150_REG_ADDRESS_DATAX_MSB = 0x43,
    BMM150_REG_ADDRESS_DATAY_LSB = 0x44,
    BMM150_REG_ADDRESS_DATAY_MSB = 0x45,
    BMM150_REG_ADDRESS_DATAZ_LSB = 0x46,
    BMM150_REG_ADDRESS_DATAZ_MSB = 0x47,
    BMM150_REG_ADDRESS_RHALL_LSB = 0x48,
    BMM150_REG_ADDRESS_RHALL_MSB = 0x49,
    BMM150_REG_ADDRESS_INT_STAT = 0x4A,
    BMM150_REG_ADDRESS_PWR_RST = 0x4B,
    BMM150_REG_ADDRESS_OPMODE = 0x4C,
    BMM150_REG_ADDRESS_INT_CTRL1 = 0x4D,
    BMM150_REG_ADDRESS_INT_CTRL2 = 0x4E,
    BMM150_REG_ADDRESS_INT_HIGH_THRESH = 0x4F,
    BMM150_REG_ADDRESS_INT_LOW_THRESH = 0x50,
    BMM150_REG_ADDRESS_AXIS_REP_XY = 0x51,
    BMM150_REG_ADDRESS_AXIS_REP_Z = 0x52,
    BMM150_REG_ADDRESS_DIG_X1 = 0x5d,
    BMM150_REG_ADDRESS_DIG_Y1 = 0x5e,
    BMM150_REG_ADDRESS_DIG_Z4_LSB = 0x62,
    BMM150_REG_ADDRESS_DIG_Z4_MSB = 0x63,
    BMM150_REG_ADDRESS_DIG_X2 = 0x64,
    BMM150_REG_ADDRESS_DIG_Y2 = 0x65,
    BMM150_REG_ADDRESS_DIG_Z2_LSB = 0x68,
    BMM150_REG_ADDRESS_DIG_Z2_MSB = 0x69,
    BMM150_REG_ADDRESS_DIG_Z1_LSB = 0x6a,
    BMM150_REG_ADDRESS_DIG_Z1_MSB = 0x6b,
    BMM150_REG_ADDRESS_DIG_XYZ1_LSB = 0x6c,
    BMM150_REG_ADDRESS_DIG_XYZ1_MSB = 0x6d,
    BMM150_REG_ADDRESS_DIG_Z3_LSB = 0x6e,
    BMM150_REG_ADDRESS_DIG_Z3_MSB = 0x6f,
    BMM150_REG_ADDRESS_XY2 = 0x70,
    BMM150_REG_ADDRESS_DIG_XY1 = 0x71,
} bmm150_reg_address_t;

typedef enum {
    BMM150_CMD_SOFTRESET = 0x82,
    BMM150_CMD_SUSPEND = 0x01,
} bmm150_cmd_t;

typedef enum {
    BMM150_ADV_SELF_TEST_NORMAL = 0b00,
    BMM150_ADV_SELF_TEST_NEGATIVE = 0b10,
    BMM150_ADV_SELF_TEST_POSITIVE = 0b11,
} bmm150_adv_self_test_t;

typedef enum {
    BMM150_OUTPUT_DATA_RATE_10HZ = 0b000,
    BMM150_OUTPUT_DATA_RATE_2HZ = 0b001,
    BMM150_OUTPUT_DATA_RATE_6HZ = 0b010,
    BMM150_OUTPUT_DATA_RATE_8HZ = 0b011,
    BMM150_OUTPUT_DATA_RATE_15HZ = 0b100,
    BMM150_OUTPUT_DATA_RATE_20HZ = 0b101,
    BMM150_OUTPUT_DATA_RATE_25HZ = 0b110,
    BMM150_OUTPUT_DATA_RATE_30HZ = 0b111,
} bmm150_output_data_rate_t;

typedef enum {
    BMM150_OPERATING_MODE_NORMAL = 0b00,
    BMM150_OPERATING_MODE_FORCED = 0b01,
    BMM150_OPERATING_MODE_SLEEP = 0b11,
} bmm150_operating_mode_t;

typedef enum {
    BMM150_DATA_READY_POLARITY_ACTIVE_LOW = 0b0,
    BMM150_DATA_READY_POLARITY_ACTIVE_HIGH = 0b01,
} bmm150_data_ready_polarity_t;

typedef enum {
    BMM150_INTERRUPT_LATCH_EN = 0b00,
    BMM150_INTERRUPT_LATCH_DIS = 0b01,
} bmm150_interrupt_latch_t;

typedef enum {
    BMM150_INTERRUPT_POLARITY_LOW = 0b00,
    BMM150_INTERRUPT_POLARITY_HIGH = 0b01,
} bmm150_interrupt_polarity_t;

typedef enum {
    BMM150_REP_XY_LOWPOWER = 1,
    BMM150_REP_XY_REGULAR = 4,
    BMM150_REP_XY_ENHANCED = 7,
    BMM150_REP_XY_HIGHACCURACY = 23,
} bmm150_rep_xy_t;

typedef enum {
    BMM150_REP_Z_LOWPOWER = 2,
    BMM150_REP_Z_REGULAR = 14,
    BMM150_REP_Z_ENHANCED = 26,
    BMM150_REP_Z_HIGHACCURACY = 82,
} bmm150_rep_z_t;

typedef struct {
    int8_t dig_x1;
    int8_t dig_y1;
    int8_t dig_x2;
    int8_t dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2;
    int16_t dig_z3;
    int16_t dig_z4;
    uint8_t dig_xy1;
    int8_t dig_xy2;
    uint16_t dig_xyz1;
} bmm150_trim_data_t;

typedef struct {
    float32_t scale;
} bmm150_config_t;

typedef struct {
    void* bus_user;
    bmm150_err_t (*bus_init)(void*);
    bmm150_err_t (*bus_deinit)(void*);
    bmm150_err_t (*bus_write)(void*, uint8_t, uint8_t const*, size_t);
    bmm150_err_t (*bus_read)(void*, uint8_t, uint8_t*, size_t);
} bmm150_interface_t;

#endif // BMM150_BMM150_CONFIG_H