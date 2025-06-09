#include "bmm150.h"
#include "bmm150_registers.h"
#include "bmm150_utility.h"
#include <assert.h>
#include <string.h>

// void init(void)
// {
//     bmm150_pwr_rst_reg_t pwr_rst_reg = self.get_pwr_rst_reg();
//     bmm150_opmode_reg_t odr_opmode_reg = self.get_odr_opmode_reg();
//     bmm150_axis_rep_xy_reg_t axis_rep_xy_reg = self.get_axis_rep_xy_reg();

// bmm150_axis_rep_z_reg_t axis_rep_z_reg = self.get_axis_rep_z_reg();

// bmm150_int_ctrl1_reg_t int_ctrl1_reg = self.get_int_ctrl1_reg();

// bmm150_int_ctrl2_reg_t int_ctrl2_reg = self.get_int_ctrl2_reg();

//     pwr_rst_reg.power_control = 1;
//     pwr_rst_reg.soft_reset_1 = 1;
//     pwr_rst_reg.soft_reset_2 = 1;
//     self.set_pwr_rst_reg(pwr_rst_reg);

//     self.delay(10);

//     odr_opmode_reg.opmode = to_underlying(OPMode::NORMAL);

//     self.set_odr_opmode_reg(odr_opmode_reg);

//     auto chip_id = self.get_chip_id();

//     if (chip_id != CHIP_ID_VALUE) {
//         return;
//     }

//     self.trim_data = self.get_trim_data();

//     pwr_rst_reg.power_control = 0;
//     self.set_pwr_rst_reg(pwr_rst_reg);
//     self.delay(10) ;

//     pwr_rst_reg.power_control = 1;
//     self.set_pwr_rst_reg(pwr_rst_reg);

//     self.delay(10);

//     odr_opmode_reg.opmode = to_underlying(OPMode::SLEEP);

//     self.set_odr_opmode_reg(odr_opmode_reg);

//     odr_opmode_reg.opmode = to_underlying(OPMode::NORMAL);

//     self.set_odr_opmode_reg(odr_opmode_reg);

//     odr_opmode_reg.data_rate = to_underlying(ODR::ODR_10HZ);

//     self.set_odr_opmode_reg(odr_opmode_reg);

//     axis_rep_xy_reg.repetitions_xy = to_underlying(RepXY::REGULAR);

//     self.set_axis_rep_xy_reg(axis_rep_xy_reg);

//     axis_rep_z_reg.repetitions_z = to_underlying(RepZ::REGULAR);

//     self.set_axis_rep_z_reg(axis_rep_z_reg);

// }

static bmm150_err_t bmm150_bus_init(bmm150_t const* bmm150)
{
    return bmm150->interface.bus_init ? bmm150->interface.bus_init(bmm150->interface.bus_user)
                                      : BMM150_ERR_NULL;
}

static bmm150_err_t bmm150_bus_deinit(bmm150_t const* bmm150)
{
    return bmm150->interface.bus_deinit ? bmm150->interface.bus_deinit(bmm150->interface.bus_user)
                                        : BMM150_ERR_NULL;
}

static bmm150_err_t bmm150_bus_write(bmm150_t const* bmm150,
                                     uint8_t address,
                                     uint8_t const* data,
                                     size_t data_size)
{
    return bmm150->interface.bus_write
               ? bmm150->interface.bus_write(bmm150->interface.bus_user, address, data, data_size)
               : BMM150_ERR_NULL;
}

static bmm150_err_t bmm150_bus_read(bmm150_t const* bmm150,
                                    uint8_t address,
                                    uint8_t* data,
                                    size_t data_size)
{
    return bmm150->interface.bus_read
               ? bmm150->interface.bus_read(bmm150->interface.bus_user, address, data, data_size)
               : BMM150_ERR_NULL;
}

bmm150_err_t bmm150_initialize(bmm150_t* bmm150,
                               bmm150_config_t const* config,
                               bmm150_interface_t const* interface)
{
    assert(bmm150 && config && interface);

    memset(bmm150, 0, sizeof(*bmm150));
    memcpy(&bmm150->config, config, sizeof(*config));
    memcpy(&bmm150->interface, interface, sizeof(*interface));

    return bmm150_bus_init(bmm150);
}

bmm150_err_t bmm150_deinitialize(bmm150_t* bmm150)
{
    assert(bmm150);

    bmm150_err_t err = bmm150_bus_deinit(bmm150);

    memset(bmm150, 0, sizeof(*bmm150));

    return err;
}

bmm150_err_t bmm150_get_mag_data_x_scaled(bmm150_t const* bmm150, float32_t* scaled)
{
    assert(bmm150 && scaled);

    int16_t raw = {};
    uint16_t rhall = {};

    bmm150_err_t err = bmm150_get_mag_data_x_raw(bmm150, &raw);
    err |= bmm150_get_rhall_data(bmm150, &rhall);

    *scaled = bmm150_x_raw_to_scaled(&bmm150->trim_data, rhall, raw);

    return err;
}

bmm150_err_t bmm150_get_mag_data_y_scaled(bmm150_t const* bmm150, float32_t* scaled)
{
    assert(bmm150 && scaled);
}

bmm150_err_t bmm150_get_mag_data_z_scaled(bmm150_t const* bmm150, float32_t* scaled)
{
    assert(bmm150 && scaled);
}

bmm150_err_t bmm150_get_mag_data_xyz_scaled(bmm150_t const* bmm150, vec3_float32_t* scaled)
{
    assert(bmm150 && scaled);
}

bmm150_err_t bmm150_get_mag_data_x_raw(bmm150_t const* bmm150, int16_t* raw)
{
    assert(bmm150 && raw);

    bmm150_data_x_reg_t reg = {};

    bmm150_err_t err = bmm150_get_data_x_reg(bmm150, &reg);

    *raw = reg.data_x;

    // sign extend to 16 bits
    if (*raw & (1 << 12)) {
        *raw |= 0xE000;
    }

    return err;
}

bmm150_err_t bmm150_get_mag_data_y_raw(bmm150_t const* bmm150, int16_t* raw)
{
    assert(bmm150 && raw);

    bmm150_data_y_reg_t reg = {};

    bmm150_err_t err = bmm150_get_data_y_reg(bmm150, &reg);

    *raw = reg.data_y;

    // sign extend to 16 bits
    if (*raw & (1 << 12)) {
        *raw |= 0xE000;
    }

    return err;
}

bmm150_err_t bmm150_get_mag_data_z_raw(bmm150_t const* bmm150, int16_t* raw)
{
    assert(bmm150 && raw);

    bmm150_data_z_reg_t reg = {};

    bmm150_err_t err = bmm150_get_data_z_reg(bmm150, &reg);

    *raw = reg.data_z;

    // sign extend to 16 bits
    if (*raw & (1 << 12)) {
        *raw |= 0xE000;
    }

    return err;
}

bmm150_err_t bmm150_get_mag_data_xyz_raw(bmm150_t const* bmm150, vec3_int16_t* raw)
{
    assert(bmm150 && raw);

    bmm150_data_xyz_reg_t reg = {};

    bmm150_err_t err = bmm150_get_data_xyz_reg(bmm150, &reg);

    raw->x = reg.data_x;
    // sign extend from 13 to 16 bits
    if (raw->x & (1 << 12)) {
        raw->x |= 0xE000;
    }

    raw->y = reg.data_y;
    // sign extend from 13 to 16 bits
    if (raw->y & (1 << 12)) {
        raw->y |= 0xE000;
    }

    raw->z = reg.data_z;
    // sign extend from 15 to 16 bits
    if (raw->z & (1 << 14)) {
        raw->z |= 0x8000;
    }

    return err;
}

bmm150_err_t bmm150_get_rhall_data(bmm150_t const* bmm150, uint16_t* data)
{
    assert(bmm150 && data);

    bmm150_rhall_reg_t reg = {};

    bmm150_err_t err = bmm150_get_rhall_reg(bmm150, &reg);

    *data = reg.rhall;

    return err;
}

bmm150_err_t bmm150_get_trim_data(bmm150_t const* bmm150, bmm150_trim_data_t* data)
{
    assert(bmm150 && data);

    uint8_t trim_x1y1[2] = {};
    uint8_t trim_xyz[4] = {};
    uint8_t trim_xy1xy2[10] = {};

    bmm150_err_t err =
        bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_DIG_X1, trim_x1y1, sizeof(trim_x1y1));
    err |= bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_DIG_Z4_LSB, trim_xyz, sizeof(trim_xyz));
    err |= bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_DIG_Z2_LSB, trim_xy1xy2, sizeof(trim_xy1xy2));

    data->dig_x1 = (int8_t)(trim_x1y1[0]);
    data->dig_y1 = (int8_t)(trim_x1y1[1]);
    data->dig_x2 = (int8_t)(trim_xyz[2]);
    data->dig_y2 = (int8_t)(trim_xyz[3]);
    data->dig_z1 = (uint16_t)((((uint16_t)trim_xy1xy2[3]) << 8) | trim_xy1xy2[2]);
    data->dig_z2 = (int16_t)((((uint16_t)trim_xy1xy2[1]) << 8) | trim_xy1xy2[0]);
    data->dig_z3 = (int16_t)((((uint16_t)trim_xy1xy2[7]) << 8) | trim_xy1xy2[6]);
    data->dig_z4 = (int16_t)((((uint16_t)trim_xyz[1]) << 8) | trim_xyz[0]);
    data->dig_xy1 = trim_xy1xy2[9];
    data->dig_xy2 = (int8_t)(trim_xy1xy2[8]);
    data->dig_xyz1 = (uint16_t)(((uint16_t)(trim_xy1xy2[5] & 0x7F) << 8) | trim_xy1xy2[4]);

    return err;
}

bmm150_err_t bmm150_get_chip_id_reg(bmm150_t const* bmm150, bmm150_chip_id_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_CHIP_ID, &data, sizeof(data));

    reg->chip_id = data & 0xFFU;

    return err;
}

bmm150_err_t bmm150_get_data_x_reg(bmm150_t const* bmm150, bmm150_data_x_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data[2] = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_DATAX_LSB, data, sizeof(data));

    reg->data_x = (int16_t)((data[0] >> 3) & 0x1F) | ((data[1] & 0xFF) << 5);
    reg->x_self_test = data[0] & 0x01U;

    return err;
}

bmm150_err_t bmm150_get_data_y_reg(bmm150_t const* bmm150, bmm150_data_y_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data[2] = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_DATAY_LSB, data, sizeof(data));

    reg->data_y = (int16_t)((data[0] >> 3) & 0x1F) | ((data[1] & 0xFF) << 5);
    reg->y_self_test = data[0] & 0x01U;

    return err;
}

bmm150_err_t bmm150_get_data_z_reg(bmm150_t const* bmm150, bmm150_data_z_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data[2] = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_DATAZ_LSB, data, sizeof(data));

    reg->data_z = (int16_t)((data[0] >> 1) & 0x7F) | ((data[1] & 0xFF) << 5);
    reg->z_self_test = data[0] & 0x01U;

    return err;
}

bmm150_err_t bmm150_get_data_xyz_reg(bmm150_t const* bmm150, bmm150_data_xyz_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data[6] = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_DATAX_LSB, data, sizeof(data));

    reg->data_x = (int16_t)((data[0] >> 3) & 0x1F) | ((data[1] & 0xFF) << 5);
    reg->data_y = (int16_t)((data[2] >> 3) & 0x1F) | ((data[3] & 0xFF) << 5);
    reg->data_z = (int16_t)((data[4] >> 1) & 0x7F) | ((data[5] & 0xFF) << 5);

    return err;
}

bmm150_err_t bmm150_get_rhall_reg(bmm150_t const* bmm150, bmm150_rhall_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data[2] = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_RHALL_LSB, data, sizeof(data));

    reg->rhall = (uint16_t)(((data[0] >> 2U) & 0x3FU) | ((data[1] & 0xFFU) << 6U));
    reg->data_ready_status = data[1] & 0x01U;

    return err;
}

bmm150_err_t bmm150_get_int_stat_reg(bmm150_t const* bmm150, bmm150_int_stat_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_INT_STAT, &data, sizeof(data));

    reg->data_overrun = (data >> 7U) & 0x01U;
    reg->overflow = (data >> 6U) & 0x01U;
    reg->high_int_z = (data >> 5U) & 0x01U;
    reg->high_int_y = (data >> 4U) & 0x01U;
    reg->high_int_x = (data >> 3U) & 0x01U;
    reg->low_int_z = (data >> 2U) & 0x01U;
    reg->low_int_y = (data >> 1U) & 0x01U;
    reg->low_int_x = data & 0x01U;

    return err;
}

bmm150_err_t bmm150_get_pwr_rst_reg(bmm150_t const* bmm150, bmm150_pwr_rst_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_PWR_RST, &data, sizeof(data));

    reg->soft_reset_1 = (data >> 7U) & 0x01U;
    reg->spi3_en = (data >> 2U) & 0x01U;
    reg->soft_reset_2 = (data >> 1U) & 0x01U;
    reg->power_control = data & 0x01U;

    return err;
}

bmm150_err_t bmm150_set_pwr_rst_reg(bmm150_t const* bmm150, bmm150_pwr_rst_reg_t const* reg)
{
    assert(bmm150 && reg);

    uint8_t data = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_PWR_RST, &data, sizeof(data));

    data &= ~((0x01U << 7U) | (0x01U << 2U) | (0x01U << 1U) | 0x01U);

    data |= (reg->soft_reset_1 & 0x01U) << 7U;
    data |= (reg->spi3_en & 0x01U) << 2U;
    data |= (reg->soft_reset_2 & 0x01U) << 1U;
    data |= reg->power_control & 0x01U;

    err |= bmm150_bus_write(bmm150, BMM150_REG_ADDRESS_PWR_RST, &data, sizeof(data));

    return err;
}

bmm150_err_t bmm150_get_odr_opmode_reg(bmm150_t const* bmm150, bmm150_opmode_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_OPMODE, &data, sizeof(data));

    reg->adv_st = (data >> 6U) & 0x03U;
    reg->data_rate = (data >> 3U) & 0x07U;
    reg->opmode = (data >> 1U) & 0x03U;
    reg->self_test = data & 0x01U;

    return err;
}

bmm150_err_t bmm150_set_odr_opmode_reg(bmm150_t const* bmm150, bmm150_opmode_reg_t const* reg)
{
    assert(bmm150 && reg);

    uint8_t data = {};

    data |= (reg->adv_st & 0x03U) << 6U;
    data |= (reg->data_rate & 0x07U) << 3U;
    data |= (reg->opmode & 0x03U) << 1U;
    data |= reg->self_test & 0x01U;

    return bmm150_bus_write(bmm150, BMM150_REG_ADDRESS_OPMODE, &data, sizeof(data));
}

bmm150_err_t bmm150_get_int_ctrl1_reg(bmm150_t const* bmm150, bmm150_int_ctrl1_reg_t* reg)
{
    assert(bmm150 && reg);

    uint8_t data = {};

    bmm150_err_t err = bmm150_bus_read(bmm150, BMM150_REG_ADDRESS_INT_CTRL1, &data, sizeof(data));

    reg->data_overrun_en = (data >> 7U) & 0x01U;
    reg->overflow_int_en = (data >> 6U) & 0x01U;
    reg->high_int_z_en = (data >> 5U) & 0x01U;
    reg->high_int_y_en = (data >> 4U) & 0x01U;
    reg->high_int_z_en = (data >> 3U) & 0x01U;
    reg->low_int_z_en = (data >> 2U) & 0x01U;
    reg->low_int_y_en = (data >> 2U) & 0x01U;
    reg->low_int_x_en = data & 0x01U;

    return err;
}

bmm150_err_t bmm150_set_int_ctrl1_reg(bmm150_t const* bmm150, bmm150_int_ctrl1_reg_t const* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_get_int_ctrl2_reg(bmm150_t const* bmm150, bmm150_int_ctrl2_reg_t* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_set_int_ctrl2_reg(bmm150_t const* bmm150, bmm150_int_ctrl2_reg_t const* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_get_int_low_thresh_reg(bmm150_t const* bmm150, bmm150_int_low_thresh_reg_t* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_set_int_low_thresh_reg(bmm150_t const* bmm150,
                                           bmm150_int_low_thresh_reg_t const* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_get_int_high_thresh_reg(bmm150_t const* bmm150,
                                            bmm150_int_high_thresh_reg_t* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_set_int_high_thresh_reg(bmm150_t const* bmm150,
                                            bmm150_int_high_thresh_reg_t const* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_get_axis_rep_xy_reg(bmm150_t const* bmm150, bmm150_axis_rep_xy_reg_t* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_set_axis_rep_xy_reg(bmm150_t const* bmm150, bmm150_axis_rep_xy_reg_t const* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_get_axis_rep_z_reg(bmm150_t const* bmm150, bmm150_axis_rep_z_reg_t* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_set_axis_rep_z_reg(bmm150_t const* bmm150, bmm150_axis_rep_z_reg_t const* reg)
{
    assert(bmm150 && reg);
}

bmm150_err_t bmm150_send_softreset_cmd(bmm150_t const* bmm150)
{
    assert(bmm150);

    uint8_t data = BMM150_CMD_SOFTRESET;

    return bmm150_bus_write(bmm150, data, NULL, 0UL);
}

bmm150_err_t bmm150_send_suspend_cmd(bmm150_t const* bmm150)
{
    assert(bmm150);

    uint8_t data = BMM150_CMD_SUSPEND;

    return bmm150_bus_write(bmm150, data, NULL, 0UL);
}
