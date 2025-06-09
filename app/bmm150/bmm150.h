#ifndef BMM150_BMM150_H
#define BMM150_BMM150_H

#include "bmm150_config.h"
#include "bmm150_registers.h"
#include "bmm150_utility.h"

typedef struct {
    bmm150_config_t config;
    bmm150_interface_t interface;
    bmm150_trim_data_t trim_data;
} bmm150_t;

bmm150_err_t bmm150_initialize(bmm150_t* bmm150,
                               bmm150_config_t const* config,
                               bmm150_interface_t const* interface);
bmm150_err_t bmm150_deinitialize(bmm150_t* bmm150);

bmm150_err_t bmm150_get_mag_data_x_scaled(bmm150_t const* bmm150, float32_t* scaled);
bmm150_err_t bmm150_get_mag_data_y_scaled(bmm150_t const* bmm150, float32_t* scaled);
bmm150_err_t bmm150_get_mag_data_z_scaled(bmm150_t const* bmm150, float32_t* scaled);
bmm150_err_t bmm150_get_mag_data_xyz_scaled(bmm150_t const* bmm150, vec3_float32_t* scaled);

bmm150_err_t bmm150_get_mag_data_x_raw(bmm150_t const* bmm150, int16_t* raw);
bmm150_err_t bmm150_get_mag_data_y_raw(bmm150_t const* bmm150, int16_t* raw);
bmm150_err_t bmm150_get_mag_data_z_raw(bmm150_t const* bmm150, int16_t* raw);
bmm150_err_t bmm150_get_mag_data_xyz_raw(bmm150_t const* bmm150, vec3_int16_t* raw);

bmm150_err_t bmm150_get_rhall_data(bmm150_t const* bmm150, uint16_t* data);

bmm150_err_t bmm150_get_trim_data(bmm150_t const* bmm150, bmm150_trim_data_t* data);

bmm150_err_t bmm150_get_chip_id_reg(bmm150_t const* bmm150, bmm150_chip_id_reg_t* reg);

bmm150_err_t bmm150_get_data_x_reg(bmm150_t const* bmm150, bmm150_data_x_reg_t* reg);
bmm150_err_t bmm150_get_data_y_reg(bmm150_t const* bmm150, bmm150_data_y_reg_t* reg);
bmm150_err_t bmm150_get_data_z_reg(bmm150_t const* bmm150, bmm150_data_z_reg_t* reg);

bmm150_err_t bmm150_get_data_xyz_reg(bmm150_t const* bmm150, bmm150_data_xyz_reg_t* reg);

bmm150_err_t bmm150_get_rhall_reg(bmm150_t const* bmm150, bmm150_rhall_reg_t* reg);

bmm150_err_t bmm150_get_int_stat_reg(bmm150_t const* bmm150, bmm150_int_stat_reg_t* reg);

bmm150_err_t bmm150_get_pwr_rst_reg(bmm150_t const* bmm150, bmm150_pwr_rst_reg_t* reg);
bmm150_err_t bmm150_set_pwr_rst_reg(bmm150_t const* bmm150, bmm150_pwr_rst_reg_t const* reg);

bmm150_err_t bmm150_get_odr_opmode_reg(bmm150_t const* bmm150, bmm150_opmode_reg_t* reg);
bmm150_err_t bmm150_set_odr_opmode_reg(bmm150_t const* bmm150, bmm150_opmode_reg_t const* reg);

bmm150_err_t bmm150_get_int_ctrl1_reg(bmm150_t const* bmm150, bmm150_int_ctrl1_reg_t* reg);
bmm150_err_t bmm150_set_int_ctrl1_reg(bmm150_t const* bmm150, bmm150_int_ctrl1_reg_t const* reg);

bmm150_err_t bmm150_get_int_ctrl2_reg(bmm150_t const* bmm150, bmm150_int_ctrl2_reg_t* reg);
bmm150_err_t bmm150_set_int_ctrl2_reg(bmm150_t const* bmm150, bmm150_int_ctrl2_reg_t const* reg);

bmm150_err_t bmm150_get_int_low_thresh_reg(bmm150_t const* bmm150,
                                           bmm150_int_low_thresh_reg_t* reg);
bmm150_err_t bmm150_set_int_low_thresh_reg(bmm150_t const* bmm150,
                                           bmm150_int_low_thresh_reg_t const* reg);

bmm150_err_t bmm150_get_int_high_thresh_reg(bmm150_t const* bmm150,
                                            bmm150_int_high_thresh_reg_t* reg);
bmm150_err_t bmm150_set_int_high_thresh_reg(bmm150_t const* bmm150,
                                            bmm150_int_high_thresh_reg_t const* reg);

bmm150_err_t bmm150_get_axis_rep_xy_reg(bmm150_t const* bmm150, bmm150_axis_rep_xy_reg_t* reg);
bmm150_err_t bmm150_set_axis_rep_xy_reg(bmm150_t const* bmm150,
                                        bmm150_axis_rep_xy_reg_t const* reg);

bmm150_err_t bmm150_get_axis_rep_z_reg(bmm150_t const* bmm150, bmm150_axis_rep_z_reg_t* reg);
bmm150_err_t bmm150_set_axis_rep_z_reg(bmm150_t const* bmm150, bmm150_axis_rep_z_reg_t const* reg);

bmm150_err_t bmm150_send_softreset_cmd(bmm150_t const* bmm150);
bmm150_err_t bmm150_send_suspend_cmd(bmm150_t const* bmm150);

#endif // BMM150_BMM150_H