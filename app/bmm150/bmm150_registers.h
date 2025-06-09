#ifndef BMM150_BMM150_REGISTERS_H
#define BMM150_BMM150_REGISTERS_H

#include <stdint.h>

typedef struct {
    uint8_t chip_id : 8;
} bmm150_chip_id_reg_t;

typedef struct {
    uint8_t x_self_test : 1;
    int16_t data_x : 13;
} bmm150_data_x_reg_t;

typedef struct {
    uint8_t y_self_test : 1;
    int16_t data_y : 13;
} bmm150_data_y_reg_t;

typedef struct {
    uint8_t z_self_test : 1;
    int16_t data_z : 15;
} bmm150_data_z_reg_t;

typedef struct {
    uint8_t data_ready_status : 1;
    uint16_t rhall : 14;
} bmm150_rhall_reg_t;

typedef struct {
    int16_t data_x : 13;
    int16_t data_y : 13;
    int16_t data_z : 15;
} bmm150_data_xyz_reg_t;

typedef struct {
    uint8_t data_overrun : 1;
    uint8_t overflow : 1;
    uint8_t high_int_z : 1;
    uint8_t high_int_y : 1;
    uint8_t high_int_x : 1;
    uint8_t low_int_z : 1;
    uint8_t low_int_y : 1;
    uint8_t low_int_x : 1;
} bmm150_int_stat_reg_t;

typedef struct {
    uint8_t soft_reset_1 : 1;
    uint8_t spi3_en : 1;
    uint8_t soft_reset_2 : 1;
    uint8_t power_control : 1;
} bmm150_pwr_rst_reg_t;

typedef struct {
    uint8_t adv_st : 2;
    uint8_t data_rate : 3;
    uint8_t opmode : 2;
    uint8_t self_test : 1;
} bmm150_opmode_reg_t;

typedef struct {
    uint8_t data_overrun_en : 1;
    uint8_t overflow_int_en : 1;
    uint8_t high_int_z_en : 1;
    uint8_t high_int_y_en : 1;
    uint8_t high_int_x_en : 1;
    uint8_t low_int_z_en : 1;
    uint8_t low_int_y_en : 1;
    uint8_t low_int_x_en : 1;
} bmm150_int_ctrl1_reg_t;

typedef struct {
    uint8_t data_ready_pin_en : 1;
    uint8_t interrupt_pin_en : 1;
    uint8_t channel_z : 1;
    uint8_t channel_y : 1;
    uint8_t channel_x : 1;
    uint8_t dr_polarity : 1;
    uint8_t interrupt_latch : 1;
    uint8_t interrupt_polarity : 1;
} bmm150_int_ctrl2_reg_t;

typedef struct {
    int8_t low_threshold : 8;
} bmm150_int_low_thresh_reg_t;

typedef struct {
    int8_t high_threshold : 8;
} bmm150_int_high_thresh_reg_t;

typedef struct {
    uint8_t repetitions_xy : 8;
} bmm150_axis_rep_xy_reg_t;

typedef struct {
    uint8_t repetitions_z : 8;
} bmm150_axis_rep_z_reg_t;

#endif // BMM150_BMM150_REGISTERS_H