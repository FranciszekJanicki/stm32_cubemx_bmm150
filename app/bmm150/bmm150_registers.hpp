#ifndef BMM150_REGISTERS_HPP
#define BMM150_REGISTERS_HPP

#include <cstdint>

#define PACKED __attribute__((__packed__))

namespace bmm150 {

    struct CHIP_ID {
        std::uint8_t chip_id : 8;
    } PACKED;

    struct DATA_X {
        std::uint8_t x_lsb : 5;
        std::uint8_t : 2;
        std::uint8_t x_self_test : 1;
        std::uint8_t x_msb : 8;
    } PACKED;

    struct DATA_Y {
        std::uint8_t y_lsb : 5;
        std::uint8_t : 2;
        std::uint8_t y_self_test : 1;
        std::uint8_t y_msb : 8;
    } PACKED;

    struct DATA_Z {
        std::uint8_t z_lsb : 7;
        std::uint8_t z_self_test : 1;
        std::uint8_t z_msb : 8;
    } PACKED;

    struct RHALL {
        std::uint8_t rhall_lsb : 6;
        std::uint8_t : 1;
        std::uint8_t data_ready_status : 1;
        std::uint8_t rhall_msb : 8;
    } PACKED;

    struct DATA_XYZ {
        DATA_X data_x;
        DATA_Y data_y;
        DATA_Z data_z;
    } PACKED;

    struct INT_STAT {
        std::uint8_t data_overrun : 1;
        std::uint8_t overflow : 1;
        std::uint8_t high_int_z : 1;
        std::uint8_t high_int_y : 1;
        std::uint8_t low_int_z : 1;
        std::uint8_t low_int_y : 1;
        std::uint8_t low_int_x : 1;
    } PACKED;

    struct PWR_RST {
        std::uint8_t soft_reset_1 : 1;
        std::uint8_t : 4;
        std::uint8_t spi3_en : 1;
        std::uint8_t soft_reset_2 : 1;
        std::uint8_t power_control : 1;
    } PACKED;

    struct ODR_OPMODE {
        std::uint8_t adv_st : 2;
        std::uint8_t data_rate : 3;
        std::uint8_t opmode : 2;
        std::uint8_t self_test : 1;
    } PACKED;

    struct INT_CTRL1 {
        std::uint8_t data_overrun_en : 1;
        std::uint8_t overflow_int_en : 1;
        std::uint8_t high_int_z_en : 1;
        std::uint8_t high_int_y_en : 1;
        std::uint8_t high_int_x_en : 1;
        std::uint8_t low_int_z_en : 1;
        std::uint8_t low_int_y_en : 1;
        std::uint8_t low_int_x_en : 1;
    } PACKED;

    struct INT_CTRL2 {
        std::uint8_t data_ready_pin_en : 1;
        std::uint8_t interrupt_pin_en : 1;
        std::uint8_t channel_z : 1;
        std::uint8_t channel_y : 1;
        std::uint8_t channel_x : 1;
        std::uint8_t dr_polarity : 1;
        std::uint8_t interrupt_latch : 1;
        std::uint8_t interrupt_polarity : 1;
    } PACKED;

    struct INT_LOW_THRESH {
        std::uint8_t low_threshold : 8;
    } PACKED;

    struct INT_HIGH_THRESH {
        std::uint8_t high_threshold : 8;
    } PACKED;

    struct AXIS_REP_XY {
        std::uint8_t repetitions_xy : 8;
    } PACKED;

    struct AXIS_REP_Z {
        std::uint8_t repetitions_z : 8;
    } PACKED;

}; // namespace bmm150

#undef PACKED

#endif // BMM150_REGISTERS_H