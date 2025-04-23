#ifndef BMM150_CONFIG_HPP
#define BMM150_CONFIG_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdfloat>

namespace bmm150 {

    constexpr auto CHIP_ID_VALUE = 0x32U;
    constexpr auto BMM150_OVERFLOW_XY_FLIP = -4096;
    constexpr auto BMM150_OVERFLOW_Z_HALL = -16384;

    enum struct SlaveAddr : std::uint8_t {
        CSB_TO_GND_SD0_TO_GND = 0x10,
        CSB_TO_GND_SD0_TO_VDD = 0x11,
        CSB_TO_VDD_SD0_TO_GND = 0x12,
        CSB_TO_VDD_SD0_TO_VDD = 0x13,
    };

    enum struct Error : std::uint8_t {
        OK = 0,
        FAIL,
    };

    enum struct RegAddr : std::uint8_t {
        CHIP_ID = 0x40,
        DATAX_LSB = 0x42,
        DATAX_MSB = 0x43,
        DATAY_LSB = 0x44,
        DATAY_MSB = 0x45,
        DATAZ_LSB = 0x46,
        DATAZ_MSB = 0x47,
        RHALL_LSB = 0x48,
        RHALL_MSB = 0x49,
        INT_STAT = 0x4A,
        PWR_RST = 0x4B,
        ODR_OPMODE = 0x4C,
        INT_CTRL1 = 0x4D,
        INT_CTRL2 = 0x4E,
        INT_HIGH_THRESH = 0x4F,
        INT_LOW_THRESH = 0x50,
        AXIS_REP_XY = 0x51,
        AXIS_REP_Z = 0x52,
        DIG_X1 = 0x5d,
        DIG_Y1 = 0x5e,
        DIG_Z4_LSB = 0x62,
        DIG_Z4_MSB = 0x63,
        DIG_X2 = 0x64,
        DIG_Y2 = 0x65,
        DIG_Z2_LSB = 0x68,
        DIG_Z2_MSB = 0x69,
        DIG_Z1_LSB = 0x6a,
        DIG_Z1_MSB = 0x6b,
        DIG_XYZ1_LSB = 0x6c,
        DIG_XYZ1_MSB = 0x6d,
        DIG_Z3_LSB = 0x6e,
        DIG_Z3_MSB = 0x6f,
        DIG_XY2 = 0x70,
        DIG_XY1 = 0x71,
    };

    enum struct Command : std::uint8_t {
        SOFTRESET = 0x82,
        SUSPEND = 0x01,
    };

    enum struct AdvST : std::uint8_t {
        NORMAL = 0b00,
        NEGATIVE = 0b10,
        POSITIVE = 0b11,
    };

    enum struct ODR : std::uint8_t {
        ODR_10HZ = 0b000,
        ODR_2HZ = 0b001,
        ODR_6HZ = 0b010,
        ODR_8HZ = 0b011,
        ODR_15HZ = 0b100,
        ODR_20HZ = 0b101,
        ODR_25HZ = 0b110,
        ODR_30HZ = 0b111,
    };

    enum struct OPMode : std::uint8_t {
        NORMAL = 0b00,
        FORCED = 0b01,
        SLEEP = 0b11,
    };

    enum struct DRPolarity : std::uint8_t {
        ACTIVE_LOW = 0b0,
        ACTIVE_HIGH = 0b01,
    };

    enum struct InterruptLatch : std::uint8_t {
        LATCH_EN = 0b00,
        LATCH_DIS = 0b01,
    };

    enum struct InterruptPolarity : std::uint8_t {
        POLARITY_LOW = 0b00,
        POLARITY_HIGH = 0b01,
    };

    enum struct RepXY : std::uint8_t {
        LOWPOWER = 1,
        REGULAR = 4,
        ENHANCED = 7,
        HIGHACCURACY = 23,
    };

    enum struct RepZ : std::uint8_t {
        LOWPOWER = 2,
        REGULAR = 14,
        ENHANCED = 26,
        HIGHACCURACY = 82,
    };

    enum struct Range : std::uint8_t {
        FULL_SCALE,
    };

    struct TrimData {
        int8_t dig_x1;
        int8_t dig_y1;
        int8_t dig_x2;
        int8_t dig_y2;
        std::uint16_t dig_z1;
        std::int16_t dig_z2;
        std::int16_t dig_z3;
        std::int16_t dig_z4;
        std::uint8_t dig_xy1;
        int8_t dig_xy2;
        std::uint16_t dig_xyz1;
    };

    using Transmit = void (*)(void const*, std::uint8_t const*, std::size_t const);
    using Write = void (*)(void const*, std::uint8_t const, std::uint8_t const*, std::size_t const);
    using Receive = void (*)(void const*, std::uint8_t*, std::size_t const);
    using Read = void (*)(void const*, std::uint8_t const, std::uint8_t*, std::size_t const);
    using Init = void (*)(void*);
    using Deinit = void (*)(void*);
    using Delay = void (*)(std::uint32_t const);

    struct Config {
        void* user = {};

        Transmit transmit = {};
        Receive receive = {};
        Read read = {};
        Write write = {};
        Init init = {};
        Deinit deinit = {};
        Delay delay = {};

        Range range = {};
    };

}; // namespace bmm150

#endif // BMM150_CONFIG_HPP