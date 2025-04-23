#include "bmm150.hpp"
#include "bmm150_registers.hpp"
#include "bmm150_utility.hpp"
#include <bit>
#include <cstring>
#include <utility>

namespace bmm150 {

    void BMM150::initialize(this BMM150& self) noexcept
    {
        self.init();

        auto pwr_rst_reg = self.get_pwr_rst_reg();
        auto odr_opmode_reg = self.get_odr_opmode_reg();
        auto axis_rep_xy_reg = self.get_axis_rep_xy_reg();
        auto axis_rep_z_reg = self.get_axis_rep_z_reg();
        auto int_ctrl1_reg = self.get_int_ctrl1_reg();
        auto int_ctrl2_reg = self.get_int_ctrl2_reg();

        pwr_rst_reg.power_control = 1;
        pwr_rst_reg.soft_reset_1 = 1;
        pwr_rst_reg.soft_reset_2 = 1;
        self.set_pwr_rst_reg(pwr_rst_reg);

        self.delay(10);

        odr_opmode_reg.opmode = std::to_underlying(OPMode::NORMAL);
        self.set_odr_opmode_reg(odr_opmode_reg);

        auto chip_id = self.get_chip_id();
        if (chip_id != CHIP_ID_VALUE) {
            return;
        }

        self.trim_data = self.get_trim_data();

        pwr_rst_reg.power_control = 0;
        self.set_pwr_rst_reg(pwr_rst_reg);

        self.delay(10);

        pwr_rst_reg.power_control = 1;
        self.set_pwr_rst_reg(pwr_rst_reg);

        self.delay(10);

        odr_opmode_reg.opmode = std::to_underlying(OPMode::SLEEP);
        self.set_odr_opmode_reg(odr_opmode_reg);

        odr_opmode_reg.opmode = std::to_underlying(OPMode::NORMAL);
        self.set_odr_opmode_reg(odr_opmode_reg);

        odr_opmode_reg.data_rate = std::to_underlying(ODR::ODR_10HZ);
        self.set_odr_opmode_reg(odr_opmode_reg);

        axis_rep_xy_reg.repetitions_xy = std::to_underlying(RepXY::REGULAR);
        self.set_axis_rep_xy_reg(axis_rep_xy_reg);

        axis_rep_z_reg.repetitions_z = std::to_underlying(RepZ::REGULAR);
        self.set_axis_rep_z_reg(axis_rep_z_reg);
    }

    void BMM150::deinitialize(this BMM150& self) noexcept
    {
        self.deinit();
    }

    std::int16_t BMM150::get_mag_x_raw(this BMM150 const& self) noexcept
    {
        auto data = self.read<2>(std::to_underlying(RegAddr::DATAX_LSB));

        return static_cast<std::int8_t>((data[0] & 0xF8) >> 3) |
               static_cast<std::int8_t>(data[1] << 5);
    }

    std::int16_t BMM150::get_mag_y_raw(this BMM150 const& self) noexcept
    {
        auto data = self.read<2>(std::to_underlying(RegAddr::DATAY_LSB));

        return static_cast<std::int8_t>((data[0] & 0xF8) >> 3) |
               static_cast<std::int8_t>(data[1] << 5);
    }

    std::int16_t BMM150::get_mag_z_raw(this BMM150 const& self) noexcept
    {
        auto data = self.read<2>(std::to_underlying(RegAddr::DATAZ_LSB));

        return static_cast<std::int8_t>((data[0] & 0xF8) >> 1) |
               static_cast<std::int8_t>(data[1] << 7);
    }

    Vec3<std::int16_t> BMM150::get_mag_xyz_raw(this BMM150 const& self) noexcept
    {
        auto data = self.read<6>(std::to_underlying(RegAddr::DATAX_LSB));

        return Vec3<std::int16_t>{.x = static_cast<std::int8_t>((data[0] & 0xF8) >> 3) |
                                       static_cast<std::int8_t>(data[1] << 5),
                                  .y = static_cast<std::int8_t>((data[2] & 0xF8) >> 3) |
                                       static_cast<std::int8_t>(data[3] << 5),
                                  .z = static_cast<std::int8_t>((data[4] & 0xF8) >> 1) |
                                       static_cast<std::int8_t>(data[5] << 7)};
    }

    std::float32_t BMM150::get_mag_x_scaled(this BMM150 const& self) noexcept
    {
        auto raw = self.get_mag_x_raw();
        auto rhall = self.get_rhall();

        return x_raw_to_scaled(self.trim_data, self.config.range, rhall, raw);
    }

    std::float32_t BMM150::get_mag_y_scaled(this BMM150 const& self) noexcept
    {
        auto raw = self.get_mag_y_raw();
        auto rhall = self.get_rhall();

        return y_raw_to_scaled(self.trim_data, self.config.range, rhall, raw);
    }

    std::float32_t BMM150::get_mag_z_scaled(this BMM150 const& self) noexcept
    {
        auto raw = self.get_mag_z_raw();
        auto rhall = self.get_rhall();

        return z_raw_to_scaled(self.trim_data, self.config.range, rhall, raw);
    }

    Vec3<std::float32_t> BMM150::get_mag_xyz_scaled(this BMM150 const& self) noexcept
    {
        auto raw = self.get_mag_xyz_raw();
        auto rhall = self.get_rhall();

        return xyz_raw_to_scaled(self.trim_data, self.config.range, rhall, raw);
    }

    std::uint16_t BMM150::get_rhall(this BMM150 const& self) noexcept
    {
        auto data = self.read<2>(std::to_underlying(RegAddr::RHALL_LSB));

        return static_cast<std::int8_t>((data[0] & 0xFC) >> 2) |
               static_cast<std::int8_t>(data[1] << 6);
    }

    TrimData BMM150::get_trim_data(this BMM150 const& self) noexcept
    {
        auto trim_x1y1 = self.read<2>(std::to_underlying(RegAddr::DIG_X1));
        auto trim_xyz = self.read<4>(std::to_underlying(RegAddr::DIG_Z4_LSB));
        auto trim_xy1xy2 = self.read<10>(std::to_underlying(RegAddr::DIG_Z2_LSB));

        return TrimData{
            .dig_x1 = static_cast<std::int8_t>(trim_x1y1[0]),
            .dig_y1 = static_cast<std::int8_t>(trim_x1y1[1]),
            .dig_x2 = static_cast<std::int8_t>(trim_xyz[2]),
            .dig_y2 = static_cast<std::int8_t>(trim_xyz[3]),
            .dig_z1 = (std::uint16_t)((((std::uint16_t)trim_xy1xy2[3]) << 8) | trim_xy1xy2[2]),
            .dig_z2 = (int16_t)((((std::uint16_t)trim_xy1xy2[1]) << 8) | trim_xy1xy2[0]),
            .dig_z3 = (int16_t)((((std::uint16_t)trim_xy1xy2[7]) << 8) | trim_xy1xy2[6]),
            .dig_z4 = (int16_t)((((std::uint16_t)trim_xyz[1]) << 8) | trim_xyz[0]),
            .dig_xy1 = trim_xy1xy2[9],
            .dig_xy2 = static_cast<std::int8_t>(trim_xy1xy2[8]),
            .dig_xyz1 =
                (std::uint16_t)(((std::uint16_t)(trim_xy1xy2[5] & 0x7f) << 8) | trim_xy1xy2[4])};
    }

    std::uint8_t BMM150::get_chip_id(this BMM150 const& self) noexcept
    {
        return self.get_chip_id_reg().chip_id;
    }

    CHIP_ID BMM150::get_chip_id_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<CHIP_ID>(
            self.read<sizeof(CHIP_ID)>(std::to_underlying(RegAddr::CHIP_ID)));
    }

    DATA_X BMM150::get_data_x_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<DATA_X>(
            self.read<sizeof(DATA_X)>(std::to_underlying(RegAddr::DATAX_LSB)));
    }

    DATA_Y BMM150::get_data_y_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<DATA_Y>(
            self.read<sizeof(DATA_Y)>(std::to_underlying(RegAddr::DATAY_LSB)));
    }

    DATA_Z BMM150::get_data_z_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<DATA_Z>(
            self.read<sizeof(DATA_Z)>(std::to_underlying(RegAddr::DATAZ_LSB)));
    }

    DATA_XYZ BMM150::get_data_xyz_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<DATA_XYZ>(
            self.read<sizeof(DATA_XYZ)>(std::to_underlying(RegAddr::DATAX_LSB)));
    }

    RHALL BMM150::get_rhall_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<RHALL>(
            self.read<sizeof(RHALL)>(std::to_underlying(RegAddr::RHALL_LSB)));
    }

    INT_STAT BMM150::get_int_stat_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<INT_STAT>(
            self.read<sizeof(INT_STAT)>(std::to_underlying(RegAddr::INT_STAT)));
    }

    PWR_RST BMM150::get_pwr_rst_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<PWR_RST>(
            self.read<sizeof(PWR_RST)>(std::to_underlying(RegAddr::PWR_RST)));
    }

    void BMM150::set_pwr_rst_reg(this BMM150 const& self, PWR_RST const reg) noexcept
    {
        self.write(std::to_underlying(RegAddr::PWR_RST),
                   std::bit_cast<std::array<std::uint8_t, sizeof(PWR_RST)>>(reg));
    }

    ODR_OPMODE BMM150::get_odr_opmode_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<ODR_OPMODE>(
            self.read<sizeof(ODR_OPMODE)>(std::to_underlying(RegAddr::ODR_OPMODE)));
    }

    void BMM150::set_odr_opmode_reg(this BMM150 const& self, ODR_OPMODE const reg) noexcept
    {
        self.write(std::to_underlying(RegAddr::ODR_OPMODE),
                   std::bit_cast<std::array<std::uint8_t, sizeof(ODR_OPMODE)>>(reg));
    }

    INT_CTRL1 BMM150::get_int_ctrl1_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<INT_CTRL1>(
            self.read<sizeof(INT_CTRL1)>(std::to_underlying(RegAddr::INT_CTRL1)));
    }

    void BMM150::set_int_ctrl1_reg(this BMM150 const& self, INT_CTRL1 const reg) noexcept
    {
        self.write(std::to_underlying(RegAddr::INT_CTRL1),
                   std::bit_cast<std::array<std::uint8_t, sizeof(INT_CTRL1)>>(reg));
    }

    INT_CTRL2 BMM150::get_int_ctrl2_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<INT_CTRL2>(
            self.read<sizeof(INT_CTRL2)>(std::to_underlying(RegAddr::INT_CTRL2)));
    }

    void BMM150::set_int_ctrl2_reg(this BMM150 const& self, INT_CTRL2 const reg) noexcept
    {
        self.write(std::to_underlying(RegAddr::INT_CTRL2),
                   std::bit_cast<std::array<std::uint8_t, sizeof(INT_CTRL2)>>(reg));
    }

    INT_LOW_THRESH BMM150::get_int_low_thresh_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<INT_LOW_THRESH>(
            self.read<sizeof(INT_LOW_THRESH)>(std::to_underlying(RegAddr::INT_LOW_THRESH)));
    }

    void BMM150::set_int_low_thresh_reg(this BMM150 const& self, INT_LOW_THRESH const reg) noexcept
    {
        self.write(std::to_underlying(RegAddr::INT_LOW_THRESH),
                   std::bit_cast<std::array<std::uint8_t, sizeof(INT_LOW_THRESH)>>(reg));
    }

    INT_HIGH_THRESH BMM150::get_int_high_thresh_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<INT_HIGH_THRESH>(
            self.read<sizeof(INT_HIGH_THRESH)>(std::to_underlying(RegAddr::INT_HIGH_THRESH)));
    }

    void BMM150::set_int_high_thresh_reg(this BMM150 const& self,
                                         INT_HIGH_THRESH const reg) noexcept
    {
        self.write(std::to_underlying(RegAddr::INT_HIGH_THRESH),
                   std::bit_cast<std::array<std::uint8_t, sizeof(INT_HIGH_THRESH)>>(reg));
    }

    AXIS_REP_XY BMM150::get_axis_rep_xy_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<AXIS_REP_XY>(
            self.read<sizeof(AXIS_REP_XY)>(std::to_underlying(RegAddr::AXIS_REP_XY)));
    }

    void BMM150::set_axis_rep_xy_reg(this BMM150 const& self, AXIS_REP_XY const reg) noexcept
    {
        self.write(std::to_underlying(RegAddr::AXIS_REP_XY),
                   std::bit_cast<std::array<std::uint8_t, sizeof(AXIS_REP_XY)>>(reg));
    }

    AXIS_REP_Z BMM150::get_axis_rep_z_reg(this BMM150 const& self) noexcept
    {
        return std::bit_cast<AXIS_REP_Z>(
            self.read<sizeof(AXIS_REP_Z)>(std::to_underlying(RegAddr::AXIS_REP_Z)));
    }

    void BMM150::set_axis_rep_z_reg(this BMM150 const& self, AXIS_REP_Z const reg) noexcept
    {
        self.write(std::to_underlying(RegAddr::PWR_RST),
                   std::bit_cast<std::array<std::uint8_t, sizeof(PWR_RST)>>(reg));
    }

    void BMM150::send_softreset_cmd(this BMM150 const& self) noexcept
    {
        self.transmit(std::bit_cast<std::array<std::uint8_t, sizeof(Command::SOFTRESET)>>(
            Command::SOFTRESET));
    }

    void BMM150::send_suspend_cmd(this BMM150 const& self) noexcept
    {
        self.transmit(
            std::bit_cast<std::array<std::uint8_t, sizeof(Command::SUSPEND)>>(Command::SUSPEND));
    }

    void BMM150::init(this BMM150& self) noexcept
    {
        self.config.init(self.config.user);
    }

    void BMM150::deinit(this BMM150& self) noexcept
    {
        self.config.deinit(self.config.user);
    }

    void BMM150::delay(this BMM150& self, std::uint32_t const delay) noexcept
    {
        self.config.delay(delay);
    }

}; // namespace bmm150