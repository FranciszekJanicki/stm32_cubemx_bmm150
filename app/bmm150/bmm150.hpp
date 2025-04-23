#ifndef BMM150_HPP
#define BMM150_HPP

#include "bmm150_config.hpp"
#include "bmm150_registers.hpp"
#include "bmm150_utility.hpp"

namespace bmm150 {

    struct BMM150 {
    public:
        void initialize(this BMM150& self) noexcept;
        void deinitialize(this BMM150& self) noexcept;

        std::int16_t get_mag_x_raw(this BMM150 const& self) noexcept;
        std::int16_t get_mag_y_raw(this BMM150 const& self) noexcept;
        std::int16_t get_mag_z_raw(this BMM150 const& self) noexcept;
        Vec3<std::int16_t> get_mag_xyz_raw(this BMM150 const& self) noexcept;

        std::float32_t get_mag_x_scaled(this BMM150 const& self) noexcept;
        std::float32_t get_mag_y_scaled(this BMM150 const& self) noexcept;
        std::float32_t get_mag_z_scaled(this BMM150 const& self) noexcept;
        Vec3<std::float32_t> get_mag_xyz_scaled(this BMM150 const& self) noexcept;

        std::uint16_t get_rhall(this BMM150 const& self) noexcept;

        TrimData get_trim_data(this BMM150 const& self) noexcept;

        std::uint8_t get_chip_id(this BMM150 const& self) noexcept;

        Config config = {};
        TrimData trim_data = {};

    private:
        void init(this BMM150& self) noexcept;
        void deinit(this BMM150& self) noexcept;

        template <std::size_t SIZE>
        void transmit(this BMM150 const& self, std::array<std::uint8_t, SIZE> const& data) noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> receive(this BMM150 const& self) noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read(this BMM150 const& self,
                                            std::uint8_t const addresse) noexcept;

        template <std::size_t SIZE>
        void write(this BMM150 const& self,
                   std::uint8_t const address,
                   std::array<std::uint8_t, SIZE> const& data) noexcept;

        void delay(this BMM150& self, std::uint32_t const delay) noexcept;

        CHIP_ID get_chip_id_reg(this BMM150 const& self) noexcept;

        DATA_X get_data_x_reg(this BMM150 const& self) noexcept;
        DATA_Y get_data_y_reg(this BMM150 const& self) noexcept;
        DATA_Z get_data_z_reg(this BMM150 const& self) noexcept;

        DATA_XYZ get_data_xyz_reg(this BMM150 const& self) noexcept;

        RHALL get_rhall_reg(this BMM150 const& self) noexcept;

        INT_STAT get_int_stat_reg(this BMM150 const& self) noexcept;

        PWR_RST get_pwr_rst_reg(this BMM150 const& self) noexcept;
        void set_pwr_rst_reg(this BMM150 const& self, PWR_RST const reg) noexcept;

        ODR_OPMODE get_odr_opmode_reg(this BMM150 const& self) noexcept;
        void set_odr_opmode_reg(this BMM150 const& self, ODR_OPMODE const reg) noexcept;

        INT_CTRL1 get_int_ctrl1_reg(this BMM150 const& self) noexcept;
        void set_int_ctrl1_reg(this BMM150 const& self, INT_CTRL1 const reg) noexcept;

        INT_CTRL2 get_int_ctrl2_reg(this BMM150 const& self) noexcept;
        void set_int_ctrl2_reg(this BMM150 const& self, INT_CTRL2 const reg) noexcept;

        INT_LOW_THRESH get_int_low_thresh_reg(this BMM150 const& self) noexcept;
        void set_int_low_thresh_reg(this BMM150 const& self, INT_LOW_THRESH const reg) noexcept;

        INT_HIGH_THRESH get_int_high_thresh_reg(this BMM150 const& self) noexcept;
        void set_int_high_thresh_reg(this BMM150 const& self, INT_HIGH_THRESH const reg) noexcept;

        AXIS_REP_XY get_axis_rep_xy_reg(this BMM150 const& self) noexcept;
        void set_axis_rep_xy_reg(this BMM150 const& self, AXIS_REP_XY const reg) noexcept;

        AXIS_REP_Z get_axis_rep_z_reg(this BMM150 const& self) noexcept;
        void set_axis_rep_z_reg(this BMM150 const& self, AXIS_REP_Z const reg) noexcept;

        void send_softreset_cmd(this BMM150 const& self) noexcept;
        void send_suspend_cmd(this BMM150 const& self) noexcept;
    };

    template <std::size_t SIZE>
    void BMM150::transmit(this BMM150 const& self,
                          std::array<std::uint8_t, SIZE> const& data) noexcept
    {
        self.config.transmit(self.config.user, data.data(), data.size());
    }

    template <std::size_t SIZE>
    std::array<std::uint8_t, SIZE> BMM150::receive(this BMM150 const& self) noexcept
    {
        auto data = std::array<std::uint8_t, SIZE>{};
        self.config.receive(self.config.user, data.data(), data.size());

        return data;
    }

    template <std::size_t SIZE>
    std::array<std::uint8_t, SIZE> BMM150::read(this BMM150 const& self,
                                                std::uint8_t const address) noexcept
    {
        auto data = std::array<std::uint8_t, SIZE>{};
        self.config.read(self.config.user, address, data.data(), data.size());

        return data;
    }

    template <std::size_t SIZE>
    void BMM150::write(this BMM150 const& self,
                       std::uint8_t const address,
                       std::array<std::uint8_t, SIZE> const& data) noexcept
    {
        self.config.write(self.config.user, address, data.data(), data.size());
    }

}; // namespace bmm150

#endif // BMM150_HPP