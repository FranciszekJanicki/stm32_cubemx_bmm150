#include "bmm150_utility.hpp"
#include "bmm150_config.hpp"

namespace bmm150 {

    std::float32_t range_to_scale(Range const range) noexcept
    {
        switch (range) {
            case Range::FULL_SCALE:
                return 1.0F / 16.0F;
            default:
                return 0.0F;
        }
    }

    std::float32_t x_raw_to_scaled(TrimData const& data,
                                   Range const range,
                                   std::uint16_t const rhall,
                                   std::int16_t const raw) noexcept
    {
        std::float32_t scaled = 0.0F;
        std::float32_t process_comp[5];

        if ((raw != BMM150_OVERFLOW_XY_FLIP) && (rhall != 0) && (data.dig_xyz1 != 0)) {
            process_comp[0] = (((std::float32_t)data.dig_xyz1) * 16384.0f / rhall);
            scaled = process_comp[0] - 16384.0f;
            process_comp[1] = ((std::float32_t)data.dig_xy2) * (scaled * scaled / 268435456.0f);
            process_comp[2] = process_comp[1] + scaled * ((std::float32_t)data.dig_xy1) / 16384.0f;
            process_comp[3] = ((std::float32_t)data.dig_x2) + 160.0f;
            process_comp[4] = raw * ((process_comp[2] + 256.0f) * process_comp[3]);
            scaled = ((process_comp[4] / 8192.0f) + (((std::float32_t)data.dig_x1) * 8.0f)) *
                     range_to_scale(range);
        }

        return scaled;
    }

    std::float32_t y_raw_to_scaled(TrimData const& data,
                                   Range const range,
                                   std::uint16_t const rhall,
                                   std::int16_t const raw) noexcept
    {
        std::float32_t scaled = 0.0F;
        std::float32_t process_comp[5];

        if ((raw != BMM150_OVERFLOW_XY_FLIP) && (rhall != 0) && (data.dig_xyz1 != 0)) {
            process_comp[0] = (((std::float32_t)data.dig_xyz1) * 16384.0f / rhall);
            scaled = process_comp[0] - 16384.0f;
            process_comp[1] = ((std::float32_t)data.dig_xy2) * (scaled * scaled / 268435456.0f);
            process_comp[2] = process_comp[1] + scaled * ((std::float32_t)data.dig_xy1) / 16384.0f;
            process_comp[3] = ((std::float32_t)data.dig_y2) + 160.0f;
            process_comp[4] = raw * ((process_comp[2] + 256.0f) * process_comp[3]);
            scaled = ((process_comp[4] / 8192.0f) + (((std::float32_t)data.dig_y1) * 8.0f)) *
                     range_to_scale(range);
        }

        return scaled;
    }

    std::float32_t z_raw_to_scaled(TrimData const& data,
                                   Range const range,
                                   std::uint16_t const rhall,
                                   std::int16_t const raw) noexcept
    {
        std::float32_t scaled = 0.0F;
        std::float32_t process_comp[6];

        if ((raw != BMM150_OVERFLOW_Z_HALL) && (data.dig_z2 != 0) && (data.dig_z1 != 0) &&
            (data.dig_xyz1 != 0) && (rhall != 0)) {
            process_comp[0] = ((std::float32_t)raw) - ((std::float32_t)data.dig_z4);
            process_comp[1] = ((std::float32_t)rhall) - ((std::float32_t)data.dig_xyz1);
            process_comp[2] = ((std::float32_t)data.dig_z3) * process_comp[1];
            process_comp[3] = ((std::float32_t)data.dig_z1) * ((std::float32_t)rhall) / 32768.0f;
            process_comp[4] = ((std::float32_t)data.dig_z2) + process_comp[3];
            process_comp[5] = (process_comp[0] * 131072.0f) - process_comp[2];
            scaled = (process_comp[5] / ((process_comp[4]) * 4.0f)) * range_to_scale(range);
        }

        return scaled;
    }

    Vec3<std::float32_t> xyz_raw_to_scaled(TrimData const& data,
                                           Range const range,
                                           std::uint16_t const rhall,
                                           Vec3<std::int16_t> const& raw) noexcept
    {
        return (Vec3<std::float32_t>){.x = x_raw_to_scaled(data, range, rhall, raw.x),
                                      .y = y_raw_to_scaled(data, range, rhall, raw.z),
                                      .z = z_raw_to_scaled(data, range, rhall, raw.z)};
    }

}; // namespace bmm150