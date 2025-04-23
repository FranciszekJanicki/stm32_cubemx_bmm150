#ifndef BMM150_UTILITY_HPP
#define BMM150_UTILITY_HPP

#include "bmm150_config.hpp"
#include <cstdint>
#include <stdfloat>

namespace bmm150 {

    template <typename T>
    struct Vec3 {
        T x;
        T y;
        T z;
    };

    std::float32_t x_raw_to_scaled(TrimData const& data,
                                   Range const range,
                                   std::uint16_t const rhall,
                                   std::int16_t const raw) noexcept;

    std::float32_t y_raw_to_scaled(TrimData const& data,
                                   Range const range,
                                   std::uint16_t const rhall,
                                   std::int16_t const raw) noexcept;

    std::float32_t z_raw_to_scaled(TrimData const& data,
                                   Range const range,
                                   std::uint16_t const rhall,
                                   std::int16_t const raw) noexcept;

    Vec3<std::float32_t> xyz_raw_to_scaled(TrimData const& data,
                                           Range const range,
                                           std::uint16_t const rhall,
                                           Vec3<std::int16_t> const& raw) noexcept;

    std::float32_t range_to_scale(Range const range) noexcept;

} // namespace bmm150

#endif // BMM150_UTILITY_HPP