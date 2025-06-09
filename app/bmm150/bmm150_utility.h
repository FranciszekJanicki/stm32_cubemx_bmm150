#ifndef BMM150_BMM150_UTILITY_H
#define BMM150_BMM150_UTILITY_H

#include "bmm150_config.h"
#include <assert.h>

inline float32_t bmm150_x_raw_to_scaled(bmm150_trim_data_t const* data, uint16_t rhall, int16_t raw)
{
    assert(data);

    float32_t scaled = {};
    float32_t process_comp[5];

    if ((raw != BMM150_OVERFLOW_XY_FLIP) && (rhall != 0) && (data->dig_xyz1 != 0)) {
        process_comp[0] = (((float32_t)data->dig_xyz1) * 16384.0f / rhall);
        scaled = process_comp[0] - 16384.0f;
        process_comp[1] = ((float32_t)data->dig_xy2) * (scaled * scaled / 268435456.0f);
        process_comp[2] = process_comp[1] + scaled * ((float32_t)data->dig_xy1) / 16384.0f;
        process_comp[3] = ((float32_t)data->dig_x2) + 160.0f;
        process_comp[4] = raw * ((process_comp[2] + 256.0f) * process_comp[3]);
        scaled =
            ((process_comp[4] / 8192.0f) + (((float32_t)data->dig_x1) * 8.0f)) * BMM150_LSB_SCALE;
    }

    return scaled;
}

inline float32_t bmm150_y_raw_to_scaled(bmm150_trim_data_t const* data, uint16_t rhall, int16_t raw)
{
    assert(data);

    float32_t scaled = {};
    float32_t process_comp[5];

    if ((raw != BMM150_OVERFLOW_XY_FLIP) && (rhall != 0) && (data->dig_xyz1 != 0)) {
        process_comp[0] = (((float32_t)data->dig_xyz1) * 16384.0f / rhall);
        scaled = process_comp[0] - 16384.0f;
        process_comp[1] = ((float32_t)data->dig_xy2) * (scaled * scaled / 268435456.0f);
        process_comp[2] = process_comp[1] + scaled * ((float32_t)data->dig_xy1) / 16384.0f;
        process_comp[3] = ((float32_t)data->dig_y2) + 160.0f;
        process_comp[4] = raw * ((process_comp[2] + 256.0f) * process_comp[3]);
        scaled =
            ((process_comp[4] / 8192.0f) + (((float32_t)data->dig_y1) * 8.0f)) * BMM150_LSB_SCALE;
    }

    return scaled;
}

inline float32_t bmm150_z_raw_to_scaled(bmm150_trim_data_t const* data, uint16_t rhall, int16_t raw)
{
    assert(data);

    float32_t scaled = {};
    float32_t process_comp[6];

    if ((raw != BMM150_OVERFLOW_Z_HALL) && (data->dig_z2 != 0) && (data->dig_z1 != 0) &&
        (data->dig_xyz1 != 0) && (rhall != 0)) {
        process_comp[0] = ((float32_t)raw) - ((float32_t)data->dig_z4);
        process_comp[1] = ((float32_t)rhall) - ((float32_t)data->dig_xyz1);
        process_comp[2] = ((float32_t)data->dig_z3) * process_comp[1];
        process_comp[3] = ((float32_t)data->dig_z1) * ((float32_t)rhall) / 32768.0f;
        process_comp[4] = ((float32_t)data->dig_z2) + process_comp[3];
        process_comp[5] = (process_comp[0] * 131072.0f) - process_comp[2];
        scaled = (process_comp[5] / ((process_comp[4]) * 4.0f)) * BMM150_LSB_SCALE;
    }

    return scaled;
}

inline vec3_float32_t bmm150_xyz_raw_to_scaled(bmm150_trim_data_t const* data,
                                               uint16_t rhall,
                                               vec3_int16_t const* raw)
{
    assert(data && raw);

    return (vec3_float32_t){.x = bmm150_x_raw_to_scaled(data, rhall, raw->x),
                            .y = bmm150_y_raw_to_scaled(data, rhall, raw->y),
                            .z = bmm150_z_raw_to_scaled(data, rhall, raw->z)};
}

#endif // BMM150_BMM150_UTILITY_H