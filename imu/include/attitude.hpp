#pragma once
#include "geometry.hpp"
#include "msg.hpp"
#include "status.hpp"

namespace imu::attitude
{
enum class source : uint8_t
{
    none,
    bmi088,
    dmimu
};
enum class phase : uint8_t
{
    starting,
    warming,
    calibrating,
    running,
    offline,
    fault
};

struct state
{
    source device = source::none;
    phase stage = phase::starting;
    pnxtypes::status result = pnxtypes::status::not_initialized;
    math::quaternion quaternion{};
    math::vector3 gyro{}; // rad/s; BMI088 includes ESKF residual-bias correction.
    math::vector3
        accel{}; // m/s^2, sensor axes, includes gravity; no mounting/lever-arm compensation.
    math::vector3 gyro_bias{};                         // ESKF residual bias, BMI088 only.
    float roll = 0, pitch = 0, yaw = 0, total_yaw = 0; // radians
    float temperature = 0;
    float nis = 0;         // BMI088 ESKF diagnostic.
    uint32_t sequence = 0; // Valid attitude publications, preserved across recovery.
    uint32_t received_ms = 0;
    uint32_t dropped_frames = 0;
    bool online = false;
    bool accel_updated = false;
    bool zero_rate_active = false;
};

// Subscribe after starting the selected device service. Only one service may publish.
// Latest-value delivery; consumers must check online, stage and timestamp.
// BMI088 uses ESKF NWU body-to-world; DMIMU retains its device frame convention.
inline msg::topic<state> output;
} // namespace imu::attitude
