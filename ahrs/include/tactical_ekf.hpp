#pragma once

#include "tactical_fusion.hpp"

#include <cstdint>
#include <cstring>

namespace ahrs::tactical
{

struct diagnostics
{
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float gyro_bias[3] = {};
    float accel_weight = 1.0f;
    float accel_direction_error = 0.0f;
    float accel_magnitude_g = 0.0f;
    float gyro_magnitude_rad_s = 0.0f;
    float accel_magnitude_variance = 0.0f;
    float gyro_magnitude_variance = 0.0f;
    float impact_acc_delta_g = 0.0f;
    float impact_gyro_delta_rad_s = 0.0f;
    std::uint8_t motion_state = 0;
    std::uint8_t impact_state = 0;
    bool paddling = false;
    bool linear_motion = false;
    float earth_acceleration[3] = {};
    float heave_velocity = 0.0f;
    float heave_position = 0.0f;
    std::uint32_t update_count = 0;
};

class ekf
{
public:
    void reset()
    {
        Tactical_Init(&system_, 2000.0f, 0.0f);
        // These are the values used by the old AHRS integration, not merely
        // the tactical fusion library defaults.
        system_.params.processNoiseAngle = 8.5e-8f;
        system_.params.processNoiseBias = 8.5e-10f;
        system_.params.measureNoiseAcc = 1.5e-5f;
        diagnostics_ = {};
    }

    void update(float gx_rad_s, float gy_rad_s, float gz_rad_s,
                float ax_m_s2, float ay_m_s2, float az_m_s2, float dt)
    {
        FusionVector gyro{};
        gyro.axis.x = gx_rad_s * 57.2957795131f;
        gyro.axis.y = gy_rad_s * 57.2957795131f;
        gyro.axis.z = gz_rad_s * 57.2957795131f;
        FusionVector accel{};
        accel.axis.x = ax_m_s2 / 9.80665f;
        accel.axis.y = ay_m_s2 / 9.80665f;
        accel.axis.z = az_m_s2 / 9.80665f;
        accel_ = accel;
        system_.samplePeriod = dt;
        Tactical_Update(&system_, gyro, accel);
        sync(gyro);
    }

    const diagnostics& state() const { return diagnostics_; }

private:
    void sync(const FusionVector& gyro)
    {
        std::memcpy(diagnostics_.quaternion, system_.quaternion.array, sizeof(diagnostics_.quaternion));
        const FusionEuler euler = FusionQuaternionToEuler(system_.quaternion);
        diagnostics_.roll = FusionDegreesToRadians(euler.angle.roll);
        diagnostics_.pitch = FusionDegreesToRadians(euler.angle.pitch);
        diagnostics_.yaw = FusionDegreesToRadians(euler.angle.yaw);
        diagnostics_.gyro_bias[0] = FusionDegreesToRadians(system_.gyroBias.axis.x);
        diagnostics_.gyro_bias[1] = FusionDegreesToRadians(system_.gyroBias.axis.y);
        diagnostics_.gyro_bias[2] = FusionDegreesToRadians(system_.gyroBias.axis.z);
        diagnostics_.accel_weight = system_.accWeight;
        diagnostics_.accel_direction_error = system_.accDirectionError;
        diagnostics_.accel_magnitude_g = system_.accMagnitude;
        diagnostics_.gyro_magnitude_rad_s = FusionDegreesToRadians(
            FusionVectorMagnitude(Tactical_GetCalibratedGyro(&system_, gyro)));
        diagnostics_.accel_magnitude_variance = system_.noise.accMagVar;
        diagnostics_.gyro_magnitude_variance = system_.noise.gyroMagVar * 0.0003046174198f;
        diagnostics_.impact_acc_delta_g = system_.impactAccDelta;
        diagnostics_.impact_gyro_delta_rad_s = FusionDegreesToRadians(system_.impactGyroDelta);
        diagnostics_.motion_state = static_cast<std::uint8_t>(system_.motionState);
        diagnostics_.impact_state = static_cast<std::uint8_t>(system_.impactState);
        diagnostics_.paddling = system_.isPaddling;
        diagnostics_.linear_motion = system_.isLinearMotion;
        const FusionVector earth = Tactical_GetEarthAcceleration(&system_, accel_);
        diagnostics_.earth_acceleration[0] = earth.axis.x;
        diagnostics_.earth_acceleration[1] = earth.axis.y;
        diagnostics_.earth_acceleration[2] = earth.axis.z;
        diagnostics_.heave_velocity = system_.heaveVelocity;
        diagnostics_.heave_position = system_.heavePosition;
        ++diagnostics_.update_count;
    }

    TacticalSystem system_{};
    FusionVector accel_{};
    diagnostics diagnostics_{};
};

} // namespace ahrs::tactical
