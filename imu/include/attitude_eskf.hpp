#pragma once
/**
 * attitude_eskf.hpp — 6-axis attitude error-state Kalman filter (header-only).
 * Requires the libs and dsp CMake targets.
 *
 * Nominal state : q (body→world, NWU, w-x-y-z), gyro bias b (rad/s)
 * Error state   : δθ (3, local/body frame), δb (3);  truth = q ⊗ Exp(δθ)
 *
 *   predict   q ← q ⊗ Exp((ω_m − b)·dt)
 *             F = [[I − [ω]×dt, −I·dt], [0, I]],  Q = diag(n_g²·dt, n_b²·dt)
 *
 *   accel     r = â × ĝ(q) ≈ Π·δθ,  Π = I − ĝĝᵀ  (rank 2: yaw is NOT observed)
 *             R = n_a²/dt + k·(|a| − 1g)²
 *             NIS = rᵀS⁻¹r gated against χ²(2); skipped for impact_hold after a jerk spike
 *
 *   zero-rate when quasi-static long enough: ω_m − b = 0, H = [0, I]  (makes z bias observable)
 *
 * Accel-correction time constant ≈ accel_noise / gyro_noise (seconds), independent of sample rate.
 */

#include <cmath>
#include <cstdint>

#include "dsp/matrix_functions.h"
#include "geometry.hpp"

namespace estimation
{

class attitude_eskf
{
public:
    struct config
    {
        float gyro_noise = 3.0e-4f;           // rad/s/√Hz      (BMI088 datasheet ≈ 2.4e-4)
        float gyro_bias_walk = 2.0e-5f;       // rad/s²/√Hz
        float accel_noise = 6.0e-4f;          // rad·√s, gravity-direction noise → τ ≈ 2 s
        float accel_magnitude_gain = 1.0f;    // R += gain·(|a|−1g)²   [rad²/g²]
        float nis_gate = 9.21f;               // χ²(2 dof, 99 %); ≤ 0 disables gating
        float reject_timeout = 0.2f;          // s of rejections while quasi-static → re-align tilt
        float impact_jerk = 400.0f;           // g/s, |d|a|/dt| treated as impact
        float impact_hold = 0.05f;            // s of skipped accel updates after an impact
        float static_accel_tolerance = 0.05f; // g
        float static_gyro_tolerance = 0.02f;  // rad/s, on low-passed bias-corrected rate
        float static_filter_time_constant = 0.05f; // s, low-pass used by the static detector
        float zero_rate_min_time = 1.0f;    // s static before zero-rate updates; ≤ 0 disables
        float initial_tilt_stddev = 0.1f;   // rad, roll/pitch (tilt) at start or re-align
        float initial_yaw_stddev = 1.0e-3f; // rad; yaw is *defined* at start, keep this small
        float initial_bias_stddev = 0.01f;  // rad/s
        float min_bias_stddev = 2.0e-4f;    // rad/s floor; keeps float32 P well conditioned
    };

    struct state
    {
        float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f, total_yaw = 0.0f; // rad
        float gyro_bias[3] = {};                                       // rad/s
        float nis = 0.0f;
        bool initialized = false;
        bool accel_updated = false;
        bool impact = false;
        bool zero_rate_active = false;
        uint32_t sequence = 0;
    };

    attitude_eskf() = default;

    explicit attitude_eskf(const config& settings) : cfg_(settings)
    {
    }

    void reset()
    {
        *this = attitude_eskf(cfg_);
    }

    config& parameters()
    {
        return cfg_;
    }

    const state& output() const
    {
        return state_;
    }

    /// gyro in rad/s, accel in m/s², dt in s.
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
    {
        if (!(dt > 0.0f) || !std::isfinite(dt) || !std::isfinite(gx) || !std::isfinite(gy) ||
            !std::isfinite(gz) || !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az))
        {
            return;
        }
        constexpr float inverse_gravity = 1.0f / 9.80665f;
        const float a[3] = {ax * inverse_gravity, ay * inverse_gravity, az * inverse_gravity};
        const float accel_norm = math::magnitude({a[0], a[1], a[2]});

        if (!state_.initialized)
        {
            if (accel_norm > 0.5f && accel_norm < 1.5f)
            {
                set_tilt(a, 0.0f);
                for (int i = 0; i < 6; ++i)
                {
                    for (int j = 0; j < 6; ++j)
                    {
                        P_[i][j] = 0.0f;
                    }
                }
                reset_attitude_covariance();
                filtered_angular_rate_[0] = filtered_angular_rate_[1] = filtered_angular_rate_[2] =
                    0.0f;
                filtered_accel_norm_ = accel_norm;
                for (int i = 3; i < 6; ++i)
                {
                    P_[i][i] = cfg_.initial_bias_stddev * cfg_.initial_bias_stddev;
                }
                previous_accel_norm_ = accel_norm;
                state_.initialized = true;
                update_output(true);
            }
            return;
        }

        const float gyro[3] = {gx, gy, gz};
        const float w[3] = {gx - gyro_bias_[0], gy - gyro_bias_[1], gz - gyro_bias_[2]};
        predict(w, dt);

        // Impact: jerk on |a| (rate-independent, unlike a per-sample delta).
        if (std::fabs(accel_norm - previous_accel_norm_) / dt > cfg_.impact_jerk)
        {
            impact_remaining_ = cfg_.impact_hold;
        }
        previous_accel_norm_ = accel_norm;
        state_.impact = impact_remaining_ > 0.0f;
        if (state_.impact)
        {
            impact_remaining_ -= dt;
        }

        // Static detection on low-passed signals, so the test does not depend on sample rate
        // (raw per-sample gyro noise at 2 kHz is already ~0.01 rad/s per axis).
        const float alpha = dt / (cfg_.static_filter_time_constant + dt);
        for (int i = 0; i < 3; ++i)
        {
            filtered_angular_rate_[i] += alpha * (w[i] - filtered_angular_rate_[i]);
        }
        filtered_accel_norm_ += alpha * (accel_norm - filtered_accel_norm_);
        const bool quasi_static =
            !state_.impact &&
            std::fabs(filtered_accel_norm_ - 1.0f) < cfg_.static_accel_tolerance &&
            math::magnitude({filtered_angular_rate_[0], filtered_angular_rate_[1],
                filtered_angular_rate_[2]}) < cfg_.static_gyro_tolerance;
        static_time_ = quasi_static ? static_time_ + dt : 0.0f;

        state_.accel_updated = false;
        if (!state_.impact && accel_norm > 0.1f)
        {
            update_accel(a, accel_norm, dt, quasi_static);
        }

        state_.zero_rate_active =
            cfg_.zero_rate_min_time > 0.0f && static_time_ >= cfg_.zero_rate_min_time;
        if (state_.zero_rate_active)
        {
            update_zero_rate(gyro, dt);
        }
        update_output(false);
    }

private:
    struct update_terms
    {
        float PHt[6][3];   // P * H^T
        float S_inv[3][3]; // Inverse innovation covariance
        float nis;         // Normalized innovation squared
    };

    config cfg_{};
    state state_{};
    math::quaternion quaternion_{};
    float gyro_bias_[3] = {};
    float P_[6][6] = {};
    float previous_accel_norm_ = 1.0f;
    float filtered_accel_norm_ = 1.0f;
    float filtered_angular_rate_[3] = {};
    float impact_remaining_ = 0.0f;
    float static_time_ = 0.0f;
    float reject_time_ = 0.0f;
    float previous_yaw_ = 0.0f;

    // ---------------------------------------------------------------- filter steps

    void predict(const float w[3], float dt)
    {
        const auto rotation = rotation_quaternion(w[0] * dt, w[1] * dt, w[2] * dt);
        quaternion_ = math::normalise(math::multiply(quaternion_, rotation));

        float F[6][6] = {};
        for (int i = 0; i < 6; ++i)
        {
            F[i][i] = 1.0f;
        }
        F[0][1] = w[2] * dt;
        F[0][2] = -w[1] * dt;
        F[1][0] = -w[2] * dt;
        F[1][2] = w[0] * dt;
        F[2][0] = w[1] * dt;
        F[2][1] = -w[0] * dt;
        F[0][3] = F[1][4] = F[2][5] = -dt;

        float FP[6][6];
        float F_transpose[6][6];
        arm_matrix_instance_f32 transition{6, 6, &F[0][0]};
        arm_matrix_instance_f32 covariance{6, 6, &P_[0][0]};
        arm_matrix_instance_f32 product{6, 6, &FP[0][0]};
        arm_matrix_instance_f32 transpose{6, 6, &F_transpose[0][0]};
        arm_mat_mult_f32(&transition, &covariance, &product);
        arm_mat_trans_f32(&transition, &transpose);
        arm_mat_mult_f32(&product, &transpose, &covariance);

        const float attitude_process_variance = cfg_.gyro_noise * cfg_.gyro_noise * dt;
        const float bias_process_variance = cfg_.gyro_bias_walk * cfg_.gyro_bias_walk * dt;
        for (int i = 0; i < 3; ++i)
        {
            P_[i][i] += attitude_process_variance;
            P_[i + 3][i + 3] += bias_process_variance;
        }
    }

    void update_accel(const float a[3], float accel_norm, float dt, bool quasi_static)
    {
        const float u[3] = {a[0] / accel_norm, a[1] / accel_norm, a[2] / accel_norm};
        float g[3];
        gravity_direction(g);

        const auto residual = math::cross({u[0], u[1], u[2]}, {g[0], g[1], g[2]});
        const float r[3] = {residual.x, residual.y, residual.z};

        float H[3][6] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                H[i][j] = (i == j ? 1.0f : 0.0f) - g[i] * g[j];
            }
        }

        const float accel_norm_error = accel_norm - 1.0f;
        const float measurement_variance =
            cfg_.accel_noise * cfg_.accel_noise / dt +
            cfg_.accel_magnitude_gain * accel_norm_error * accel_norm_error;

        update_terms terms;
        if (!compute_update_terms(H, r, measurement_variance, terms))
        {
            return;
        }
        state_.nis = terms.nis;

        if (cfg_.nis_gate > 0.0f && terms.nis > cfg_.nis_gate)
        {
            if (!quasi_static)
            {
                reject_time_ = 0.0f;
                return;
            }
            reject_time_ += dt;
            if (reject_time_ >= cfg_.reject_timeout)
            {
                // Still, ~1 g, yet persistently inconsistent: the estimate has diverged.
                set_tilt(a, state_.yaw);
                reset_attitude_covariance();
                reject_time_ = 0.0f;
            }
            return;
        }
        reject_time_ = 0.0f;
        apply_update(H, r, measurement_variance, terms);
        state_.accel_updated = true;
    }

    void update_zero_rate(const float gyro[3], float dt)
    {
        float H[3][6] = {};
        H[0][3] = H[1][4] = H[2][5] = 1.0f;
        const float residual[3] = {gyro[0] - gyro_bias_[0], gyro[1] - gyro_bias_[1],
            gyro[2] - gyro_bias_[2]};
        const float measurement_variance = cfg_.gyro_noise * cfg_.gyro_noise / dt;

        update_terms terms;
        if (compute_update_terms(H, residual, measurement_variance, terms))
        {
            apply_update(H, residual, measurement_variance, terms);
        }
    }

    bool compute_update_terms(const float H[3][6], const float residual[3],
        float measurement_variance, update_terms& terms) const
    {
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                float s = 0.0f;
                for (int k = 0; k < 6; ++k)
                {
                    s += P_[i][k] * H[j][k];
                }
                terms.PHt[i][j] = s;
            }
        }

        float S[3][3];
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                float s = (i == j) ? measurement_variance : 0.0f;
                for (int k = 0; k < 6; ++k)
                {
                    s += H[i][k] * terms.PHt[k][j];
                }
                S[i][j] = s;
            }
        }
        const float determinant = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) +
                                  S[0][1] * (S[1][2] * S[2][0] - S[1][0] * S[2][2]) +
                                  S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
        if (!(std::fabs(determinant) > 1e-30f))
        {
            return false;
        }
        // CMSIS-DSP inversion overwrites S; it is local scratch storage.
        arm_matrix_instance_f32 covariance{3, 3, &S[0][0]};
        arm_matrix_instance_f32 inverse{3, 3, &terms.S_inv[0][0]};
        if (arm_mat_inverse_f32(&covariance, &inverse) != ARM_MATH_SUCCESS)
        {
            return false;
        }

        terms.nis = 0.0f;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                terms.nis += residual[i] * terms.S_inv[i][j] * residual[j];
            }
        }
        return true;
    }

    // K = P Hᵀ S⁻¹, inject, Joseph form P = (I−KH) P (I−KH)ᵀ + rv·K Kᵀ.
    void apply_update(const float H[3][6], const float residual[3], float measurement_variance,
        const update_terms& terms)
    {
        float K[6][3];
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                K[i][j] = terms.PHt[i][0] * terms.S_inv[0][j] +
                          terms.PHt[i][1] * terms.S_inv[1][j] + terms.PHt[i][2] * terms.S_inv[2][j];
            }
        }

        float state_correction[6];
        for (int i = 0; i < 6; ++i)
        {
            state_correction[i] =
                K[i][0] * residual[0] + K[i][1] * residual[1] + K[i][2] * residual[2];
        }

        const auto rotation =
            rotation_quaternion(state_correction[0], state_correction[1], state_correction[2]);
        quaternion_ = math::normalise(math::multiply(quaternion_, rotation));
        gyro_bias_[0] += state_correction[3];
        gyro_bias_[1] += state_correction[4];
        gyro_bias_[2] += state_correction[5];

        float A[6][6];
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                A[i][j] = (i == j ? 1.0f : 0.0f) -
                          (K[i][0] * H[0][j] + K[i][1] * H[1][j] + K[i][2] * H[2][j]);
            }
        }

        float AP[6][6];
        float A_transpose[6][6];
        arm_matrix_instance_f32 transition{6, 6, &A[0][0]};
        arm_matrix_instance_f32 covariance{6, 6, &P_[0][0]};
        arm_matrix_instance_f32 product{6, 6, &AP[0][0]};
        arm_matrix_instance_f32 transpose{6, 6, &A_transpose[0][0]};
        arm_mat_mult_f32(&transition, &covariance, &product);
        arm_mat_trans_f32(&transition, &transpose);
        arm_mat_mult_f32(&product, &transpose, &covariance);
        for (int i = 0; i < 6; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                P_[i][j] += measurement_variance *
                            (K[i][0] * K[j][0] + K[i][1] * K[j][1] + K[i][2] * K[j][2]);
            }
        }
        stabilize_covariance();
    }

    // ---------------------------------------------------------------- helpers

    void set_tilt(const float a[3], float yaw)
    {
        const float roll = std::atan2(a[1], a[2]);
        const float pitch = std::atan2(-a[0], std::sqrt(a[1] * a[1] + a[2] * a[2]));
        const float cr = std::cos(0.5f * roll), sr = std::sin(0.5f * roll);
        const float cp = std::cos(0.5f * pitch), sp = std::sin(0.5f * pitch);
        const float cy = std::cos(0.5f * yaw), sy = std::sin(0.5f * yaw);
        quaternion_.w = cr * cp * cy + sr * sp * sy;
        quaternion_.x = sr * cp * cy - cr * sp * sy;
        quaternion_.y = cr * sp * cy + sr * cp * sy;
        quaternion_.z = cr * cp * sy - sr * sp * cy;
    }

    // P_θθ = σ_tilt²·(I − ĝĝᵀ) + σ_yaw²·ĝĝᵀ. A large variance along gravity is unobservable
    // and only destroys float32 conditioning, so yaw gets its own small σ.
    void reset_attitude_covariance()
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                P_[i][j] = P_[j][i] = 0.0f;
            }
        }
        float g[3];
        gravity_direction(g);
        const float tilt_variance = cfg_.initial_tilt_stddev * cfg_.initial_tilt_stddev;
        const float sy = cfg_.initial_yaw_stddev * cfg_.initial_yaw_stddev;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                P_[i][j] =
                    tilt_variance * ((i == j ? 1.0f : 0.0f) - g[i] * g[j]) + sy * g[i] * g[j];
            }
        }
    }

    void stabilize_covariance()
    {
        for (int i = 0; i < 6; ++i)
        {
            for (int j = i + 1; j < 6; ++j)
            {
                const float m = 0.5f * (P_[i][j] + P_[j][i]);
                P_[i][j] = P_[j][i] = m;
            }
        }
        const float min_bias_variance = cfg_.min_bias_stddev * cfg_.min_bias_stddev;
        for (int i = 3; i < 6; ++i)
        {
            if (P_[i][i] < min_bias_variance)
            {
                P_[i][i] = min_bias_variance;
            }
        }
    }

    void gravity_direction(float g[3]) const // Rᵀ·e_z, unit length
    {
        const float w = quaternion_.w, x = quaternion_.x, y = quaternion_.y, z = quaternion_.z;
        g[0] = 2.0f * (x * z - w * y);
        g[1] = 2.0f * (y * z + w * x);
        g[2] = w * w - x * x - y * y + z * z;
    }

    void update_output(bool initial)
    {
        state_.quaternion[0] = quaternion_.w;
        state_.quaternion[1] = quaternion_.x;
        state_.quaternion[2] = quaternion_.y;
        state_.quaternion[3] = quaternion_.z;
        for (int i = 0; i < 3; ++i)
        {
            state_.gyro_bias[i] = gyro_bias_[i];
        }
        const auto angles = math::to_euler(quaternion_);
        state_.roll = angles.roll;
        state_.pitch = angles.pitch;
        state_.yaw = angles.yaw;

        constexpr float pi = math::pi;
        float yaw_delta = initial ? state_.yaw : state_.yaw - previous_yaw_;
        if (yaw_delta > pi)
        {
            yaw_delta -= 2.0f * pi;
        }
        if (yaw_delta < -pi)
        {
            yaw_delta += 2.0f * pi;
        }
        state_.total_yaw = initial ? state_.yaw : state_.total_yaw + yaw_delta;
        previous_yaw_ = state_.yaw;
        ++state_.sequence;
    }

    static math::quaternion rotation_quaternion(float vx, float vy, float vz)
    {
        const float angle = math::magnitude({vx, vy, vz});
        if (angle < 1e-6f)
        {
            return {1.0f, 0.5f * vx, 0.5f * vy, 0.5f * vz};
        }
        const float s = std::sin(0.5f * angle) / angle;
        return {std::cos(0.5f * angle), vx * s, vy * s, vz * s};
    }
};

} // namespace estimation
