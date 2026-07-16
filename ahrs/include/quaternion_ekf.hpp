#pragma once

#include "constants.hpp"

#include "arm_math.h"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace ahrs
{

class quaternion_ekf
{
public:
    quaternion_ekf()
    {
        init_matrices();
        reset();
    }

    void reset()
    {
        std::memset(x_hat_data_, 0, sizeof(x_hat_data_));
        std::memset(x_hat_minus_data_, 0, sizeof(x_hat_minus_data_));
        std::memset(q_data_, 0, sizeof(q_data_));

        x_hat_data_[0] = 1.0f;
        x_hat_minus_data_[0] = 1.0f;
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        GyroBias[0] = 0.0f;
        GyroBias[1] = 0.0f;
        GyroBias[2] = 0.0f;
        yaw = 0.0f;
        pitch = 0.0f;
        roll = 0.0f;
        total_yaw = 0.0f;
        prev_yaw_ = 0.0f;
        yaw_round_count_ = 0;
        Initialized = false;
        ConvergeFlag = false;
        StableFlag = false;
        SkipPPredict = false;
        ErrorCount = 0;
        UpdateCount = 0;
        gyro_norm = 0.0f;
        accl_norm = 0.0f;
        AdaptiveGainScale = 0.0f;
        dt = 0.0f;

        std::memcpy(f_data_, f_init_, sizeof(f_init_));
        std::memcpy(p_data_, p_init_, sizeof(p_init_));
        std::memcpy(r_data_, r_init_, sizeof(r_init_));
    }

    void update_kalman(float gx, float gy, float gz, float ax, float ay, float az, float delta_time)
    {
        dt = delta_time > 0.0f ? delta_time : 0.001f;

        gyro_[0] = gx - GyroBias[0];
        gyro_[1] = gy - GyroBias[1];
        gyro_[2] = gz - GyroBias[2];

        const float half_gx_dt = 0.5f * gyro_[0] * dt;
        const float half_gy_dt = 0.5f * gyro_[1] * dt;
        const float half_gz_dt = 0.5f * gyro_[2] * dt;

        std::memcpy(f_data_, f_init_, sizeof(f_init_));
        f_data_[1] = -half_gx_dt;
        f_data_[2] = -half_gy_dt;
        f_data_[3] = -half_gz_dt;
        f_data_[6] = half_gx_dt;
        f_data_[8] = half_gz_dt;
        f_data_[9] = -half_gy_dt;
        f_data_[12] = half_gy_dt;
        f_data_[13] = -half_gz_dt;
        f_data_[15] = half_gx_dt;
        f_data_[18] = half_gz_dt;
        f_data_[19] = half_gy_dt;
        f_data_[20] = -half_gx_dt;

        accel_[0] = ax;
        accel_[1] = ay;
        accel_[2] = az;

        const float accel_norm_sq = ax * ax + ay * ay + az * az;
        if (accel_norm_sq <= 1e-9f)
        {
            return;
        }

        const float accel_inv_norm = math::inv_sqrt(accel_norm_sq);
        measured_vector_[0] = accel_[0] * accel_inv_norm;
        measured_vector_[1] = accel_[1] * accel_inv_norm;
        measured_vector_[2] = accel_[2] * accel_inv_norm;

        gyro_norm = 1.0f / math::inv_sqrt(gyro_[0] * gyro_[0] + gyro_[1] * gyro_[1] + gyro_[2] * gyro_[2]);
        accl_norm = 1.0f / accel_inv_norm;
        StableFlag = gyro_norm < 0.3f && accl_norm > 9.3f && accl_norm < 10.3f;

        q_data_[0] = q1_ * dt;
        q_data_[7] = q1_ * dt;
        q_data_[14] = q1_ * dt;
        q_data_[21] = q1_ * dt;
        q_data_[28] = q2_ * dt;
        q_data_[35] = q2_ * dt;

        const float yaw_rate = std::fabs(gz - GyroBias[2]);
        const float r_scale = 1.0f + yaw_rate * yaw_rate;
        r_data_[0] = 100000.0f * r_scale;
        r_data_[4] = 100000.0f * r_scale;
        r_data_[8] = 100000.0f * r_scale;

        update();

        q[0] = filtered_value_[0];
        q[1] = filtered_value_[1];
        q[2] = filtered_value_[2];
        q[3] = filtered_value_[3];
        GyroBias[0] = filtered_value_[4];
        GyroBias[1] = filtered_value_[5];
        GyroBias[2] = 0.0f;

        yaw = std::atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
                         q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -std::asin(clamp(2.0f * (q[1] * q[3] - q[0] * q[2]), -1.0f, 1.0f));
        roll = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                          q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

        if (yaw - prev_yaw_ > math::pi)
        {
            --yaw_round_count_;
        }
        else if (yaw - prev_yaw_ < -math::pi)
        {
            ++yaw_round_count_;
        }
        total_yaw = static_cast<float>(yaw_round_count_) * math::two_pi + yaw;
        prev_yaw_ = yaw;
        ++UpdateCount;
    }

    bool Initialized = false;
    bool ConvergeFlag = false;
    bool StableFlag = false;
    bool SkipPPredict = false;
    uint64_t ErrorCount = 0;
    uint64_t UpdateCount = 0;

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float GyroBias[3] = {0.0f, 0.0f, 0.0f};
    float OrientationCosine[3] = {};

    float yaw = 0.0f;
    float total_yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float gyro_norm = 0.0f;
    float accl_norm = 0.0f;
    float AdaptiveGainScale = 0.0f;
    float dt = 0.0f;

private:
    static constexpr uint8_t state_size_ = 6;
    static constexpr uint8_t measurement_size_ = 3;
    static constexpr float q1_ = 10.0f;
    static constexpr float q2_ = 0.001f;
    static constexpr float chi_square_test_threshold_ = 1e-6f;
    static constexpr float lambda_ = 1.0f;

    static constexpr float f_init_[36] = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
    };

    static constexpr float p_init_[36] = {
        100000, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
        0.1f, 100000, 0.1f, 0.1f, 0.1f, 0.1f,
        0.1f, 0.1f, 100000, 0.1f, 0.1f, 0.1f,
        0.1f, 0.1f, 0.1f, 100000, 0.1f, 0.1f,
        0.1f, 0.1f, 0.1f, 0.1f, 100, 0.1f,
        0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100,
    };

    static constexpr float r_init_[9] = {
        100000.0f, 0.0f, 0.0f,
        0.0f, 100000.0f, 0.0f,
        0.0f, 0.0f, 100000.0f,
    };

    static float clamp(float value, float min, float max)
    {
        return value < min ? min : (value > max ? max : value);
    }

    void init_matrices()
    {
        arm_mat_init_f32(&x_hat_, state_size_, 1, x_hat_data_);
        arm_mat_init_f32(&x_hat_minus_, state_size_, 1, x_hat_minus_data_);
        arm_mat_init_f32(&measurement_, measurement_size_, 1, measurement_data_);
        arm_mat_init_f32(&p_, state_size_, state_size_, p_data_);
        arm_mat_init_f32(&p_minus_, state_size_, state_size_, p_minus_data_);
        arm_mat_init_f32(&f_, state_size_, state_size_, f_data_);
        arm_mat_init_f32(&f_t_, state_size_, state_size_, f_t_data_);
        arm_mat_init_f32(&h_, measurement_size_, state_size_, h_data_);
        arm_mat_init_f32(&h_t_, state_size_, measurement_size_, h_t_data_);
        arm_mat_init_f32(&q_matrix_, state_size_, state_size_, q_data_);
        arm_mat_init_f32(&r_, measurement_size_, measurement_size_, r_data_);
        arm_mat_init_f32(&k_, state_size_, measurement_size_, k_data_);
        arm_mat_init_f32(&s_, measurement_size_, measurement_size_, s_data_);
        arm_mat_init_f32(&temp_matrix_, state_size_, state_size_, temp_matrix_data_);
        arm_mat_init_f32(&temp_matrix1_, state_size_, state_size_, temp_matrix1_data_);
        arm_mat_init_f32(&temp_vector_, state_size_, 1, temp_vector_data_);
        arm_mat_init_f32(&temp_vector1_, state_size_, 1, temp_vector1_data_);
    }

    void update()
    {
        std::memcpy(measurement_data_, measured_vector_, sizeof(measured_vector_));
        update_x_hat_minus();
        update_p_minus();
        update_k();
        update_x_hat();
        update_p();
        std::memcpy(filtered_value_, x_hat_data_, sizeof(filtered_value_));
    }

    void update_x_hat_minus()
    {
        arm_mat_mult_f32(&f_, &x_hat_, &x_hat_minus_);

        float q0 = x_hat_minus_data_[0];
        float q1 = x_hat_minus_data_[1];
        float q2 = x_hat_minus_data_[2];
        float q3 = x_hat_minus_data_[3];

        const float q_inv_norm = math::inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        x_hat_minus_data_[0] *= q_inv_norm;
        x_hat_minus_data_[1] *= q_inv_norm;
        x_hat_minus_data_[2] *= q_inv_norm;
        x_hat_minus_data_[3] *= q_inv_norm;

        q0 = x_hat_minus_data_[0];
        q1 = x_hat_minus_data_[1];
        q2 = x_hat_minus_data_[2];
        q3 = x_hat_minus_data_[3];

        f_data_[4] = q1 * dt * 0.5f;
        f_data_[5] = q2 * dt * 0.5f;
        f_data_[10] = -q0 * dt * 0.5f;
        f_data_[11] = q3 * dt * 0.5f;
        f_data_[16] = -q3 * dt * 0.5f;
        f_data_[17] = -q0 * dt * 0.5f;
        f_data_[22] = q2 * dt * 0.5f;
        f_data_[23] = -q1 * dt * 0.5f;

        p_data_[28] /= lambda_;
        p_data_[35] /= lambda_;

        if (p_data_[28] > 10000.0f)
        {
            p_data_[28] = 10000.0f;
        }
        if (p_data_[35] > 10000.0f)
        {
            p_data_[35] = 10000.0f;
        }
    }

    void update_p_minus()
    {
        arm_mat_trans_f32(&f_, &f_t_);
        arm_mat_mult_f32(&f_, &p_, &p_minus_);
        set_matrix_shape(temp_matrix_, p_minus_.numRows, f_t_.numCols);
        arm_mat_mult_f32(&p_minus_, &f_t_, &temp_matrix_);
        arm_mat_add_f32(&temp_matrix_, &q_matrix_, &p_minus_);
    }

    void update_k()
    {
        const float double_q0 = 2.0f * x_hat_minus_data_[0];
        const float double_q1 = 2.0f * x_hat_minus_data_[1];
        const float double_q2 = 2.0f * x_hat_minus_data_[2];
        const float double_q3 = 2.0f * x_hat_minus_data_[3];

        std::memset(h_data_, 0, sizeof(h_data_));
        h_data_[0] = -double_q2;
        h_data_[1] = double_q3;
        h_data_[2] = -double_q0;
        h_data_[3] = double_q1;
        h_data_[6] = double_q1;
        h_data_[7] = double_q0;
        h_data_[8] = double_q3;
        h_data_[9] = double_q2;
        h_data_[12] = double_q0;
        h_data_[13] = -double_q1;
        h_data_[14] = -double_q2;
        h_data_[15] = double_q3;

        arm_mat_trans_f32(&h_, &h_t_);
        set_matrix_shape(temp_matrix_, h_.numRows, p_minus_.numCols);
        arm_mat_mult_f32(&h_, &p_minus_, &temp_matrix_);
        set_matrix_shape(temp_matrix1_, temp_matrix_.numRows, h_t_.numCols);
        arm_mat_mult_f32(&temp_matrix_, &h_t_, &temp_matrix1_);
        arm_mat_add_f32(&temp_matrix1_, &r_, &s_);
        arm_mat_inverse_f32(&s_, &temp_matrix1_);
    }

    void update_x_hat()
    {
        const float q0 = x_hat_minus_data_[0];
        const float q1 = x_hat_minus_data_[1];
        const float q2 = x_hat_minus_data_[2];
        const float q3 = x_hat_minus_data_[3];

        set_matrix_shape(temp_vector_, 3, 1);
        temp_vector_data_[0] = 2.0f * (q1 * q3 - q0 * q2);
        temp_vector_data_[1] = 2.0f * (q0 * q1 + q2 * q3);
        temp_vector_data_[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        for (uint8_t i = 0; i < 3; ++i)
        {
            OrientationCosine[i] = std::acos(std::fabs(clamp(temp_vector_data_[i], -1.0f, 1.0f)));
        }

        set_matrix_shape(temp_vector1_, 3, 1);
        arm_mat_sub_f32(&measurement_, &temp_vector_, &temp_vector1_);

        set_matrix_shape(temp_matrix_, 3, 1);
        arm_mat_mult_f32(&temp_matrix1_, &temp_vector1_, &temp_matrix_);

        float chi_square = 0.0f;
        arm_dot_prod_f32(temp_vector1_.pData, temp_matrix_.pData, 3, &chi_square);

        if (chi_square < 0.5f * chi_square_test_threshold_)
        {
            ConvergeFlag = true;
        }

        if (chi_square > chi_square_test_threshold_ && ConvergeFlag)
        {
            ErrorCount = StableFlag ? ErrorCount + 1 : 0;
            if (ErrorCount > 50)
            {
                ConvergeFlag = false;
                SkipPPredict = true;
            }
            else
            {
                std::memcpy(x_hat_data_, x_hat_minus_data_, sizeof(x_hat_data_));
                std::memcpy(p_data_, p_minus_data_, sizeof(p_data_));
                SkipPPredict = false;
                restore_vector_shapes();
                return;
            }
        }
        else
        {
            if (chi_square > 0.1f * chi_square_test_threshold_ && ConvergeFlag)
            {
                AdaptiveGainScale = (chi_square_test_threshold_ - chi_square) /
                                    (0.9f * chi_square_test_threshold_);
            }
            else
            {
                AdaptiveGainScale = 1.0f;
            }
            SkipPPredict = false;
            ErrorCount = 0;
        }

        set_matrix_shape(temp_matrix_, 6, 3);
        arm_mat_mult_f32(&p_minus_, &h_t_, &temp_matrix_);
        arm_mat_mult_f32(&temp_matrix_, &temp_matrix1_, &k_);

        for (uint8_t i = 0; i < k_.numRows * k_.numCols; ++i)
        {
            k_data_[i] *= AdaptiveGainScale;
        }

        for (uint8_t i = 4; i < 6; ++i)
        {
            for (uint8_t j = 0; j < 3; ++j)
            {
                k_data_[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f;
            }
        }

        set_matrix_shape(temp_vector_, 6, 1);
        arm_mat_mult_f32(&k_, &temp_vector1_, &temp_vector_);

        if (ConvergeFlag)
        {
            for (uint8_t i = 4; i < 6; ++i)
            {
                temp_vector_data_[i] = clamp(temp_vector_data_[i], -1e-2f * dt, 1e-2f * dt);
            }
        }

        temp_vector_data_[3] = 0.0f;
        arm_mat_add_f32(&x_hat_minus_, &temp_vector_, &x_hat_);

        const float q0_new = x_hat_data_[0];
        const float q1_new = x_hat_data_[1];
        const float q2_new = x_hat_data_[2];
        const float q3_new = x_hat_data_[3];
        const float norm_sq = q0_new * q0_new + q1_new * q1_new + q2_new * q2_new + q3_new * q3_new;
        if (norm_sq > 1e-9f)
        {
            const float inv_norm = math::inv_sqrt(norm_sq);
            x_hat_data_[0] *= inv_norm;
            x_hat_data_[1] *= inv_norm;
            x_hat_data_[2] *= inv_norm;
            x_hat_data_[3] *= inv_norm;
        }

        restore_vector_shapes();
    }

    void update_p()
    {
        if (SkipPPredict)
        {
            return;
        }

        set_matrix_shape(temp_matrix_, k_.numRows, h_.numCols);
        set_matrix_shape(temp_matrix1_, temp_matrix_.numRows, p_minus_.numCols);
        arm_mat_mult_f32(&k_, &h_, &temp_matrix_);
        arm_mat_mult_f32(&temp_matrix_, &p_minus_, &temp_matrix1_);
        arm_mat_sub_f32(&p_minus_, &temp_matrix1_, &p_);
    }

    static void set_matrix_shape(arm_matrix_instance_f32& matrix, uint16_t rows, uint16_t cols)
    {
        matrix.numRows = rows;
        matrix.numCols = cols;
    }

    void restore_vector_shapes()
    {
        set_matrix_shape(temp_vector_, state_size_, 1);
        set_matrix_shape(temp_vector1_, state_size_, 1);
    }

    float gyro_[3] = {};
    float accel_[3] = {};
    float measured_vector_[3] = {};
    float filtered_value_[6] = {};
    int16_t yaw_round_count_ = 0;
    float prev_yaw_ = 0.0f;

    arm_matrix_instance_f32 x_hat_{};
    arm_matrix_instance_f32 x_hat_minus_{};
    arm_matrix_instance_f32 measurement_{};
    arm_matrix_instance_f32 p_{};
    arm_matrix_instance_f32 p_minus_{};
    arm_matrix_instance_f32 f_{};
    arm_matrix_instance_f32 f_t_{};
    arm_matrix_instance_f32 h_{};
    arm_matrix_instance_f32 h_t_{};
    arm_matrix_instance_f32 q_matrix_{};
    arm_matrix_instance_f32 r_{};
    arm_matrix_instance_f32 k_{};
    arm_matrix_instance_f32 s_{};
    arm_matrix_instance_f32 temp_matrix_{};
    arm_matrix_instance_f32 temp_matrix1_{};
    arm_matrix_instance_f32 temp_vector_{};
    arm_matrix_instance_f32 temp_vector1_{};

    float x_hat_data_[6] = {};
    float x_hat_minus_data_[6] = {};
    float measurement_data_[3] = {};
    float p_data_[36] = {};
    float p_minus_data_[36] = {};
    float f_data_[36] = {};
    float f_t_data_[36] = {};
    float h_data_[18] = {};
    float h_t_data_[18] = {};
    float q_data_[36] = {};
    float r_data_[9] = {};
    float k_data_[18] = {};
    float s_data_[9] = {};
    float temp_matrix_data_[36] = {};
    float temp_matrix1_data_[36] = {};
    float temp_vector_data_[6] = {};
    float temp_vector1_data_[6] = {};
};

} // namespace ahrs
