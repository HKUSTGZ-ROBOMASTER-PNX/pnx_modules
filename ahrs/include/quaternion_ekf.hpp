#pragma once

#include "kalman_filter.hpp"
#include "numeric.hpp"

#include "arm_math.h"
#include <cstdint>
#include <cstring>



namespace modules {

class QuaternionEkf : public filter::KalmanFilter
{
private:

    const float F_Init[36] = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1
    };

    const float P_Init[36] = {
        100000, 0.1, 0.1, 0.1, 0.1, 0.1,
        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
        0.1, 0.1, 0.1, 0.1, 100, 0.1,
        0.1, 0.1, 0.1, 0.1, 0.1, 100
    };

    const float R_Init[9] = {
        100000.0f, 0, 0,
        0, 100000.0f, 0,
        0, 0, 100000.0f
    };

    const float Q1 = 10;    // Quaternion process noise
    const float Q2 = 0.001; // Gyro bias process noise
    const float ChiSquareTestThreshold = 1e-6; // Chi-square test threshold
    const float lambda = 1;                 // Fading coefficient
    // const float accLPFcoef = 0; // Accel low-pass filter coefficient

    int16_t yawroundcount = 0;
    float prev_yaw = 0.0f;
    float Gyro[3];
    float Accel[3];

public:

    float yaw = 0.0f;
    float total_yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;

    QuaternionEkf() 
        : filter::KalmanFilter(6, 0, 3)
    {
        this->x_hat_data_[0] = 1.0f;
        this->x_hat_data_[1] = 0.0f;
        this->x_hat_data_[2] = 0.0f;
        this->x_hat_data_[3] = 0.0f;

        this->x_hat_minus_data_[0] = 1.0f;
        this->x_hat_minus_data_[1] = 0.0f;
        this->x_hat_minus_data_[2] = 0.0f;
        this->x_hat_minus_data_[3] = 0.0f;

        std::memcpy(this->f_data_, F_Init, sizeof(F_Init));
        std::memcpy(this->r_data_, R_Init, sizeof(R_Init));
        std::memcpy(this->p_data_, P_Init, sizeof(P_Init));
    }

    void update_kalman(float gx, float gy, float gz, float ax, float ay, float az, float dt)
    {
        this->dt = dt;

        Gyro[0] = gx - GyroBias[0];
        Gyro[1] = gy - GyroBias[1];
        Gyro[2] = gz - GyroBias[2];

        // Pre-calculate F matrix elements (linearization point)
        float halfgxdt = 0.5f * Gyro[0] * dt;
        float halfgydt = 0.5f * Gyro[1] * dt;
        float halfgzdt = 0.5f * Gyro[2] * dt;

        std::memcpy(f_data_, F_Init, 36 * sizeof(float));

        this->f_data_[1] = -halfgxdt;
        this->f_data_[2] = -halfgydt;
        this->f_data_[3] = -halfgzdt;

        this->f_data_[6] = halfgxdt;
        this->f_data_[8] = halfgzdt;
        this->f_data_[9] = -halfgydt;

        this->f_data_[12] = halfgydt;
        this->f_data_[13] = -halfgzdt;
        this->f_data_[15] = halfgxdt;

        this->f_data_[18] = halfgzdt;
        this->f_data_[19] = halfgydt;
        this->f_data_[20] = -halfgxdt;

        // Accel LPF
        // if (UpdateCount == 0)
        // {
        //     Accel[0] = ax;
        //     Accel[1] = ay;
        //     Accel[2] = az;
        // }
        // Accel[0] = Accel[0] * accLPFcoef / (dt + accLPFcoef) + ax * dt / (dt + accLPFcoef);
        // Accel[1] = Accel[1] * accLPFcoef / (dt + accLPFcoef) + ay * dt / (dt + accLPFcoef);
        // Accel[2] = Accel[2] * accLPFcoef / (dt + accLPFcoef) + az * dt / (dt + accLPFcoef);
        Accel[0] = ax;
        Accel[1] = ay;
        Accel[2] = az;

        // Normalize Accel to get measurement vector z
        float accelInvNorm = numeric::inv_sqrt(Accel[0] * Accel[0] + Accel[1] * Accel[1] + Accel[2] * Accel[2]);
        measured_vector_[0] = Accel[0] * accelInvNorm;
        measured_vector_[1] = Accel[1] * accelInvNorm;
        measured_vector_[2] = Accel[2] * accelInvNorm;

        // Stability check
        gyro_norm = 1.0f / numeric::inv_sqrt(Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2]);
        accl_norm = 1.0f / accelInvNorm;

        if (gyro_norm < 0.3f && accl_norm > 9.8f - 0.5f && accl_norm < 9.8f + 0.5f)
        {
            StableFlag = true;
        }
        else
        {
            StableFlag = false;
        }

        // Update Q and R
        this->q_data_[0] = Q1 * dt;
        this->q_data_[7] = Q1 * dt;
        this->q_data_[14] = Q1 * dt;
        this->q_data_[21] = Q1 * dt;
        this->q_data_[28] = Q2 * dt;
        this->q_data_[35] = Q2 * dt;

        // ??yaw???
        float yaw_rate = fabsf(gz - GyroBias[2]); // ?z�?
        
        // ????R
        float r_scale = 1.0f + yaw_rate * yaw_rate; // ??????????�?
        this->r_data_[0] = 100000.0f * r_scale;
        this->r_data_[4] = 100000.0f * r_scale;
        this->r_data_[8] = 100000.0f * r_scale;

        // Call base Update
        this->update();

        // Post-process
        q[0] = filtered_value_[0];
        q[1] = filtered_value_[1];
        q[2] = filtered_value_[2];
        q[3] = filtered_value_[3];
        GyroBias[0] = filtered_value_[4];
        GyroBias[1] = filtered_value_[5];
        GyroBias[2] = 0;

        yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

        if (yaw - prev_yaw > numeric::kPi) {
            yawroundcount--;
        } else if (yaw - prev_yaw < -numeric::kPi) {
            yawroundcount++;
        }
        total_yaw = static_cast<float>(yawroundcount) * numeric::kPiX2 + yaw;
        prev_yaw = yaw;

        UpdateCount++;
    }

    // Public data members to maintain compatibility with previous struct usage
    bool Initialized = false;
    bool ConvergeFlag = false;
    bool StableFlag = false;
    bool SkipPPredict = false;
    uint64_t ErrorCount = 0;
    uint64_t UpdateCount = 0;

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};        // Quaternion estimate
    float GyroBias[3] = {0.0f, 0.0f, 0.0f}; // Gyro bias estimate

    float OrientationCosine[3];

    float gyro_norm = 0;
    float accl_norm = 0;
    float AdaptiveGainScale = 0;

    float dt = 0; // Update period
    
protected:

    // Override virtual functions from KalmanFilter
    void update_x_hat_minus() override
    {
        // Standard prediction: x- = F * x
        arm_mat_mult_f32(&f_, &x_hat_, &x_hat_minus_);
        
        // Custom linearization and fading
        float q0 = x_hat_minus_data_[0];
        float q1 = x_hat_minus_data_[1];
        float q2 = x_hat_minus_data_[2];
        float q3 = x_hat_minus_data_[3];

        float qInvNorm = numeric::inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        this->x_hat_minus_data_[0] *= qInvNorm;
        this->x_hat_minus_data_[1] *= qInvNorm;
        this->x_hat_minus_data_[2] *= qInvNorm;
        this->x_hat_minus_data_[3] *= qInvNorm;
        
        q0 = this->x_hat_minus_data_[0];
        q1 = this->x_hat_minus_data_[1];
        q2 = this->x_hat_minus_data_[2];
        q3 = this->x_hat_minus_data_[3];

        // Update F matrix for P update
        this->f_data_[4] = q1 * dt / 2;
        this->f_data_[5] = q2 * dt / 2;

        this->f_data_[10] = -q0 * dt / 2;
        this->f_data_[11] = q3 * dt / 2;
        
        this->f_data_[16] = -q3 * dt / 2;
        this->f_data_[17] = -q0 * dt / 2;

        this->f_data_[22] = q2 * dt / 2;
        this->f_data_[23] = -q1 * dt / 2;

        // Fading filter
        this->p_data_[28] /= lambda;
        this->p_data_[35] /= lambda;

        // Limit P
        if (this->p_data_[28] > 10000) 
            this->p_data_[28] = 10000;
        if (this->p_data_[35] > 10000) 
            this->p_data_[35] = 10000;
    }

    void update_k() override
    {
        float doubleq0 = 2 * this->x_hat_minus_data_[0];
        float doubleq1 = 2 * this->x_hat_minus_data_[1];
        float doubleq2 = 2 * this->x_hat_minus_data_[2];
        float doubleq3 = 2 * this->x_hat_minus_data_[3];

        std::memset(this->h_data_, 0, sizeof(float) * this->measurement_size_ * this->state_size_);

        this->h_data_[0] = -doubleq2;
        this->h_data_[1] = doubleq3;
        this->h_data_[2] = -doubleq0;
        this->h_data_[3] = doubleq1;

        this->h_data_[6] = doubleq1;
        this->h_data_[7] = doubleq0;
        this->h_data_[8] = doubleq3;
        this->h_data_[9] = doubleq2;

        this->h_data_[12] = doubleq0;
        this->h_data_[13] = -doubleq1;
        this->h_data_[14] = -doubleq2;
        this->h_data_[15] = doubleq3;

        arm_mat_trans_f32(&h_, &h_t_);
        temp_matrix_.numRows = h_.numRows;
        temp_matrix_.numCols = p_minus_.numCols;
        arm_mat_mult_f32(&h_, &p_minus_, &temp_matrix_); // temp_matrix_ = H�P'(k)
        temp_matrix1_.numRows = temp_matrix_.numRows;
        temp_matrix1_.numCols = h_t_.numCols;
        arm_mat_mult_f32(&temp_matrix_, &h_t_, &temp_matrix1_); // temp_matrix1_ = H�P'(k)�h_t_
        s_.numRows = r_.numRows;
        s_.numCols = r_.numCols;
        arm_mat_add_f32(&temp_matrix1_, &r_, &s_); // S = H P'(k) h_t_ + R
        arm_mat_inverse_f32(&s_, &temp_matrix1_);     // temp_matrix1_ = inv(H�P'(k)�h_t_ + R)
    }

    void update_x_hat() override
    {
        // 1. Calculate h(x-) and OrientationCosine
        float q0 = this->x_hat_minus_data_[0];
        float q1 = this->x_hat_minus_data_[1];
        float q2 = this->x_hat_minus_data_[2];
        float q3 = this->x_hat_minus_data_[3];

        // Predicted gravity direction (h(x-))
        // Using temp_vector_ for h(x-) (3x1)
        temp_vector_.numRows = 3;
        temp_vector_.numCols = 1;
        temp_vector_data_[0] = 2 * (q1 * q3 - q0 * q2);
        temp_vector_data_[1] = 2 * (q0 * q1 + q2 * q3);
        temp_vector_data_[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Calculate OrientationCosine
        for (uint8_t i = 0; i < 3; i++)
        {
            OrientationCosine[i] = acosf(fabsf(temp_vector_data_[i]));
        }

        // 2. Calculate residual y = z - h(x-)
        // z is in this->z (3x1)
        // Store residual in temp_vector1_ (3x1)
        temp_vector1_.numRows = 3;
        temp_vector1_.numCols = 1;
        arm_mat_sub_f32(&measurement_, &temp_vector_, &temp_vector1_);

        // 3. Chi-square test
        // invS is in temp_matrix1_ (3x3) from setK
        // temp_matrix_ (3x1) = invS * residual
        temp_matrix_.numRows = 3;
        temp_matrix_.numCols = 1;
        arm_mat_mult_f32(&temp_matrix1_, &temp_vector1_, &temp_matrix_);
        
        // chiSquare = residual' * temp_matrix_
        float chiSquare = 0;
        arm_dot_prod_f32(temp_vector1_.pData, temp_matrix_.pData, 3, &chiSquare);

        // 4. Logic & Flags
        if (chiSquare < 0.5f * ChiSquareTestThreshold)
        {
            ConvergeFlag = true;
        }

        if (chiSquare > ChiSquareTestThreshold && ConvergeFlag)
        {
            if (StableFlag)
            {
                ErrorCount++;
            }
            else
            {
                ErrorCount = 0;
            }

            if (ErrorCount > 50)
            {
                ConvergeFlag = false;
                SkipPPredict = true;
            }
            else
            {
                // Reject update
                std::memcpy(x_hat_data_, x_hat_minus_data_, sizeof(float) * state_size_);
                std::memcpy(p_data_, p_minus_data_, sizeof(float) * state_size_ * state_size_);
                SkipPPredict = false;
                return;
            }
        }
        else
        {
            // Adaptive Gain
            if (chiSquare > 0.1f * ChiSquareTestThreshold && ConvergeFlag)
            {
                AdaptiveGainScale = (ChiSquareTestThreshold - chiSquare) / (0.9f * ChiSquareTestThreshold);
            }
            else
            {
                AdaptiveGainScale = 1.0f;
            }
            SkipPPredict = false;
            ErrorCount = 0;
        }

        // 5. Calculate K
        // K = P- * h_t_ * invS
        // temp_matrix_ (6x3) = P- * h_t_
        temp_matrix_.numRows = 6;
        temp_matrix_.numCols = 3;
        arm_mat_mult_f32(&p_minus_, &h_t_, &temp_matrix_);
        
        // K (6x3) = temp_matrix_ * invS
        // invS is temp_matrix1_ (3x3)
        arm_mat_mult_f32(&temp_matrix_, &temp_matrix1_, &k_);

        // 6. Apply Scaling to K
        for (uint8_t i = 0; i < k_.numRows * k_.numCols; i++)
        {
            k_data_[i] *= AdaptiveGainScale;
        }
        
        // Scale Gyro Bias parts (rows 4, 5 -> indices 4, 5 in 0-based rows)
        for (uint8_t i = 4; i < 6; i++)
        {
            for (uint8_t j = 0; j < 3; j++)
            {
                k_data_[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f;
            }
        }

        // 7. Calculate Correction
        // correction (6x1) = K * residual
        // residual is temp_vector1_
        // Reuse temp_vector_ for correction
        temp_vector_.numRows = 6;
        temp_vector_.numCols = 1;
        arm_mat_mult_f32(&k_, &temp_vector1_, &temp_vector_);

        // 8. Limit Bias Correction
        if (ConvergeFlag)
        {
            for (uint8_t i = 4; i < 6; i++)
            {
                if (temp_vector_data_[i] > 1e-2f * dt)
                    temp_vector_data_[i] = 1e-2f * dt;
                if (temp_vector_data_[i] < -1e-2f * dt)
                    temp_vector_data_[i] = -1e-2f * dt;
            }
        }

        // 9. Zero out Yaw correction
        temp_vector_data_[3] = 0.0f;

        // 10. Update State
        arm_mat_add_f32(&x_hat_minus_, &temp_vector_, &x_hat_);

        // 11. Normalization
        float q0_new = x_hat_data_[0];
        float q1_new = x_hat_data_[1];
        float q2_new = x_hat_data_[2];
        float q3_new = x_hat_data_[3];
        float normSq = q0_new * q0_new + q1_new * q1_new + q2_new * q2_new + q3_new * q3_new;
        if (normSq > 1e-9f)
        {
            float invNorm = numeric::inv_sqrt(normSq);
            x_hat_data_[0] *= invNorm;
            x_hat_data_[1] *= invNorm;
            x_hat_data_[2] *= invNorm;
            x_hat_data_[3] *= invNorm;
        }
        
        // Restore dimensions
        temp_vector_.numRows = state_size_;
        temp_vector1_.numRows = state_size_;
    }

    void update_p() override
    {
        if (!SkipPPredict)
        {
            temp_matrix_.numRows = k_.numRows;
            temp_matrix_.numCols = h_.numCols;
            temp_matrix1_.numRows = temp_matrix_.numRows;
            temp_matrix1_.numCols = p_minus_.numCols;
            arm_mat_mult_f32(&k_, &h_, &temp_matrix_);                 // temp_matrix_ = K(k)�H
            arm_mat_mult_f32(&temp_matrix_, &p_minus_, &temp_matrix1_); // temp_matrix1_ = K(k)�H�P'(k)
            arm_mat_sub_f32(&p_minus_, &temp_matrix1_, &p_);
        }
    }
};



}  // namespace modules
