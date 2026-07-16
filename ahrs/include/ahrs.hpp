#pragma once

#include "bmi088.hpp"
#include "msg.hpp"
#include "quaternion_ekf.hpp"
#include "tx_api.h"

namespace ahrs
{

struct config
{
    float imu_offset_x = 0.0f;
    uint32_t imu_thread_priority = 3;
    uint32_t temp_thread_priority = 4;
    float target_temp = 45.0f;
    bool temperature_control_enabled = true;
};

struct message
{
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float total_yaw = 0.0f;
    float gyro_r = 0.0f;
    float gyro_p = 0.0f;
    float gyro_y = 0.0f;
    float accel[3] = {};
};

class service
{
public:
    static service& instance();

    bool init(const config& cfg = {});
    TX_SEMAPHORE* heartbeat_sem() { return &heartbeat_sem_; }
    imu::bmi088& imu() { return imu_; }

private:
    static void imu_thread_entry(ULONG arg);
    static void gyro_data_ready_callback(void* user);
    bool create_resources();
    void fill_msg(message& msg, const quaternion_ekf& ekf, const imu::reading& reading);

    config cfg_{};
    imu::bmi088 imu_{};

    TX_THREAD imu_thread_{};
    TX_SEMAPHORE heartbeat_sem_{};
    TX_SEMAPHORE gyro_data_ready_sem_{};

    alignas(8) uint8_t imu_stack_[3072]{};

    msg::topic* ahrs_topic_ = nullptr;
    bool initialized_ = false;
};

} // namespace ahrs
