#pragma once

#include "bmi088.hpp"
#include "msg.hpp"
#include "quaternion_ekf.hpp"
#include "runtime_monitor.hpp"
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

struct telemetry
{
    bool initialized = false;
    bool resources_created = false;
    bool imu_configured = false;
    bool accel_chip_ok = false;
    bool gyro_chip_ok = false;
    bool temperature_ready = false;
    bool calibrated = false;
    bool read_ok = false;
    bool solved = false;
    uint32_t update_count = 0;
    uint32_t gyro_ready_count = 0;
    uint32_t gyro_ready_drained = 0;
    uint32_t loop_runtime_us = 0;
    uint32_t loop_runtime_max_us = 0;
    uint32_t loop_runtime_overruns = 0;
    float loop_runtime_avg_us = 0.0f;
    float dt_s = 0.0f;
    float dt_min_s = 0.0f;
    float dt_max_s = 0.0f;
    float temperature = 0.0f;
    float accel_norm = 0.0f;
    float gyro_norm = 0.0f;
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float total_yaw = 0.0f;
};

class service
{
public:
    static service& instance();

    bool init(const config& cfg = {});
    TX_SEMAPHORE* heartbeat_sem() { return &heartbeat_sem_; }
    imu::bmi088& imu() { return imu_; }
    const telemetry& diagnostics() const { return telemetry_; }

private:
    static void imu_thread_entry(ULONG arg);
    static void gyro_data_ready_callback(void* user);
    bool create_resources();
    void wait_for_gyro_data_ready();
    void fill_msg(message& msg, const quaternion_ekf& ekf, const imu::reading& reading);

    config cfg_{};
    imu::bmi088 imu_{};

    TX_THREAD imu_thread_{};
    TX_SEMAPHORE heartbeat_sem_{};
    TX_SEMAPHORE gyro_data_ready_sem_{};

    alignas(8) uint8_t imu_stack_[3072]{};

    msg::topic* ahrs_topic_ = nullptr;
    telemetry telemetry_{};
    runtime::monitor imu_loop_monitor_{500U};
    bool initialized_ = false;
};

} // namespace ahrs
