#pragma once

#include "bmi088.hpp"
#include "config.hpp"
#if HAS_DMIMU
#include "dmimu.hpp"
#endif
#include "msg.hpp"
#include "quaternion_ekf.hpp"
#include "runtime_monitor.hpp"
#include "tactical_ekf.hpp"
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
    uint32_t sample_error_count = 0;
    uint32_t imu_spi_read_error_count = 0;
    uint32_t imu_spi_write_error_count = 0;
    uint32_t imu_spi_lock_error_count = 0;
    uint32_t update_count = 0;
    uint32_t gyro_ready_count = 0;
    uint32_t gyro_ready_drained = 0;
    uint32_t gyro_ready_startup_drained = 0;
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
    float gyro[3] = {};
    float calibrated_gyro_offset[3] = {};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float total_yaw = 0.0f;
    tactical::diagnostics tactical{};
};

extern "C" telemetry ahrs_debug_telemetry;

#if HAS_DMIMU
// DMIMU uses a distinct payload type so its direct device attitude and the
// BMI088 local-fusion result remain independent topics even when both services
// run at the same time.
struct dmimu_message
{
    float quaternion[4] = {};
    float yaw = 0.0f;       // rad
    float pitch = 0.0f;     // rad
    float roll = 0.0f;      // rad
    float total_yaw = 0.0f; // rad; DMIMU does not provide a multi-turn yaw value.
    float gyro_r = 0.0f;    // rad/s
    float gyro_p = 0.0f;    // rad/s
    float gyro_y = 0.0f;    // rad/s
    float accel[3] = {};     // m/s^2
    uint32_t sequence = 0U;
    ULONG received_tick = 0U;
    bool online = false;
};

struct dmimu_service_config
{
    uint32_t thread_priority = 3U;
    ULONG receive_wait_ticks = 1U;
    ULONG request_period_ticks = 1U;
};

struct dmimu_telemetry
{
    bool initialized = false;
    bool resources_created = false;
    bool thread_started = false;
    bool online = false;
    bool snapshot_received = false;
    uint32_t processed_frame_count = 0U;
    uint32_t request_cycle_count = 0U;
    uint32_t request_error_count = 0U;
    uint32_t publish_count = 0U;
    uint32_t publish_error_count = 0U;
    uint32_t offline_zero_publish_count = 0U;
    uint32_t last_sequence = 0U;
    ULONG last_publish_tick = 0U;
    uint32_t loop_runtime_us = 0U;
    uint32_t loop_runtime_max_us = 0U;
    uint32_t loop_runtime_overruns = 0U;
    float loop_runtime_avg_us = 0.0f;
    imu::dmimu::diagnostics device{};
};

extern "C" dmimu_telemetry dmimu_debug_telemetry;
#endif

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

    // The IMU thread owns both the production quaternion EKF and the shadow
    // Tactical ESKF.  Their covariance workspaces are intentionally local to
    // this thread, so reserve enough DTCM stack for their peak update frames.
    alignas(8) uint8_t imu_stack_[8192]{};

    msg::topic* ahrs_topic_ = nullptr;
    telemetry& telemetry_ = ahrs_debug_telemetry;
    tactical::ekf tactical_ekf_{};
    runtime::monitor imu_loop_monitor_{500U};
    bool initialized_ = false;
};

#if HAS_DMIMU
class dmimu_service
{
public:
    static dmimu_service& instance();

    // Calling init is the enable switch for this service. The later robot
    // configuration layer must omit this call when DMIMU is not configured.
    bool init(const imu::dmimu::config& dmimu_cfg,
              const dmimu_service_config& service_cfg = {});
    TX_SEMAPHORE* heartbeat_sem() { return &heartbeat_sem_; }
    imu::dmimu& dmimu() { return dmimu_; }
    const dmimu_telemetry& diagnostics() const { return telemetry_; }

private:
    static void thread_entry(ULONG arg);
    bool create_resources();
    bool request_snapshot();
    void publish_snapshot(const imu::dmimu::snapshot& snapshot);
    void publish_offline_zero();
    void update_runtime_telemetry();

    imu::dmimu::config dmimu_cfg_{};
    dmimu_service_config service_cfg_{};
    imu::dmimu dmimu_{};

    TX_THREAD thread_{};
    TX_SEMAPHORE heartbeat_sem_{};
    alignas(8) uint8_t stack_[2048]{};

    msg::topic* topic_ = nullptr;
    dmimu_telemetry& telemetry_ = dmimu_debug_telemetry;
    runtime::monitor loop_monitor_{1000U};
    bool initialized_ = false;
};
#endif

} // namespace ahrs
