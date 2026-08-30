#include "ahrs.hpp"

#include "bsp_dwt.hpp"
#include "constants.hpp"

#include <cmath>
#include <cstring>

namespace ahrs
{

extern "C" {
telemetry ahrs_debug_telemetry{};
#if HAS_DMIMU
dmimu_telemetry dmimu_debug_telemetry{};
#endif
}

namespace
{

uint32_t ins_count = 0;

float solve_delta_s(uint32_t* last)
{
    if (!bsp::dwt::initialized())
    {
        if (bsp::dwt::init() != types::status::ok)
        {
            return 0.0005f;
        }
    }

    const float dt = bsp::dwt::delta_s(last);
    return dt > 0.0f ? dt : 0.0005f;
}

void update_dt_telemetry(telemetry& out, float dt)
{
    out.dt_s = dt;
    out.dt_min_s = out.update_count == 0U || dt < out.dt_min_s ? dt : out.dt_min_s;
    out.dt_max_s = dt > out.dt_max_s ? dt : out.dt_max_s;
}

float vector_norm(float x, float y, float z)
{
    return std::sqrt(x * x + y * y + z * z);
}

constexpr float degrees_to_radians(float degrees)
{
    return degrees * (math::pi / 180.0f);
}

} // namespace

service& service::instance()
{
    static service inst;
    return inst;
}

bool service::create_resources()
{
    if (!imu_.initialize_transport())
    {
        return false;
    }

    if (tx_semaphore_create(&heartbeat_sem_, const_cast<CHAR*>("ahrs_hb"), 0) != TX_SUCCESS)
    {
        return false;
    }
    if (tx_semaphore_create(&gyro_data_ready_sem_, const_cast<CHAR*>("ahrs_gyro"), 0) != TX_SUCCESS)
    {
        return false;
    }

    ahrs_topic_ = msg::create<message>();
    if (ahrs_topic_ == nullptr)
    {
        return false;
    }

    if (tx_thread_create(&imu_thread_, const_cast<CHAR*>("ahrs_imu"), imu_thread_entry,
                         reinterpret_cast<ULONG>(this),
                         imu_stack_, sizeof(imu_stack_), cfg_.imu_thread_priority,
                         cfg_.imu_thread_priority, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }

    imu::bmi088::temp_control_config temp_cfg{};
    temp_cfg.enabled = cfg_.temperature_control_enabled;
    temp_cfg.target_temp = cfg_.target_temp;
    temp_cfg.thread_priority = cfg_.temp_thread_priority;
    if (!imu_.start_temperature_control(temp_cfg))
    {
        return false;
    }

    return true;
}

bool service::init(const config& cfg)
{
    if (initialized_)
    {
        return true;
    }
    cfg_ = cfg;
    if (msg::init() != types::status::ok)
    {
        return false;
    }
    if (!create_resources())
    {
        return false;
    }
    initialized_ = true;
    telemetry_.initialized = true;
    telemetry_.resources_created = true;
    return true;
}

void service::fill_msg(message& msg, const quaternion_ekf& ekf, const imu::reading& reading)
{
    std::memcpy(msg.quaternion, ekf.q, sizeof(ekf.q));
    msg.yaw = ekf.yaw;
    msg.pitch = ekf.pitch;
    msg.roll = ekf.roll;
    msg.total_yaw = ekf.total_yaw;
    msg.gyro_r = reading.gyro.x;
    msg.gyro_p = reading.gyro.y;
    msg.gyro_y = reading.gyro.z;
    msg.accel[0] = reading.accel.x;
    msg.accel[1] = reading.accel.y;
    msg.accel[2] = reading.accel.z;
}

void service::gyro_data_ready_callback(void* user)
{
    auto* self = static_cast<service*>(user);
    if (self == nullptr)
    {
        return;
    }
    ++self->telemetry_.gyro_ready_count;
    tx_semaphore_put(&self->gyro_data_ready_sem_);
}

void service::wait_for_gyro_data_ready()
{
    tx_semaphore_get(&gyro_data_ready_sem_, TX_WAIT_FOREVER);
    while (tx_semaphore_get(&gyro_data_ready_sem_, TX_NO_WAIT) == TX_SUCCESS)
    {
        ++telemetry_.gyro_ready_drained;
    }
}

void service::imu_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<service*>(arg);
    if (self == nullptr)
    {
        self = &instance();
    }

    message msg{};
    imu::reading reading{};
    quaternion_ekf ekf;
    self->tactical_ekf_.reset();

    self->imu_.test.acc_chip_id_err = true;
    self->imu_.test.acc_data_err = true;
    self->imu_.test.gyro_chip_id_err = true;
    self->imu_.test.gyro_data_err = true;
    self->imu_.test.init_err = true;
    self->imu_.test.calibrate_err = false;
    self->imu_.test.temp_ctrl_err = false;
    self->imu_.set_gyro_data_ready_callback(gyro_data_ready_callback, self);

    self->imu_.configure();
    self->telemetry_.imu_configured = true;
    self->imu_.verify_acc_chip_id();
    self->telemetry_.accel_chip_ok = !self->imu_.test.acc_chip_id_err;
    self->imu_.verify_gyro_chip_id();
    self->telemetry_.gyro_chip_ok = !self->imu_.test.gyro_chip_id_err;

    while (!self->imu_.temperature_ready())
    {
        self->telemetry_.temperature = self->imu_.temperature();
        self->telemetry_.temperature_ready = false;
        tx_thread_sleep(100);
    }
    self->telemetry_.temperature_ready = true;
    self->telemetry_.temperature = self->imu_.temperature();
    self->imu_.test.init_err = false;
    tx_thread_sleep(100);
    self->imu_.calibrate();
    self->telemetry_.calibrated = !self->imu_.test.calibrate_err;
    const auto gyro_offset = self->imu_.calibrated_gyro_offset();
    self->telemetry_.calibrated_gyro_offset[0] = gyro_offset.x;
    self->telemetry_.calibrated_gyro_offset[1] = gyro_offset.y;
    self->telemetry_.calibrated_gyro_offset[2] = gyro_offset.z;
    while (tx_semaphore_get(&self->gyro_data_ready_sem_, TX_NO_WAIT) == TX_SUCCESS)
    {
        ++self->telemetry_.gyro_ready_startup_drained;
    }

    if (!bsp::dwt::initialized())
    {
        bsp::dwt::init();
    }
    bsp::dwt::delta_s(&ins_count);

    for (;;)
    {
        self->wait_for_gyro_data_ready();
        self->imu_loop_monitor_.begin();
        if (!self->imu_.test.init_err)
        {
            self->telemetry_.read_ok = self->imu_.read(reading);
            self->telemetry_.imu_spi_read_error_count = self->imu_.spi_read_error_count();
            self->telemetry_.imu_spi_write_error_count = self->imu_.spi_write_error_count();
            self->telemetry_.imu_spi_lock_error_count = self->imu_.spi_lock_error_count();
            if (!self->telemetry_.read_ok)
            {
                ++self->telemetry_.sample_error_count;
            }
            else
            {
                if (self->cfg_.imu_offset_x != 0.0f)
                {
                    reading.accel.x += self->cfg_.imu_offset_x * reading.gyro.z * reading.gyro.z;
                }
                const float dt = solve_delta_s(&ins_count);
                ekf.update_kalman(reading.gyro.x, reading.gyro.y, reading.gyro.z,
                                  reading.accel.x, reading.accel.y, reading.accel.z,
                                  dt);
                self->tactical_ekf_.update(reading.gyro.x, reading.gyro.y, reading.gyro.z,
                                           reading.accel.x, reading.accel.y, reading.accel.z,
                                           dt);
                update_dt_telemetry(self->telemetry_, dt);
                self->telemetry_.solved = ekf.UpdateCount > 0U;
                self->telemetry_.update_count = static_cast<uint32_t>(ekf.UpdateCount);
                self->telemetry_.temperature = reading.temperature;
                self->telemetry_.accel_norm = vector_norm(reading.accel.x, reading.accel.y, reading.accel.z);
                self->telemetry_.gyro_norm = vector_norm(reading.gyro.x, reading.gyro.y, reading.gyro.z);
                self->telemetry_.gyro[0] = reading.gyro.x;
                self->telemetry_.gyro[1] = reading.gyro.y;
                self->telemetry_.gyro[2] = reading.gyro.z;
                std::memcpy(self->telemetry_.quaternion, ekf.q, sizeof(ekf.q));
                self->telemetry_.yaw = ekf.yaw;
                self->telemetry_.pitch = ekf.pitch;
                self->telemetry_.roll = ekf.roll;
                self->telemetry_.total_yaw = ekf.total_yaw;
                self->telemetry_.tactical = self->tactical_ekf_.state();
            }
        }

        tx_semaphore_put(&self->heartbeat_sem_);
        self->fill_msg(msg, ekf, reading);
        msg::publish(self->ahrs_topic_, msg);
        self->imu_loop_monitor_.end();
        const auto& loop_stats = self->imu_loop_monitor_.stats();
        self->telemetry_.loop_runtime_us = loop_stats.last_us;
        self->telemetry_.loop_runtime_max_us = loop_stats.max_us;
        self->telemetry_.loop_runtime_avg_us = loop_stats.average_us();
        self->telemetry_.loop_runtime_overruns = loop_stats.overrun_count;
    }
}

#if HAS_DMIMU
dmimu_service& dmimu_service::instance()
{
    static dmimu_service inst;
    return inst;
}

bool dmimu_service::create_resources()
{
    if (!dmimu_.initialize(dmimu_cfg_))
    {
        return false;
    }

    if (tx_semaphore_create(&heartbeat_sem_, const_cast<CHAR*>("dmimu_hb"), 0) != TX_SUCCESS)
    {
        return false;
    }

    topic_ = msg::create<dmimu_message>();
    if (topic_ == nullptr)
    {
        return false;
    }

    telemetry_.resources_created = true;
    if (tx_thread_create(&thread_, const_cast<CHAR*>("ahrs_dmimu"), thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         service_cfg_.thread_priority, service_cfg_.thread_priority,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }
    telemetry_.thread_started = true;
    return true;
}

bool dmimu_service::init(const imu::dmimu::config& dmimu_cfg,
                         const dmimu_service_config& service_cfg)
{
    if (initialized_)
    {
        return true;
    }
    if (service_cfg.receive_wait_ticks == TX_NO_WAIT ||
        service_cfg.receive_wait_ticks == TX_WAIT_FOREVER ||
        (dmimu_cfg.runtime.communication_mode == imu::dmimu::mode::request &&
         service_cfg.request_period_ticks == 0U))
    {
        return false;
    }

    dmimu_cfg_ = dmimu_cfg;
    service_cfg_ = service_cfg;
    telemetry_ = {};
    if (msg::init() != types::status::ok)
    {
        return false;
    }
    if (!create_resources())
    {
        return false;
    }

    initialized_ = true;
    telemetry_.initialized = true;
    return true;
}

bool dmimu_service::request_snapshot()
{
    ++telemetry_.request_cycle_count;
    bool ok = true;
    const auto request = [this, &ok](types::status status) {
        if (status != types::status::ok)
        {
            ++telemetry_.request_error_count;
            ok = false;
        }
    };

    request(dmimu_.request_accel());
    request(dmimu_.request_gyro());
    request(dmimu_.request_euler());
    request(dmimu_.request_quaternion());
    return ok;
}

void dmimu_service::publish_snapshot(const imu::dmimu::snapshot& snapshot)
{
    dmimu_message output{};
    std::memcpy(output.quaternion, snapshot.quaternion, sizeof(output.quaternion));
    // The device protocol exposes Euler angles in degrees. The AHRS-facing
    // message contract uses radians, matching the existing BMI088 output.
    output.yaw = degrees_to_radians(snapshot.yaw_deg);
    output.pitch = degrees_to_radians(snapshot.pitch_deg);
    output.roll = degrees_to_radians(snapshot.roll_deg);
    output.gyro_r = snapshot.gyro.x;
    output.gyro_p = snapshot.gyro.y;
    output.gyro_y = snapshot.gyro.z;
    output.accel[0] = snapshot.accel.x;
    output.accel[1] = snapshot.accel.y;
    output.accel[2] = snapshot.accel.z;
    output.sequence = snapshot.sequence;
    output.received_tick = snapshot.received_tick;
    output.online = true;

    if (msg::publish(topic_, output) == types::status::ok)
    {
        ++telemetry_.publish_count;
        telemetry_.last_sequence = output.sequence;
        telemetry_.last_publish_tick = tx_time_get();
    }
    else
    {
        ++telemetry_.publish_error_count;
    }
}

void dmimu_service::publish_offline_zero()
{
    // A value-initialized message deliberately clears quaternion, Euler,
    // angular velocity, acceleration and metadata together on disconnect.
    const dmimu_message output{};
    if (msg::publish(topic_, output) == types::status::ok)
    {
        ++telemetry_.publish_count;
        ++telemetry_.offline_zero_publish_count;
        telemetry_.last_sequence = 0U;
        telemetry_.last_publish_tick = tx_time_get();
    }
    else
    {
        ++telemetry_.publish_error_count;
    }
}

void dmimu_service::update_runtime_telemetry()
{
    const auto& stats = loop_monitor_.stats();
    telemetry_.loop_runtime_us = stats.last_us;
    telemetry_.loop_runtime_max_us = stats.max_us;
    telemetry_.loop_runtime_avg_us = stats.average_us();
    telemetry_.loop_runtime_overruns = stats.overrun_count;
}

void dmimu_service::thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<dmimu_service*>(arg);
    if (self == nullptr)
    {
        self = &instance();
    }

    ULONG last_request_tick = tx_time_get();
    if (self->dmimu_cfg_.runtime.communication_mode == imu::dmimu::mode::request)
    {
        last_request_tick -= self->service_cfg_.request_period_ticks;
    }
    bool was_online = false;

    for (;;)
    {
        self->loop_monitor_.begin();
        const ULONG now = tx_time_get();
        if (self->dmimu_cfg_.runtime.communication_mode == imu::dmimu::mode::request &&
            (now - last_request_tick) >= self->service_cfg_.request_period_ticks)
        {
            self->request_snapshot();
            last_request_tick = now;
        }

        uint32_t processed = 0U;
        if (self->dmimu_.process_next(self->service_cfg_.receive_wait_ticks))
        {
            processed = 1U + self->dmimu_.process_pending();
            self->telemetry_.processed_frame_count += processed;
        }

        imu::dmimu::snapshot snapshot{};
        const bool has_snapshot = self->dmimu_.take_snapshot(snapshot);
        const bool is_online = self->dmimu_.audit_online();
        self->telemetry_.snapshot_received = has_snapshot;
        self->telemetry_.online = is_online;

        if (has_snapshot)
        {
            self->publish_snapshot(snapshot);
        }
        else if (was_online && !is_online)
        {
            self->publish_offline_zero();
        }
        was_online = is_online;

        self->telemetry_.device = self->dmimu_.diagnostics_snapshot();
        tx_semaphore_put(&self->heartbeat_sem_);
        self->loop_monitor_.end();
        self->update_runtime_telemetry();
    }
}
#endif

} // namespace ahrs
