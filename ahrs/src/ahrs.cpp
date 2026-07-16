#include "ahrs.hpp"

#include "bsp_dwt.hpp"
#include "constants.hpp"

#include <cstring>

namespace ahrs
{

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

} // namespace

service& service::instance()
{
    static service inst;
    return inst;
}

bool service::create_resources()
{
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
    tx_semaphore_put(&self->gyro_data_ready_sem_);
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

    self->imu_.test.acc_chip_id_err = true;
    self->imu_.test.acc_data_err = true;
    self->imu_.test.gyro_chip_id_err = true;
    self->imu_.test.gyro_data_err = true;
    self->imu_.test.init_err = true;
    self->imu_.test.calibrate_err = false;
    self->imu_.test.temp_ctrl_err = false;
    self->imu_.set_gyro_data_ready_callback(gyro_data_ready_callback, self);

    self->imu_.configure();
    self->imu_.verify_acc_chip_id();
    self->imu_.verify_gyro_chip_id();

    while (!self->imu_.temperature_ready())
    {
        tx_thread_sleep(100);
    }
    self->imu_.test.init_err = false;
    self->imu_.calibrate();
    while (tx_semaphore_get(&self->gyro_data_ready_sem_, TX_NO_WAIT) == TX_SUCCESS)
    {
        continue;
    }

    if (!bsp::dwt::initialized())
    {
        bsp::dwt::init();
    }
    bsp::dwt::delta_s(&ins_count);

    for (;;)
    {
        tx_semaphore_get(&self->gyro_data_ready_sem_, TX_WAIT_FOREVER);
        while (tx_semaphore_get(&self->gyro_data_ready_sem_, TX_NO_WAIT) == TX_SUCCESS)
        {
            continue;
        }
        if (!self->imu_.test.init_err)
        {
            self->imu_.read(reading);
            if (self->cfg_.imu_offset_x != 0.0f)
            {
                reading.accel.x += self->cfg_.imu_offset_x * reading.gyro.z * reading.gyro.z;
            }
            const float dt = solve_delta_s(&ins_count);
            ekf.update_kalman(reading.gyro.x, reading.gyro.y, reading.gyro.z,
                              reading.accel.x, reading.accel.y, reading.accel.z,
                              dt);
        }

        tx_semaphore_put(&self->heartbeat_sem_);
        self->fill_msg(msg, ekf, reading);
        msg::publish(self->ahrs_topic_, msg);
    }
}

} // namespace ahrs
