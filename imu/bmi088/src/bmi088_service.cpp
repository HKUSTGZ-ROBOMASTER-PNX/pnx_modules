#include "attitude.hpp"
#include "attitude_eskf.hpp"
#include "bmi088.hpp"
#include "bsp_dwt.hpp"
#include "user_config.hpp"
#include <cmath>

namespace imu
{
namespace
{
namespace cfg = user_config::ahrs;
using attitude::output;
using attitude::phase;
using attitude::source;
using attitude::state;
using pnxtypes::status;
imu::bmi088 sensor{user_config::bmi088};
estimation::attitude_eskf filter;
TX_THREAD worker{};
alignas(8) uint8_t stack[cfg::bmi088_stack_size]{};
alignas(8) uint8_t heater_stack[user_config::bmi088_heater_stack_size]{};
bool attempted = false;
status initialization = status::not_initialized;

void run(ULONG)
{
    state current{};
    current.device = source::bmi088;
    sensor.gyro_offset = user_config::bmi088_gyro_offset;
    sensor.accel_offset = user_config::bmi088_accel_offset;
    auto& parameters = filter.parameters();
    parameters.gyro_noise = cfg::eskf::gyro_noise;
    parameters.gyro_bias_walk = cfg::eskf::gyro_bias_walk;
    parameters.accel_noise = cfg::eskf::accel_noise;
    parameters.accel_magnitude_gain = cfg::eskf::accel_magnitude_gain;
    parameters.nis_gate = cfg::eskf::nis_gate;
    parameters.reject_timeout = cfg::eskf::reject_timeout;
    parameters.impact_jerk = cfg::eskf::impact_jerk;
    parameters.impact_hold = cfg::eskf::impact_hold;
    parameters.static_accel_tolerance = cfg::eskf::static_accel_tolerance;
    parameters.static_gyro_tolerance = cfg::eskf::static_gyro_tolerance;
    parameters.static_filter_time_constant = cfg::eskf::static_filter_time_constant;
    parameters.zero_rate_min_time = cfg::eskf::zero_rate_min_time;
    parameters.initial_tilt_stddev = cfg::eskf::initial_tilt_stddev;
    parameters.initial_yaw_stddev = cfg::eskf::initial_yaw_stddev;
    parameters.initial_bias_stddev = cfg::eskf::initial_bias_stddev;
    parameters.min_bias_stddev = cfg::eskf::min_bias_stddev;

    for (;;)
    {
        imu::bmi088_heater::disable();
        filter.reset();
        current.online = false;
        current.stage = phase::starting;
        current.result = status::busy;
        (void)output.publish(current);

        auto result = sensor.configure();
        const auto started_ms = HAL_GetTick();
        uint32_t last_temperature_ms = started_ms - user_config::bmi088_heater.period_ms;
        uint64_t last_sample_us = bsp::dwt::timeline_us();
        imu::bmi088::calibration calibration{};
        bool calibration_complete = !cfg::calibrate_on_start;
        bool warmup_complete = !user_config::bmi088_heater_enabled;

        while (result == status::ok)
        {
            const auto now_ms = HAL_GetTick();
            if (now_ms - last_temperature_ms >= user_config::bmi088_heater.period_ms)
            {
                last_temperature_ms = now_ms;
                result = sensor.read_temperature(&current.temperature);
                if (result != status::ok)
                {
                    break;
                }
                if (user_config::bmi088_heater_enabled)
                {
                    imu::bmi088_heater::submit(current.temperature, HAL_GetTick());
                }
            }
            if (user_config::bmi088_heater_enabled)
            {
                const auto heater = imu::bmi088_heater::read();
                if (heater.result != status::ok && heater.result != status::timeout &&
                    heater.result != status::not_initialized)
                {
                    result = heater.result;
                    break;
                }
                // Once warmed, normal temperature fluctuations do not restart calibration.
                warmup_complete = warmup_complete || heater.stable;
                if (!warmup_complete && now_ms - started_ms >= cfg::warmup_timeout_ms)
                {
                    result = status::timeout;
                    break;
                }
            }

            math::vector3 accel{};
            math::vector3 gyro{};
            result = sensor.read_acc(&accel);
            if (result == status::ok)
            {
                result = sensor.read_gyro(&gyro);
            }
            if (result != status::ok)
            {
                break;
            }
            const auto sample_us = bsp::dwt::timeline_us();
            const float dt = static_cast<float>(sample_us - last_sample_us) * 1.0e-6f;
            last_sample_us = sample_us;
            current.online = false;
            if (!warmup_complete)
            {
                current.stage = phase::warming;
            }
            else if (!calibration_complete)
            {
                current.stage = phase::calibrating;
                const auto progress = sensor.calibrate(calibration, gyro, accel);
                if (progress == status::ok)
                {
                    calibration_complete = true;
                    filter.reset();
                    // This gyro sample still uses the previous offset.
                    tx_thread_sleep(cfg::sample_period_ticks);
                    continue;
                }
                if (progress != status::busy)
                {
                    result = progress;
                    break;
                }
            }
            else
            {
                if (!(dt > 0) || dt > cfg::max_dt)
                {
                    result = status::timeout;
                    break;
                }
                filter.update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, dt);
                const auto& attitude = filter.output();
                const auto& q = attitude.quaternion;
                const float norm_squared = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
                if (!std::isfinite(norm_squared) || norm_squared < 0.5f || norm_squared > 1.5f)
                {
                    result = status::error;
                    break;
                }
                current.quaternion = {attitude.quaternion[0], attitude.quaternion[1],
                    attitude.quaternion[2], attitude.quaternion[3]};
                current.roll = attitude.roll;
                current.pitch = attitude.pitch;
                current.yaw = attitude.yaw;
                current.total_yaw = attitude.total_yaw;
                current.gyro_bias = {attitude.gyro_bias[0], attitude.gyro_bias[1],
                    attitude.gyro_bias[2]};
                current.gyro = math::subtract(gyro, current.gyro_bias);
                current.accel = accel;
                current.nis = attitude.nis;
                current.accel_updated = attitude.accel_updated;
                current.zero_rate_active = attitude.zero_rate_active;
                current.online = attitude.initialized;
                current.stage = attitude.initialized ? phase::running : phase::starting;
                if (current.online)
                {
                    ++current.sequence;
                }
            }

            current.received_ms = HAL_GetTick();
            current.result = current.online ? status::ok : status::busy;
            (void)output.publish(current);
            tx_thread_sleep(cfg::sample_period_ticks);
        }

        // End the failed acquisition session before waiting to configure the sensor again.
        imu::bmi088_heater::disable();
        const auto stopped = sensor.shutdown();
        current.online = false;
        current.stage = phase::fault;
        current.result = stopped == status::ok ? result : stopped;
        (void)output.publish(current);
        const auto failed_ms = HAL_GetTick();
        while (HAL_GetTick() - failed_ms < cfg::retry_period_ms)
        {
            tx_thread_sleep(cfg::sample_period_ticks);
        }
    }
}
} // namespace

pnxtypes::status start_bmi088()
{
    if (attempted)
    {
        return initialization;
    }
    attempted = true;
    initialization = output.init();
    if (initialization != status::ok)
    {
        return initialization;
    }
    if (user_config::bmi088_heater_enabled)
    {
        initialization = imu::bmi088_heater::init(board::bmi088::heater_timer,
            board::bmi088::heater_channel, user_config::bmi088_thermal, user_config::bmi088_heater,
            heater_stack, sizeof(heater_stack));
        if (initialization != status::ok)
        {
            return initialization;
        }
    }
    const auto created = tx_thread_create(&worker, const_cast<CHAR*>("ahrs bmi088"), run, 0, stack,
        sizeof(stack), cfg::thread_priority, cfg::thread_priority, TX_NO_TIME_SLICE, TX_AUTO_START);
    initialization = created == TX_SUCCESS ? status::ok : status::error;
    return initialization;
}
} // namespace imu
