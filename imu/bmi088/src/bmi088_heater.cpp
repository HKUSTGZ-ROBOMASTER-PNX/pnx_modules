#include "bmi088_heater.hpp"
#include "bsp_pwm.hpp"
#include "pid.hpp"
#include "main.h"
#include <cmath>

namespace imu::bmi088_heater
{
namespace
{
enum class heat_phase
{
    boost,
    approach,
    hold,
    overtemperature
};

struct
{
    bsp::pwm::handle heater_timer;
    bsp::pwm::channel heater_channel;
} wiring{};

bmi088_thermal thermal{};
bmi088_heater_config cfg{};
control::pid temperature_pid{300.0f, 0.06f, 100.0f, 90.0f, 200.0f,
    control::pid_mode::position | control::pid_mode::integral_limit |
        control::pid_mode::changing_integral_rate | control::pid_mode::derivative_on_measurement};
heat_phase heat_state = heat_phase::boost;
float heater_duty = 0;
bool heater_started = false;
TX_THREAD worker{};
bool attempted = false;
pnxtypes::status initialization = pnxtypes::status::not_initialized;

struct temperature_sample
{
    float temperature = 0;
    uint32_t received_ms = 0;
    bool valid = false;
} latest{};

state published{};

pnxtypes::status stop_heater()
{
    heat_state = heat_phase::boost;
    heater_duty = 0.0f;
    temperature_pid.clear();
    if (wiring.heater_timer == bsp::pwm::handle::none &&
        wiring.heater_channel == bsp::pwm::channel::none)
    {
        return pnxtypes::status::ok;
    }
    const auto zero = bsp::pwm::set_duty(wiring.heater_timer, wiring.heater_channel, 0.0f);
    const auto stopped = bsp::pwm::stop(wiring.heater_timer, wiring.heater_channel);
    if (stopped == pnxtypes::status::ok)
    {
        heater_started = false;
    }
    return zero != pnxtypes::status::ok ? zero : stopped;
}

pnxtypes::status temperature_control(float temp)
{
    if (!std::isfinite(temp) || temp < -20.0f || temp > 85.0f)
    {
        const auto stopped = stop_heater();
        return stopped == pnxtypes::status::ok ? pnxtypes::status::invalid_arg : stopped;
    }
    const auto previous = heat_state;
    const float approach =
        thermal.approach_end + (thermal.approach_start - thermal.approach_end) *
                                   math::clamp((thermal.target - 0.1f - temp) / 3.9f, 0.0f, 1.0f);
    switch (heat_state)
    {
        case heat_phase::boost:
            if (temp < thermal.target - 4.0f)
            {
                heater_duty = thermal.boost;
            }
            else
            {
                heat_state = heat_phase::approach;
                heater_duty = approach;
            }
            break;
        case heat_phase::approach:
        case heat_phase::hold:
            if (temp >= thermal.target + 0.2f)
            {
                heat_state = heat_phase::overtemperature;
                heater_duty = 0.0f;
            }
            else if (temp < thermal.target - 4.0f)
            {
                heat_state = heat_phase::boost;
                heater_duty = thermal.reboost;
            }
            else if (heat_state == heat_phase::hold)
            {
                temperature_pid.ref = thermal.target;
                temperature_pid.fdb = temp;
                temperature_pid.update();
                heater_duty = math::clamp(temperature_pid.result / 999.0f, 0.0f, thermal.hold_max);
            }
            else if (temp >= thermal.target - 0.1f)
            {
                heat_state = heat_phase::hold;
            }
            else
            {
                heater_duty = approach;
            }
            break;
        case heat_phase::overtemperature:
            heater_duty = 0.0f;
            if (temp <= thermal.target - 0.2f)
            {
                heat_state = heat_phase::approach;
            }
            break;
    }
    // Also stop immediately when a first call arrives with an already-hot sensor.
    if (temp >= thermal.target + 0.2f)
    {
        heat_state = heat_phase::overtemperature;
        heater_duty = 0.0f;
    }
    if (previous != heat_state &&
        (previous == heat_phase::hold || heat_state == heat_phase::overtemperature))
    {
        temperature_pid.reset_state(thermal.target, temp);
    }
    auto result = bsp::pwm::set_duty(wiring.heater_timer, wiring.heater_channel, heater_duty);
    if (result == pnxtypes::status::ok && !heater_started)
    {
        result = bsp::pwm::start(wiring.heater_timer, wiring.heater_channel);
        heater_started = result == pnxtypes::status::ok;
    }
    if (result != pnxtypes::status::ok)
    {
        const auto stopped = stop_heater();
        return stopped == pnxtypes::status::ok ? result : stopped;
    }
    return result;
}

void run(ULONG)
{
    uint32_t last_control = HAL_GetTick() - cfg.period_ms;
    uint32_t stable_since = 0;
    uint32_t last_sample = 0;
    bool consumed = false;
    bool in_band = false;
    state output{};
    for (;;)
    {
        TX_INTERRUPT_SAVE_AREA
        TX_DISABLE
        const auto sample = latest;
        TX_RESTORE
        const auto now = HAL_GetTick();
        if (!sample.valid || now - sample.received_ms >= cfg.timeout_ms)
        {
            const auto stopped = stop_heater();
            output.result = stopped == pnxtypes::status::ok ? pnxtypes::status::timeout : stopped;
            output.stable = false;
            in_band = false;
            consumed = false;
        }
        else if (now - last_control >= cfg.period_ms &&
                 (!consumed || sample.received_ms != last_sample))
        {
            last_control = now;
            last_sample = sample.received_ms;
            consumed = true;
            output.temperature = sample.temperature;
            output.result = temperature_control(sample.temperature);
            const bool near_target =
                output.result == pnxtypes::status::ok &&
                std::fabs(sample.temperature - thermal.target) <= cfg.stable_tolerance;
            if (near_target && !in_band)
            {
                stable_since = now;
            }
            in_band = near_target;
            output.stable = in_band && now - stable_since >= cfg.stable_time_ms;
        }
        output.active = heater_started;
        output.duty = heater_duty;
        TX_DISABLE
        published = output;
        TX_RESTORE
        tx_thread_sleep(1);
    }
}
} // namespace

pnxtypes::status init(bsp::pwm::handle timer, bsp::pwm::channel channel,
    const bmi088_thermal& settings, const bmi088_heater_config& config, void* stack,
    ULONG stack_size)
{
    if (attempted)
    {
        return initialization;
    }
    attempted = true;
    if (timer == bsp::pwm::handle::none || channel == bsp::pwm::channel::none || stack == nullptr ||
        stack_size < TX_MINIMUM_STACK || config.period_ms != 125 ||
        config.timeout_ms <= config.period_ms || config.timeout_ms > INT32_MAX ||
        config.stable_time_ms == 0 || config.stable_time_ms > INT32_MAX ||
        config.thread_priority >= TX_MAX_PRIORITIES || !std::isfinite(config.stable_tolerance) ||
        config.stable_tolerance <= 0 || !std::isfinite(settings.target) || settings.target < 20 ||
        settings.target > 60)
    {
        return initialization = pnxtypes::status::invalid_arg;
    }
    const float limits[]{settings.boost, settings.reboost, settings.approach_start,
        settings.approach_end, settings.hold_max};
    for (float duty : limits)
    {
        if (!std::isfinite(duty) || duty < 0 || duty > 1)
        {
            return initialization = pnxtypes::status::invalid_arg;
        }
    }
    wiring = {timer, channel};
    cfg = config;
    thermal = settings;
    temperature_pid.scalar_a = 3.5f;
    temperature_pid.scalar_b = 0.2f;
    initialization = stop_heater();
    if (initialization != pnxtypes::status::ok)
    {
        return initialization;
    }
    initialization =
        tx_thread_create(&worker, const_cast<CHAR*>("bmi088 heater"), run, 0, stack, stack_size,
            cfg.thread_priority, cfg.thread_priority, TX_NO_TIME_SLICE, TX_AUTO_START) == TX_SUCCESS
            ? pnxtypes::status::ok
            : pnxtypes::status::error;
    return initialization;
}

void submit(float temperature, uint32_t received_ms)
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    latest = {temperature, received_ms, true};
    TX_RESTORE
}

void disable()
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    latest.valid = false;
    TX_RESTORE
}

state read()
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    const auto result = published;
    TX_RESTORE
    return result;
}
} // namespace imu::bmi088_heater
