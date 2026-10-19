#pragma once
#include "config.hpp"
#include "status.hpp"
#include "tx_api.h"

namespace imu
{
// Defaults match penix H723. Set the C-board values before enabling heating.
struct bmi088_thermal
{
    float target = 45.0f;
    float boost = 0.06f;
    float reboost = 0.05f;
    float approach_start = 0.044f;
    float approach_end = 0.029f;
    float hold_max = 0.04f;
};

struct bmi088_heater_config
{
    uint32_t period_ms = 125; // PID gains are tuned for 125 ms; other periods are rejected.
    uint32_t timeout_ms = 500;
    uint32_t stable_time_ms = 2000;
    float stable_tolerance = 0.5f;
    uint32_t thread_priority = 2;
};

namespace bmi088_heater
{
struct state
{
    float temperature = 0;
    float duty = 0;
    bool stable = false;
    bool active = false;
    pnxtypes::status result = pnxtypes::status::not_initialized;
};

// Serialized startup, static-lifetime stack; owns this PWM channel for firmware lifetime.
// No SPI access. Do not terminate the worker externally or write its PWM channel elsewhere.
pnxtypes::status init(bsp::pwm::handle timer, bsp::pwm::channel channel,
    const bmi088_thermal& thermal, const bmi088_heater_config& config, void* stack,
    ULONG stack_size);
// Single sampling-thread producer. Submit only newly read temperatures with HAL tick timestamps.
void submit(float temperature, uint32_t received_ms);
// Invalidate temperature; the worker stops PWM on its next tick. Fresh submit re-enables it.
void disable();
state read();
} // namespace bmi088_heater
} // namespace imu
