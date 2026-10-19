#include "bmi088.hpp"
#include "bsp_pwm.hpp"
#include "pid.hpp"
#include "main.h"
#include <cmath>
#if defined(STM32H723xx)
#include "memory.hpp"
#include <cstring>
#endif

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

constexpr auto heater_timer = board::bmi088::heater_timer;
constexpr auto heater_channel = board::bmi088::heater_channel;

bmi088_thermal thermal{};
bmi088_heater_config cfg{};
control::pid temperature_pid{300.0f, 0.06f, 100.0f, 90.0f, 200.0f,
    control::pid_mode::position | control::pid_mode::integral_limit |
        control::pid_mode::changing_integral_rate | control::pid_mode::derivative_on_measurement};
heat_phase heat_state = heat_phase::boost;
float heater_duty = 0;
bool heater_started = false;
#if defined(STM32H723xx)
static_assert(bsp::peripherals::tim3.channel_mask == 0x08U, "TIM3 must be dedicated to the heater");
bool protection_ready = false;
alignas(uint32_t) memory::buffer<sizeof(uint32_t)> zero_source RAM_D1_BSS;
auto* const timeout_timer = bsp::peripherals::heater_timeout_timer;
#endif
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

#if defined(STM32H723xx)
pnxtypes::status init_protection()
{
    using pnxtypes::status;
    auto* dma = timeout_timer->hdma[TIM_DMA_ID_UPDATE];
    const uint64_t divider = static_cast<uint64_t>(timeout_timer->Init.Prescaler) + 1U;
    const uint64_t period = static_cast<uint64_t>(timeout_timer->Init.Period) + 1U;
    const uint64_t duration = divider * period * 1000U;
    const uint64_t clock = bsp::peripherals::heater_timeout_clock_hz;
    if (timeout_timer->Instance == nullptr || timeout_timer->State == HAL_TIM_STATE_RESET ||
        clock == 0 || duration <= clock * cfg.period_ms || duration > clock * 250U ||
        dma == nullptr || dma->State != HAL_DMA_STATE_READY ||
        dma->Init.Request != DMA_REQUEST_TIM6_UP || dma->Init.Direction != DMA_MEMORY_TO_PERIPH ||
        dma->Init.PeriphInc != DMA_PINC_DISABLE || dma->Init.MemInc != DMA_MINC_DISABLE ||
        dma->Init.PeriphDataAlignment != DMA_PDATAALIGN_WORD ||
        dma->Init.MemDataAlignment != DMA_MDATAALIGN_WORD || dma->Init.Mode != DMA_NORMAL ||
        !memory::valid(zero_source))
    {
        return status::error;
    }
    std::memset(zero_source.data(), 0, zero_source.capacity());
    bsp::clean_cache(zero_source.data(), zero_source.capacity());
    __DSB();
    // The deadline and heater PWM must both keep counting while the CPU is halted.
    __HAL_DBGMCU_UnFreeze_TIM6();
    __HAL_DBGMCU_UnFreeze_TIM3();
    CLEAR_BIT(timeout_timer->Instance->CR1, TIM_CR1_CEN);
    CLEAR_BIT(timeout_timer->Instance->DIER, TIM_DIER_UDE | TIM_DIER_UIE);
    SET_BIT(timeout_timer->Instance->CR1, TIM_CR1_OPM);
    protection_ready = true;
    return status::ok;
}

// Call only with the heater output stopped; aborting DMA removes the hardware deadline.
pnxtypes::status stop_protection()
{
    CLEAR_BIT(timeout_timer->Instance->CR1, TIM_CR1_CEN);
    CLEAR_BIT(timeout_timer->Instance->DIER, TIM_DIER_UDE);
    auto* dma = timeout_timer->hdma[TIM_DMA_ID_UPDATE];
    // Without a completion IRQ, HAL can remain BUSY after the hardware transfer finished.
    if (dma->State == HAL_DMA_STATE_BUSY)
    {
        return bsp::to_status(HAL_DMA_Abort(dma));
    }
    return dma->State == HAL_DMA_STATE_READY ? pnxtypes::status::ok : pnxtypes::status::error;
}

pnxtypes::status start_heater()
{
    // Enable PWM at zero before arming DMA, so resuming a breakpoint cannot reopen a timed-out
    // gate.
    auto result = bsp::pwm::set_duty(heater_timer, heater_channel, 0.0f);
    if (result != pnxtypes::status::ok)
    {
        return result;
    }
    auto* heater = bsp::peripherals::tim_handle(heater_timer);
    heater->Instance->EGR = TIM_EGR_UG;
    result = bsp::pwm::start(heater_timer, heater_channel);
    if (result != pnxtypes::status::ok)
    {
        return result;
    }
    heater_started = true;
    __HAL_TIM_SET_COUNTER(timeout_timer, 0U);
    timeout_timer->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(timeout_timer, TIM_FLAG_UPDATE);
    auto* dma = timeout_timer->hdma[TIM_DMA_ID_UPDATE];
    // TIM3 is dedicated to the heater. Gate its output so later CCR writes cannot undo a timeout.
    const auto source = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(zero_source.data()));
    const auto destination =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&heater->Instance->CCER));
    const auto started = bsp::to_status(HAL_DMA_Start(dma, source, destination, 1U));
    if (started != pnxtypes::status::ok)
    {
        return started;
    }
    SET_BIT(timeout_timer->Instance->DIER, TIM_DIER_UDE);
    SET_BIT(timeout_timer->Instance->CR1, TIM_CR1_OPM | TIM_CR1_CEN);
    return pnxtypes::status::ok;
}
#endif

pnxtypes::status stop_heater()
{
    heat_state = heat_phase::boost;
    heater_duty = 0.0f;
    temperature_pid.clear();
    const auto zero = bsp::pwm::set_duty(heater_timer, heater_channel, 0.0f);
    const auto stopped = bsp::pwm::stop(heater_timer, heater_channel);
    auto protection = pnxtypes::status::ok;
#if defined(STM32H723xx)
    // Keep the hardware deadline alive if software could not close the output.
    if (protection_ready && stopped == pnxtypes::status::ok)
    {
        protection = stop_protection();
    }
#endif
    if (stopped == pnxtypes::status::ok)
    {
        heater_started = false;
    }
    if (zero != pnxtypes::status::ok)
    {
        return zero;
    }
    if (stopped != pnxtypes::status::ok)
    {
        return stopped;
    }
    return protection;
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
    auto result = pnxtypes::status::ok;
#if defined(STM32H723xx)
    if (!heater_started)
    {
        result = start_heater();
    }
#endif
    if (result == pnxtypes::status::ok)
    {
        result = bsp::pwm::set_duty(heater_timer, heater_channel, heater_duty);
#if defined(STM32H723xx)
        // The same DMA stays armed throughout heating; a timeout latches the output off.
        __HAL_TIM_SET_COUNTER(timeout_timer, 0U);
#endif
    }
    if (result == pnxtypes::status::ok && !heater_started)
    {
        result = bsp::pwm::start(heater_timer, heater_channel);
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
#if defined(STM32H723xx)
        // HAL ticks stop at a breakpoint. Discard the old sample when hardware timed out.
        if ((timeout_timer->Instance->DIER & TIM_DIER_UDE) != 0U &&
            (timeout_timer->Instance->SR & TIM_SR_UIF) != 0U)
        {
            disable();
        }
#endif
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

pnxtypes::status init(const bmi088_thermal& settings, const bmi088_heater_config& config, void* stack,
    ULONG stack_size)
{
    if (attempted)
    {
        return initialization;
    }
    attempted = true;
    if (stack == nullptr || stack_size < TX_MINIMUM_STACK || config.period_ms != 125 ||
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
    cfg = config;
    thermal = settings;
    temperature_pid.scalar_a = 3.5f;
    temperature_pid.scalar_b = 0.2f;
    initialization = stop_heater();
    if (initialization != pnxtypes::status::ok)
    {
        return initialization;
    }
#if defined(STM32H723xx)
    initialization = init_protection();
    if (initialization != pnxtypes::status::ok)
    {
        return initialization;
    }
#endif
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
