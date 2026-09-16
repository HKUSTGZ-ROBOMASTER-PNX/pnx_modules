#include "ps2_gpio_async.hpp"

namespace remoter
{
namespace
{

constexpr std::uint32_t edge_delay_us = 5U;
constexpr std::uint32_t cs_setup_us = 20U;
constexpr std::uint32_t cs_hold_us = 20U;
constexpr ULONG transfer_timeout_ticks = 20U;

} // namespace

types::status ps2_gpio_async::init()
{
    if (initialized_)
    {
        return types::status::ok;
    }
    auto status = bsp::gpio::set_active(cs_, false);
    if (status == types::status::ok)
    {
        status = bsp::gpio::write(clk_, true);
    }
    if (status == types::status::ok)
    {
        status = bsp::gpio::write(cmd_, true);
    }
    if (status != types::status::ok)
    {
        return status;
    }

    if (tx_semaphore_create(&complete_, const_cast<CHAR*>("ps2_done"), 0U) != TX_SUCCESS)
    {
        return types::status::error;
    }
    status = bsp::timer::init(timer_, core::callback<void()>::bind<
        ps2_gpio_async, &ps2_gpio_async::on_timer>(this));
    if (status != types::status::ok)
    {
        tx_semaphore_delete(&complete_);
        return status;
    }
    initialized_ = true;
    return types::status::ok;
}

types::status ps2_gpio_async::transfer(const std::uint8_t* tx, std::uint8_t* rx,
                                     std::size_t size)
{
    if (!initialized_) return types::status::not_initialized;
    if (tx == nullptr || rx == nullptr || size == 0U) return types::status::invalid_arg;
    if (tx_thread_identify() == nullptr) return types::status::invalid_context;

    // One poll thread owns this transport. Protect setup/timeout against its ISR.
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    if (phase_ != phase::idle)
    {
        TX_RESTORE
        return types::status::busy;
    }
    tx_semaphore_get(&complete_, TX_NO_WAIT);
    tx_ = tx;
    rx_ = rx;
    size_ = size;
    byte_ = 0U;
    bit_ = 0U;
    rx_[0] = 0U;
    const auto status = bsp::gpio::set_active(cs_, true);
    if (status == types::status::ok) wait_for(phase::set_cmd, cs_setup_us);
    else finish(status);
    TX_RESTORE

    const auto wait_status = tx_semaphore_get(&complete_, transfer_timeout_ticks);
    TX_DISABLE
    if (wait_status != TX_SUCCESS)
    {
        bsp::timer::cancel(timer_);
        phase_ = phase::idle;
        bsp::gpio::set_active(cs_, false);
        bsp::gpio::write(clk_, true);
        result_ = types::status::error;
    }
    tx_ = nullptr;
    rx_ = nullptr;
    const auto result = result_;
    TX_RESTORE
    return result;
}

void ps2_gpio_async::wait_for(phase next, std::uint32_t delay_us)
{
    phase_ = next;
    const auto status = bsp::timer::arm(timer_, delay_us);
    if (status != types::status::ok) finish(status);
}

void ps2_gpio_async::finish(types::status result)
{
    bsp::timer::cancel(timer_);
    const auto release_status = bsp::gpio::set_active(cs_, false);
    const auto clock_status = bsp::gpio::write(clk_, true);
    if (result == types::status::ok) result = release_status;
    if (result == types::status::ok) result = clock_status;
    result_ = result;
    phase_ = phase::idle;
    tx_semaphore_put(&complete_);
}

void ps2_gpio_async::on_timer()
{
    types::status status = types::status::ok;
    switch (phase_)
    {
    case phase::next_bit:
        if (++bit_ == 8U)
        {
            bit_ = 0U;
            if (++byte_ == size_)
            {
                wait_for(phase::release_cs, cs_hold_us);
                return;
            }
            rx_[byte_] = 0U;
        }
        [[fallthrough]];

    case phase::set_cmd:
        status = bsp::gpio::write(cmd_, (tx_[byte_] & (1U << bit_)) != 0U);
        if (status == types::status::ok) wait_for(phase::lower_clk, edge_delay_us);
        break;

    case phase::lower_clk:
        status = bsp::gpio::write(clk_, false);
        if (status == types::status::ok) wait_for(phase::sample_data, edge_delay_us);
        break;

    case phase::sample_data:
    {
        bool high = false;
        status = bsp::gpio::read(data_, high);
        if (status != types::status::ok) break;
        if (high) rx_[byte_] |= static_cast<std::uint8_t>(1U << bit_);
        status = bsp::gpio::write(clk_, true);
        if (status == types::status::ok) wait_for(phase::next_bit, edge_delay_us);
        break;
    }

    case phase::release_cs:
        finish(types::status::ok);
        return;

    case phase::idle:
        return;
    }
    if (status != types::status::ok) finish(status);
}

} // namespace remoter
