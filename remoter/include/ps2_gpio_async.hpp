#pragma once

#include "bsp_gpio.hpp"
#include "bsp_timer.hpp"
#include "tx_api.h"

#include <cstddef>
#include <cstdint>

namespace remoter
{

// GPIO timing advances in the timer ISR. transfer() suspends its calling thread
// until completion; the buffers must remain valid until transfer() returns.
class ps2_gpio_async
{
public:
    constexpr ps2_gpio_async(bsp::gpio::output cmd, bsp::gpio::input data,
                            bsp::gpio::output clk, bsp::gpio::output cs,
                            bsp::timer::channel timer) noexcept
        : cmd_(cmd), data_(data), clk_(clk), cs_(cs), timer_(timer)
    {
    }

    types::status init();
    types::status transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t size);

private:
    enum class phase : std::uint8_t { idle, set_cmd, lower_clk, sample_data, next_bit, release_cs };

    void on_timer();
    void finish(types::status result);
    void wait_for(phase next, std::uint32_t delay_us);

    bsp::gpio::output cmd_;
    bsp::gpio::input data_;
    bsp::gpio::output clk_;
    bsp::gpio::output cs_;
    bsp::timer::channel timer_;
    TX_SEMAPHORE complete_{};
    bool initialized_ = false;
    volatile phase phase_ = phase::idle;
    volatile types::status result_ = types::status::ok;
    const std::uint8_t* tx_ = nullptr;
    std::uint8_t* rx_ = nullptr;
    std::size_t size_ = 0;
    std::size_t byte_ = 0;
    std::uint8_t bit_ = 0;
};

} // namespace remoter
