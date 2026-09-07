#pragma once

#include "bsp_gpio.hpp"
#include "bsp_spi.hpp"
#include "config.hpp"
#include "msg.hpp"
#include "types.hpp"
#include "tx_api.h"
#include "usertypes.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace remoter
{

struct ps2_config
{
    std::uint32_t thread_priority = params::remoter::thread_priority;
};

struct ps2_controller_state
{
    bool select = false;
    bool start = false;
    bool up = false;
    bool down = false;
    bool left = false;
    bool right = false;
    bool l1 = false;
    bool l2 = false;
    bool r1 = false;
    bool r2 = false;
    bool triangle = false;
    bool circle = false;
    bool cross = false;
    bool square = false;
    bool l3 = false;
    bool r3 = false;
    float lx = 0.0f;
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
};

// Shared synchronous-PS2 protocol. Physical transports only provide transfer().
class ps2_protocol
{
public:
    static constexpr std::size_t frame_size = 9U;

    template <typename Transport>
    static types::status poll(Transport& transport, ps2_controller_state& out,
                              ps2_raw_frame* raw = nullptr)
    {
        constexpr std::array<std::uint8_t, frame_size> query{
            0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
        ps2_raw_frame response{};
        const types::status status =
            transport.transfer(query.data(), response.bytes.data(), response.bytes.size());
        if (raw != nullptr)
        {
            *raw = response;
        }
        return status == types::status::ok ? decode(response, out) : status;
    }

    static types::status decode(const ps2_raw_frame& raw, ps2_controller_state& out) noexcept;
};

class ps2_spi
{
public:
    constexpr ps2_spi(bsp::spi::bus spi_bus, bsp::gpio::output cs) noexcept
        : spi_bus_(spi_bus), cs_(cs)
    {
    }

    types::status init();
    types::status transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t size);

private:
    bsp::spi::bus spi_bus_;
    bsp::gpio::output cs_;
};

class ps2_gpio
{
public:
    constexpr ps2_gpio(bsp::gpio::output cmd, bsp::gpio::input data,
                       bsp::gpio::output clk, bsp::gpio::output cs) noexcept
        : cmd_(cmd), data_(data), clk_(clk), cs_(cs)
    {
    }

    types::status init();
    types::status transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t size);

private:
    types::status transfer_byte(std::uint8_t tx, std::uint8_t& rx);

    bsp::gpio::output cmd_;
    bsp::gpio::input data_;
    bsp::gpio::output clk_;
    bsp::gpio::output cs_;
};

// Small, static type-erased transport injection. The generated binding chooses
// one concrete transport at compile time; there is no runtime backend branch.
class ps2
{
public:
    template <typename Transport>
    explicit constexpr ps2(Transport& transport) noexcept
        : transport_(&transport), init_transport_([](void* context) {
              return static_cast<Transport*>(context)->init();
          }), transfer_([](void* context, const std::uint8_t* tx, std::uint8_t* rx,
                           std::size_t size) {
              return static_cast<Transport*>(context)->transfer(tx, rx, size);
          })
    {
    }

    types::status init(const ps2_config& cfg = {});
    msg::channel<ps2_state>& output() noexcept { return output_; }

private:
    static void poll_thread_entry(ULONG arg);
    bool create_resources();
    types::status poll_once(ps2_state& message);

    void* transport_ = nullptr;
    types::status (*init_transport_)(void*) = nullptr;
    types::status (*transfer_)(void*, const std::uint8_t*, std::uint8_t*, std::size_t) = nullptr;
    ps2_config cfg_{};
    TX_THREAD thread_{};
    alignas(8) std::uint8_t stack_[768]{};
    msg::channel<ps2_state> output_{};
    bool initialized_ = false;
};

} // namespace remoter
