#pragma once

#include "bsp_usb.hpp"
#include "callback.hpp"
#include "host_protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace host_protocol::usb
{

using request_callback = core::callback<types::status(const host_packet&)>;

struct config
{
    // Invoked in the USB read thread after a complete, validated host packet.
    // Return value is written into the generated device response.
    request_callback on_request{};
};

struct runtime_state
{
    std::uint32_t rx_count = 0;
    std::uint32_t tx_count = 0;
    std::uint32_t error_count = 0;
    std::uint32_t tx_pending_seq = 0;
    bool tx_pending = false;
    types::status last_status = types::status::not_configured;
    host_packet last_rx{};
    device_packet last_tx{};
};

class service
{
public:
    static service& instance();

    types::status init(const config& cfg = {}) noexcept;
    const runtime_state& state() const noexcept { return state_; }

private:
    static void rx_entry(const uint8_t* data, uint16_t len);
    static void tx_result_entry(const bsp::usb::tx_result& result);

    void process_packet(const host_packet& packet);
    void handle_tx_result(const bsp::usb::tx_result& result);

    config cfg_{};
    runtime_state state_{};
    std::array<uint8_t, sizeof(host_packet)> rx_packet_bytes_{};
    std::size_t rx_packet_size_ = 0U;
    bool initialized_ = false;
};

} // namespace host_protocol::usb
