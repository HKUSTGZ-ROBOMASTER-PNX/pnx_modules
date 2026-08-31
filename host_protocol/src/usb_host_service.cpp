#include "usb_host_service.hpp"

#include <cstring>

namespace host_protocol::usb
{

service& service::instance()
{
    static service instance;
    return instance;
}

types::status service::init(const config& cfg) noexcept
{
    if (initialized_)
    {
        return types::status::ok;
    }

    cfg_ = cfg;
    state_ = {};
    rx_packet_size_ = 0U;

    bsp::usb::config usb_config{};
    usb_config.on_rx = bsp::usb::rx_callback::bind<&rx_entry>();
    usb_config.on_tx_result =
        core::callback<void(const bsp::usb::tx_result&)>::bind<&tx_result_entry>();
    const types::status status = bsp::usb::init(usb_config);
    state_.last_status = status;
    if (status != types::status::ok)
    {
        ++state_.error_count;
        return status;
    }

    initialized_ = true;
    return types::status::ok;
}

void service::rx_entry(const uint8_t* data, uint16_t len)
{
    auto& self = instance();
    for (uint16_t index = 0U; index < len; ++index)
    {
        self.rx_packet_bytes_[self.rx_packet_size_++] = data[index];
        if (self.rx_packet_size_ == self.rx_packet_bytes_.size())
        {
            host_packet packet{};
            std::memcpy(&packet, self.rx_packet_bytes_.data(), sizeof(packet));
            self.rx_packet_size_ = 0U;
            self.process_packet(packet);
        }
    }
}

void service::process_packet(const host_packet& packet)
{
    state_.last_rx = packet;
    if (!validate(packet, usb_host_magic))
    {
        ++state_.error_count;
        state_.last_status = types::status::invalid_arg;
        return;
    }

    ++state_.rx_count;
    const types::status request_status = cfg_.on_request ? cfg_.on_request(packet) : types::status::ok;
    const device_packet response = make_response(
        usb_device_magic, packet, request_status, bsp::usb::connected(), state_.rx_count,
        state_.tx_count + 1U, state_.error_count);
    const types::status tx_status = bsp::usb::try_send(response);
    state_.last_tx = response;
    state_.last_status = tx_status;
    if (tx_status == types::status::ok)
    {
        ++state_.tx_pending_seq;
        state_.tx_pending = true;
    }
    else
    {
        ++state_.error_count;
    }
}

void service::tx_result_entry(const bsp::usb::tx_result& result)
{
    instance().handle_tx_result(result);
}

void service::handle_tx_result(const bsp::usb::tx_result& result)
{
    state_.tx_pending = false;
    if (result.success() && result.requested_len == sizeof(device_packet) &&
        result.actual_len == result.requested_len)
    {
        ++state_.tx_count;
        state_.last_status = types::status::ok;
        return;
    }

    ++state_.error_count;
    state_.last_status = result.code == bsp::usb::tx_result_code::not_connected
                             ? types::status::not_connected
                             : types::status::error;
}

} // namespace host_protocol::usb
