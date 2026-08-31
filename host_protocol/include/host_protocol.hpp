#pragma once

#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace host_protocol
{

inline constexpr std::uint32_t usart_host_magic = 0x31535544U;   // "DUS1"
inline constexpr std::uint32_t usart_device_magic = 0x31535552U; // "RUS1"
inline constexpr std::uint32_t usb_host_magic = 0x31425344U;     // "DSB1"
inline constexpr std::uint32_t usb_device_magic = 0x31425352U;   // "RSB1"
inline constexpr std::uint32_t checksum_seed = 0xA5A51234U;

struct host_packet
{
    std::uint32_t magic = 0;
    std::uint32_t seq = 0;
    float value = 0.0f;
    std::uint8_t flag = 0;
    std::uint8_t reserved[3] = {};
    std::uint32_t counter = 0;
    std::uint32_t checksum = 0;
};

struct device_packet
{
    std::uint32_t magic = 0;
    std::uint32_t seq = 0;
    std::uint8_t status = 0;
    std::uint8_t ready = 0;
    std::uint8_t flag = 0;
    std::uint8_t reserved = 0;
    float value = 0.0f;
    std::uint32_t rx_count = 0;
    std::uint32_t tx_count = 0;
    std::uint32_t error_count = 0;
    std::uint32_t checksum = 0;
};

static_assert(std::is_trivially_copyable_v<host_packet>);
static_assert(std::is_trivially_copyable_v<device_packet>);
static_assert(sizeof(host_packet) == 24);
static_assert(sizeof(device_packet) == 32);
static_assert(offsetof(host_packet, checksum) == sizeof(host_packet) - sizeof(std::uint32_t));
static_assert(offsetof(device_packet, checksum) == sizeof(device_packet) - sizeof(std::uint32_t));

inline std::uint32_t checksum_bytes(const void* data, std::size_t len) noexcept
{
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::uint32_t checksum = checksum_seed;
    for (std::size_t i = 0; i < len; ++i)
    {
        checksum = (checksum << 5U) ^ (checksum >> 2U) ^ bytes[i];
    }
    return checksum;
}

template <typename Packet>
inline std::uint32_t checksum_of(const Packet& packet) noexcept
{
    return checksum_bytes(&packet, sizeof(Packet) - sizeof(std::uint32_t));
}

inline bool validate(const host_packet& packet, std::uint32_t expected_magic) noexcept
{
    return packet.magic == expected_magic && packet.checksum == checksum_of(packet);
}

inline std::uint8_t status_code(types::status status) noexcept
{
    return static_cast<std::uint8_t>(status);
}

inline device_packet make_response(std::uint32_t magic, const host_packet& request,
                                   types::status status, bool ready, std::uint32_t rx_count,
                                   std::uint32_t tx_count, std::uint32_t error_count) noexcept
{
    device_packet response{};
    response.magic = magic;
    response.seq = request.seq;
    response.status = status_code(status);
    response.ready = ready ? 1U : 0U;
    response.flag = request.flag;
    response.value = request.value;
    response.rx_count = rx_count;
    response.tx_count = tx_count;
    response.error_count = error_count;
    response.checksum = checksum_of(response);
    return response;
}

inline void copy_host_packet(const void* src, host_packet& out) noexcept
{
    std::memcpy(&out, src, sizeof(out));
}

} // namespace host_protocol
