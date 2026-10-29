#include "dmimu.hpp"
#include "quantize.hpp"
#include <cmath>

// Decoder ported from penix DMIMU: L1 16-bit vectors and packed 14-bit WXYZ.
// Temperature follows the integer-Celsius acceleration byte used by that driver.

namespace imu
{
namespace
{
// Vector and Euler frames use little-endian unsigned 16-bit components.
float decode_u16(const uint8_t* bytes, float minimum, float maximum)
{
    const uint16_t raw = static_cast<uint16_t>(bytes[0] | (bytes[1] << 8));
    return math::dequantize(raw, minimum, maximum, 16U);
}

math::vector3 decode_vector(const uint8_t data[8], float range)
{
    return {decode_u16(data + 2, -range, range), decode_u16(data + 4, -range, range),
        decode_u16(data + 6, -range, range)};
}

} // namespace

pnxtypes::status dmimu::configure(const config& cfg)
{
    if (cfg.bus == bsp::can::handle::none || cfg.bus > bsp::can::handle::fdcan3 ||
        cfg.can_id > 255 || cfg.master_id > 255 || cfg.communication > mode::active ||
        cfg.assembly_timeout_ms == 0 || cfg.offline_timeout_ms < cfg.assembly_timeout_ms ||
        cfg.offline_timeout_ms > INT32_MAX)
    {
        return pnxtypes::status::invalid_arg;
    }
    cfg_ = cfg;
    configured_ = true;
    clear();
    return pnxtypes::status::ok;
}

void dmimu::clear()
{
    working_ = {};
    completed_ = {};
    pending_ = 0;
    available_ = false;
    have_sample_ = false;
    reply_available_ = false;
}

pnxtypes::status dmimu::receive(bsp::can::handle bus, const bsp::can::rx_frame& frame,
    uint32_t received_ms)
{
    if (!configured_)
    {
        return pnxtypes::status::not_initialized;
    }
    if (bus != cfg_.bus || frame.id != cfg_.master_id || frame.len != 8 || frame.remote ||
        frame.id_kind != bsp::can::id_type::standard ||
        frame.format != bsp::can::frame_mode::classic)
    {
        return pnxtypes::status::invalid_arg;
    }

    const auto* data = frame.data;
    if (data[0] == 0xCC)
    {
        if (data[2] != 0xDD)
        {
            return pnxtypes::status::error;
        }
        reply_.reg = static_cast<register_id>(data[1]);
        reply_.ack = data[3] <= 3 ? static_cast<ack_code>(data[3]) : ack_code::unknown;
        reply_.data = uint32_t(data[4]) | (uint32_t(data[5]) << 8) | (uint32_t(data[6]) << 16) |
                      (uint32_t(data[7]) << 24);
        reply_available_ = true;
        return pnxtypes::status::ok;
    }

    if (data[0] < 1 || data[0] > 4)
    {
        return pnxtypes::status::error;
    }
    const uint8_t component_bit = static_cast<uint8_t>(1U << (data[0] - 1));
    // A repeated component starts a new candidate rather than mixing cycles.
    // The protocol has no sampling sequence: the window bounds age, not simultaneity.
    const bool repeated_component = (pending_ & component_bit) != 0;
    const bool assembly_expired =
        pending_ != 0 && received_ms - working_.received_ms > cfg_.assembly_timeout_ms;
    if (repeated_component || assembly_expired)
    {
        pending_ = 0;
    }
    if (pending_ == 0)
    {
        working_ = {};
        working_.received_ms = received_ms;
    }
    switch (data[0])
    {
        case 1:
            working_.accel = decode_vector(data, 235.2f);
            working_.temperature = static_cast<float>(data[1]); // Integer degrees Celsius.
            break;
        case 2:
            working_.gyro = decode_vector(data, 34.88f);
            break;
        case 3:
            // Wire order is Pitch, Yaw, Roll; convert device degrees to radians.
            working_.pitch = math::degrees_to_radians(decode_u16(data + 2, -90.0f, 90.0f));
            working_.yaw = math::degrees_to_radians(decode_u16(data + 4, -180.0f, 180.0f));
            working_.roll = math::degrees_to_radians(decode_u16(data + 6, -180.0f, 180.0f));
            break;
        case 4:
            decode_quaternion(data);
            break;
    }
    pending_ |= component_bit;
    constexpr uint8_t complete_sample = 0x0F; // Acceleration, gyro, Euler angles and quaternion.
    if (pending_ != complete_sample)
    {
        return pnxtypes::status::ok;
    }

    pending_ = 0;
    const auto q = working_.quaternion;
    const float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!std::isfinite(norm) || norm < 0.5f || norm > 1.5f)
    {
        clear();
        return pnxtypes::status::error;
    }
    working_.quaternion = {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
    working_.sequence = ++sequence_;
    completed_ = working_;
    available_ = true;
    have_sample_ = true;
    return pnxtypes::status::ok;
}

bool dmimu::online(uint32_t now_ms) const
{
    return configured_ && have_sample_ &&
           now_ms - completed_.received_ms <= cfg_.offline_timeout_ms;
}

bool dmimu::take_sample(sample& out, uint32_t now_ms)
{
    if (!available_)
    {
        return false;
    }
    available_ = false;
    if (!online(now_ms))
    {
        return false;
    }
    out = completed_;
    return true;
}

bool dmimu::take_reply(reply& out)
{
    if (!reply_available_)
    {
        return false;
    }
    out = reply_;
    reply_available_ = false;
    return true;
}

pnxtypes::status dmimu::send_command(register_id reg, bool write, uint32_t value)
{
    if (!configured_)
    {
        return pnxtypes::status::not_initialized;
    }
    const uint8_t data[]{0xCC, static_cast<uint8_t>(reg), static_cast<uint8_t>(write), 0xDD,
        static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)};
    return bsp::can::transmit(cfg_.bus, cfg_.can_id, data, sizeof(data));
}

pnxtypes::status dmimu::read_register(register_id reg)
{
    return send_command(reg, false, 0);
}

pnxtypes::status dmimu::write_register(register_id reg, uint32_t data)
{
    return send_command(reg, true, data);
}

void dmimu::decode_quaternion(const uint8_t data[8])
{
    // DATA[2][7:2] contains W[5:0]. The V1.2 example's 0xF8 drops W bit 0;
    // follow the protocol table and retain all six bits.
    const uint16_t w =
        static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 6U) | ((data[2] & 0xFCU) >> 2U));
    const uint16_t x =
        static_cast<uint16_t>(((data[2] & 0x03U) << 12U) | (static_cast<uint16_t>(data[3]) << 4U) |
                              ((data[4] & 0xF0U) >> 4U));
    const uint16_t y =
        static_cast<uint16_t>(((data[4] & 0x0FU) << 10U) | (static_cast<uint16_t>(data[5]) << 2U) |
                              ((data[6] & 0xC0U) >> 6U));
    const uint16_t z = static_cast<uint16_t>(((data[6] & 0x3FU) << 8U) | data[7]);

    working_.quaternion.w = math::dequantize(w, -1.0f, 1.0f, 14U);
    working_.quaternion.x = math::dequantize(x, -1.0f, 1.0f, 14U);
    working_.quaternion.y = math::dequantize(y, -1.0f, 1.0f, 14U);
    working_.quaternion.z = math::dequantize(z, -1.0f, 1.0f, 14U);
}

} // namespace imu
