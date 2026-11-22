#include "bmi270.hpp"
#include <cmath>
#include <cstring>
#include <limits>

namespace imu
{
namespace bmi270_protocol
{
// CRC-32/ISO-HDLC over gyro XYZ followed by accel XYZ, IEEE754 little-endian floats.
uint32_t bias_checksum(const math::vector3& gyro, const math::vector3& accel)
{
    const float axes[]{gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z};
    uint32_t crc = UINT32_MAX;
    for (float axis : axes)
    {
        uint32_t bits;
        std::memcpy(&bits, &axis, sizeof(bits));
        for (unsigned byte = 0; byte < 4; ++byte)
        {
            crc ^= (bits >> (byte * 8)) & 0xffU;
            for (unsigned bit = 0; bit < 8; ++bit)
            {
                crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
            }
        }
    }
    return ~crc;
}
} // namespace bmi270_protocol

namespace
{
uint16_t u16(const uint8_t* bytes)
{
    return static_cast<uint16_t>(uint16_t(bytes[0]) | (uint16_t(bytes[1]) << 8));
}

uint32_t u32(const uint8_t* bytes)
{
    return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) | (uint32_t(bytes[2]) << 16) |
           (uint32_t(bytes[3]) << 24);
}

float f32(const uint8_t* bytes)
{
    static_assert(sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559);
    const uint32_t bits = u32(bytes);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

int32_t i16(const uint8_t* bytes)
{
    const auto value = u16(bytes);
    return value < 0x8000 ? int32_t(value) : int32_t(value) - 0x10000;
}
} // namespace

pnxtypes::status bmi270::configure(const config& cfg)
{
    if (cfg.bus == bsp::can::handle::none || cfg.bus > bsp::can::handle::fdcan3 ||
        cfg.command_id != command_id || cfg.target_id != gyro_id || cfg.assembly_timeout_ms == 0 ||
        cfg.assembly_timeout_ms > INT32_MAX || cfg.gyro_timeout_ms == 0 ||
        cfg.gyro_timeout_ms > INT32_MAX || cfg.quaternion_timeout_ms == 0 ||
        cfg.quaternion_timeout_ms > INT32_MAX || cfg.accel_timeout_ms == 0 ||
        cfg.accel_timeout_ms > INT32_MAX || cfg.euler_timeout_ms == 0 ||
        cfg.euler_timeout_ms > INT32_MAX)
    {
        return pnxtypes::status::invalid_arg;
    }
    const float offsets[]{cfg.gyro_offset.x, cfg.gyro_offset.y, cfg.gyro_offset.z};
    for (float offset : offsets)
    {
        if (!std::isfinite(offset) || std::fabs(offset) > 0.15f)
        {
            return pnxtypes::status::invalid_arg;
        }
    }
    const float accel_offsets[]{cfg.accel_offset.x, cfg.accel_offset.y, cfg.accel_offset.z};
    for (float offset : accel_offsets)
    {
        if (!std::isfinite(offset) || std::fabs(offset) > 100.0f)
        {
            return pnxtypes::status::invalid_arg;
        }
    }
    cfg_ = cfg;
    configured_ = true;
    clear();
    return pnxtypes::status::ok;
}

void bmi270::clear()
{
    feedback_.gyro_valid = false;
    feedback_.quaternion_valid = false;
    feedback_.accel_valid = false;
    feedback_.euler_valid = false;
    feedback_.flags = 0;
    feedback_.accel_flags = 0;
    feedback_.euler_flags = 0;
    next_fragment_ = 0;
    ack_available_ = false;
    calibration_available_ = false;
}

pnxtypes::status bmi270::receive(bsp::can::handle bus, const bsp::can::rx_frame& frame,
    uint32_t received_ms)
{
    using pnxtypes::status;
    if (!configured_)
    {
        return status::not_initialized;
    }
    if (bus != cfg_.bus ||
        (frame.id != gyro_id && frame.id != quaternion_id && frame.id != accel_id &&
            frame.id != euler_id && frame.id != reply_id) ||
        frame.len != 8 || frame.remote || frame.id_kind != bsp::can::id_type::standard ||
        frame.format != bsp::can::frame_mode::classic)
    {
        return status::invalid_arg;
    }

    const auto* data = frame.data;
    if (frame.id == gyro_id)
    {
        if (!cfg_.receive_gyro)
        {
            return status::ok;
        }
        constexpr uint8_t gyro_valid = 0x01;
        constexpr uint8_t quat_valid = 0x02;
        constexpr uint8_t sensor_fault = 0x10;
        constexpr uint8_t reserved = 0xe0;
        feedback_.gyro_sequence = data[6];
        feedback_.flags = data[7];
        if ((data[7] & (sensor_fault | reserved)) != 0 || (data[7] & quat_valid) == 0)
        {
            feedback_.quaternion_valid = false;
        }
        const auto x = i16(data);
        const auto y = i16(data + 2);
        const auto z = i16(data + 4);
        if ((data[7] & (sensor_fault | reserved)) != 0 || (data[7] & gyro_valid) == 0 ||
            x == -32768 || y == -32768 || z == -32768)
        {
            feedback_.gyro_valid = false;
            return status::error;
        }
        constexpr float scale = (2000.0f * math::pi / 180.0f) / 32767.0f;
        feedback_.gyro = {x * scale, y * scale, z * scale};
        feedback_.gyro_received_ms = received_ms;
        feedback_.gyro_valid = true;
        ++feedback_.gyro_updates;
        return status::ok;
    }

    if (frame.id == quaternion_id)
    {
        if (!cfg_.receive_quaternion)
        {
            return status::ok;
        }
        const auto w = i16(data);
        const auto x = i16(data + 2);
        const auto y = i16(data + 4);
        const auto z = i16(data + 6);
        if (w == -32768 || x == -32768 || y == -32768 || z == -32768)
        {
            feedback_.quaternion_valid = false;
            return status::error;
        }
        constexpr float scale = 1.0f / 32767.0f;
        math::quaternion q{w * scale, x * scale, y * scale, z * scale};
        const float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (norm < 0.99f || norm > 1.01f)
        {
            feedback_.quaternion_valid = false;
            return status::error;
        }
        q = {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
        const auto angles = math::to_euler(q);
        feedback_.quaternion = q;
        feedback_.roll = angles.roll;
        feedback_.pitch = angles.pitch;
        feedback_.yaw = angles.yaw;
        feedback_.quaternion_received_ms = received_ms;
        feedback_.quaternion_valid = true;
        ++feedback_.quaternion_updates;
        return status::ok;
    }

    if (frame.id == accel_id || frame.id == euler_id)
    {
        const bool is_accel = frame.id == accel_id;
        if (is_accel ? !cfg_.receive_accel : !cfg_.receive_euler)
        {
            return status::ok;
        }
        auto& sequence = is_accel ? feedback_.accel_sequence : feedback_.euler_sequence;
        auto& flags = is_accel ? feedback_.accel_flags : feedback_.euler_flags;
        auto& valid = is_accel ? feedback_.accel_valid : feedback_.euler_valid;
        sequence = data[6];
        flags = data[7];
        const auto x = i16(data);
        const auto y = i16(data + 2);
        const auto z = i16(data + 4);
        if ((flags & 0xf0) != 0 || (flags & 0x01) == 0 || (!is_accel && (flags & 0x02) == 0) ||
            x == -32768 || y == -32768 || z == -32768)
        {
            valid = false;
            return status::error;
        }
        if (is_accel)
        {
            constexpr float scale = 78.4532f / 32767.0f;
            feedback_.accel = {x * scale, y * scale, z * scale};
            feedback_.accel_received_ms = received_ms;
            ++feedback_.accel_updates;
        }
        else
        {
            constexpr float scale = math::pi / 32767.0f;
            feedback_.euler = {x * scale, y * (scale / 2.0f), z * scale};
            feedback_.euler_received_ms = received_ms;
            ++feedback_.euler_updates;
        }
        valid = true;
        return status::ok;
    }

    if (data[0] == 0x40)
    {
        if (data[2] != uint8_t(command::calibrate) && data[2] != uint8_t(command::read_status) &&
            data[2] != uint8_t(command::read_last_calibration) &&
            data[2] != uint8_t(command::set_offsets))
        {
            return status::error;
        }
        ack_ = {static_cast<command>(data[2]), data[1],
            data[3] <= 10 ? static_cast<result>(data[3]) : result::unknown, u32(data + 4),
            received_ms};
        ack_available_ = true;
        return status::ok;
    }

    if (data[0] >= 0xa0 && data[0] <= 0xa9)
    {
        const uint8_t index = data[0] - 0xa0;
        if (index == 0)
        {
            next_fragment_ = 0;
            fragment_sequence_ = data[1];
            status_started_ms_ = received_ms;
        }
        else if (next_fragment_ != index || data[1] != fragment_sequence_ ||
                 received_ms - status_started_ms_ >= cfg_.assembly_timeout_ms)
        {
            next_fragment_ = 0;
            return status::error;
        }
        const uint32_t offset = uint32_t(index) * 6;
        const uint32_t length = index == 9 ? 2 : 6;
        std::memcpy(status_bytes_ + offset, data + 2, length);
        next_fragment_ = index + 1;
        if (next_fragment_ != 10)
        {
            return status::ok;
        }
        next_fragment_ = 0;
        if (u16(status_bytes_) != 5 || u16(status_bytes_ + 2) != gyro_id || status_bytes_[40] > 1 ||
            status_bytes_[41] > 5 || status_bytes_[42] > 8 || status_bytes_[43] > 5 ||
            !std::isfinite(f32(status_bytes_ + 24)) || !std::isfinite(f32(status_bytes_ + 28)) ||
            !std::isfinite(f32(status_bytes_ + 32)) || !std::isfinite(f32(status_bytes_ + 36)) ||
            !std::isfinite(f32(status_bytes_ + 44)) || !std::isfinite(f32(status_bytes_ + 48)) ||
            !std::isfinite(f32(status_bytes_ + 52)))
        {
            return status::error;
        }
        calibration_.version = u16(status_bytes_);
        calibration_.node_id = u16(status_bytes_ + 2);
        calibration_.generation = u32(status_bytes_ + 4);
        calibration_.operation = u32(status_bytes_ + 8);
        calibration_.samples = u32(status_bytes_ + 12);
        calibration_.required_samples = u32(status_bytes_ + 16);
        calibration_.elapsed_ms = u32(status_bytes_ + 20);
        calibration_.gyro_offset = {f32(status_bytes_ + 24), f32(status_bytes_ + 28),
            f32(status_bytes_ + 32)};
        calibration_.temperature = f32(status_bytes_ + 36);
        calibration_.accel_offset = {f32(status_bytes_ + 44), f32(status_bytes_ + 48),
            f32(status_bytes_ + 52)};
        calibration_.valid = status_bytes_[40] != 0;
        calibration_.state = static_cast<calibration_state>(status_bytes_[41]);
        calibration_.code = static_cast<result>(status_bytes_[42]);
        calibration_.stage = static_cast<phase>(status_bytes_[43]);
        calibration_.sequence = ++status_sequence_;
        calibration_.received_ms = status_started_ms_;
        calibration_available_ = true;
        return status::ok;
    }

    return status::error;
}

bmi270::sample bmi270::read(uint32_t now_ms) const
{
    auto value = feedback_;
    value.gyro_valid = value.gyro_valid && now_ms - value.gyro_received_ms < cfg_.gyro_timeout_ms;
    value.quaternion_valid = value.quaternion_valid &&
                             now_ms - value.quaternion_received_ms < cfg_.quaternion_timeout_ms;
    value.accel_valid =
        value.accel_valid && now_ms - value.accel_received_ms < cfg_.accel_timeout_ms;
    value.euler_valid =
        value.euler_valid && now_ms - value.euler_received_ms < cfg_.euler_timeout_ms;
    return value;
}

bool bmi270::take_ack(ack& out)
{
    if (!ack_available_)
    {
        return false;
    }
    out = ack_;
    ack_available_ = false;
    return true;
}

bool bmi270::take_calibration(calibration& out)
{
    if (!calibration_available_)
    {
        return false;
    }
    out = calibration_;
    calibration_available_ = false;
    return true;
}

pnxtypes::status bmi270::send(command operation, uint8_t token, uint8_t part)
{
    if (!configured_)
    {
        return pnxtypes::status::not_initialized;
    }
    if (operation != command::calibrate && operation != command::read_status &&
        operation != command::read_last_calibration && operation != command::set_stream &&
        operation != command::set_offsets)
    {
        return pnxtypes::status::invalid_arg;
    }
    uint8_t data[]{static_cast<uint8_t>(operation), token, uint8_t(cfg_.target_id),
        uint8_t(cfg_.target_id >> 8), 0, 0, 0, 0};
    if (operation == command::set_offsets)
    {
        if (part > 6)
        {
            return pnxtypes::status::invalid_arg;
        }
        uint32_t bits;
        if (part < 6)
        {
            const float axes[]{cfg_.gyro_offset.x, cfg_.gyro_offset.y, cfg_.gyro_offset.z,
                cfg_.accel_offset.x, cfg_.accel_offset.y, cfg_.accel_offset.z};
            std::memcpy(&bits, &axes[part], sizeof(bits));
            data[0] = 0x10 + part;
        }
        else
        {
            bits = bmi270_protocol::bias_checksum(cfg_.gyro_offset, cfg_.accel_offset);
        }
        for (unsigned byte = 0; byte < 4; ++byte)
        {
            data[4 + byte] = static_cast<uint8_t>(bits >> (byte * 8));
        }
    }
    if (operation == command::set_stream)
    {
        data[1] = 0;
        data[4] = (cfg_.receive_gyro ? 0x01U : 0U) | (cfg_.receive_quaternion ? 0x02U : 0U) |
                  (cfg_.receive_accel ? 0x04U : 0U) | (cfg_.receive_euler ? 0x08U : 0U);
    }
    return bsp::can::transmit(cfg_.bus, cfg_.command_id, data, sizeof(data));
}
} // namespace imu
