#include "referee.hpp"

#include "crc.hpp"
#include "memory.h"

#include <cstring>

namespace referee
{

namespace
{

constexpr std::size_t rx_buffer_size = 256;
uint8_t rx_buffer_storage[rx_buffer_size] RAM_D1_BSS{};

enum class decode_result : uint8_t
{
    known,
    unknown,
    invalid_length,
};

template <typename Packet>
bool copy_fixed_packet(Packet& destination, const uint8_t* source, uint16_t length) noexcept
{
    static_assert(std::is_trivially_copyable_v<Packet>);

    if (source == nullptr || length != sizeof(Packet))
    {
        return false;
    }
    std::memcpy(&destination, source, sizeof(Packet));
    return true;
}

bool copy_robot_interaction(robot_interaction_data& destination, const uint8_t* source,
                            uint16_t length) noexcept
{
    if (source == nullptr || length < sizeof(destination.header) ||
        length > protocol::max_data_length)
    {
        return false;
    }

    std::memcpy(&destination.header, source, sizeof(destination.header));
    destination.payload_length = static_cast<uint16_t>(length - sizeof(destination.header));
    std::memset(destination.payload, 0, sizeof(destination.payload));
    std::memcpy(destination.payload, source + sizeof(destination.header),
                destination.payload_length);
    return true;
}

decode_result decode_packet(packet_store& packets, uint16_t command_id, const uint8_t* payload,
                            uint16_t payload_length) noexcept
{
    bool copied = false;
    switch (command_id)
    {
    case RefereeID::GameStatus:
        copied = copy_fixed_packet(packets.game_status, payload, payload_length);
        break;
    case RefereeID::GameRobotStatus:
        copied = copy_fixed_packet(packets.game_robot_status, payload, payload_length);
        break;
    case RefereeID::PowerHeatData:
        copied = copy_fixed_packet(packets.power_heat_data, payload, payload_length);
        break;
    case RefereeID::Buff:
        copied = copy_fixed_packet(packets.buff, payload, payload_length);
        break;
    case RefereeID::RobotHurt:
        copied = copy_fixed_packet(packets.robot_hurt, payload, payload_length);
        break;
    case RefereeID::DartInfo:
        copied = copy_fixed_packet(packets.dart_info, payload, payload_length);
        break;
    case RefereeID::DartClientCmd:
        copied = copy_fixed_packet(packets.dart_client_command, payload, payload_length);
        break;
    case RefereeID::RoboInteractData:
        copied = copy_robot_interaction(packets.robot_interaction, payload, payload_length);
        break;
    default:
        return decode_result::unknown;
    }

    return copied ? decode_result::known : decode_result::invalid_length;
}

void update_legacy_data(data& legacy, const packet_store& packets, uint16_t command_id,
                        const receive_config& receive) noexcept
{
    switch (command_id)
    {
    case RefereeID::GameStatus:
        if (receive.game_status)
        {
            legacy.game_status = packets.game_status;
        }
        break;
    case RefereeID::GameRobotStatus:
        if (receive.game_robot_status)
        {
            legacy.robot_status = packets.game_robot_status;
        }
        break;
    case RefereeID::PowerHeatData:
        if (receive.power_heat)
        {
            legacy.heat_now = packets.power_heat_data.shoot_id1_17mm_cooling_heat;
            legacy.power_buffer = packets.power_heat_data.chassis_power_buffer;
        }
        break;
    case RefereeID::Buff:
        if (receive.buff)
        {
            legacy.remained_energy = packets.buff.remained_energy;
        }
        break;
    case RefereeID::RobotHurt:
        if (receive.robot_hurt)
        {
            legacy.robot_hurt = packets.robot_hurt;
        }
        break;
    case RefereeID::DartInfo:
        if (receive.dart_info)
        {
            legacy.dart_info = packets.dart_info;
        }
        break;
    case RefereeID::DartClientCmd:
        if (receive.dart_client_command)
        {
            legacy.dart_client_command = packets.dart_client_command;
        }
        break;
    case RefereeID::RoboInteractData:
        if (receive.robot_interaction)
        {
            legacy.robot_interaction = packets.robot_interaction;
        }
        break;
    default:
        break;
    }
}

} // namespace

service& service::instance()
{
    static service inst;
    return inst;
}

void service::push_rx(const uint8_t* data, uint16_t len)
{
    if (data != nullptr && len > 0)
    {
        rx_fifo_.push(data, len);
    }
}

void service::referee_rx_callback(bsp::usart::port, const bsp::usart::rx_frame& frame, void*)
{
    if (frame.data != nullptr && frame.len > 0)
    {
        instance().push_rx(frame.data, static_cast<uint16_t>(frame.len));
    }
}

bool service::create_resources()
{
    if (tx_semaphore_create(&heartbeat_sem_, const_cast<CHAR*>("referee_hb"), 0) != TX_SUCCESS)
    {
        return false;
    }

    referee_topic_ = msg::create<data>();
    if (referee_topic_ == nullptr)
    {
        return false;
    }

    std::memset(rx_buffer_storage, 0, sizeof(rx_buffer_storage));
    rx_buffer_ = rx_buffer_storage;

    if (bsp::usart::init(cfg_.uart_port, bsp::usart::mode::dma) != types::status::ok)
    {
        return false;
    }
    if (bsp::usart::start_rx_to_idle(cfg_.uart_port, rx_buffer_, rx_buffer_size, referee_rx_callback,
                                     nullptr, nullptr) != types::status::ok)
    {
        return false;
    }

    if (tx_thread_create(&thread_, const_cast<CHAR*>("referee"), referee_thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         cfg_.thread_priority, cfg_.thread_priority, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }

    return true;
}

bool service::init(const config& cfg)
{
    if (initialized_)
    {
        return true;
    }
    cfg_ = cfg;
    if (msg::init() != types::status::ok)
    {
        return false;
    }
    if (!create_resources())
    {
        return false;
    }
    initialized_ = true;
    return true;
}

void service::referee_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<service*>(arg);
    if (self == nullptr)
    {
        self = &instance();
    }

    data referee_data{};
    uint16_t reset_hurt_count = 0;
    uint8_t rx_byte = 0;
    UnpackState state = STEP_HEADER_SOF;
    uint8_t buffer[256]{};
    uint16_t data_length = 0;
    uint16_t index = 0;

    for (;;)
    {
        bool got_valid_frame = false;

        while (self->rx_fifo_.pop(rx_byte))
        {
            switch (state)
            {
            case STEP_HEADER_SOF:
                if (rx_byte == 0xA5)
                {
                    index = 0;
                    buffer[index++] = rx_byte;
                    state = STEP_LENGTH_SEQ;
                }
                break;

            case STEP_LENGTH_SEQ:
                buffer[index++] = rx_byte;
                if (index == sizeof(FrameHeader))
                {
                    if (crc::verify_crc8_checksum(buffer, sizeof(FrameHeader)) == 0)
                    {
                        ++self->status_.crc8_error_count;
                        state = STEP_HEADER_SOF;
                        break;
                    }

                    data_length = static_cast<uint16_t>(buffer[1] | (buffer[2] << 8));
                    if (data_length > protocol::max_data_length)
                    {
                        ++self->status_.length_error_count;
                        state = STEP_HEADER_SOF;
                        break;
                    }
                    state = STEP_DATA_CRC16;
                }
                break;

            case STEP_DATA_CRC16:
                buffer[index++] = rx_byte;
                if (index == static_cast<uint16_t>(sizeof(FrameHeader) + sizeof(uint16_t) +
                                                   data_length + sizeof(uint16_t)))
                {
                    if (crc::verify_crc16_checksum(buffer, index) == 0)
                    {
                        ++self->status_.crc16_error_count;
                        state = STEP_HEADER_SOF;
                        break;
                    }

                    uint16_t command_id = 0;
                    std::memcpy(&command_id, &buffer[sizeof(FrameHeader)], sizeof(command_id));
                    const uint8_t* payload =
                        &buffer[sizeof(FrameHeader) + sizeof(command_id)];
                    const decode_result result =
                        decode_packet(self->packets_, command_id, payload, data_length);
                    if (result == decode_result::invalid_length)
                    {
                        ++self->status_.length_error_count;
                        state = STEP_HEADER_SOF;
                        break;
                    }

                    got_valid_frame = true;
                    self->status_.online = true;
                    ++self->status_.valid_frame_count;
                    if (result == decode_result::unknown)
                    {
                        ++self->status_.unknown_packet_count;
                    }
                    else
                    {
                        update_legacy_data(referee_data, self->packets_, command_id,
                                           self->cfg_.receive);
                    }

                    const update_info update{command_id, true, result == decode_result::known};
                    if (self->cfg_.on_update != nullptr)
                    {
                        // Runs synchronously in the referee receive thread and must not block.
                        self->cfg_.on_update(self->packets_, update, self->cfg_.update_context);
                    }
                    state = STEP_HEADER_SOF;
                }
                break;

            default:
                state = STEP_HEADER_SOF;
                break;
            }
        }

        if (got_valid_frame)
        {
            referee_data.online = true;
            ++referee_data.update_count;
        }
        msg::publish(self->referee_topic_, referee_data);

        ++reset_hurt_count;
        if (reset_hurt_count == 1000)
        {
            std::memset(&referee_data.robot_hurt, 0, sizeof(referee_data.robot_hurt));
            reset_hurt_count = 0;
        }

        tx_semaphore_put(&self->heartbeat_sem_);
        tx_thread_sleep(1);
    }
}

} // namespace referee
