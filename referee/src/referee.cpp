#include "referee.hpp"

#include "crc.hpp"
#include "memory.h"

#include <cstring>

namespace referee
{

namespace
{

constexpr size_t rx_buffer_size = 256;
uint8_t rx_buffer_storage[rx_buffer_size] RAM_D1_BSS{};

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
                         reinterpret_cast<ULONG>(this), stack_,
                         sizeof(stack_), cfg_.thread_priority, cfg_.thread_priority,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
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
    uint16_t reset_hurt_cnt = 0;
    GameRobotStatus_t game_robot_status{};
    PowerHeatData_t power_heat_data{};
    Buff_t buff{};
    RobotHurt_t robot_hurt{};

    for (;;)
    {
        static uint8_t rx_byte = 0;
        static UnpackState state = STEP_HEADER_SOF;
        static uint8_t buffer[256];
        static uint16_t data_len = 0;
        static uint16_t index = 0;
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
                if (index == 5)
                {
                    if (crc::verify_crc8_checksum(buffer, 5) != 0)
                    {
                        data_len = static_cast<uint16_t>(buffer[1] | (buffer[2] << 8));
                        state = (data_len > 200) ? STEP_HEADER_SOF : STEP_DATA_CRC16;
                    }
                    else
                    {
                        state = STEP_HEADER_SOF;
                    }
                }
                break;

            case STEP_DATA_CRC16:
                buffer[index++] = rx_byte;
                if (index == static_cast<uint16_t>(5 + 2 + data_len + 2))
                {
                    if (crc::verify_crc16_checksum(buffer, index) != 0)
                    {
                        got_valid_frame = true;
                        uint8_t* msg_ptr = &buffer[5];
                        uint16_t cmd_id = 0;
                        std::memcpy(&cmd_id, msg_ptr, sizeof(uint16_t));
                        msg_ptr += sizeof(uint16_t);
                        switch (cmd_id)
                        {
                        case RefereeID::PowerHeatData:
                            if (!self->cfg_.receive.power_heat)
                            {
                                break;
                            }
                            std::memcpy(&power_heat_data, msg_ptr, sizeof(power_heat_data));
                            break;
                        case RefereeID::GameRobotStatus:
                            if (!self->cfg_.receive.game_robot_status)
                            {
                                break;
                            }
                            std::memcpy(&game_robot_status, msg_ptr, sizeof(game_robot_status));
                            break;
                        case RefereeID::Buff:
                            if (!self->cfg_.receive.buff)
                            {
                                break;
                            }
                            std::memcpy(&buff, msg_ptr, sizeof(buff));
                            break;
                        case RefereeID::RobotHurt:
                            if (!self->cfg_.receive.robot_hurt)
                            {
                                break;
                            }
                            std::memcpy(&robot_hurt, msg_ptr, sizeof(robot_hurt));
                            break;
                        default:
                            break;
                        }
                    }
                    state = STEP_HEADER_SOF;
                }
                break;

            default:
                state = STEP_HEADER_SOF;
                break;
            }
        }

        referee_data.robot_status = game_robot_status;
        referee_data.heat_now = power_heat_data.shoot_id1_17mm_cooling_heat;
        referee_data.power_buffer = power_heat_data.chassis_power_buffer;
        referee_data.remained_energy = buff.remained_energy;
        referee_data.robot_hurt = robot_hurt;
        if (got_valid_frame)
        {
            referee_data.online = true;
            ++referee_data.update_count;
        }

        msg::publish(self->referee_topic_, referee_data);

        ++reset_hurt_cnt;
        if (reset_hurt_cnt == 1000)
        {
            std::memset(&robot_hurt, 0, sizeof(RobotHurt_t));
            reset_hurt_cnt = 0;
        }

        tx_semaphore_put(&self->heartbeat_sem_);
        tx_thread_sleep(1);
    }
}

} // namespace referee
