#pragma once

#include "bsp_usart.hpp"
#include "msg.hpp"
#include "referee_protocol.hpp"
#include "tx_api.h"
#include "config.hpp"

namespace referee
{

struct receive_config
{
    bool game_robot_status = true;
    bool power_heat = true;
    bool buff = true;
    bool robot_hurt = true;
};

struct config
{
    bsp::usart::port uart_port = app::uart::referee;
    uint32_t thread_priority = 8;
    receive_config receive{};
};

struct data
{
    bool online = false;
    uint32_t update_count = 0;
    GameRobotStatus_t robot_status{};
    uint16_t heat_now = 0;
    uint16_t power_buffer = 0;
    uint8_t remained_energy = 0;
    RobotHurt_t robot_hurt{};
};

class service
{
public:
    static service& instance();

    bool init(const config& cfg = {});
    TX_SEMAPHORE* heartbeat_sem() { return &heartbeat_sem_; }

    void push_rx(const uint8_t* data, uint16_t len);

private:
    static void referee_thread_entry(ULONG arg);
    bool create_resources();
    static void referee_rx_callback(bsp::usart::port port, const bsp::usart::rx_frame& frame,
                                    void* user);

    config cfg_{};
    TX_THREAD thread_{};
    TX_SEMAPHORE heartbeat_sem_{};
    alignas(8) uint8_t stack_[1536]{};

    RefereeRingBuffer rx_fifo_{};
    msg::topic* referee_topic_ = nullptr;
    uint8_t* rx_buffer_ = nullptr;
    bool initialized_ = false;
};

} // namespace referee
