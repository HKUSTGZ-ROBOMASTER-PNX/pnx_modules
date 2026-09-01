#pragma once

#include "bsp_usart.hpp"
#include "callback.hpp"
#include "referee_protocol.hpp"
#include "tx_api.h"
#include "config.hpp"

#include <type_traits>

namespace referee
{

struct robot_interaction_data
{
    RoboInteractData_t header{};
    uint16_t payload_length = 0;
    uint8_t payload[protocol::max_robot_interaction_payload]{};
};

struct packet_store
{
    GameStatus_t game_status{};
    GameRobotStatus_t game_robot_status{};
    PowerHeatData_t power_heat_data{};
    Buff_t buff{};
    RobotHurt_t robot_hurt{};
    DartInfo_t dart_info{};
    DartClientCmd_t dart_client_command{};
    robot_interaction_data robot_interaction{};
};

struct update_info
{
    uint16_t command_id = 0;
    bool got_valid_frame = false;
    bool is_known_packet = false;
};

struct communication_status
{
    bool online = false;
    uint32_t valid_frame_count = 0;
    uint32_t crc8_error_count = 0;
    uint32_t crc16_error_count = 0;
    uint32_t length_error_count = 0;
    uint32_t unknown_packet_count = 0;
};

using update_callback = core::callback<void(const packet_store&, const update_info&)>;

struct config
{
    bsp::usart::port uart_port = app::uart::referee;
    uint32_t thread_priority = 8;
    // Invoked synchronously in the referee receive thread after a valid frame is processed.
    // The callback must not block. Configuration is fixed by the first successful init().
    update_callback on_update_callback{};
};

static_assert(std::is_trivially_copyable_v<robot_interaction_data>);
static_assert(std::is_trivially_copyable_v<packet_store>);
static_assert(std::is_trivially_copyable_v<update_info>);
static_assert(std::is_trivially_copyable_v<communication_status>);

class service
{
public:
    static service& instance();

    types::status init(const config& cfg = {});
    TX_SEMAPHORE* heartbeat_sem() { return &heartbeat_sem_; }
    const packet_store& packets() const noexcept { return packets_; }
    const communication_status& status() const noexcept { return status_; }

    void push_rx(const uint8_t* data, uint16_t len);

private:
    static void referee_thread_entry(ULONG arg);
    bool create_resources();
    static void referee_rx_callback(bsp::usart::port port, const bsp::usart::rx_frame& frame);

    config cfg_{};
    TX_THREAD thread_{};
    TX_SEMAPHORE heartbeat_sem_{};
    alignas(8) uint8_t stack_[1536]{};

    RefereeRingBuffer rx_fifo_{};
    packet_store packets_{};
    communication_status status_{};
    uint8_t* rx_buffer_ = nullptr;
    bool initialized_ = false;
};

} // namespace referee
