#pragma once
#include "callback.hpp"
#include "referee_protocol.hpp"
#include "bsp_usart.hpp"

namespace referee
{
// Only wire packet types declared in this protocol have a command mapping.
template <typename T> inline constexpr uint16_t command_id = 0xffff;

template <> inline constexpr uint16_t command_id<GameStatus_t> = RefereeID::GameStatus;
template <> inline constexpr uint16_t command_id<GameResult_t> = RefereeID::GameResult;
template <> inline constexpr uint16_t command_id<RobotHP_t> = RefereeID::RobotHP;
template <> inline constexpr uint16_t command_id<EventData_t> = RefereeID::EventData;
template <> inline constexpr uint16_t command_id<RefereeWarning_t> = RefereeID::RefereeWarning;
template <> inline constexpr uint16_t command_id<DartInfo_t> = RefereeID::DartInfo;
template <> inline constexpr uint16_t command_id<GameRobotStatus_t> = RefereeID::GameRobotStatus;
template <> inline constexpr uint16_t command_id<PowerHeatData_t> = RefereeID::PowerHeatData;
template <> inline constexpr uint16_t command_id<GameRobotPos_t> = RefereeID::GameRobotPos;
template <> inline constexpr uint16_t command_id<Buff_t> = RefereeID::Buff;
template <> inline constexpr uint16_t command_id<RobotHurt_t> = RefereeID::RobotHurt;
template <> inline constexpr uint16_t command_id<ShootData_t> = RefereeID::ShootData;
template <>
inline constexpr uint16_t command_id<BulletRemaining_t> = RefereeID::ProjectileAllowance;
template <> inline constexpr uint16_t command_id<RfidStatus_t> = RefereeID::RfidStatus;
template <> inline constexpr uint16_t command_id<DartClientCmd_t> = RefereeID::DartClientCmd;
template <>
inline constexpr uint16_t command_id<GroundRobotPosition_t> = RefereeID::GroundRobotPosition;
template <> inline constexpr uint16_t command_id<RadarMarkData_t> = RefereeID::RadarMarkData;
template <> inline constexpr uint16_t command_id<SentryInfo_t> = RefereeID::SentryInfo;
template <> inline constexpr uint16_t command_id<RadarInfo_t> = RefereeID::RadarInfo;

struct robot_interaction_data
{
    RoboInteractData_t header{};
    uint16_t payload_length = 0;
    uint8_t payload[protocol::max_robot_interaction_payload]{};
};

template <>
inline constexpr uint16_t command_id<robot_interaction_data> = RefereeID::RoboInteractData;

// Latest value of one packet type. Timestamps use HAL milliseconds and may wrap.
template <typename T> struct sample
{
    T data{};
    uint32_t received_ms = 0;
    uint32_t sequence = 0;
};

// Thread-safe snapshot. False before the first valid packet; destination stays unchanged.
// Cached values survive link loss: check received_ms and link diagnostics before use.
template <typename T> bool read(sample<T>& destination);

// Called in the receive thread after caching this type. No blocking or retained references.
// Bind only the packet types that need per-frame handling; other types are still cached.
template <typename T> using receive_callback = pnxtypes::callback<void(const sample<T>&)>;

struct callbacks
{
    receive_callback<GameStatus_t> game_status{};
    receive_callback<GameResult_t> game_result{};
    receive_callback<RobotHP_t> robot_hp{};
    receive_callback<EventData_t> event_data{};
    receive_callback<RefereeWarning_t> referee_warning{};
    receive_callback<DartInfo_t> dart_info{};
    receive_callback<GameRobotStatus_t> game_robot_status{};
    receive_callback<PowerHeatData_t> power_heat_data{};
    receive_callback<GameRobotPos_t> game_robot_pos{};
    receive_callback<Buff_t> buff{};
    receive_callback<RobotHurt_t> robot_hurt{};
    receive_callback<ShootData_t> shoot_data{};
    receive_callback<BulletRemaining_t> projectile_allowance{};
    receive_callback<RfidStatus_t> rfid_status{};
    receive_callback<DartClientCmd_t> dart_client_cmd{};
    receive_callback<GroundRobotPosition_t> ground_robot_position{};
    receive_callback<RadarMarkData_t> radar_mark_data{};
    receive_callback<SentryInfo_t> sentry_info{};
    receive_callback<RadarInfo_t> radar_info{};
    receive_callback<robot_interaction_data> robo_interact_data{};
};

struct config
{
    bsp::usart::config uart;
    uint32_t thread_priority = 5;
    ULONG period_ticks = 1;
    uint32_t offline_timeout_ms = 1000;
};

struct state
{
    uint32_t crc8_error_count = 0;
    uint32_t crc16_error_count = 0;
    uint32_t length_error_count = 0;
    uint32_t frame_timeout_count = 0;
    uint32_t rx_dropped_bytes = 0;
    uint32_t sequence = 0;
    uint32_t received_ms = 0;
    uint32_t updated_ms = 0;
    bool online = false;
};

// Serialized startup; owns UART RX for firmware lifetime. Initialization failure is latched.
// Do not terminate the worker externally. Later calls return the first result, without rebinding.
pnxtypes::status init(const config& settings, const callbacks& handlers = {});
// Consistent link diagnostics; read(sample<T>&) retrieves cached packet data.
state read();
} // namespace referee
