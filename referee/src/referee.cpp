#include "referee.hpp"

#include "crc.hpp"

#include <cstddef>
#include <cstring>

namespace modules {

namespace {

constexpr size_t kRefereeRxBufferSize = 256;

uint8_t g_referee_rx_buffer[kRefereeRxBufferSize]
    __attribute__((section(".dma_buffer"), aligned(32)));

}  // namespace

Referee& Referee::instance() {
  static Referee inst;
  return inst;
}

void Referee::push_rx(const uint8_t* data, uint16_t len) {
  if (data != nullptr && len > 0) {
    rx_fifo_.push(data, len);
  }
}

void Referee::uart_rx_callback(bsp::UartPort, uint8_t* data, size_t len, void*) {
  if (data != nullptr && len > 0) {
    instance().push_rx(data, static_cast<uint16_t>(len));
  }
}

os::Status Referee::create_resources() {
  if (os::sem_create(&heartbeat_sem_, "referee_heartbeat", 0) != os::Status::Ok) {
    return os::Status::Error;
  }

  referee_topic_ = os::create_topic("referee", sizeof(MsgReferee));
  if (referee_topic_ == nullptr) {
    return os::Status::Error;
  }

  std::memset(g_referee_rx_buffer, 0, sizeof(g_referee_rx_buffer));
  rx_buffer_ = g_referee_rx_buffer;

  if (bsp::usart_init_impl(config_.uart_port, bsp::UartMode::Dma) != bsp::Status::Ok) {
    return os::Status::Error;
  }
  if (bsp::usart_start_rx_to_idle(config_.uart_port, rx_buffer_, kRefereeRxBufferSize,
                                  uart_rx_callback, nullptr, nullptr) != bsp::Status::Ok) {
    return os::Status::Error;
  }

  if (os::thread_create(&thread_, "referee", thread_entry, 0, stack_, sizeof(stack_),
                        config_.thread_priority) != os::Status::Ok) {
    return os::Status::Error;
  }

  return os::Status::Ok;
}

os::Status Referee::init(const RefereeConfig& config) {
  if (initialized_) {
    return os::Status::Ok;
  }
  config_ = config;
  const os::Status status = create_resources();
  if (status == os::Status::Ok) {
    initialized_ = true;
  }
  return status;
}

void Referee::thread_entry(uint32_t) { instance().thread_loop(); }

void Referee::thread_loop() {
  MsgReferee referee_data{};
  uint16_t reset_hurt_cnt = 0;

  for (;;) {
    static uint8_t rx_byte = 0;
    static UnpackState state = STEP_HEADER_SOF;
    static uint8_t buffer[256];
    static uint16_t data_len = 0;
    static uint16_t index = 0;

    GameStatus_t game_status{};
    GameResult_t game_result{};
    RobotHP_t robot_hp{};
    EventData_t event_data{};
    RefereeWarning_t referee_warning{};
    DartInfo_t dart_info{};
    GameRobotStatus_t game_robot_status{};
    PowerHeatData_t power_heat_data{};
    GameRobotPos_t game_robot_pos{};
    Buff_t buff{};
    RobotHurt_t robot_hurt{};
    ShootData_t shoot_data{};
    RfidStatus_t rfid_status{};
    DartClientCmd_t dart_client_cmd{};
    RoboInteractData_t robo_interact_data{};

    while (rx_fifo_.pop(rx_byte)) {
      switch (state) {
        case STEP_HEADER_SOF:
          if (rx_byte == 0xA5) {
            index = 0;
            buffer[index++] = rx_byte;
            state = STEP_LENGTH_SEQ;
          }
          break;

        case STEP_LENGTH_SEQ:
          buffer[index++] = rx_byte;
          if (index == 5) {
            if (crc::verify_crc8_checksum(buffer, 5) != 0) {
              data_len = static_cast<uint16_t>(buffer[1] | (buffer[2] << 8));
              if (data_len > 200) {
                state = STEP_HEADER_SOF;
              } else {
                state = STEP_DATA_CRC16;
              }
            } else {
              state = STEP_HEADER_SOF;
            }
          }
          break;

        case STEP_DATA_CRC16:
          buffer[index++] = rx_byte;
          if (index == static_cast<uint16_t>(5 + 2 + data_len + 2)) {
            if (crc::verify_crc16_checksum(buffer, index) != 0) {
              uint8_t* msg_ptr = &buffer[5];
              uint16_t cmd_id = 0;
              std::memcpy(&cmd_id, msg_ptr, sizeof(uint16_t));
              msg_ptr += sizeof(uint16_t);
              switch (cmd_id) {
                case RefereeID::GameStatus:
                  std::memcpy(&game_status, msg_ptr, sizeof(game_status));
                  break;
                case RefereeID::GameResult:
                  std::memcpy(&game_result, msg_ptr, sizeof(game_result));
                  break;
                case RefereeID::RobotHP:
                  std::memcpy(&robot_hp, msg_ptr, sizeof(robot_hp));
                  break;
                case RefereeID::EventData:
                  std::memcpy(&event_data, msg_ptr, sizeof(event_data));
                  break;
                case RefereeID::RefereeWarning:
                  std::memcpy(&referee_warning, msg_ptr, sizeof(referee_warning));
                  break;
                case RefereeID::DartInfo:
                  std::memcpy(&dart_info, msg_ptr, sizeof(dart_info));
                  break;
                case RefereeID::GameRobotPos:
                  std::memcpy(&game_robot_pos, msg_ptr, sizeof(game_robot_pos));
                  break;
                case RefereeID::Buff:
                  std::memcpy(&buff, msg_ptr, sizeof(buff));
                  break;
                case RefereeID::RobotHurt:
                  std::memcpy(&robot_hurt, msg_ptr, sizeof(robot_hurt));
                  break;
                case RefereeID::ShootData:
                  std::memcpy(&shoot_data, msg_ptr, sizeof(shoot_data));
                  break;
                case RefereeID::RfidStatus:
                  std::memcpy(&rfid_status, msg_ptr, sizeof(rfid_status));
                  break;
                case RefereeID::DartClientCmd:
                  std::memcpy(&dart_client_cmd, msg_ptr, sizeof(dart_client_cmd));
                  break;
                case RefereeID::RoboInteractData:
                  std::memcpy(&robo_interact_data, msg_ptr, sizeof(robo_interact_data));
                  break;
                case RefereeID::PowerHeatData:
                  std::memcpy(&power_heat_data, msg_ptr, sizeof(power_heat_data));
                  break;
                case RefereeID::GameRobotStatus:
                  std::memcpy(&game_robot_status, msg_ptr, sizeof(game_robot_status));
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

    os::publish(referee_topic_, &referee_data, sizeof(referee_data));

    ++reset_hurt_cnt;
    if (reset_hurt_cnt == 1000) {
      std::memset(&robot_hurt, 0, sizeof(RobotHurt_t));
      reset_hurt_cnt = 0;
    }

    os::sem_give(&heartbeat_sem_);
    os::thread_sleep(1);
  }
}

}  // namespace modules
