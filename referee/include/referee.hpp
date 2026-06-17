#pragma once

#include "messages.hpp"
#include "os.hpp"
#include "bsp_usart.hpp"
#include "referee_protocol.hpp"

namespace modules {

struct RefereeConfig {
  bsp::UartPort uart_port = bsp::UartPort::Usart1;
  uint32_t thread_priority = 8;
};

class Referee {
 public:
  static Referee& instance();

  os::Status init(const RefereeConfig& config = {});
  os::Semaphore* heartbeat_sem() { return &heartbeat_sem_; }
  os::Topic* referee_topic() { return referee_topic_; }

  void push_rx(const uint8_t* data, uint16_t len);

 private:
  static void thread_entry(uint32_t arg);
  void thread_loop();
  os::Status create_resources();
  static void uart_rx_callback(bsp::UartPort port, uint8_t* data, size_t len, void* user);

  RefereeConfig config_{};
  os::Thread thread_{};
  os::Semaphore heartbeat_sem_{};
  alignas(8) uint8_t stack_[2048]{};

  os::Topic* referee_topic_ = nullptr;
  RefereeRingBuffer rx_fifo_{};
  uint8_t* rx_buffer_ = nullptr;

  bool initialized_ = false;
};

}  // namespace modules
