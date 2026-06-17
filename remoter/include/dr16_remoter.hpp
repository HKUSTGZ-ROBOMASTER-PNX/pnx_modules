#pragma once

#include "bsp_usart.hpp"
#include "messages.hpp"
#include "os.hpp"
#include "remoter_types.hpp"

#include <cstddef>

namespace modules {

struct Dr16RemoterConfig {
  bsp::UartPort uart_port = bsp::UartPort::Uart5;
  uint32_t thread_priority = 2;
  uint32_t rx_timeout_ticks = 100;
};

class Dr16Remoter {
 public:
  static Dr16Remoter& instance();

  os::Status init(const Dr16RemoterConfig& config = {});
  os::Semaphore* rx_sem() { return &rx_sem_; }
  os::Topic* remoter_topic() { return remoter_topic_; }

 private:
  static void thread_entry(uint32_t arg);
  static void rx_callback(bsp::UartPort port, uint8_t* data, size_t len, void* user_data);
  void thread_loop();
  os::Status create_resources();

  Dr16RemoterConfig config_{};
  os::Thread thread_{};
  os::Semaphore rx_sem_{};
  alignas(8) uint8_t stack_[1024]{};

  os::Topic* remoter_topic_ = nullptr;
  uint8_t* rx_buffer_ = nullptr;

  bool initialized_ = false;
};

}  // namespace modules
