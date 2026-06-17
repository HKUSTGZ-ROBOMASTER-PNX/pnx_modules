#pragma once

#include "bsp_usart.hpp"
#include "messages.hpp"
#include "os.hpp"
#include "remoter_types.hpp"

namespace modules {

struct Vt03RemoterConfig {
  bsp::UartPort uart_port = bsp::UartPort::Usart1;
  uint32_t thread_priority = 2;
};

class Vt03Remoter {
 public:
  static Vt03Remoter& instance();

  os::Status init(const Vt03RemoterConfig& config = {});
  os::Semaphore* rx_sem() { return &rx_sem_; }
  os::Topic* remoter_topic() { return remoter_topic_; }

 private:
  static void thread_entry(uint32_t arg);
  void thread_loop();
  os::Status create_resources();
  void fill_vt03(MsgRemoter& raw, const Vt03Data& now);

  Vt03RemoterConfig config_{};
  os::Thread thread_{};
  os::Semaphore rx_sem_{};
  alignas(8) uint8_t stack_[1024]{};

  os::Topic* remoter_topic_ = nullptr;
  alignas(4) uint8_t rx_buffer_[kVt03DataSize]{};

  bool initialized_ = false;
};

}  // namespace modules
