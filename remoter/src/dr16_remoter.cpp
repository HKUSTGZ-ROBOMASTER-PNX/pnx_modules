#include "dr16_remoter.hpp"

#include "remoter_common.hpp"

#include <cstring>

extern "C" {
volatile uint8_t debug_dr16_rx_bytes[18]{};
volatile uint16_t debug_dr16_ch0 = 0;
volatile uint16_t debug_dr16_ch1 = 0;
volatile uint16_t debug_dr16_ch2 = 0;
volatile uint16_t debug_dr16_ch3 = 0;
volatile uint16_t debug_dr16_s1 = 0;
volatile uint16_t debug_dr16_s2 = 0;
volatile uint32_t debug_dr16_rx_count = 0;
volatile uint32_t debug_dr16_timeout_count = 0;
volatile uint32_t debug_dr16_last_rx_size = 0;
}

namespace modules {

namespace {

constexpr size_t kDr16DmaBufferSize = 32;

uint8_t g_dr16_rx_buffer[kDr16DmaBufferSize]
    __attribute__((section(".dma_buffer"), aligned(32)));

}  // namespace

Dr16Remoter& Dr16Remoter::instance() {
  static Dr16Remoter inst;
  return inst;
}

os::Status Dr16Remoter::create_resources() {
  if (os::sem_create(&rx_sem_, "dr16_remoter_rx", 0) != os::Status::Ok) {
    return os::Status::Error;
  }

  remoter_topic_ = os::create_topic("remoter", sizeof(MsgRemoter));
  if (remoter_topic_ == nullptr) {
    return os::Status::Error;
  }

  std::memset(g_dr16_rx_buffer, 0, sizeof(g_dr16_rx_buffer));
  rx_buffer_ = g_dr16_rx_buffer;

  if (bsp::usart_init_impl(config_.uart_port, bsp::UartMode::Dma) != bsp::Status::Ok) {
    return os::Status::Error;
  }
  if (bsp::usart_start_rx_to_idle(config_.uart_port, rx_buffer_, kDr16DataSize, rx_callback,
                                  this, nullptr) != bsp::Status::Ok) {
    return os::Status::Error;
  }

  if (os::thread_create(&thread_, "dr16_remoter", thread_entry, 0, stack_, sizeof(stack_),
                        config_.thread_priority) != os::Status::Ok) {
    return os::Status::Error;
  }

  return os::Status::Ok;
}

os::Status Dr16Remoter::init(const Dr16RemoterConfig& config) {
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

void Dr16Remoter::thread_entry(uint32_t) { instance().thread_loop(); }

void Dr16Remoter::rx_callback(bsp::UartPort, uint8_t*, size_t len, void* user_data) {
  debug_dr16_last_rx_size = static_cast<uint32_t>(len);
  auto* self = static_cast<Dr16Remoter*>(user_data);
  if (self != nullptr) {
    os::sem_give(&self->rx_sem_);
  }
}

void Dr16Remoter::thread_loop() {
  MsgRemoter msg{};
  msg.offline = true;

  for (;;) {
    while (os::sem_take(&rx_sem_, static_cast<os::Wait>(config_.rx_timeout_ticks)) !=
           os::Status::Ok) {
      ++debug_dr16_timeout_count;
      msg.offline = true;
      bsp::usart_restart_rx(config_.uart_port);
      os::publish(remoter_topic_, &msg, sizeof(msg));
      os::thread_sleep(3);
      bsp::usart_start_rx_to_idle(config_.uart_port, rx_buffer_, kDr16DataSize, rx_callback,
                                  this, nullptr);
    }

    const auto& dr16 = *reinterpret_cast<const Dr16Data*>(rx_buffer_);
    ++debug_dr16_rx_count;
    for (uint8_t i = 0; i < kDr16DataSize; ++i) {
      debug_dr16_rx_bytes[i] = rx_buffer_[i];
    }
    debug_dr16_ch0 = dr16.ch_0;
    debug_dr16_ch1 = dr16.ch_1;
    debug_dr16_ch2 = dr16.ch_2;
    debug_dr16_ch3 = dr16.ch_3;
    debug_dr16_s1 = dr16.s1;
    debug_dr16_s2 = dr16.s2;

    msg.offline = false;
    msg.ctrl_sw = process_ctrl(msg.last_ctrl_sw, static_cast<CtrlState>(dr16.s2));
    msg.jump_sw = process_jump(msg.last_jump_sw, static_cast<JumpState>(dr16.s1));

    msg.right_x =
        (static_cast<float>(dr16.ch_0) - static_cast<float>(kRcChValueOffset)) / kRcChOffsetMax;
    msg.right_y =
        (static_cast<float>(dr16.ch_1) - static_cast<float>(kRcChValueOffset)) / kRcChOffsetMax;
    msg.left_x =
        (static_cast<float>(dr16.ch_2) - static_cast<float>(kRcChValueOffset)) / kRcChOffsetMax;
    msg.left_y =
        (static_cast<float>(dr16.ch_3) - static_cast<float>(kRcChValueOffset)) / kRcChOffsetMax;

    msg.mouse_x = static_cast<float>(dr16.mouse_x);
    msg.mouse_y = static_cast<float>(dr16.mouse_y);
    msg.mouse_z = static_cast<float>(dr16.mouse_z);
    msg.mouse_left = dr16.mouse_left != 0;
    msg.mouse_right = dr16.mouse_right != 0;

    std::memcpy(&msg.key, &dr16.key, sizeof(msg.key));
    os::publish(remoter_topic_, &msg, sizeof(msg));
    msg.last_ctrl_sw = msg.ctrl_sw;
    msg.last_jump_sw = msg.jump_sw;
    std::memcpy(&msg.last_key, &msg.key, sizeof(msg.last_key));
    os::thread_sleep(1);
  }
}

}  // namespace modules
