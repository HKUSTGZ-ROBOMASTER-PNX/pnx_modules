#include "vt03_remoter.hpp"

#include "remoter_common.hpp"

#include <cstring>

namespace modules {

namespace {

constexpr int kLostThreshold = 100;

bool update_online(bool got, uint32_t& lost_cnt) {
  if (got) {
    lost_cnt = 0;
    return true;
  }
  ++lost_cnt;
  return lost_cnt < static_cast<uint32_t>(kLostThreshold);
}

}  // namespace

Vt03Remoter& Vt03Remoter::instance() {
  static Vt03Remoter inst;
  return inst;
}

os::Status Vt03Remoter::create_resources() {
  if (os::sem_create(&rx_sem_, "vt03_remoter_rx", 0) != os::Status::Ok) {
    return os::Status::Error;
  }

  remoter_topic_ = os::create_topic("remoter", sizeof(MsgRemoter));
  if (remoter_topic_ == nullptr) {
    return os::Status::Error;
  }

  if (bsp::usart_init_impl(config_.uart_port, bsp::UartMode::Dma) != bsp::Status::Ok) {
    return os::Status::Error;
  }
  if (bsp::usart_start_rx_to_idle(config_.uart_port, rx_buffer_, sizeof(rx_buffer_), nullptr,
                                  nullptr, &rx_sem_) != bsp::Status::Ok) {
    return os::Status::Error;
  }

  if (os::thread_create(&thread_, "vt03_remoter", thread_entry, 0, stack_, sizeof(stack_),
                        config_.thread_priority) != os::Status::Ok) {
    return os::Status::Error;
  }

  return os::Status::Ok;
}

os::Status Vt03Remoter::init(const Vt03RemoterConfig& config) {
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

void Vt03Remoter::thread_entry(uint32_t) { instance().thread_loop(); }

void Vt03Remoter::fill_vt03(MsgRemoter& raw, const Vt03Data& now) {
  raw.ctrl_sw = static_cast<CtrlState>(now.mode_sw);
  raw.right_x =
      (static_cast<float>(now.ch_0) - static_cast<float>(kRcChValueOffset)) / kRcChOffsetMax;
  raw.right_y =
      (static_cast<float>(now.ch_1) - static_cast<float>(kRcChValueOffset)) / kRcChOffsetMax;
  raw.left_x =
      (static_cast<float>(now.ch_3) - static_cast<float>(kRcChValueOffset)) / kRcChOffsetMax;
  raw.left_y =
      (static_cast<float>(now.ch_2) - static_cast<float>(kRcChValueOffset)) / kRcChOffsetMax;
  raw.mouse_x = static_cast<float>(now.mouse_x);
  raw.mouse_y = static_cast<float>(now.mouse_y);
  raw.mouse_z = static_cast<float>(now.mouse_z);
  raw.mouse_left = now.mouse_left != 0;
  raw.mouse_right = now.mouse_right != 0;
  raw.fn_1 = now.fn_1 != 0;
  raw.fn_2 = now.fn_2 != 0;
  raw.button = now.trigger != 0;
  raw.pause = now.pause != 0;
  std::memcpy(&raw.key, &now.key, sizeof(raw.key));
}

void Vt03Remoter::thread_loop() {
  MsgRemoter output{};
  MsgRemoter vt03_raw{};
  uint32_t lost_cnt = 0;
  output.offline = true;

  for (;;) {
    const auto vt03_data = *reinterpret_cast<const Vt03Data*>(rx_buffer_);
    const bool vt03_got = (os::sem_take(&rx_sem_, os::Wait::NoWait) == os::Status::Ok) &&
                          vt03_data.sof_1 == 0xA9 && vt03_data.sof_2 == 0x53;
    const bool online = update_online(vt03_got, lost_cnt);

    if (vt03_got) {
      fill_vt03(vt03_raw, vt03_data);
    }

    if (online) {
      output.ctrl_sw = process_ctrl(output.last_ctrl_sw, vt03_raw.ctrl_sw);
      output.right_x = vt03_raw.right_x;
      output.right_y = vt03_raw.right_y;
      output.left_x = vt03_raw.left_x;
      output.left_y = vt03_raw.left_y;
      output.mouse_x = vt03_raw.mouse_x;
      output.mouse_y = vt03_raw.mouse_y;
      output.mouse_z = vt03_raw.mouse_z;
      output.mouse_left = vt03_raw.mouse_left;
      output.mouse_right = vt03_raw.mouse_right;
      output.fn_1 = vt03_raw.fn_1;
      output.fn_2 = vt03_raw.fn_2;
      output.button = vt03_raw.button;
      output.pause = vt03_raw.pause;
      std::memcpy(&output.key, &vt03_raw.key, sizeof(output.key));
      output.offline = false;
    } else {
      output.offline = true;
    }

    os::publish(remoter_topic_, &output, sizeof(output));
    output.last_ctrl_sw = output.ctrl_sw;
    std::memcpy(&output.last_key, &output.key, sizeof(output.key));
    os::thread_sleep(1);
  }
}

}  // namespace modules
