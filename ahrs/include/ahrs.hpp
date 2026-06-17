#pragma once

#include "os.hpp"
#include "bmi088.hpp"
#include "quaternion_ekf.hpp"
#include "messages.hpp"

namespace modules {

struct AhrsConfig {
  float imu_offset_x = 0.0f;
  uint32_t imu_thread_priority = 3;
  uint32_t temp_thread_priority = 4;
};

class Ahrs {
 public:
  static Ahrs& instance();

  os::Status init(const AhrsConfig& config = {});
  os::Semaphore* heartbeat_sem() { return &heartbeat_sem_; }
  os::Topic* ahrs_topic() { return ahrs_topic_; }
  devices::Bmi088& imu() { return imu_; }

 private:
  static void imu_thread_entry(uint32_t arg);
  static void temp_thread_entry(uint32_t arg);

  void imu_thread_loop();
  void temp_thread_loop();
  os::Status create_resources();

  void fill_ahrs_msg(MsgAhrs& msg, const QuaternionEkf& ekf);

  AhrsConfig config_{};
  devices::Bmi088 imu_{};

  os::Thread imu_thread_{};
  os::Thread temp_thread_{};
  os::Semaphore heartbeat_sem_{};

  alignas(8) uint8_t imu_stack_[4096]{};
  alignas(8) uint8_t temp_stack_[1024]{};

  os::Topic* ahrs_topic_ = nullptr;
  bool initialized_ = false;
};

}  // namespace modules
