#include "ahrs.hpp"

#include "bsp_dwt.hpp"
#include "bsp_pwm.hpp"
#include "pid.hpp"

#include <cstring>
modules::MsgAhrs debug_ahrs_msg{};
volatile float debug_gyro_data_x = 0.0f;
volatile float debug_gyro_data_y = 0.0f;
volatile float debug_acc_data_z = 0.0f;

namespace modules {
namespace {

constexpr bool kDisableTemperatureControlForBringup = false;
constexpr auto kImuHeaterPwm = bsp::PwmInstance::Tim3_Ch4;


}  // namespace
// volatile float debug_imu_temp=0;
extern "C" volatile float debug_imu_temp = 0.0f;
Ahrs& Ahrs::instance() {
  static Ahrs inst;
  return inst;
}

os::Status Ahrs::create_resources() {
  if (os::sem_create(&heartbeat_sem_, "ahrs_heartbeat", 0) != os::Status::Ok) {
    return os::Status::Error;
  }

  ahrs_topic_ = os::create_topic("ahrs", sizeof(MsgAhrs));
  if (ahrs_topic_ == nullptr) {
    return os::Status::Error;
  }

  imu_thread_.entry = imu_thread_entry;
  imu_thread_.user_arg = 0;
  if (os::thread_create(&imu_thread_, "ahrs_imu", imu_thread_entry, 0, imu_stack_,
                        sizeof(imu_stack_), config_.imu_thread_priority) != os::Status::Ok) {
    return os::Status::Error;
  }

  temp_thread_.entry = temp_thread_entry;
  temp_thread_.user_arg = 0;
  if (os::thread_create(&temp_thread_, "ahrs_temp", temp_thread_entry, 0, temp_stack_,
                        sizeof(temp_stack_), config_.temp_thread_priority) != os::Status::Ok) {
    return os::Status::Error;
  }

  return os::Status::Ok;
}

os::Status Ahrs::init(const AhrsConfig& config) {
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

void Ahrs::imu_thread_entry(uint32_t) { instance().imu_thread_loop(); }

void Ahrs::temp_thread_entry(uint32_t) { instance().temp_thread_loop(); }

void Ahrs::fill_ahrs_msg(MsgAhrs& msg, const QuaternionEkf& ekf) {
  std::memcpy(msg.quaternion, ekf.q, sizeof(ekf.q));
  msg.yaw = ekf.yaw;
  msg.pitch = ekf.pitch;
  msg.roll = ekf.roll;
  msg.total_yaw = ekf.total_yaw;
  msg.gyro_r = imu_.gyro_data_.x;
  msg.gyro_p = imu_.gyro_data_.y;
  msg.gyro_y = imu_.gyro_data_.z;
  msg.accel[0] = imu_.acc_data_.x;
  msg.accel[1] = imu_.acc_data_.y;
  msg.accel[2] = imu_.acc_data_.z;
}

void Ahrs::imu_thread_loop() {
  MsgAhrs msg{};
  QuaternionEkf ekf;

  auto& self_test = imu_.self_test_;
  self_test.acc_chip_id_err = true;
  self_test.acc_data_err = true;
  self_test.gyro_chip_id_err = true;
  self_test.gyro_data_err = true;
  self_test.init_err = true;
  self_test.calibrate_err = false;
  self_test.temp_ctrl_err = false;

  imu_.config();
  imu_.verify_acc_chip_id();
  imu_.verify_gyro_chip_id();
  imu_.read_acc_temperature(&imu_.acc_data_.temperature);
  if (kDisableTemperatureControlForBringup && imu_.acc_data_.temperature < 20.0f) {
    imu_.acc_data_.temperature = 20.0f;
  }

  while (imu_.acc_data_.temperature < 20.0f) {//NOTE：原本是40，温度过低不是正常工作
    os::thread_sleep(100);
  }
  self_test.init_err = false;

  imu_.calibrate();

  uint32_t ins_count = 0;
  bsp::dwt_delta_s(&ins_count);

  for (;;) {
    if (!self_test.init_err) {
      imu_.read_acc_data(&imu_.acc_data_);
      imu_.read_gyro_data(&imu_.gyro_data_);
      if (config_.imu_offset_x != 0.0f) {
        imu_.acc_data_.x +=
            config_.imu_offset_x * imu_.gyro_data_.z * imu_.gyro_data_.z;
      }
      debug_gyro_data_x = imu_.gyro_data_.x;
      debug_gyro_data_y = imu_.gyro_data_.y;
      debug_acc_data_z = imu_.acc_data_.z;
      ekf.update_kalman(imu_.gyro_data_.x, imu_.gyro_data_.y, imu_.gyro_data_.z,
                        imu_.acc_data_.x, imu_.acc_data_.y, imu_.acc_data_.z,
                        bsp::dwt_delta_s(&ins_count));
    }

    os::sem_give(&heartbeat_sem_);
    os::sem_give(&heartbeat_sem_);
    os::sem_give(&heartbeat_sem_);

    fill_ahrs_msg(msg, ekf);
    memcpy(&debug_ahrs_msg, &msg, sizeof(debug_ahrs_msg));
    os::publish(ahrs_topic_, &msg, sizeof(msg));
    os::thread_sleep(1);
  }
}

void Ahrs::temp_thread_loop() {
  if (kDisableTemperatureControlForBringup) {
    bsp::pwm_set_duty(kImuHeaterPwm, 0.0f);
    bsp::pwm_stop(kImuHeaterPwm);
    for (;;) {
      imu_.read_acc_temperature(&imu_.acc_data_.temperature);
      os::thread_sleep(500);
    }
  }

  auto& pid = imu_.temp_pid_;
  pid.mode_ = pid::Position | pid::IntegralLimit | pid::ChangingIntegralRate |
              pid::DerivativeOnMeasurement;
  pid.kp_ = 650.0f;
  pid.ki_ = 0.06f;
  pid.kd_ = 10.0f;
  pid.max_out_ = 300.0f;
  pid.max_i_out_ = 300.0f;
  pid.scalar_a_ = 3.5f;
  pid.scalar_b_ = 0.08f;

  imu_.target_temp_ = 45.0f;
  bsp::pwm_init<bsp::PwmInstance::Tim3_Ch4>();
  bsp::pwm_start(kImuHeaterPwm);

  float tmp_last = imu_.acc_data_.temperature;
  os::thread_sleep(1000);
  imu_.read_acc_temperature(&imu_.acc_data_.temperature);
  if (tmp_last == imu_.acc_data_.temperature) {
    for (;;) {
      os::thread_sleep(1000);
      // debug_imu_temp = imu_.acc_data_.temperature;
    }
  }//现在阻塞了之后的温控线程
  // if(tmp_last == imu_.acc_data_.temperature){
  //   imu_.self_test_.temp_ctrl_err = true;
  //   bsp::pwm_set_duty(kImuHeaterPwm, 0.0f);
  // }

  uint32_t init_heating_count = 0;

  for (;;) {
    imu_.read_acc_temperature(&imu_.acc_data_.temperature);
    debug_imu_temp = imu_.acc_data_.temperature;
    if (imu_.acc_data_.temperature > 55.0f) {
      bsp::pwm_set_duty(kImuHeaterPwm, 0.0f);
    } else if (imu_.acc_data_.temperature < 45.0f) {
      if (init_heating_count < 2) {
        ++init_heating_count;
        imu_.temperature_control(imu_.target_temp_);
      } else {
        ++init_heating_count;
        bsp::pwm_set_duty(kImuHeaterPwm, 0.0f);
        if (init_heating_count == 8) {
          init_heating_count = 0;
        }
      }
    } else {
      imu_.temperature_control(imu_.target_temp_);
      //TODO：关掉温控
    }

    os::thread_sleep(125);
  }
}

}  // namespace modules
