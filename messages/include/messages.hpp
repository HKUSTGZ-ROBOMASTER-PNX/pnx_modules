#pragma once

#include "referee_protocol.hpp"

#include <cstdint>

namespace modules {

enum class CtrlState : uint8_t {
  Spin = 1,
  Relax = 2,
  Normal = 3,
  R2N = 4,
  N2R = 5,
  N2S = 6,
  S2N = 7,
};

enum class JumpState : uint8_t {
  Jump = 1,
  None = 2,
  Prepared = 3,
  N2P = 4,
  P2J = 5,
  J2P = 6,
  P2N = 7,
};

struct RemoterKey {
  uint16_t W : 1;
  uint16_t S : 1;
  uint16_t A : 1;
  uint16_t D : 1;
  uint16_t SHIFT : 1;
  uint16_t CTRL : 1;
  uint16_t Q : 1;
  uint16_t E : 1;
  uint16_t R : 1;
  uint16_t F : 1;
  uint16_t G : 1;
  uint16_t Z : 1;
  uint16_t X : 1;
  uint16_t C : 1;
  uint16_t V : 1;
  uint16_t B : 1;
} __attribute__((packed));

struct MsgRemoter {
  CtrlState ctrl_sw = CtrlState::Relax;
  JumpState jump_sw = JumpState::None;
  CtrlState last_ctrl_sw = CtrlState::Relax;
  JumpState last_jump_sw = JumpState::None;
  float left_x = 0.0f;
  float left_y = 0.0f;
  float right_x = 0.0f;
  float right_y = 0.0f;
  float mouse_x = 0.0f;
  float mouse_y = 0.0f;
  float mouse_z = 0.0f;
  bool mouse_left = false;
  bool mouse_right = false;
  RemoterKey key{};
  RemoterKey last_key{};
  bool offline = true;
  bool fn_1 = false;
  bool fn_2 = false;
  bool button = false;
  bool pause = false;
};

struct MsgAhrs {
  float quaternion[4]{};
  float roll = 0.0f;       // rad
  float pitch = 0.0f;      // rad
  float yaw = 0.0f;        // rad
  float total_yaw = 0.0f;  // rad
  float gyro_r = 0.0f;     // rad/s
  float gyro_p = 0.0f;     // rad/s
  float gyro_y = 0.0f;     // rad/s
  float accel[3]{};
};

struct MsgReferee {
  GameRobotStatus_t robot_status{};
  RobotHurt_t robot_hurt{};
  uint16_t heat_now = 0;
  uint16_t power_buffer = 0;
  uint8_t remained_energy = 0;
};

struct MsgWheelLegged {
  float N = 0.0f;
  float x = 0.0f;
  float v = 0.0f;
  float len = 0.0f;
  bool flying = false;
  bool normal = false;
  bool recovered = false;
  bool stairing = false;
  bool spinning = false;
  float lalpha = 0.0f;
  float ralpha = 0.0f;
  float pitch = 0.0f;
};

using MsgPendulum = MsgWheelLegged;

struct MsgMotor {
  int16_t yaw_current = 0;
  int8_t trigger_speed = 0;
};

enum class BulletFreq : uint8_t {
  Low = 0,
  High,
};

struct MsgCmd {
  float x = 0.0f;
  float v = 0.0f;
  float len = 0.0f;
  float dlen = 0.0f;
  float yaw = 0.0f;
  float dyaw = 0.0f;
  float dpitch = 0.0f;
  float roll = 0.0f;
  float tri_spd = 0.0f;
  uint16_t yawmotor_cur = 0;
  bool iffly = false;
  bool move = false;
  bool spin = false;
  bool gostair = false;
  bool inair = false;
  bool ifjump = false;
  bool shoot = false;
  bool fire = false;
  bool auto_aim = false;
  bool auto_shoot = false;
  bool shoot_check = false;
  bool aim_rune = false;
  bool ifturn = false;
  bool ifreverse = false;
  bool ifgimbalonly = false;
  BulletFreq bullet_freq = BulletFreq::High;
};

struct MsgChassisUi {
  float relative_angle = 0.0f;
  float dist = 0.0f;
  bool jumping = false;
};

struct MsgVisionRx {
  uint8_t header = 0;
  uint8_t tracking : 1;
  uint8_t fire : 1;
  uint8_t id : 4;
  uint8_t reserved : 2;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float pitch_acc = 0.0f;
  float pitch_vel = 0.0f;
  float yaw_acc = 0.0f;
  float yaw_vel = 0.0f;
  float project_x = 0.0f;
  float project_y = 0.0f;
  float distance = 0.0f;
  uint16_t checksum = 0;
} __attribute__((packed));

struct MsgVisionTx {
  uint8_t header = 0;
  uint8_t detect_color : 1;
  uint8_t reset_tracker : 1;
  uint8_t set_target : 4;
  uint8_t reserved : 2;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
  float q4 = 0.0f;
  float gyro_yaw = 0.0f;
  float gyro_pitch = 0.0f;
  uint16_t checksum = 0;
} __attribute__((packed));

}  // namespace modules
