#pragma once

#include <cstdint>

namespace modules {

constexpr uint16_t kRcChValueMin = 364;
constexpr uint16_t kRcChValueOffset = 1024;
constexpr uint16_t kRcChValueMax = 1684;
constexpr uint16_t kRcChOffsetMax = 660;
constexpr uint16_t kMouseOffsetMax = 32767;

constexpr uint8_t kDr16DataSize = 18;
constexpr uint8_t kVt03DataSize = 21;

struct Dr16Data {
  uint16_t ch_0 : 11;
  uint16_t ch_1 : 11;
  uint16_t ch_2 : 11;
  uint16_t ch_3 : 11;
  uint16_t s1 : 2;
  uint16_t s2 : 2;

  int16_t mouse_x : 16;
  int16_t mouse_y : 16;
  int16_t mouse_z : 16;
  uint8_t mouse_left : 8;
  uint8_t mouse_right : 8;
  struct __attribute__((packed)) {
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
  } key;
  uint16_t reserve : 16;
} __attribute__((packed));

struct Vt03Data {
  uint8_t sof_1;
  uint8_t sof_2;
  uint64_t ch_0 : 11;
  uint64_t ch_1 : 11;
  uint64_t ch_2 : 11;
  uint64_t ch_3 : 11;
  uint64_t mode_sw : 2;
  uint64_t pause : 1;
  uint64_t fn_1 : 1;
  uint64_t fn_2 : 1;
  uint64_t wheel : 11;
  uint64_t trigger : 1;

  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_left : 2;
  uint8_t mouse_right : 2;
  uint8_t mouse_middle : 2;
  struct __attribute__((packed)) {
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
  } key;
  uint16_t crc16;
} __attribute__((packed));

}  // namespace modules
