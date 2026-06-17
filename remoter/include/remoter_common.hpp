#pragma once

#include "messages.hpp"

namespace modules {

inline CtrlState process_ctrl(CtrlState last, CtrlState now) {
  if (last == CtrlState::Relax && now == CtrlState::Normal) {
    return CtrlState::R2N;
  }
  if (last == CtrlState::Normal && now == CtrlState::Relax) {
    return CtrlState::N2R;
  }
  if (last == CtrlState::Normal && now == CtrlState::Spin) {
    return CtrlState::N2S;
  }
  if (last == CtrlState::Spin && now == CtrlState::Normal) {
    return CtrlState::S2N;
  }
  return now;
}

inline JumpState process_jump(JumpState last, JumpState now) {
  if (last == JumpState::Prepared && now == JumpState::Jump) {
    return JumpState::N2P;
  }
  if (last == JumpState::Jump && now == JumpState::Prepared) {
    return JumpState::P2N;
  }
  if (last == JumpState::Prepared && now == JumpState::None) {
    return JumpState::P2J;
  }
  if (last == JumpState::None && now == JumpState::Prepared) {
    return JumpState::J2P;
  }
  return now;
}

}  // namespace modules
