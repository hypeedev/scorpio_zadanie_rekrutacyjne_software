#pragma once

#include <memory>
#include <variant>
#include <cstdint>

namespace solution {
  using Motor_t = std::shared_ptr<backend_interface::Component<int8_t, uint16_t>>;

  enum MotorType { HORIZONTAL, VERTICAL };
  enum HorizontalDirection { LEFT, RIGHT };
  enum VerticalDirection { DOWN, UP };

  struct MotorState {
    Motor_t motor;
    MotorType motor_type;
    int32_t current_units = 0;
    int32_t target_units = 0;
    bool assigned_target = false;
    std::variant<HorizontalDirection, VerticalDirection> rotation_direction;
  };
}