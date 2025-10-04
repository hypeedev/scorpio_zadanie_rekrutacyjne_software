#pragma once

#include <cstdint>
#include <memory>
#include <variant>
#include <cmath>

#include "component.hpp"
#include "tester.hpp"

enum HorizontalDirection {
  LEFT,
  RIGHT
};

enum VerticalDirection {
  UP,
  DOWN
};

enum MotorType {
  HORIZONTAL,
  VERTICAL
};

using Motor_t = std::shared_ptr<backend_interface::Component<int8_t, uint16_t>>;

struct MotorState {
  Motor_t motor;
  MotorType motor_type;
  uint16_t target_units;
  uint16_t current_units;
  std::variant<HorizontalDirection, VerticalDirection> rotation_direction;
  double target_rotation_angle;
  bool assigned_target;
};

struct PointState {
  Point point;
  bool horizontal_motor_done;
  bool vertical_motor_done;
};

using PointQueue_t = std::vector<PointState>;

constexpr int ENCODER_UNITS_DISTANCE_TOLERANCE = 3;
constexpr double ENCODER_UNITS_PER_DEGREE = 4096.0 / 360.0;

inline double degrees_to_encoder_units(const double degrees) {
  return degrees * ENCODER_UNITS_PER_DEGREE;
}

inline double calculate_yaw_from_point(const Point& point) {
  return atan2(-point.y, point.x) * (180 / M_PI);
}

inline double calculate_pitch_from_point(const Point& point) {
  const double distance = std::hypot(point.x, point.y);
  return atan2(point.z, distance) * (180 / M_PI);
}

inline uint16_t calculate_target_encoder_units(const Point& point, const MotorType motor_type) {
  const double degrees = (motor_type == HORIZONTAL)
    ? calculate_yaw_from_point(point)
    : calculate_pitch_from_point(point);
  return static_cast<uint16_t>(degrees_to_encoder_units(degrees)) % 4096;
}