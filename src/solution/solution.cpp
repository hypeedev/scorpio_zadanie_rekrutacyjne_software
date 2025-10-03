#include "tester.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <variant>
#include "../../include/backend/motor.hpp"

enum HorizontalDirection {
  LEFT = 0,
  RIGHT = 1
};

enum VerticalDirection {
  UP = 0,
  DOWN = 1
};

typedef std::shared_ptr<backend_interface::Component<int8_t, uint16_t>> Motor_t;

struct MotorState {
  Motor_t motor;
  uint16_t target_units;
  uint16_t current_units;
  std::variant<HorizontalDirection, VerticalDirection> rotation_direction;
  double target_rotation_angle;
};

constexpr int ENCODER_UNITS_DISTANCE_TOLERANCE = 3;
constexpr double ENCODER_UNITS_PER_DEGREE = 4096.0 / 360.0;

double degrees_to_encoder_units(const double degrees) {
  return degrees * ENCODER_UNITS_PER_DEGREE;
}

double calculate_yaw_from_point(const MotorState &motor, const Point &point) {
  return atan2(-point.y, point.x) * (180 / M_PI);
}

double calculate_pitch_from_point(const MotorState &motor, const Point &point) {
  const double distance = sqrt(point.x * point.x + point.y * point.y);
  return atan2(point.z, distance) * (180 / M_PI);
}

uint16_t calculate_target_horizontal_encoder_units(const MotorState &motor, const Point &point) {
  const double degrees = calculate_yaw_from_point(motor, point);
  return static_cast<uint16_t>(degrees_to_encoder_units(degrees)) % 4096;
}

uint16_t calculate_target_vertical_encoder_units(const MotorState &motor, const Point &point) {
  const double degrees = calculate_pitch_from_point(motor, point);
  return static_cast<uint16_t>(degrees_to_encoder_units(degrees)) % 4096;
}

int solver(const std::shared_ptr<backend_interface::Tester> tester, const bool preempt) {
  bool command_received = false;

  std::cout << (preempt ? "Preempt" : "Queue") << '\n';

  const auto commands = tester->get_commands();

  MotorState horizontal_motor_state{ .motor = tester->get_motor_1() };
  MotorState vertical_motor_state{ .motor = tester->get_motor_2() };

  horizontal_motor_state.motor->add_data_callback([&command_received, &horizontal_motor_state](const uint16_t& data) {
    if (!command_received) return;

    std::cout << "Motor 1 data: " << static_cast<int>(data) << ", horizontal target units: " << horizontal_motor_state.target_units << "\n";
    horizontal_motor_state.current_units = data;

    const auto distance = abs(horizontal_motor_state.current_units - horizontal_motor_state.target_units);
    // TODO: adjust motor speed dynamically based on distance to not overshoot
    if (distance <= ENCODER_UNITS_DISTANCE_TOLERANCE) {
      horizontal_motor_state.motor->send_data(0);
    } else if (distance <= 10) {
      const auto rotation_direction = std::get<HorizontalDirection>(horizontal_motor_state.rotation_direction);
      const auto speed = static_cast<signed char>((rotation_direction == RIGHT) ? 60 : -60);
      horizontal_motor_state.motor->send_data(speed);
    }
  });

  vertical_motor_state.motor->add_data_callback([&command_received, &vertical_motor_state](const uint16_t& data) {
    if (!command_received) return;

    std::cout << "Motor 2 data: " << static_cast<int>(data) << ", vertical target units: " << vertical_motor_state.target_units << "\n";
    vertical_motor_state.current_units = data;

    const auto distance = abs(vertical_motor_state.current_units - vertical_motor_state.target_units);
    // TODO: adjust motor speed dynamically based on distance to not overshoot
    if (distance <= ENCODER_UNITS_DISTANCE_TOLERANCE) {
      vertical_motor_state.motor->send_data(0);
    } else if (distance <= 10) {
      const auto rotation_direction = std::get<VerticalDirection>(vertical_motor_state.rotation_direction);
      const auto speed = static_cast<signed char>((rotation_direction == UP) ? 60 : -60);
      vertical_motor_state.motor->send_data(speed);
    }
  });

  commands->add_data_callback([&command_received, &horizontal_motor_state, &vertical_motor_state](const Point& point) {
    command_received = true;

    std::cout << "Command point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";

    horizontal_motor_state.target_units = calculate_target_horizontal_encoder_units(horizontal_motor_state, point);
    if (horizontal_motor_state.target_units != horizontal_motor_state.current_units) {
      auto delta = horizontal_motor_state.target_units - horizontal_motor_state.current_units;
      if (delta > 2048) delta -= 4096;
      else if (delta < -2048) delta += 4096;

      horizontal_motor_state.rotation_direction = delta > 0 ? RIGHT : LEFT;
      const auto horizontal_speed = static_cast<signed char>(delta > 0 ? 127 : -128);
      horizontal_motor_state.motor->send_data(horizontal_speed);
    }

    vertical_motor_state.target_units = calculate_target_vertical_encoder_units(vertical_motor_state, point);
    if (vertical_motor_state.target_units != vertical_motor_state.current_units) {
      auto delta = vertical_motor_state.target_units - vertical_motor_state.current_units;
      if (delta > 2048) delta -= 4096;
      else if (delta < -2048) delta += 4096;

      vertical_motor_state.rotation_direction = delta > 0 ? UP : DOWN;
      const auto vertical_speed = static_cast<signed char>(delta > 0 ? 127 : -128);
      vertical_motor_state.motor->send_data(vertical_speed);
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(3600 * 1000));

  return 0;
}
