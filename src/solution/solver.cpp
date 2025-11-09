#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include "solver.hpp"
#include "tester.hpp"

namespace solution {
  static constexpr int ENCODER_UNITS_DISTANCE_TOLERANCE = 5;

  Solver::Solver(std::shared_ptr<backend_interface::Tester> tester, const bool preempt)
    : tester_(std::move(tester)), preempt_(preempt),
      horizontal_motor_state_{ .motor = tester_->get_motor_1(), .motor_type = HORIZONTAL },
      vertical_motor_state_{ .motor = tester_->get_motor_2(), .motor_type = VERTICAL } {}

  int Solver::start() {
    std::cout << (preempt_ ? "Preempt" : "Queue") << '\n';

    horizontal_motor_state_.motor->add_data_callback([this](const uint16_t& data) {
      const auto process_next_point = encoder_data_callback(data, horizontal_motor_state_);
      if (process_next_point && !queued_points_.empty()) {
        process_point(queued_points_.front().point);
      }
    });

    vertical_motor_state_.motor->add_data_callback([this](const uint16_t& data) {
      const auto process_next_point = encoder_data_callback(data, vertical_motor_state_);
      if (process_next_point && !queued_points_.empty()) {
        process_point(queued_points_.front().point);
      }
    });

    tester_->get_commands()->add_data_callback([this](const Point& point) {
      if (!preempt_) {
        queued_points_.push_back({ point });
      }
      process_point(point);
    });

    std::this_thread::sleep_for(std::chrono::hours(1));
    return 0;
  }

  void Solver::update_motor_speed(MotorState& motor_state, const Point& point) {
    motor_state.target_units = calculate_target_encoder_units(point, motor_state.motor_type);
    if (motor_state.target_units == motor_state.current_units) return;

    motor_state.assigned_target = true;
    auto delta = motor_state.target_units - motor_state.current_units;
    if (delta > 2048) delta -= 4096;
    else if (delta < -2048) delta += 4096;

    if (delta > 0) motor_state.target_units += ENCODER_UNITS_DISTANCE_TOLERANCE / 3;
    else if (delta < 0) motor_state.target_units -= ENCODER_UNITS_DISTANCE_TOLERANCE / 3;

    if (motor_state.motor_type == HORIZONTAL) {
      motor_state.rotation_direction = std::variant<HorizontalDirection, VerticalDirection>(delta > 0 ? RIGHT : LEFT);
    } else {
      motor_state.rotation_direction = std::variant<HorizontalDirection, VerticalDirection>(delta > 0 ? UP : DOWN);
    }
    motor_state.motor->send_data(delta > 0 ? 127 : -128);
  }

  bool Solver::encoder_data_callback(const uint16_t& data, MotorState& motor_state) {
    if (!motor_state.assigned_target) return false;

    std::cout << (motor_state.motor_type == HORIZONTAL ? "Horizontal" : "Vertical") << " motor data: " << static_cast<int>(data) << ", target units: " << motor_state.target_units << "\n";
    motor_state.current_units = data;

    const auto distance = std::abs(motor_state.current_units - motor_state.target_units);
    if (distance <= ENCODER_UNITS_DISTANCE_TOLERANCE) {
      motor_state.motor->send_data(0);
      motor_state.assigned_target = false;
      std::cout << "Reached " << (motor_state.motor_type == HORIZONTAL ? "horizontal" : "vertical") << " target\n";
      if (preempt_) return false;

      if (queued_points_.empty()) return false;
      auto& last_point = queued_points_.front();
      if (motor_state.motor_type == HORIZONTAL) last_point.horizontal_motor_done = true;
      else last_point.vertical_motor_done = true;

      if (!last_point.horizontal_motor_done || !last_point.vertical_motor_done) return false;

      queued_points_.erase(queued_points_.begin());
      return true;
    }

    if (distance <= 10) {
      const auto is_positive_direction = (motor_state.motor_type == HORIZONTAL)
        ? std::get<HorizontalDirection>(motor_state.rotation_direction) == RIGHT
        : std::get<VerticalDirection>(motor_state.rotation_direction) == UP;
      motor_state.motor->send_data(is_positive_direction ? 60 : -60);
    }

    return false;
  }

  void Solver::process_point(const Point& point) {
    if (!preempt_) {
      std::cout << "Queued points:\n";
      for (const auto& qp : queued_points_) {
        std::cout << "\t(" << qp.point.x << ", " << qp.point.y << ", " << qp.point.z << ") - H done: " << qp.horizontal_motor_done << ", V done: " << qp.vertical_motor_done << "\n";
      }
    } else {
      std::cout << "Processing point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    if (preempt_ || queued_points_.size() == 1) {
      update_motor_speed(horizontal_motor_state_, point);
      update_motor_speed(vertical_motor_state_, point);
    }
  }

  double Solver::degrees_to_encoder_units(const double degrees) {
    return degrees * ENCODER_UNITS_PER_DEGREE;
  }

  double Solver::calculate_yaw_from_point(const Point& point) {
    return atan2(-point.y, point.x) * (180 / M_PI);
  }

  double Solver::calculate_pitch_from_point(const Point& point) {
    const double distance = std::hypot(point.x, point.y);
    return atan2(point.z, distance) * (180 / M_PI);
  }

  uint16_t Solver::calculate_target_encoder_units(const Point& point, const MotorType motor_type) {
    const double degrees = (motor_type == HORIZONTAL)
      ? calculate_yaw_from_point(point)
      : calculate_pitch_from_point(point);
    return static_cast<uint16_t>(degrees_to_encoder_units(degrees)) % 4096;
  }
}
