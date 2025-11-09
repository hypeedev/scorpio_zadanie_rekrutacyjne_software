#pragma once

#include <memory>

#include "../backend/motor.hpp"
#include "point_queue.hpp"
#include "motor_state.hpp"

namespace solution {
  class Solver {
  public:
    Solver(std::shared_ptr<backend_interface::Tester> tester, bool preempt);

    int start();

  private:
    std::shared_ptr<backend_interface::Tester> tester_;
    bool preempt_;
    PointQueue_t queued_points_;
    MotorState horizontal_motor_state_;
    MotorState vertical_motor_state_;

    static constexpr double ENCODER_UNITS_PER_DEGREE = 4096.0 / 360.0;

    static void update_motor_speed(MotorState& motor_state, const Point& point);
    bool encoder_data_callback(const uint16_t& data, MotorState& motor_state);
    void process_point(const Point& point);

    static double degrees_to_encoder_units(double degrees);
    static double calculate_yaw_from_point(const Point& point);
    static double calculate_pitch_from_point(const Point& point);
    static uint16_t calculate_target_encoder_units(const Point& point, MotorType motor_type);
  };
}