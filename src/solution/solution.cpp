#include "tester.hpp"
#include "../../include/backend/motor.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <variant>

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

typedef std::shared_ptr<backend_interface::Component<int8_t, uint16_t>> Motor_t;

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

typedef std::vector<PointState> PointQueue_t;

constexpr int ENCODER_UNITS_DISTANCE_TOLERANCE = 3;
constexpr double ENCODER_UNITS_PER_DEGREE = 4096.0 / 360.0;

double degrees_to_encoder_units(const double degrees) {
  return degrees * ENCODER_UNITS_PER_DEGREE;
}

double calculate_yaw_from_point(const Point& point) {
  return atan2(-point.y, point.x) * (180 / M_PI);
}

double calculate_pitch_from_point(const Point& point) {
  const double distance = sqrt(point.x * point.x + point.y * point.y);
  return atan2(point.z, distance) * (180 / M_PI);
}

uint16_t calculate_target_encoder_units(const Point& point, const MotorType motor_type) {
  const double degrees = (motor_type == HORIZONTAL)
    ? calculate_yaw_from_point(point)
    : calculate_pitch_from_point(point);
  return static_cast<uint16_t>(degrees_to_encoder_units(degrees)) % 4096;
}

void update_motor_speed(MotorState& motor_state, const Point& point) {
  motor_state.target_units = calculate_target_encoder_units(point, motor_state.motor_type);
  if (motor_state.target_units != motor_state.current_units) {
    motor_state.assigned_target = true;

    auto delta = motor_state.target_units - motor_state.current_units;
    if (delta > 2048) delta -= 4096;
    else if (delta < -2048) delta += 4096;

    // Slightly adjust target to avoid undershooting
    if (delta > 0) {
      motor_state.target_units += ENCODER_UNITS_DISTANCE_TOLERANCE / 3;
    } else if (delta < 0) {
      motor_state.target_units -= ENCODER_UNITS_DISTANCE_TOLERANCE / 3;
    }

    motor_state.rotation_direction = (motor_state.motor_type == HORIZONTAL)
      ? std::variant<HorizontalDirection, VerticalDirection>(delta > 0 ? RIGHT : LEFT)
      : std::variant<HorizontalDirection, VerticalDirection>(delta > 0 ? UP : DOWN);
    const auto horizontal_speed = static_cast<signed char>(delta > 0 ? 127 : -128);
    motor_state.motor->send_data(horizontal_speed);
  }
}

void print_queued_points(const PointQueue_t& points) {
  std::cout << "Queued points:\n";
  for (const auto &point_state : points) {
    const auto &point = point_state.point;
    std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ") - H done: " << point_state.horizontal_motor_done << ", V done: " << point_state.vertical_motor_done << "\n";
  }
}

void process_point(const Point& point, const bool preempt, MotorState& horizontal_motor_state, MotorState& vertical_motor_state, const PointQueue_t& queued_points) {
  if (!preempt) {
    print_queued_points(queued_points);
  } else {
    std::cout << "Processing point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
  }

  if (preempt || queued_points.size() == 1) {
    update_motor_speed(horizontal_motor_state, point);
    update_motor_speed(vertical_motor_state, point);
  }
}

bool encoder_data_callback(const uint16_t& data, MotorState& motor_state, const bool preempt, PointQueue_t& queued_points) {
  if (!motor_state.assigned_target) return false;

  std::cout << (motor_state.motor_type == HORIZONTAL ? "Horizontal" : "Vertical") << " motor data: " << static_cast<int>(data) << ", target units: " << motor_state.target_units << "\n";
  motor_state.current_units = data;

  const auto distance = abs(motor_state.current_units - motor_state.target_units);
  // TODO: adjust motor speed dynamically based on distance to not under/overshoot
  if (distance <= ENCODER_UNITS_DISTANCE_TOLERANCE) {
    motor_state.motor->send_data(0);
    motor_state.assigned_target = false;
    std::cout << "Reached " << (motor_state.motor_type == HORIZONTAL ? "horizontal" : "vertical") << " target\n";

    if (preempt) {
      return false;
    }

    auto &last_point = queued_points.front();

    if (motor_state.motor_type == HORIZONTAL) {
      last_point.horizontal_motor_done = true;
    } else if (motor_state.motor_type == VERTICAL) {
      last_point.vertical_motor_done = true;
    }

    if (!last_point.horizontal_motor_done || !last_point.vertical_motor_done) {
      return false;
    }

    queued_points.erase(queued_points.begin());

    print_queued_points(queued_points);

    return true;
  }

  if (distance <= 10) {
    const auto is_positive_direction = (motor_state.motor_type == HORIZONTAL)
                                         ? std::get<HorizontalDirection>(motor_state.rotation_direction) == RIGHT
                                         : std::get<VerticalDirection>(motor_state.rotation_direction) == UP;
    const auto speed = static_cast<signed char>(is_positive_direction ? 60 : -60);
    motor_state.motor->send_data(speed);
  }

  return false;
}

int solver(const std::shared_ptr<backend_interface::Tester> tester, const bool preempt) {
  PointQueue_t queued_points;

  std::cout << (preempt ? "Preempt" : "Queue") << '\n';

  const auto commands = tester->get_commands();

  MotorState horizontal_motor_state{ .motor = tester->get_motor_1(), .motor_type = HORIZONTAL };
  MotorState vertical_motor_state{ .motor = tester->get_motor_2(), .motor_type = VERTICAL };

  horizontal_motor_state.motor->add_data_callback([&](const uint16_t& data) {
    const auto process_next_point = encoder_data_callback(data, horizontal_motor_state, preempt, queued_points);
    if (process_next_point && !queued_points.empty()) {
      const auto next_point = queued_points.front().point;
      process_point(next_point, preempt, horizontal_motor_state, vertical_motor_state, queued_points);
    }
  });

  vertical_motor_state.motor->add_data_callback([&](const uint16_t& data) {
    const auto process_next_point = encoder_data_callback(data, vertical_motor_state, preempt, queued_points);
    if (process_next_point && !queued_points.empty()) {
      const auto next_point = queued_points.front().point;
      process_point(next_point, preempt, horizontal_motor_state, vertical_motor_state, queued_points);
    }
  });

  commands->add_data_callback([&](const Point& point) {
    if (!preempt) {
      queued_points.push_back({ point, false, false });
    }

    process_point(point, preempt, horizontal_motor_state, vertical_motor_state, queued_points);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(3600 * 1000));

  return 0;
}
