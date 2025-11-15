// VehicleController.cpp
// Implementation of VehicleController
// Based on original code provided by RWTH Aachen ACDC exercise
// Minor cleanups, comments and initialization added for robustness.

#include "VehicleController.h"

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <algorithm>

VehicleController::VehicleController()
  : target_velocity_(0.0),
    target_steering_angle_(0.0)
{
  for (size_t i = 0; i < 5; ++i) sensor_distances_[i] = 0.0f;
}

void VehicleController::computeTargetVelocity() {
  // -- Longitudinal control: distance-based velocity scheduling --
  // Use only the LiDAR distances we have: right, right_front, front, left_front, left
  const float& right_distance = this->sensor_distances_[0];
  const float& right_front_distance = this->sensor_distances_[1];
  const float& front_distance = this->sensor_distances_[2];
  const float& left_front_distance = this->sensor_distances_[3];
  const float& left_distance = this->sensor_distances_[4];

  static float max_error = 0.0f;
  static float max_VELOCITY = 0.0f;
  static float max_rightShift = 0.0f;
  static float max_lookAhead = 0.0f;
  static float max_distance_factor = 0.0f;
  static float max_target_velocity = 0.0f;

  // Combine side offsets and look-ahead into a single error measure to penalize cornering
  float rightShift = left_distance - right_distance;
  float lookAhead = (left_front_distance - right_front_distance);
  float error = 0.80f * rightShift + 0.20f * lookAhead;

  // If an obstacle is directly in front, be conservative with lateral adjustments
  if (front_distance < 10.0f) {
    error = 0.0f;
  }

  // Distance factor: base on front distance minus absolute lateral error
  float distance_factor = 1.0f * front_distance - 0.4f * std::abs(error);

  // Transfer function: tanh to saturate gently; scale and then square to map to usable velocity
  const float FACTOR = 7.0f;
  const float FACTOR2 = 0.06f;
  const double MIN_VELOCITY = 8.50f;
  float VELOCITY = FACTOR * std::tanh(distance_factor * FACTOR2);

  // Keep some minimum velocity so the simulated car doesn't stall; square adds stronger separation between low/high speeds
  this->target_velocity_ = std::max(std::pow(VELOCITY, 2.0f), static_cast<float>(MIN_VELOCITY));

  // Update some diagnostic maxima for debugging/tuning
  if (std::abs(error) > max_error) max_error = std::abs(error);
  if (VELOCITY > max_VELOCITY) max_VELOCITY = VELOCITY;
  if (std::abs(rightShift) > 0.0f) max_rightShift += std::abs(rightShift);
  if (std::abs(lookAhead) > 0.0f) max_lookAhead += std::abs(lookAhead);
  if (this->target_velocity_ > max_target_velocity) max_target_velocity = static_cast<float>(this->target_velocity_);
}

void VehicleController::computeTargetSteeringAngle() {
  // -- Lateral control: filtered sensor reads + P/filtered-D controller --

  // Raw sensor references
  const float& raw_right_distance = this->sensor_distances_[0];
  const float& raw_right_front_distance = this->sensor_distances_[1];
  const float& front_distance = this->sensor_distances_[2];
  const float& raw_left_front_distance = this->sensor_distances_[3];
  const float& raw_left_distance = this->sensor_distances_[4];

  // Steering limiting and velocity reference
  const float max_steering_change = 1.25f; // radians per control cycle (unused but kept as design note)
  const float velocity = static_cast<float>(this->target_velocity_);

  // Persistent previous samples for spike filtering
  static float previous_right_distance = 0.0f;
  static float previous_right_front_distance = 0.0f;
  static float previous_left_front_distance = 0.0f;
  static float previous_left_distance = 0.0f;
  static float previous_steering = 0.0f;

  // Debug maxima
  static float max_error = 0.0f;
  static float max_steering_correction = 0.0f;
  static float max_rightShift = 0.0f;
  static float max_lookAhead = 0.0f;

  // Spike filtering parameters
  static float weird_deviation = 3.50f; // threshold for detecting spikes
  float weird_factor = 0.20f;          // when spike detected, attenuate new reading

  // Filter sensor spikes per-channel (prevents sudden jumps from influencing output)
  float right_distance = 0.0f;
  if ((std::abs(raw_right_distance - previous_right_distance) > weird_deviation) && (front_distance > 4.0f)) {
    right_distance = previous_right_distance;
    previous_right_distance = weird_factor * raw_right_distance;
  } else {
    right_distance = raw_right_distance;
    previous_right_distance = raw_right_distance;
  }

  float right_front_distance = 0.0f;
  if ((std::abs(raw_right_front_distance - previous_right_front_distance) > weird_deviation) && (front_distance > 4.0f)) {
    right_front_distance = previous_right_front_distance;
    previous_right_front_distance = weird_factor * raw_right_front_distance;
  } else {
    right_front_distance = raw_right_front_distance;
    previous_right_front_distance = raw_right_front_distance;
  }

  float left_front_distance = 0.0f;
  if ((std::abs(raw_left_front_distance - previous_left_front_distance) > weird_deviation) && (front_distance > 4.0f)) {
    left_front_distance = previous_left_front_distance;
    previous_left_front_distance = weird_factor * raw_left_front_distance;
  } else {
    left_front_distance = raw_left_front_distance;
    previous_left_front_distance = raw_left_front_distance;
  }

  float left_distance = 0.0f;
  if ((std::abs(raw_left_distance - previous_left_distance) > weird_deviation) && (front_distance > 4.0f)) {
    left_distance = previous_left_distance;
    previous_left_distance = weird_factor * raw_left_distance;
  } else {
    left_distance = raw_left_distance;
    previous_left_distance = raw_left_distance;
  }

  // Compute lateral error (rightShift = desired offset to the right)
  float rightShift = (left_distance - right_distance);
  float lookAhead = (left_front_distance - right_front_distance);

  // Normalize error by front distance to avoid overreaction when close to obstacles
  float error = std::tanh((0.75f * rightShift + 0.5f * lookAhead) / (3.00f * std::max(1.0f, front_distance)));

  // PID-style state with derivative filtering
  static float integral_error = 0.0f;
  static float previous_error = 0.0f;
  static float previous_time = 0.0f;
  static float d_filtered = 0.0f;

  auto now = std::chrono::steady_clock::now();
  double now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();

  // Gains (tuned empirically in original exercise)
  float Kp = 1.25f;
  float Ki = 0.0000f;
  float Kd = 0.01f;

  // Compute time delta
  float dt = 1e-3f;
  if (previous_time != 0.0f) {
    dt = std::max(1e-3f, static_cast<float>(now_sec - previous_time));
  }

  // Integral with anti-windup
  integral_error += error * dt;
  const float integral_limit = 2.2f;
  integral_error = std::clamp(integral_error, -integral_limit, integral_limit);

  // Derivative with first-order low-pass filter
  float raw_derivative = (error - previous_error) / dt;
  const float tau = 0.05f;  // filter time constant
  float alpha = dt / (tau + dt);
  d_filtered += alpha * (raw_derivative - d_filtered);

  // Compute steering correction (P + I + D-filtered)
  float steering_correction = Kp * error + Ki * integral_error + Kd * d_filtered;

  // Optional rate-limiting on steering change could be applied here (commented out originally)
  previous_steering = steering_correction;

  // If very close to obstacles and lateral offsets are small, keep straight
  if (front_distance > 4.0f && (std::abs(rightShift) < 0.2f || std::abs(lookAhead) < 0.2f)) {
    steering_correction = 0.0f;
  }

  // Clamp final steering to safe range
  this->target_steering_angle_ = std::max(std::min(0.45f, steering_correction), -0.45f);

  // Update internal state for next iteration
  previous_error = error;
  previous_time = now_sec;

  static float iterations = 0.0f;
  iterations += 1.0f;

  // Track maxima for debugging/tuning output
  if (std::abs(error) > max_error) max_error = std::abs(error);
  if (steering_correction > max_steering_correction) max_steering_correction = steering_correction;
  if (std::abs(rightShift) != 0.0f) max_rightShift += std::abs(rightShift);
  if (std::abs(lookAhead) != 0.0f) max_lookAhead += std::abs(lookAhead);

  // Debug printout (keeps the same telemetry format used in the original exercise)
  std::cout << "\n==================== Lateral Control (Simple P) ====================" << std::endl;
  std::cout << "Right Distance: " << right_distance << std::endl;
  std::cout << "Left Distance: " << left_distance << std::endl;
  std::cout << "Look Ahead: " << lookAhead << std::endl;
  std::cout << "Right Shift: " << rightShift << std::endl;
  std::cout << "Error: " << error << std::endl;
  std::cout << "Kp: " << Kp << std::endl;
  std::cout << "Ki: " << Ki << std::endl;
  std::cout << "Kd: " << Kd << std::endl;
  std::cout << "Target Steering Angle: " << this->target_steering_angle_ << std::endl;
  std::cout << "Velocity: " << velocity << std::endl;
  std::cout << "Time delta (dt): " << dt << std::endl;
  std::cout << "iterations: " << iterations << std::endl;

  std::cout << "Maximum right shift so far: " << max_rightShift << std::endl;
  std::cout << "Maximum look ahead so far: " << max_lookAhead << std::endl;
  std::cout << "Maximum error so far: " << max_error << std::endl;
  std::cout << "Maximum steering correction so far: " << max_steering_correction << std::endl;
  std::cout << "====================================================================\n" << std::endl;
}

void VehicleController::overwriteLidarDistances(const float distances[5]) {
  for (size_t i = 0; i < 5; i++) {
    this->sensor_distances_[i] = distances[i];
  }
}

void VehicleController::computeTargetValues() {
  computeTargetVelocity();
  computeTargetSteeringAngle();
}

double VehicleController::getTargetVelocity() {
  return this->target_velocity_;
}

double VehicleController::getTargetSteeringAngle() {
  return this->target_steering_angle_;
}
