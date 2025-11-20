#include "rp_controller/differential_drive_controller.h"

#include <iostream>

void DiffDriveController::update(const Eigen::Isometry2f& current_pose) {
  if (_done) {
    return;
  }

  std::cout << "Current pose: " << current_pose.translation().x() << " "
            << current_pose.translation().y() << std::endl;

  // TODO: Get the target pose in the world frame;
  const auto& target_pose_in_world = _waypoints[_current_waypoint];

  // TODO: Get the target pose in the robot frame;
  const auto& target_pose_in_robot = current_pose.inverse() * target_pose_in_world;

  Eigen::Vector2f delta_pos = Eigen::Vector2f::Zero();
  // TODO: Compute the delta in translation;
  delta_pos = target_pose_in_robot.translation();

  // TODO: Compute the delta in rotation (atan2);
  const auto delta_yaw = atan2(delta_pos.y(), delta_pos.x());

  // HINT: The greater the distances to the target, the greater the velocities (use the gains)
  _output_v = _k_rho * delta_pos.norm(); // TODO: Compute the linear velocity;
  _output_w = _k_w * delta_yaw; // TODO: Compute the angular velocity;

  //***************  Begin of obstacle avoidance procedure ***************//
  if (_obstacle_avoidance) {
    // HINTS:
    // - Use the laser measurements to avoid obstacles
    // - Activate the obstacle avoidance when the robot is close to an obstacle
    // - Check if there are more points on the left or on the right (the azimuth of a point in the laser scan frame is atan2(y, x))
    // - If there are more points on the right, turn left and vice versa
    // - The rotational velocity should be proportional to the angle to the closest obstacle (0 when the obstacle is at +/-3.14 radians)

    // Note: The above HINTS are taken from our implementation, you can come up with your own solution
    
    // Identifying the closest obstacle
    float closest_obstacle_distance = std::numeric_limits<float>::max();
     float closest_obstacle_angle = 0.0f;
    int left_count = 0, right_count = 0;

    for (const auto& point : _measurements) {
      float distance = point.norm();   // Distance to the obstacle
      float azimuth = atan2(point.y(), point.x());  // Angle of the point in the laser frame

      if (distance < closest_obstacle_distance) {
        closest_obstacle_distance = distance;
        closest_obstacle_angle = azimuth;
      }
    }

    for (const auto& point : _measurements) {
      float azimuth = atan2(point.y(), point.x());  // Angle of the point in the laser frame
      if (azimuth > closest_obstacle_angle) {
        left_count++;
      } else {
        right_count++;
      }
    }

    // Activate obstacle avoidance if too close
    if (closest_obstacle_distance < _tolerance*0.5f) {
      _output_v = 0.1f;  // decelerate

      // Turn left or right based on obstacle distribution
      if (left_count > right_count) {
        _output_w = -_k_w * closest_obstacle_angle * 5.0f;  // Turn right
      } else {
        _output_w = _k_w * closest_obstacle_angle * 5.0f;   // Turn left
      }
    }
  }

  //***************  End of obstacle avoidance procedure ***************//

  // Check if the robot is close to the target waypoint
  if (delta_pos.norm() < _tolerance) {
    // Move to the next waypoint
    _current_waypoint++;
    std::cerr << "New waypoint" << std::endl;
    if (_current_waypoint >= _waypoints.size()) {
      std::cerr << "Completed path" << std::endl;
      _done = true;
      _output_v = 0.0f;
      _output_w = 0.0f;
    }
  }

  return;
}
