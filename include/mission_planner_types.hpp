#pragma once

#include "ros/ros.h"

struct parameters {
  float horizon_length = 0;        // number of steps
  int n_drones = 1;                // number of drones
  float step_size = 0.1;           // seconds
  float planning_rate = 0.0;       // sec
  float visualization_rate = 0.0;  // sec
  int drone_id = 0;
  float vel_max = 0.0;
  float acc_max = 0.0;
  std::string frame;
  float inspection_dist = 1;  // meters
  int leader_id = 1;
  float inc_distance = 0.1;
  float inc_angle = 0.01;
};

struct state {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation;
};

inline Eigen::Quaterniond eulerToQuat(const double roll, const double pitch,
                                      const double yaw) {
  return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

inline Eigen::Vector3d quatToEuler(Eigen::Quaterniond q) {
  return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

template <typename T>
inline bool safeGetParam(ros::NodeHandle &_nh, std::string const &_param_name,
                         T &_param_value) {
  if (!_nh.getParam(_param_name, _param_value)) {
    ROS_ERROR("Failed to find parameter: %s",
              _nh.resolveName(_param_name, true).c_str());
    exit(1);
  }
  return true;
}

inline bool trajectoryHasNan(std::vector<state> &trajectory) {
  for (const auto &pose : trajectory) {
    if (std::isnan(pose.pos(0)) || std::isinf(pose.pos(0)) ||
        std::isnan(pose.pos(1)) || std::isinf(pose.pos(1)) ||
        std::isnan(pose.pos(2)) || std::isinf(pose.pos(2)) ||
        std::isnan(pose.vel(0)) || std::isinf(pose.vel(0)) ||
        std::isnan(pose.vel(1)) || std::isinf(pose.vel(1)) ||
        std::isnan(pose.vel(2)) || std::isinf(pose.vel(2))) {
      return true;
    }
  }
  return false;
}

inline Eigen::Vector3d rotateEig(const Eigen::Vector3d &eigen_to_rotate,
                                 const float angle) {
  Eigen::Quaterniond rotation = eulerToQuat(0, 0, angle);
  Eigen::Matrix3d rotMat = rotation.toRotationMatrix();
  return rotMat * eigen_to_rotate;
}