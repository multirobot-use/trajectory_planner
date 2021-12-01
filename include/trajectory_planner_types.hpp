#pragma once

#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <chrono>

namespace trajectory_planner{
struct parameters {
  float horizon_length = 40;       // number of steps
  int n_drones = 1;                // number of drones
  float step_size = 0.1;           // seconds
  float planning_rate = 1.0;       // sec
  float visualization_rate = 1.0;  // sec
  float clock_rate  = 0.01;        // sec
  float topics_rate = 0.1;         // sec
  int drone_id = 1;
  int flight_mode = 1;
  float vel_max = 5.0;
  float acc_max = 5.0;
  std::string frame = "map";
  std::string pcd_file_path = "";
};

struct state {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  float time_stamp;
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

inline boost::shared_ptr<nav_msgs::Path> vectorToPath(const std::vector<state> &initial_path){
  nav_msgs::PathPtr initial_path_ptr = boost::make_shared<nav_msgs::Path>();
  geometry_msgs::PoseStamped aux;
  for(auto state : initial_path){
    aux.pose.position.x = state.pos.x();
    aux.pose.position.y = state.pos.y();
    aux.pose.position.z = state.pos.z();
    initial_path_ptr->poses.push_back(aux);
  }
  return initial_path_ptr;
}
}
// inline std::vector<Eigen::Vector3f> stateVectorToEigenf(const std::vector<state> &_initial_path){
//   std::vector<Eigen::Vector3f> path_to_return;
//   Eigen::Vector3f aux_pose;
//   for(auto &&state : _initial_path){
//     aux_pose[0] = state.pos[0];
//     aux_pose[1] = state.pos[1];
//     aux_pose[2] = state.pos[2];
//     path_to_return.push_back(aux_pose);
//   }
//   return path_to_return;
// }
// }