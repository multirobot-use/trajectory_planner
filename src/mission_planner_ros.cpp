#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh)
    : nh_(_nh) {
  // ros params
  safeGetParam(nh_, "horizon_length", param_.horizon_length);
  safeGetParam(nh_, "n_drones", param_.n_drones);
  safeGetParam(nh_, "step_size", param_.step_size);
  safeGetParam(nh_, "planning_rate", param_.planning_rate);
  safeGetParam(nh_, "drone_id", param_.drone_id);
  safeGetParam(nh_, "vel_max", param_.vel_max);
  safeGetParam(nh_, "acc_max", param_.acc_max);
  safeGetParam(nh_, "frame", param_.frame);
  safeGetParam(nh_, "drone_id", param_.drone_id);

  mission_planner_ptr_ = std::make_unique<MissionPlanner>(param_);


  // Subscribers
  for (int drone = 1; drone <= param_.n_drones; drone++) {
    cur_pose_sub_[drone] = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/drone_" + std::to_string(drone) + "/ual/pose", 1,
        std::bind(&MissionPlannerRos::uavPoseCallback, this,
                  std::placeholders::_1, drone));

    cur_vel_sub_[drone] = nh_.subscribe<geometry_msgs::TwistStamped>(
        "/drone_" + std::to_string(drone) + "/ual/velocity", 1,
        std::bind(&MissionPlannerRos::uavVelocityCallback, this,
                  std::placeholders::_1, drone));

    if (drone != param_.drone_id) {
      solved_trajectories_sub_[drone] = nh_.subscribe<nav_msgs::Path>(
          "/drone_" + std::to_string(drone) +
              "/mission_planner_ros/solved_traj",
          1,
          std::bind(&MissionPlannerRos::solvedTrajCallback, this,
                    std::placeholders::_1, drone));
    }
  }
  // create timer
  planTimer_ = nh_.createTimer(ros::Duration(param_.planning_rate),
                               &MissionPlannerRos::replanCB, this);
  planTimer_.stop();

  // publishers
  pub_path_ = nh_.advertise<nav_msgs::Path>("solved_traj", 1);
  pub_ref_path_ = nh_.advertise<nav_msgs::Path>("ref_traj", 1);
  tracking_pub_ = nh_.advertise<nav_msgs::Path>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/upat_follower/follower/trajectory_to_follow",
      1);
  tracking_pub_trajectory_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/trajectory_follower_node/trajectory_to_follow",
      1);

  // Services
  service_activate_planner = nh_.advertiseService(
      "activate_planner", &MissionPlannerRos::activationPlannerServiceCallback,
      this);
  service_waypoint = nh_.advertiseService(
      "add_waypoint", &MissionPlannerRos::addWaypointServiceCallback, this);
  clear_waypoints = nh_.advertiseService(
      "clear_waypoints", &MissionPlannerRos::clearWaypointsServiceCallback,
      this);
}

MissionPlannerRos::~MissionPlannerRos() {}

// Callbacks
bool MissionPlannerRos::activationPlannerServiceCallback(
    std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("[%s]: Activation planner service called.",
           ros::this_node::getName().c_str());

  res.success = true;

  if (req.data == false) {
    ROS_INFO("[%s]: Planning deactivated.", ros::this_node::getName().c_str());
    res.message = "Planning deactivated.";
    planTimer_.stop();
  } else {
    ROS_INFO("[%s]: Planning activated.", ros::this_node::getName().c_str());
    res.message = "Planning activated.";
    planTimer_.start();
  }
  return true;
}

bool MissionPlannerRos::addWaypointServiceCallback(
    trajectory_planner::WaypointSrv::Request &req,
    trajectory_planner::WaypointSrv::Response &res) {
  ROS_INFO("[%s]: Add waypoint service called.",
           ros::this_node::getName().c_str());
  geometry_msgs::Point point;
  point.x = req.waypoint.pose.pose.position.x;
  point.y = req.waypoint.pose.pose.position.y;
  point.z = req.waypoint.pose.pose.position.z;
  points_.push_back(std::move(point));

  state state_req;

  state_req.pos(0) = req.waypoint.pose.pose.position.x;
  state_req.pos(1) = req.waypoint.pose.pose.position.y;
  state_req.pos(2) = req.waypoint.pose.pose.position.z;
  state_req.vel[0] = req.waypoint.twist.twist.linear.x;
  state_req.vel[1] = req.waypoint.twist.twist.linear.y;
  state_req.vel[2] = req.waypoint.twist.twist.linear.z;

  mission_planner_ptr_->appendGoal(state_req);

  res.success = true;
}

bool MissionPlannerRos::clearWaypointsServiceCallback(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("[%s]: Clear waypoints service called.",
           ros::this_node::getName().c_str());

  mission_planner_ptr_->clearGoals();
}


void MissionPlannerRos::replanCB(const ros::TimerEvent &e) {
  if (mission_planner_ptr_->getStatus() != PlannerStatus::FIRST_PLAN) {
    publishTrajectoryJoint(tracking_pub_trajectory_,
                           mission_planner_ptr_->getLastTrajectory());
    publishPath(pub_path_, mission_planner_ptr_->getLastTrajectory());
    publishPath(pub_ref_path_, mission_planner_ptr_->getReferenceTrajectory());
  }
  mission_planner_ptr_->plan();
}

void MissionPlannerRos::pubVisCB(const ros::TimerEvent &e) {
  // publish commanded waypoint
  publishPoints(points_pub_, points_, Colors::RED);

  // publish transformed waypoints
  std::vector<state> goals = mission_planner_ptr_->getGoals();

  geometry_msgs::Point point;
  std::vector<geometry_msgs::Point> points;

  for (auto const &goal : goals) {
    point.x = goal.pos(0);
    point.y = goal.pos(1);
    point.z = goal.pos(2);
    points.push_back(point);
  }
}


void MissionPlannerRos::solvedTrajCallback(const nav_msgs::Path::ConstPtr &msg,
                                           int id) {
  state aux_state;
  std::vector<state> path;
  for (auto pose : msg->poses) {
    aux_state.pos(0) = pose.pose.position.x;
    aux_state.pos(1) = pose.pose.position.y;
    aux_state.pos(2) = pose.pose.position.z;
    path.push_back(aux_state);
  }
  mission_planner_ptr_->setSolvedTrajectories(path, id);
}
void MissionPlannerRos::uavPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg, int id) {
  mission_planner_ptr_->states_[id].pos[0] = msg->pose.position.x;
  mission_planner_ptr_->states_[id].pos[1] = msg->pose.position.y;
  mission_planner_ptr_->states_[id].pos[2] = msg->pose.position.z;
}

void MissionPlannerRos::uavVelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg, int id) {
  mission_planner_ptr_->states_[id].vel[0] = msg->twist.linear.x;
  mission_planner_ptr_->states_[id].vel[1] = msg->twist.linear.y;
  mission_planner_ptr_->states_[id].vel[2] = msg->twist.linear.z;
}

void MissionPlannerRos::setMarkerColor(visualization_msgs::Marker &marker,
                                       const Colors &color) {
  switch (color) {
    case Colors::RED:
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      break;
    case Colors::BLUE:
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      break;
    case Colors::YELLOW:
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      break;
    default:
      break;
  }
}

void MissionPlannerRos::publishPoints(
    const ros::Publisher &pub_points,
    const std::vector<geometry_msgs::Point> &_points, Colors color) {
  visualization_msgs::Marker points;

  // markers config
  points.header.frame_id = param_.frame;
  points.header.stamp = ros::Time::now();
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  setMarkerColor(points, color);
  points.color.g = 1.0f;
  points.color.a = 1.0;

  points.points = _points;
  pub_points.publish(points);
}

void MissionPlannerRos::publishPath(const ros::Publisher &pub_path,
                                    const std::vector<state> &trajectory) {
  nav_msgs::Path path_to_publish;
  geometry_msgs::PoseStamped aux_pose;
  path_to_publish.header.frame_id = param_.frame;
  path_to_publish.header.stamp = ros::Time::now();
  for (const auto &state : trajectory) {
    aux_pose.pose.position.x = state.pos(0);
    aux_pose.pose.position.y = state.pos(1);
    aux_pose.pose.position.z = state.pos(2);
    aux_pose.pose.orientation.x = state.orientation.x();
    aux_pose.pose.orientation.y = state.orientation.y();
    aux_pose.pose.orientation.z = state.orientation.z();
    aux_pose.pose.orientation.w = state.orientation.w();
    path_to_publish.poses.push_back(aux_pose);
  }
  try {
    pub_path.publish(path_to_publish);
  } catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'",
              pub_path.getTopic().c_str());
  }
}

void MissionPlannerRos::publishTrajectoryJoint(
    const ros::Publisher &pub_path, const std::vector<state> &trajectory) {
  trajectory_msgs::JointTrajectory trajectory_to_follow;
  trajectory_msgs::JointTrajectoryPoint point_to_follow;
  for (auto &point : trajectory) {
    point_to_follow.positions.clear();
    point_to_follow.velocities.clear();
    point_to_follow.positions.push_back(point.pos(0));
    point_to_follow.positions.push_back(point.pos(1));
    point_to_follow.positions.push_back(point.pos(2));
    point_to_follow.positions.push_back(point.orientation.x());
    point_to_follow.positions.push_back(point.orientation.y());
    point_to_follow.positions.push_back(point.orientation.z());
    point_to_follow.positions.push_back(point.orientation.w());

    point_to_follow.velocities.push_back(point.vel(0));
    point_to_follow.velocities.push_back(point.vel(1));
    point_to_follow.velocities.push_back(point.vel(2));
    trajectory_to_follow.points.push_back(point_to_follow);
  }
  try {
    pub_path.publish(trajectory_to_follow);
  } catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'",
              pub_path.getTopic().c_str());
  }
}
