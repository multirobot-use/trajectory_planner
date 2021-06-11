#include "trajectory_planner_ros.hpp"

int main(int argc, char **argv) {
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "trajectory_planner");
  ros::NodeHandle nh("~");

  TrajectoryPlannerRos TrajectoryPlannerRos(nh);

  ros::spin();
  return 0;
}