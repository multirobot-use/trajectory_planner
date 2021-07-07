#include "trajectory_planner_ros.hpp"

int main(int argc, char **argv) {
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "trajectory_planner");
  ros::NodeHandle nh("~");

  trajectory_planner::TrajectoryPlannerRos TrajectoryPlannerRos(nh);

  ros::AsyncSpinner async_spinner{2, &TrajectoryPlannerRos.pcd_queue_};

  async_spinner.start();

  ros::spin();
  return 0;
}