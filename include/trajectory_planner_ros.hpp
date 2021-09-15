#pragma once
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <trajectory_planner/WaypointSrv.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
#include <trajectory_planner.hpp>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>

namespace trajectory_planner {

enum Colors { RED = 0, BLUE = 2, YELLOW = 3 };

//!  TrajectoryPlannerRos class.
/*!
  A class to handle the functionality of the mission planner on ROS.
*/

class TrajectoryPlannerRos {
 public:
  //! TrajectoryPlannerRos constructor
  TrajectoryPlannerRos(ros::NodeHandle _nh);

  //! TrajectoryPlannerRos destructor
  ~TrajectoryPlannerRos();


 private:
  // Declarations
  ros::NodeHandle nh_;
  parameters param_;
  std::unique_ptr<TrajectoryPlanner> trajectory_planner_ptr_;
  ros::Timer planTimer_;
  ros::Timer pubVis_;

  std::vector<geometry_msgs::Point> points_;

  // Subscriptions
  std::map<int, ros::Subscriber> cur_pose_sub_;
  std::map<int, ros::Subscriber> cur_vel_sub_;
  std::map<int, ros::Subscriber> solved_trajectories_sub_;
  
  ros::Subscriber pcd_sub_;

  // Publishers
  ros::Publisher points_pub_;
  ros::Publisher pub_path_;
  ros::Publisher pub_ref_path_;
  ros::Publisher tracking_pub_;
  ros::Publisher tracking_pub_trajectory_;
  ros::Publisher corridor_pub_;
  ros::Publisher pub_point_cloud_;
  // Services
  ros::ServiceServer service_activate_planner;
  ros::ServiceServer service_waypoint;
  ros::ServiceServer clear_waypoints;

  // queues
  ros::CallbackQueue pcd_queue_;
  ros::AsyncSpinner async_spinner_{2, &pcd_queue_};

  tf2_ros::Buffer tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;    


  //! Callback prototypes

  /**
   * @brief Callback for the solved trajectories from others
   *
   * @param msg trajectory
   * @param id drone id
   */
  void solvedTrajCallback(const nav_msgs::Path::ConstPtr &msg, int id);

  /*! \brief Callback for the activate planner service.
   *   \param req structure of the request message
   *   \param res structure of the response message
   *   \return planning_activated
   */
  bool activationPlannerServiceCallback(std_srvs::SetBool::Request &req,
                                        std_srvs::SetBool::Response &res);

  /*! \brief Callback for the waypoint service. It adds a desired waypoint
   *   \param req new desired waypoint
   *   \param res success
   *   \return success
   */
  bool addWaypointServiceCallback(trajectory_planner::WaypointSrv::Request &req,
                                  trajectory_planner::WaypointSrv::Response &res);

  /*! \brief Callback for the clean waypoints service. It cleans all the
   * waypoints queued \param req request \param res success \return success
   */
  bool clearWaypointsServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res);

  /*! \brief Callback for timer that publishes rviz markers
   * \param TimerEvent structure passed to callback invoked by ros::Timer
   */
  void pubVisCB(const ros::TimerEvent &e);
  /*! \brief Callback for plan timer.
   *   \param TimerEvent structure passed to callback invoked by ros::Timer
   */
  void replanCB(const ros::TimerEvent &e);

  /*! \brief Callback for drone's pose
   *   \param msg drone's pose, geometry_msgs/PoseStamped
   *   \param id  identifier of the drone
   **/
  void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);

  /*! \brief Callback for drone's velocity
   *   \param msg drone's velocity, geometry_msgs/TwistStamped
   *   \param id  identifier of the drone
   **/
  void uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg,
                           int id);

  void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /*! \brief function to publish last solved trajectory
   */
  void publishPath(const ros::Publisher &pub_path,
                   const std::vector<state> &trajectory);

  /*! \brief function to publish trajectory to the solver
   */
  void publishTrajectoryJoint(const ros::Publisher &pub_path,
                              const std::vector<state> &trajectory);

  /*! \brief function to publish points on RViz
   *   \param pub_points publisher
   *   \param _points vector of points to publish
   *   \param color color of points
   */
  void publishPoints(const ros::Publisher &pub_points,
                     const std::vector<geometry_msgs::Point> &_points,
                     Colors color);

  /*! \brief function to set the marker's color on RViz
   *   \param marker marker
   *   \param color color of the marker
   */
  void setMarkerColor(visualization_msgs::Marker &marker, const Colors &color);
};

}
