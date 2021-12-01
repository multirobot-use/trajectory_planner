#pragma once

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <acado/acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <map>
#include "log.h"
#include "trajectory_planner_types.hpp"
#include <safe_corridor_generator/safe_corridor_generator.h>
#include <mutex>


namespace trajectory_planner {

//! TrajectoryPlanner class
/*!
 * Abstract base class for mission planner.
 */

enum PlannerStatus { FIRST_PLAN = 0, REPLANNED = 2 };
enum MissionStatus { GO_TO = 0, MISSION_ZONE = 1 };

class TrajectoryPlanner {
 public:
  std::unique_ptr<Logger> logger_;
  std::vector<state> reference_traj;
  std::map<int, state> states_;
  float start_plan_time_;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_ptr_;
  std::shared_ptr<safe_corridor_generator::SafeCorridorGenerator>  safe_corridor_generator_;
  /**
   * @brief default constructor
   * 
   */
  TrajectoryPlanner();
  /**
   * @brief constructor of the class
   */
  TrajectoryPlanner(const parameters _param);

  /**
   * @brief destructor of the class
   */
  virtual ~TrajectoryPlanner();

  /**
   * @brief Update map using safe corridor generator library
   * 
   */
  void updateMap(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _pcd_input);

  /**
   * @brief pushes back a new goal on the vector of goal points to reach
   *
   * @param _goal new waypoint to add
   */
  void appendGoal(const state &_goal) {
    goals_.push_back(std::move(_goal));
  }

  /**
   * @brief clears all the waypoints on the queue
   */
  void clearGoals() { goals_.clear(); }

  /**
   * @brief gives the remaining goals to reach
   *
   * @return std::vector<state> goals remaining to reach
   */
  std::vector<state> getGoals() { return goals_; }

  /**
   * @brief gives the reference trajectory to follow
   *
   * @return std::vector<state> reference trajectory to follow
   */
  std::vector<state> getReferenceTrajectory() { return reference_traj; }

  /**
   * @brief gives the last reference trajectory done
   *
   * @return std::vector<state> last reference trajectory done
   */
  std::vector<state> getLastTrajectory() {
    return solved_trajectories_[param_.drone_id];
  }


  /**
   * @brief gives the status of the planner
   *
   * @return value of the PlannerStatus
   */
  int getStatus() { return planner_state_; }

  /**
   * @brief executes the planner of the drone
   */
  virtual void plan();

  /**
   * @brief sets the current time
   */
  void setCurrentTime(float time_) {current_time_ = time_;}

  /**
   * @brief sets the solved trajectories for each drone
   *
   * @param solved_trajectory trajectory solved for each drone
   * @param _drone_id drone
   */
  void setSolvedTrajectories(const std::vector<state> &solved_trajectory,
                             int _drone_id) {
    solved_trajectories_[_drone_id] = solved_trajectory;
  }

 protected:
  const parameters param_;
  ACADO::Grid my_grid_;
  std::vector<state> goals_;
  bool init = true;
  std::map<int, std::vector<state>> solved_trajectories_;
  Eigen::Vector3d init_point_;
  int planner_state_ = PlannerStatus::FIRST_PLAN;
  float current_time_;

  // std::mutex mtx_jps_map_;
  // std::mutex mtx_leader_traj_;

  /**
   * @brief Calculate a path from one point to another at cte velocity
   *
   * @param initial point
   * @param final point
   * @return std::vector<state> path
   */
  std::vector<state> pathFromPointToAnother(const Eigen::Vector3d &initial,
                                            const Eigen::Vector3d &final);

  /**
   * @brief Function that add the desired orientation to the trajectory
   * 
   * @param traj solved trajectory
   */
  virtual void initialOrientation(std::vector<state> &traj) {}

  /**
   * @brief check formation poses
   *
   * @return true if all poses are received
   * @return false
   */
  bool hasPose() { return (states_.size() == param_.n_drones); }

  /**
   * @brief check if there is a solver trajectories for each drone
   *
   * @return true if all the drones have a trajectory to follow
   * @return false if any of the drones does not have a trajectory to follow
   */
  bool hasSolvedTrajectories() {
    return (solved_trajectories_.size() == param_.n_drones);
  }

  /**
   * @brief check if there is still a goal to reach
   *
   * @return true if there is still a goal to reach
   * @return false if there is no goals
   */
  bool hasGoal() { return !goals_.empty(); }

  /**
   * @brief check if there drone is close to the waypoint to reach according to
   * REACH_TOL value
   *
   * @param point current point
   * @param waypoint waypoint to reach
   * @return true if is close enough
   * @return false if is not close enough
   */
  bool waypointReached(const state &point, const state &waypoint) {
    return ((point.pos - waypoint.pos).norm() < REACH_TOL);
  }

 private:

  std::chrono::time_point<std::chrono::steady_clock> start_time_cycle_;
  const float REACH_TOL = 2;  //! tolerance to reach waypoints

  /**
   * @brief returns an initial straight trajectory for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  virtual std::vector<state> initialTrajectory(const state &_initial_pose) {
    return pathFromPointToAnother(_initial_pose.pos, goals_[0].pos);
  }

  /**
   * @brief check if the initial trajectory given is the optimal trajectory
   *
   * @param initial_trajectory initial trajectory
   * @return true if is optimal
   * @return false if is not optimal
   */
  bool optimalTrajectory(const std::vector<state> &initial_trajectory);

  /**
   * @brief gives the next waypoint to reach
   *
   * @return the next waypoint to reach
   */
  state nextGoal() { return goals_[0]; }

  /**
   * @brief virtual function that makes the following checks
   *
   * @return true if all checks are passed
   * @return false if any ot the check is not passed
   */
  virtual bool checks(); 

  /**
   * @brief gives an initial orientation according to a trajectory given
   *
   * @param traj trajectory
   */
  // virtual void initialOrientation(std::vector<state> &traj);

  /**
   * @brief gives an optimal orientation according to a trajectory given
   *
   * @param traj_to_optimize trajectory to optimize
   */
  void optimalOrientation(const std::vector<state> &traj_to_optimize);


  /**
   * @brief returns the value of the closest point according to a given
   * trajectory (vector of points)
   *
   * @param initial_trajectory initial trajectory (vector of states)
   * @param point point
   * @return the value of the closer point of the initial_trajectory to the
   * point
   */
  int closestPoint(const std::vector<state> &initial_trajectory,
                   const state point);
  void polyhedronsToACADO(ACADO::OCP &_ocp, const vec_E<Polyhedron<3>> &_vector_of_polyhedrons, const std::vector<Eigen::Vector3d> &path_free, ACADO::DifferentialState &_px, ACADO::DifferentialState &_py, ACADO::DifferentialState &_pz);

  void lastPointOccupied(std::vector<state> &_initial_trajectory);
};

}
