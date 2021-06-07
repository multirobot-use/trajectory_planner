#pragma once

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <acado/acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <map>
#include "log.h"
#include "mission_planner_types.hpp"

//! MissionPlanner class
/*!
 * Abstract base class for mission planner. It cannot be instantiated
 */

enum PlannerStatus { FIRST_PLAN = 0, REPLANNED = 2 };
enum MissionStatus { GO_TO = 0, MISSION_ZONE = 1 };

class MissionPlanner {
 public:
  std::unique_ptr<Logger> logger_;
  std::vector<state> reference_traj;
  std::map<int, state> states_;

  /**
   * @brief constructor of the class
   */
  MissionPlanner(const parameters _param);

  /**
   * @brief destructor of the class
   */
  virtual ~MissionPlanner();

  /**
   * @brief pushes back a new goal on the vector of goal points to reach
   *
   * @param _goal new waypoint to add
   */
  virtual void appendGoal(const state &_goal) {
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
  void plan();

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
  const ACADO::Grid my_grid_;
  std::vector<state> goals_;
  bool init = true;
  Eigen::Vector3d init_point_;
  std::map<int, std::vector<state>> solved_trajectories_;
  int planner_state_ = PlannerStatus::FIRST_PLAN;

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
  const float REACH_TOL = 1;  //! tolerance to reach waypoints

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
  virtual bool optimalTrajectory(const std::vector<state> &initial_trajectory);

  /**
   * @brief gives the next waypoint to reach
   *
   * @return the next waypoint to reach
   */
  virtual state nextGoal() { return goals_[0]; }

  /**
   * @brief virtual function that makes the following checks
   *
   * @return true if all checks are passed
   * @return false if any ot the check is not passed
   */
  virtual bool checks() {
    std::cout << "check mission planner abstract" << std::endl;
  }

  /**
   * @brief gives an initial orientation according to a trajectory given
   *
   * @param traj trajectory
   */
  void initialOrientation(std::vector<state> &traj);

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
};