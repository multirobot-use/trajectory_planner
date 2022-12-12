#include "trajectory_planner.hpp"
#include <pcl/common/transforms.h>

using namespace trajectory_planner;

TrajectoryPlanner::TrajectoryPlanner(const parameters _param)
    : param_(_param),
      my_grid_(0.0, (param_.horizon_length - 1) * param_.step_size,
               param_.horizon_length),
      pcl_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>()) {
  
  // initialize solved trajectory
  // solved_trajectories_[param_.drone_id] =
  //     std::vector<state>(_param.horizon_length);

  // initialize logger
  logger_ = std::make_unique<Logger>(param_.drone_id);

  std::string world_frame = "map";
  double main_loop_rate;
  double segment_margin = 0.0;
  std::vector<double> _local_bbox{2.0, 2.0, 2.0};
  std::vector<float> _map_frame_coordinates{0, 0, 0};
  double size_x{100};
  double size_y{100};
  double size_z{30};
  double min_z{0.0};
  double max_z{8};
  double jps_inflation{1.25};
  double map_resolution{0.5};
  double decompose_inflation{1.0};
  bool test_ = false;
  double max_sampling_distance{0.3};
  int max_jps_expansions{500};

  safe_corridor_generator_ =
      std::make_shared<safe_corridor_generator::SafeCorridorGenerator>();

  safe_corridor_generator_->initialize(param_.pcd_file_path,
      world_frame, decompose_inflation, segment_margin, _local_bbox, size_x,
      size_y, size_z, max_z, min_z, _map_frame_coordinates, jps_inflation,
      map_resolution, max_sampling_distance, max_jps_expansions);

   safe_corridor_generator_->updateMaps();
}

TrajectoryPlanner::TrajectoryPlanner()
    : my_grid_(0.0, (param_.horizon_length - 1) * param_.step_size,
               param_.horizon_length) {
  // Initialize logger
  logger_ = std::make_unique<Logger>(param_.drone_id);
}

TrajectoryPlanner::~TrajectoryPlanner() {}


void TrajectoryPlanner::plan() {
  auto start_time = std::chrono::steady_clock::now();
  auto aux_start_plan_time_ = ros::Time::now();
  start_plan_time_ = aux_start_plan_time_.toSec();
  logger_->log(start_time_cycle_, "plan cycle");
  start_time_cycle_ = std::chrono::steady_clock::now();

  std::cout << "Planner status: " << planner_state_ << std::endl;

  int size_of_solved_trajectory   = solved_trajectories_[param_.drone_id].size();

  if (planner_state_ == PlannerStatus::INSPECTING) {
    if (inspecting())   std::cout << "Inspecting..." << std::endl;
    else {
      std::cout << "Inspecting: close enough!" << std::endl;
    }
  } else if ((solved_trajectories_[param_.drone_id][0].pos -
              solved_trajectories_[param_.drone_id][size_of_solved_trajectory].pos).norm() < 0.5) { // This check is done because, sometimes, follower UAVs generate empty trajectories
      std::cout << "FIRST PLAN" << std::endl;
      planner_state_ = PlannerStatus::FIRST_PLAN;
      }

  if (!hasGoal()) {
    planner_state_ = PlannerStatus::FIRST_PLAN;
    std::cout << "there's no goals" << std::endl;
    return;
  }

  if (!checks()) return;

  // reference_trajectories.clear();
  state initial_pose;

  if (planner_state_ == PlannerStatus::FIRST_PLAN) {
    initial_pose = states_[param_.drone_id];
  } else {
    // Put some security method to detect if the closest point on the trajectory does not make sense
    int shift = closestPoint(solved_trajectories_[param_.drone_id],
                             states_[param_.drone_id]);

    if (shift > (solved_trajectories_[param_.drone_id].size() - (param_.planning_rate/param_.step_size))){
      // std::cout << "Taking end of trajectory" << std::endl;
      auto end_pose = solved_trajectories_[param_.drone_id].size();
      initial_pose  = solved_trajectories_[param_.drone_id][end_pose - 1];
    }
    else {
      initial_pose =
          solved_trajectories_[param_.drone_id]
                              [param_.planning_rate / param_.step_size + shift];
    }
  
  }

  if (planner_state_ == PlannerStatus::INSPECTING){
    std::cout << "Enter Inspection" << std::endl;
    reference_trajectories_[param_.drone_id] = inspectionTrajectory(initial_pose);
    std::cout << "Exit Inspection" << std::endl;
  }
  else{
    reference_trajectories_[param_.drone_id] = initialTrajectory(initial_pose);
  }
  
  if (reference_trajectories_[param_.drone_id].empty()) {
    if (planner_state_ != PlannerStatus::INSPECTING){
      planner_state_ == PlannerStatus::FIRST_PLAN;
    }
    std::cout << "Initial trajectory empty...breaking" << std::endl;
    return;
  } else {
    if (trajectoryHasNan(reference_trajectories_[param_.drone_id])) {
      std::cout << "Initial trajectory has nan" << std::endl;
      logger_->log(reference_trajectories_[param_.drone_id], "Nan or Inf found in ref trajectory");
      return;
    }
    // Calculate optimal trajectory
    lastPointOccupied(reference_trajectories_[param_.drone_id]);
    bool solver_success = optimalTrajectory(reference_trajectories_[param_.drone_id]);
    if (solver_success != 0) {
      logger_->log(solver_success, "Error solving the ocp: ");
    }
  }
  // Calculate orientation
  initialOrientation(solved_trajectories_[param_.drone_id]);

  // Optimize orientation (check without it, about the change of quadrant while flying)
  if (param_.opt_orientation){
    optimalOrientation(solved_trajectories_[param_.drone_id]);
  }
  

  if (planner_state_ != PlannerStatus::INSPECTING)  {planner_state_ = PlannerStatus::REPLANNED;}

  std::cout << " Planner state: "  << planner_state_ << std::endl;
  std::cout << " Operation mode: " << operation_mode_ << std::endl;

  logger_->log(start_time, "solving cycle");
}

void TrajectoryPlanner::updateMap(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _pcd_input) {
  auto start_time = std::chrono::steady_clock::now();
  const std::vector<float> my_pose{float(states_[param_.drone_id].pos.x()),
                                   float(states_[param_.drone_id].pos.y()),
                                   float(states_[param_.drone_id].pos.z())};

  pcl_cloud_ptr_ = _pcd_input;
  mtx_jps_map_.lock();

  // Should not get any parameters?
  safe_corridor_generator_ -> updateMaps(pcl_cloud_ptr_, my_pose);
  // safe_corridor_generator_ -> updateMaps();
  mtx_jps_map_.unlock();
  logger_->log(start_time, "update maps time");
}


void TrajectoryPlanner::optimalOrientation(
    const std::vector<state> &traj_to_optimize) {
  ACADO::DifferentialState heading, pitch, v_heading, v_pitch;
  ACADO::Control a_heading, a_pitch;

  float a_limit     = 0.5;
  float v_limit     = 0.5;
  float pitch_limit = 1.57;
  ACADO::DifferentialEquation model;

  model << dot(heading) == v_heading;
  model << dot(v_heading) == a_heading;
  model << dot(pitch) == v_pitch;
  model << dot(v_pitch) == a_pitch;

  ACADO::OCP ocp(my_grid_);
  ocp.subjectTo(model);
  ocp.subjectTo(-a_limit     <= a_heading <= a_limit);
  ocp.subjectTo(-a_limit     <= a_pitch   <= a_limit);
  ocp.subjectTo(-v_limit     <= v_heading <= v_limit);
  ocp.subjectTo(-v_limit     <= v_pitch   <= v_limit);
  ocp.subjectTo(-pitch_limit <= pitch     <= pitch_limit);

  ACADO::VariablesGrid reference_trajectory(4, my_grid_); // 2 yaw, 2 pitch
  ACADO::DVector       reference_point(4);                // 2 yaw, 2 pitch
  reference_trajectory.setVector(0, reference_point);
  
  Eigen::Vector3d initial_orientation, orientation_aux;
  std::vector<float> yaw_values, adapted_yaw_values;

  initial_orientation = trajectory_planner::quatToEuler(traj_to_optimize[0].orientation);

  // Be aware with heading angle. Can pass from M_PI to -M_PI. Adapt for the optimization problem
  // Adapt heading angles
  for (int k = 0; k < traj_to_optimize.size(); k++) {
    orientation_aux = trajectory_planner::quatToEuler(traj_to_optimize[k].orientation);
    yaw_values.push_back(orientation_aux(2));
  }

  adapted_yaw_values = adaptYawValues(yaw_values);

  for (int k = 0; k < traj_to_optimize.size(); k++) {
    orientation_aux = trajectory_planner::quatToEuler(traj_to_optimize[k].orientation);
    reference_point(0) = adapted_yaw_values[k]; // Yaw/heading
    reference_point(1) = 0.0;
    reference_point(2) = orientation_aux(1);    // Pitch
    reference_point(3) = 0.0;
    reference_trajectory.setVector(k, reference_point);
  }

  ocp.subjectTo(ACADO::AT_START, heading   == initial_orientation(2));
  ocp.subjectTo(ACADO::AT_START, pitch     == initial_orientation(1));
  ocp.subjectTo(ACADO::AT_START, v_heading == 0.0);
  ocp.subjectTo(ACADO::AT_START, v_pitch   == 0.0);
  ocp.subjectTo(ACADO::AT_START, a_heading == 0.0);
  ocp.subjectTo(ACADO::AT_START, a_pitch   == 0.0);

  ACADO::Function rf;

  rf << heading << a_heading << pitch << a_pitch;

  ACADO::DMatrix S(4, 4);

  S.setIdentity();

  ocp.minimizeLSQ(S, rf, reference_trajectory);

  ACADO::OptimizationAlgorithm solver_(ocp);

  // No prints and set MAX_TIME
  solver_.set(ACADO::MAX_TIME, 2.0);
  solver_.set(ACADO::PRINT_INTEGRATOR_PROFILE, false);
  // solver_.set(ACADO::CONIC_SOLVER_PRINT_LEVEL, ACADO::NONE);
  // solver_.set(ACADO::RELAXATION_PARAMETER, 30.0);
  solver_.set(ACADO::PRINTLEVEL, ACADO::NONE);
  solver_.set(ACADO::PRINT_COPYRIGHT, ACADO::NONE);
  solver_.set(ACADO::INTEGRATOR_PRINTLEVEL, ACADO::NONE);

  // reference_trajectory.print();

  // Call the solver
  bool value = solver_.solve();
  // Get solution
  ACADO::VariablesGrid output_states, output_control;

  solver_.getDifferentialStates(output_states);
  solver_.getControls(output_control);


  Eigen::Quaterniond quaternion_aux;

  for (int k = 0; k < traj_to_optimize.size(); k++) {
    quaternion_aux = trajectory_planner::eulerToQuat(0.0, output_states(k, 2), output_states(k, 0));
    solved_trajectories_[param_.drone_id][k].orientation = quaternion_aux;
  }

  heading.clearStaticCounters();
  v_heading.clearStaticCounters();
  pitch.clearStaticCounters();
  v_pitch.clearStaticCounters();

  // int success_value = value;
  // ROS_INFO("[Acado]: Acado angular optimization took %.3f, success = %d ",
  // (ros::Time::now() - start).toSec(), success_value); return success_value;

}


std::vector<state> TrajectoryPlanner::pathFromPointToAnother(
    const Eigen::Vector3d &initial, const Eigen::Vector3d &final) {
  std::vector<state> trajectory_to_optimize;
  state aux_point;
  const Eigen::Vector3d vel((final - initial) / (final - initial).norm());

  for (int i = 0; i < param_.horizon_length; i++) {
    aux_point.pos = initial + i * vel * param_.vel_max * param_.step_size;
    trajectory_to_optimize.push_back(std::move(aux_point));
  }

  return trajectory_to_optimize;
}


bool TrajectoryPlanner::optimalTrajectory(
    const std::vector<state> &initial_trajectory) {
  // if (initial_trajectory.size() != param_.horizon_length) return -2;

  solved_trajectories_[param_.drone_id] = std::vector<state>(initial_trajectory.size());

  ACADO::DifferentialState px_, py_, pz_, vx_, vy_, vz_;
  ACADO::Control ax_, ay_, az_;

  ACADO::DifferentialEquation model;

  ACADO::Grid my_grid_(0.0, (initial_trajectory.size() - 1) * param_.step_size,
              initial_trajectory.size());

  // define the model
  model << dot(px_) == vx_;
  model << dot(py_) == vy_;
  model << dot(pz_) == vz_;
  model << dot(vx_) == ax_;
  model << dot(vy_) == ay_;
  model << dot(vz_) == az_;

  ACADO::OCP ocp(my_grid_);
  ocp.subjectTo(model);
  ocp.subjectTo(-param_.acc_max <= ax_ <= param_.acc_max);
  ocp.subjectTo(-param_.acc_max <= ay_ <= param_.acc_max);
  ocp.subjectTo(-param_.acc_max <= az_ <= param_.acc_max);
  ocp.subjectTo(-param_.vel_max <= vx_ <= param_.vel_max);
  ocp.subjectTo(-param_.vel_max <= vy_ <= param_.vel_max);
  ocp.subjectTo(-param_.vel_max <= vz_ <= param_.vel_max);

  ocp.subjectTo(ACADO::AT_START, px_ == initial_trajectory[0].pos(0));
  ocp.subjectTo(ACADO::AT_START, py_ == initial_trajectory[0].pos(1));
  ocp.subjectTo(ACADO::AT_START, pz_ == initial_trajectory[0].pos(2));
  ocp.subjectTo(ACADO::AT_START, vx_ == initial_trajectory[0].vel(0));
  ocp.subjectTo(ACADO::AT_START, vy_ == initial_trajectory[0].vel(1));
  ocp.subjectTo(ACADO::AT_START, vz_ == initial_trajectory[0].vel(2));
  ocp.subjectTo(ACADO::AT_START, ax_ == initial_trajectory[0].acc(0));
  ocp.subjectTo(ACADO::AT_START, ay_ == initial_trajectory[0].acc(1));
  ocp.subjectTo(ACADO::AT_START, az_ == initial_trajectory[0].acc(2));

  // Generate polyhedrons
  // NOTE: Check what really happens when a huge change of formation angle reference
  // on the followers: it seems that the UAV pose is not the correct one (?)

  // Setup reference trajectory
  ACADO::VariablesGrid reference_trajectory(6, my_grid_);
  ACADO::DVector reference_point(6);

  nav_msgs::PathPtr collision_free_path;

  if (param_.obstacle_avoidance){
    mtx_jps_map_.lock();
    vec_E<Polyhedron<3>> polyhedron_vector =
        safe_corridor_generator_->getSafeCorridorPolyhedronVector(
            vectorToPath(initial_trajectory));  // get polyhedrons
    mtx_jps_map_.unlock();

    // Get JPS path along which the polyhedrons were generated - needed to
    // Generation of correct constraints
    collision_free_path = safe_corridor_generator_->getLastPath();

    std::vector<Eigen::Vector3d> collision_free_path_vector;

    for (int i = 0; i < collision_free_path->poses.size(); i++) {
      collision_free_path_vector.push_back(
          Eigen::Vector3d(collision_free_path->poses[i].pose.position.x,
                collision_free_path->poses[i].pose.position.y,
                collision_free_path->poses[i].pose.position.z));
      // std::cout << " x: " << collision_free_path->poses[i].pose.position.x
      //           << " y: " << collision_free_path->poses[i].pose.position.y
      //           << " z: " << collision_free_path->poses[i].pose.position.z
      //           << std::endl;
    }
    polyhedronsToACADO(ocp, polyhedron_vector, collision_free_path_vector, px_, py_, pz_);

    // OPERATING WITH COLLISION FREE PATH
    for (int k = 0; k < collision_free_path->poses.size(); k++) {
      reference_point(0) = collision_free_path->poses[k].pose.position.x;
      reference_point(1) = collision_free_path->poses[k].pose.position.y;
      reference_point(2) = collision_free_path->poses[k].pose.position.z;
      reference_point(3) = 0.0;
      reference_point(4) = 0.0;
      reference_point(5) = 0.0;
      reference_trajectory.setVector(k, reference_point);
    }
  }
  else{
    // OPERATING WITH INITIAL TRAJECTORY
    for (int k = 0; k < initial_trajectory.size(); k++) {
      reference_point(0) = initial_trajectory[k].pos(0);
      reference_point(1) = initial_trajectory[k].pos(1);
      reference_point(2) = initial_trajectory[k].pos(2);
      reference_point(3) = 0.0;
      reference_point(4) = 0.0;
      reference_point(5) = 0.0;
      reference_trajectory.setVector(k, reference_point);  // TODO: check
    }
  }

  // DEFINE LSQ function to minimize diff from desired trajectory
  ACADO::Function rf;

  rf << px_ << py_ << pz_ << ax_ << ay_ << az_;

  ACADO::DMatrix S(6, 6);

  S.setIdentity();

  ocp.minimizeLSQ(S, rf, reference_trajectory);

  ACADO::OptimizationAlgorithm solver(ocp);

  // No prints and set MAX_TIME
  solver.set(ACADO::MAX_TIME, 2.0);
  solver.set(ACADO::PRINT_INTEGRATOR_PROFILE, false);
  // solver.set(ACADO::CONIC_SOLVER_PRINT_LEVEL, ACADO::NONE);
  // solver.set(ACADO::RELAXATION_PARAMETER, 30.0);
  solver.set(ACADO::PRINTLEVEL, ACADO::NONE);
  solver.set(ACADO::PRINT_COPYRIGHT, ACADO::NONE);
  solver.set(ACADO::INTEGRATOR_PRINTLEVEL, ACADO::NONE);

  bool solver_success = solver.solve();

  ACADO::VariablesGrid output_states, output_control;

  solver.getDifferentialStates(output_states);
  solver.getControls(output_control);

  // Start mutex
  // if (param_.drone_id == 1){
  //   mtx_leader_traj_.lock();
  // }

  int size_for;

  if (param_.obstacle_avoidance){
    size_for = collision_free_path->poses.size();
  }
  else{
    size_for = initial_trajectory.size();
  }

  for (int k = 0; k < size_for; k++) {
    // solved_trajectories_[param_.drone_id][k].time_stamp = current_time_ + k*param_.step_size;
    solved_trajectories_[param_.drone_id][k].time_stamp = initial_trajectory[0].time_stamp + k*param_.step_size;
    // std::cout << "Current time: " << current_time_ << "   k=" << k << ": " << solved_trajectories_[param_.drone_id][k].time_stamp << std::endl;  
    solved_trajectories_[param_.drone_id][k].pos(0) = output_states(k, 0);
    solved_trajectories_[param_.drone_id][k].pos(1) = output_states(k, 1);
    solved_trajectories_[param_.drone_id][k].pos(2) = output_states(k, 2);
    solved_trajectories_[param_.drone_id][k].vel(0) = output_states(k, 3);
    solved_trajectories_[param_.drone_id][k].vel(1) = output_states(k, 4);
    solved_trajectories_[param_.drone_id][k].vel(2) = output_states(k, 5);
    solved_trajectories_[param_.drone_id][k].acc(0) = output_control(k, 0);
    solved_trajectories_[param_.drone_id][k].acc(1) = output_control(k, 1);
    solved_trajectories_[param_.drone_id][k].acc(2) = output_control(k, 2);
  }

  // End mutex
  // if (param_.drone_id == 1){
  //   mtx_leader_traj_.unlock();
  // }

  px_.clearStaticCounters();
  py_.clearStaticCounters();
  pz_.clearStaticCounters();
  vx_.clearStaticCounters();
  vy_.clearStaticCounters();
  vz_.clearStaticCounters();
  ax_.clearStaticCounters();
  ay_.clearStaticCounters();
  az_.clearStaticCounters();
};

int TrajectoryPlanner::closestPoint(
    const std::vector<state> &initial_trajectory, const state point) {
  float dist = INFINITY;
  float aux_dist = 0;
  int idx = 0;

  for (int i = 0; i < initial_trajectory.size(); i++) {
    aux_dist = (initial_trajectory[i].pos - point.pos).norm();
    // std::cout<<aux_dist<<std::endl;

    if (aux_dist < dist) {
      dist = aux_dist;
      idx = i;
    }
  }
  return idx;
}
void TrajectoryPlanner::polyhedronsToACADO(
    ACADO::OCP &_ocp, const vec_E<Polyhedron<3>> &_vector_of_polyhedrons,
    const std::vector<Eigen::Vector3d> &path_free, ACADO::DifferentialState &_px,
    ACADO::DifferentialState &_py, ACADO::DifferentialState &_pz) {
  // Convert to inequality constraints Ax < b
  // Taken from decomp test node
  // ROS_INFO("[Acado]: polyhedrons to acado ");

  for (size_t i = 0; i < path_free.size() - 1; i++) {
    // ROS_INFO(
    //     "[Acado]: polyhedrons to acado - iter i = %lu, initial path size = "
    //     "%lu, polyhedrons size =%lu ",
    //     i, path_free.size(), _vector_of_polyhedrons.size());
    const auto pt_inside = path_free[i + 1];

    LinearConstraint3D cs(pt_inside, _vector_of_polyhedrons[i].hyperplanes());
    for (size_t k = 0; k < cs.b().size();
         k++) {  // each polyhedron i is subject to k constraints
      // ROS_INFO("[Acado]: polyhedrons to acado - iter k = %lu ", k);
      if (!std::isnan(cs.A()(k, 0)) && !std::isnan(cs.A()(k, 1)) &&
          !std::isnan(cs.A()(k, 2)) && !std::isnan(cs.b()[k])) {
        _ocp.subjectTo(i + 1, cs.A()(k, 0) * _px + cs.A()(k, 1) * _py +
                                      cs.A()(k, 2) * _pz <=
                                  cs.b()[k]);
      } else {
        ROS_ERROR("[Acado]: NaNs detected in polyhedrons ");
      }
      // ROS_INFO(
      //     "[Acado]: Adding constraint: %.2f * px + %.2f * py + %.2f * pz <= "
      //     "%.2f",
      //     cs.A()(k, 0), cs.A()(k, 1), cs.A()(k, 2), cs.b()[k]);
      // ROS_INFO("[Acado]: Point: [%.2f, %.2f, %.2f], result = %.2f",
      //          pt_inside(0), pt_inside(1), pt_inside(2),
      //          cs.A()(k, 0) * pt_inside(0) + cs.A()(k, 1) * pt_inside(1) +
      //              cs.A()(k, 2) * pt_inside(2));
    }
  }
}

std::vector<float> TrajectoryPlanner::adaptYawValues(std::vector<float> &v_yaw){
  float yaw;
  float previous_loop_value = v_yaw[0];
  bool add_plus_2pi  = false;
  bool add_minus_2pi = false;
  std::vector<float> adapted_yaw_vector;

  adapted_yaw_vector.push_back(previous_loop_value);

  for (int i = 1; i < v_yaw.size(); i++){
    if (abs(previous_loop_value - v_yaw[i]) > M_PI){
      if (previous_loop_value > 0)  add_plus_2pi  = true;
      else                          add_minus_2pi = true;
    }

    if (add_plus_2pi)         yaw = v_yaw[i] + 2*M_PI;
    else if (add_minus_2pi)   yaw = v_yaw[i] - 2*M_PI;
    else                      yaw = v_yaw[i];

    previous_loop_value = v_yaw[i];
    adapted_yaw_vector.push_back(yaw);
  }

  return adapted_yaw_vector;

}

// change to free last point
void TrajectoryPlanner::lastPointOccupied(
    std::vector<state> &_initial_trajectory) {
  if (safe_corridor_generator_->isPointOccupied(
          _initial_trajectory.back().pos)) {
    Eigen::Vector3d last_point = _initial_trajectory.back().pos;
    const Eigen::Vector3d first_point = _initial_trajectory[0].pos;
    const Eigen::Vector3d point_dir = (last_point - first_point).normalized();
    while (safe_corridor_generator_->isPointOccupied(last_point)) {
      last_point = last_point + point_dir;
    }
    state aux_state;
    aux_state.pos = last_point;
    _initial_trajectory.back() = aux_state;
  }
}
bool TrajectoryPlanner::checks(){
  // check waypoints to remove or not the waypoints to follow
  if (waypointReached(goals_[0], states_[param_.drone_id])) {
    std::cout << "Removed waypoint" << std::endl;
    init_point_ = goals_[0].pos;
    goals_.erase(goals_.begin());
    if (!hasGoal()) return false;
  }
  return true;
}
