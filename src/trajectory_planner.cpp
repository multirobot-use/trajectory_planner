#include "trajectory_planner.hpp"
#include <pcl/common/transforms.h>

using namespace trajectory_planner;

TrajectoryPlanner::TrajectoryPlanner(const parameters _param)
    : param_(_param),
      my_grid_(0.0, (param_.horizon_length - 1) * param_.step_size,
               param_.horizon_length),
      pcl_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>()) {
  // initialize solved trajectory
  solved_trajectories_[param_.drone_id] =
      std::vector<state>(_param.horizon_length);
  // initialize logger
  logger_ = std::make_unique<Logger>(param_.drone_id);

  std::string world_frame = "map";
  double main_loop_rate;
  double segment_margin = 0.0;
  std::vector<double> _local_bbox{2.0, 2.0, 2.0};
  std::vector<float> _map_frame_coordinates{0, 0, 0};
  double size_x{50};
  double size_y{50};
  double size_z{20};
  double min_z{0.0};
  double max_z{20};
  double jps_inflation{1.2};
  double map_resolution{0.25};
  double decompose_inflation{1.00};
  bool test_ = false;
  double max_sampling_distance{0.3};
  int max_jps_expansions{500};

  safe_corridor_generator_ =
      std::make_shared<safe_corridor_generator::SafeCorridorGenerator>();

  safe_corridor_generator_->initialize(
      world_frame, decompose_inflation, segment_margin, _local_bbox, size_x,
      size_y, size_z, max_z, min_z, _map_frame_coordinates, jps_inflation,
      map_resolution, max_sampling_distance, max_jps_expansions);
}

TrajectoryPlanner::TrajectoryPlanner()
    : my_grid_(0.0, (param_.horizon_length - 1) * param_.step_size,
               param_.horizon_length) {
  // initialize logger
  logger_ = std::make_unique<Logger>(param_.drone_id);
}

TrajectoryPlanner::~TrajectoryPlanner() {}

void TrajectoryPlanner::plan() {
  auto start_time = std::chrono::steady_clock::now();
  logger_->log(start_time_cycle_, "plan cycle");
  start_time_cycle_ = std::chrono::steady_clock::now();
  if (!hasGoal()) {
    planner_state_ = PlannerStatus::FIRST_PLAN;
    std::cout << "there's no goals" << std::endl;
    return;
  }

  if (!checks()) return;

  reference_traj.clear();
  state initial_pose;

  if (planner_state_ == PlannerStatus::FIRST_PLAN) {
    initial_pose = states_[param_.drone_id];
  } else {
    int shift = closestPoint(solved_trajectories_[param_.drone_id],
                             states_[param_.drone_id]);
    // std::cout<<shift<<std::endl;
    initial_pose =
        solved_trajectories_[param_.drone_id]
                            [param_.planning_rate / param_.step_size + shift];
    // std::cout<<"i: "<<param_.planning_rate/param_.step_size+shift<<std::endl;
  }

  reference_traj = initialTrajectory(initial_pose);
  if (reference_traj.empty()) {
    planner_state_ == PlannerStatus::FIRST_PLAN;
    std::cout << "Initial trajectory empty...breaking" << std::endl;
    return;
  } else {
    if (trajectoryHasNan(reference_traj)) {
      std::cout << "Initial trajectory has nan" << std::endl;
      logger_->log(reference_traj, "Nan or Inf found in ref trajectory");
      return;
    }
    // calculate optimal trajectory
    lastPointOccupied(reference_traj);
    bool solver_success = optimalTrajectory(reference_traj);
    if (solver_success != 0) {
      logger_->log(solver_success, "Error solving the ocp: ");
    }
  }
  // calculate orientation
  initialOrientation(solved_trajectories_[param_.drone_id]);

  planner_state_ = PlannerStatus::REPLANNED;
  logger_->log(start_time, "solving cycle");
}

void TrajectoryPlanner::updateMap(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _pcd_input) {
  auto start_time = std::chrono::steady_clock::now();
  const std::vector<float> my_pose{float(states_[param_.drone_id].pos.x()),
                                   float(states_[param_.drone_id].pos.y()),
                                   float(states_[param_.drone_id].pos.z())};

  pcl_cloud_ptr_ = _pcd_input;

  safe_corridor_generator_->updateMaps(pcl_cloud_ptr_, my_pose);
  mtx_jps_map_.unlock();
  logger_->log(start_time, "update maps time");
}

void TrajectoryPlanner::optimalOrientation(
    const std::vector<state> &traj_to_optimize) {
  // // DifferentialState heading, pitch, v_heading, v_pitch;
  // DifferentialState heading, v_heading;
  // // Control a_heading, a_pitch;
  // Control a_heading;

  // float a_limit     = 0.5;
  // float v_limit     = 0.5;
  // float pitch_limit = 1.57;
  // DifferentialEquation model;

  // model << dot(heading) == v_heading;
  // model << dot(v_heading) == a_heading;
  // // model << dot(pitch) == v_pitch;
  // // model << dot(v_pitch) == a_pitch;

  // OCP ocp(my_grid_);
  // ocp.subjectTo(model);
  // ocp.subjectTo(-a_limit <= a_heading <= a_limit);
  // // ocp.subjectTo(-a_limit <= a_pitch <= a_limit);
  // ocp.subjectTo(-v_limit <= v_heading <= v_limit);
  // // ocp.subjectTo(-v_limit <= v_pitch <= v_limit);
  // // ocp.subjectTo(-pitch_limit <= pitch <= pitch_limit);

  // VariablesGrid reference_trajectory(2, my_grid_); // 2 for pitch
  // DVector       reference_point(2); // 2 for pitch
  // reference_trajectory.setVector(0, reference_point);  // TODO: check
  // indexing for (int k = 0; k < TIME_HORIZON; k++) {
  //   reference_point(0) =
  //   mrs_lib::geometry::radians::unwrap(_desired_trajectory[k].heading,
  //   reference_point(0)); reference_point(1) = 0;
  //   reference_trajectory.setVector(k, reference_point);  // TODO: check
  //   indexing
  // }

  // // set initial guess so that it meats the constraints
  // // _initial_guess[0].v_heading = _initial_guess[0].v_heading < -v_limit ?
  // -v_limit : _initial_guess[0].v_heading;
  // // _initial_guess[0].v_pitch   = _initial_guess[0].v_pitch > v_limit ?
  // v_limit : _initial_guess[0].v_pitch;
  // // _initial_guess[0].a_heading = _initial_guess[0].a_heading < -a_limit ?
  // -a_limit : _initial_guess[0].a_heading;
  // // _initial_guess[0].a_pitch   = _initial_guess[0].a_pitch > a_limit ?
  // a_limit : _initial_guess[0].a_pitch;

  // ocp.subjectTo(AT_START, heading == _initial_guess[0].heading);
  // ocp.subjectTo(AT_START, pitch == _initial_guess[0].pitch);
  // ocp.subjectTo(AT_START, v_heading == _initial_guess[0].v_heading);
  // ocp.subjectTo(AT_START, v_pitch == _initial_guess[0].v_pitch);
  // ocp.subjectTo(AT_START, a_heading == _initial_guess[0].a_heading);
  // ocp.subjectTo(AT_START, a_pitch == _initial_guess[0].a_pitch);

  // Function rf;

  // rf << heading << a_heading;
  // // rf << pitch;

  // DMatrix S(2, 2);
  // // DMatrix S(2, 2);

  // S.setIdentity();
  // S(0, 0) = 1.0;
  // S(1, 1) = 1.0;

  // ocp.minimizeLSQ(S, rf, reference_trajectory);

  // OptimizationAlgorithm solver_(ocp);

  // // reference_trajectory.print();
  // // call the solver
  // returnValue value = solver_.solve();
  // // get solution
  // VariablesGrid output_states, output_control;

  // solver_.getDifferentialStates(output_states);
  // solver_.getControls(output_control);
  // ROS_INFO("[Acado]: Output states: ");
  // output_states.print();
  // for (uint i = 0; i < N; i++) {
  //   _robot_states[i].heading   = output_states(i, 0);
  //   _robot_states[i].pitch     = output_states(i, 1);
  //   _robot_states[i].v_heading = output_states(i, 2);
  //   _robot_states[i].v_pitch   = output_states(i, 3);
  //   _robot_states[i].a_heading = output_control(i, 0);
  //   _robot_states[i].a_pitch   = output_control(i, 1);
  // }

  // heading.clearStaticCounters();
  // v_heading.clearStaticCounters();
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
  if (initial_trajectory.size() != param_.horizon_length) return -2;
  ACADO::DifferentialState px_, py_, pz_, vx_, vy_, vz_;
  ACADO::Control ax_, ay_, az_;

  ACADO::DifferentialEquation model;

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

  // generate polyhedrons
  mtx_jps_map_.lock();
  vec_E<Polyhedron<3>> polyhedron_vector =
      safe_corridor_generator_->getSafeCorridorPolyhedronVector(
          vectorToPath(initial_trajectory));  // get polyhedrons
  mtx_jps_map_.unlock();
  // get JPS path along which the polyhedrons were generated - needed to
  // generation of correct constraints
  nav_msgs::PathPtr collision_free_path =
      safe_corridor_generator_->getLastPath();

  std::vector<Eigen::Vector3f> collision_free_path_vector;

  for (int i = 0; i < collision_free_path->poses.size();
       i++) {  // nav_msgs to eigen vector
    collision_free_path_vector.push_back(
        Eigen::Vector3f(collision_free_path->poses[i].pose.position.x,
              collision_free_path->poses[i].pose.position.y,
              collision_free_path->poses[i].pose.position.z));
    std::cout << "x: " << collision_free_path->poses[i].pose.position.x
              << " y: " << collision_free_path->poses[i].pose.position.y
              << " z: " << collision_free_path->poses[i].pose.position.z
              << std::endl;
  }
  polyhedronsToACADO(ocp, polyhedron_vector, collision_free_path_vector, px_, py_, pz_);
  // setup reference trajectory
  ACADO::VariablesGrid reference_trajectory(6, my_grid_);
  ACADO::DVector reference_point(6);
  for (int k = 0; k < param_.horizon_length; k++) {
    reference_point(0) = initial_trajectory[k].pos(0);
    reference_point(1) = initial_trajectory[k].pos(1);
    reference_point(2) = initial_trajectory[k].pos(2);
    reference_point(3) = 0.0;
    reference_point(4) = 0.0;
    reference_point(5) = 0.0;
    reference_trajectory.setVector(k, reference_point);  // TODO: check
  }

  // DEFINE LSQ function to minimize diff from desired trajectory
  ACADO::Function rf;

  rf << px_ << py_ << pz_ << ax_ << ay_ << az_;

  ACADO::DMatrix S(6, 6);

  S.setIdentity();

  ocp.minimizeLSQ(S, rf, reference_trajectory);

  ACADO::OptimizationAlgorithm solver(ocp);
  solver.set(ACADO::MAX_TIME, 2.0);  // TODO: have it as parameter
  // solver.set(ACADO::PRINT_INTEGRATOR_PROFILE, false);
  // solver.set(ACADO::CONIC_SOLVER_PRINT_LEVEL, ACADO::NONE);
  // solver.set(ACADO::RELAXATION_PARAMETER, 30.0);
  // solver.set(ACADO::PRINTLEVEL, ACADO::NONE);
  // solver.set(ACADO::PRINT_COPYRIGHT, ACADO::NONE);
  // solver.set(ACADO::INTEGRATOR_PRINTLEVEL, ACADO::NONE);

  bool solver_success = solver.solve();

  ACADO::VariablesGrid output_states, output_control;

  solver.getDifferentialStates(output_states);
  solver.getControls(output_control);

  for (int k = 0; k < param_.horizon_length; k++) {
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
  ROS_INFO("[Acado]: polyhedrons to acado ");

  for (size_t i = 0; i < path_free.size() - 1; i++) {
    ROS_INFO(
        "[Acado]: polyhedrons to acado - iter i = %lu, initial path size = "
        "%lu, polyhedrons size =%lu ",
        i, path_free.size(), _vector_of_polyhedrons.size());
    const auto pt_inside = path_free[i + 1];

    LinearConstraint3D cs(pt_inside, _vector_of_polyhedrons[i].hyperplanes());
    for (size_t k = 0; k < cs.b().size();
         k++) {  // each polyhedron i is subject to k constraints
      ROS_INFO("[Acado]: polyhedrons to acado - iter k = %lu ", k);
      if (!std::isnan(cs.A()(k, 0)) && !std::isnan(cs.A()(k, 1)) &&
          !std::isnan(cs.A()(k, 2)) && !std::isnan(cs.b()[k])) {
        _ocp.subjectTo(i + 1, cs.A()(k, 0) * _px + cs.A()(k, 1) * _py +
                                      cs.A()(k, 2) * _pz <=
                                  cs.b()[k]);
      } else {
        ROS_ERROR("[Acado]: NaNs detected in polyhedrons ");
      }
      ROS_INFO(
          "[Acado]: Adding constraint: %.2f * px + %.2f * py + %.2f * pz <= "
          "%.2f",
          cs.A()(k, 0), cs.A()(k, 1), cs.A()(k, 2), cs.b()[k]);
      ROS_INFO("[Acado]: Point: [%.2f, %.2f, %.2f], result = %.2f",
               pt_inside(0), pt_inside(1), pt_inside(2),
               cs.A()(k, 0) * pt_inside(0) + cs.A()(k, 1) * pt_inside(1) +
                   cs.A()(k, 2) * pt_inside(2));
    }
  }
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