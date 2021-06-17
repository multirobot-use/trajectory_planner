# Trajectory planner
That trajectory planner contains a global planner that guides a horizon local planner to a desired location. The global planner used in this work is Jump Point Search (JPS). JPS finds the shortest piecewise linear path between two points in a 3D uniformly-weighted voxel grid, guaranteeing optimality and completeness. Then, we define an optimization problem which uses the path calculated by JPS as reference trajectory, and also, minimizes the accelerations. We use ACADO, which is a software environment and algorithm collection for automatic control and dynamic optimization, to solve the optimization problem.

