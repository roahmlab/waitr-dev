#include "Dynamics.h"
#include "CollisionChecking.h"

// const std::string pathname = "/home/roahmlab/Documents/armour-dev/kinova_src/kinova_simulator_interfaces/kinova_planner_realtime/buffer/";
const std::string pathname = "/home/baiyuew/ROAHM/armour-dev/kinova_src/kinova_simulator_interfaces/kinova_planner_realtime/buffer/";
const std::string inputfilename = pathname + "armour.in";
const std::string outputfilename1 = pathname + "armour.out";
const std::string outputfilename2 = pathname + "armour_joint_position_center.out";
const std::string outputfilename3 = pathname + "armour_joint_position_radius.out";
const std::string outputfilename4 = pathname + "armour_control_input_radius.out";
const std::string outputfilename5 = pathname + "armour_constraints.out";
const std::string outputfilename6 = pathname + "armour_wrench_values.out";
const std::string outputfilename7 = pathname + "armour_force_constraint_radius.out";
const std::string outputfilename8 = pathname + "armour_desired_sliced.out";

int main() {
/*
Section I:
    Parse input
    There is no check and warning, so be careful!
*/
    // Here is an example of the required input
    // double q0[NUM_FACTORS] = {0.6543, -0.0876, -0.4837, -1.2278, -1.5735, -1.0720, 0};
    // double qd0[NUM_FACTORS] = {0, 0, 0, 0, 0, 0, 0};
    // double qdd0[NUM_FACTORS] = {0, 0, 0, 0, 0, 0, 0};
    // double q_des[NUM_FACTORS] = {0.6831, 0.009488, -0.2471, -0.9777, -1.414, -0.9958, 0};

    // const int num_obstacles = 10;
    // const double obstacles[num_obstacles * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3] = {-0.28239,  -0.33281, 0.88069, 0.069825, 0, 0, 0,  0.09508, 0, 0, 0, 0.016624,
    //                                                                             -0.19033,  0.035391,  1.3032,  0.11024, 0, 0, 0, 0.025188, 0, 0, 0, 0.014342,
    //                                                                             0.67593, -0.085841, 0.43572,  0.17408, 0, 0, 0,  0.07951, 0, 0, 0,  0.18012,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             -0.28239,  -0.33281, 0.88069, 0.069825, 0, 0, 0,  0.09508, 0, 0, 0, 0.016624,
    //                                                                             -0.19033,  0.035391,  1.3032,  0.11024, 0, 0, 0, 0.025188, 0, 0, 0, 0.014342,
    //                                                                             0.67593, -0.085841, 0.43572,  0.17408, 0, 0, 0,  0.07951, 0, 0, 0,  0.18012,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981};

    // declare this first and make sure we always have a new output
    std::ofstream outputstream1(outputfilename1);

    Eigen::VectorXd q0(NUM_FACTORS); q0.setZero();
    Eigen::VectorXd qd0(NUM_FACTORS); qd0.setZero();
    Eigen::VectorXd qdd0(NUM_FACTORS); qdd0.setZero();
    Eigen::VectorXd q_des(NUM_FACTORS); q_des.setZero();

    int num_obstacles = 0;
    double obstacles[MAX_OBSTACLE_NUM * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3] = {0.0};

    std::ifstream inputstream(inputfilename);
    if (!inputstream.is_open()) {
        WARNING_PRINT("        CUDA & C++: Error reading input files !\n");
        outputstream1 << -1;
        outputstream1.close();
        throw;
    }
    for (int i = 0; i < NUM_FACTORS; i++) {
        inputstream >> q0[i];
    }
    for (int i = 0; i < NUM_FACTORS; i++) {
        inputstream >> qd0[i];
        qd0[i] = qd0[i];
    }
    for (int i = 0; i < NUM_FACTORS; i++) {
        inputstream >> qdd0[i];
        qdd0[i] = qdd0[i];
    }
    for (int i = 0; i < NUM_FACTORS; i++) {
        inputstream >> q_des[i];
    }
    inputstream >> num_obstacles;
    if (num_obstacles > MAX_OBSTACLE_NUM || num_obstacles < 0) {
        WARNING_PRINT("        CUDA & C++: Number of obstacles larger than MAX_OBSTACLE_NUM !\n");
        outputstream1 << -1;
        outputstream1.close();
        throw;
    }
    if (num_obstacles > 0) {
        for (int i = 0; i < num_obstacles * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3; i++) {
            inputstream >> obstacles[i];
        }
    }

    inputstream.close();

    double t_plan = 1.0; // optimize the distance between q_des and the desired trajectories at t_plan
    // Kinova Hardware Demo Values: u_s = 0.609382421; surf_rad =  0.058;
    double u_s = 0.609382421; // 0.5; // static coefficient of friction between tray and object
    double surf_rad =  0.058/2; // 0.0762; // radius of contact area between tray and object (area assumed to be circular)
    // Note: might want to change this to be input to the C++ code from matlab?

/*
Section II:
    Initialize all polynomial zonotopes, including links and torques
*/
    auto start1 = std::chrono::high_resolution_clock::now();

    omp_set_num_threads(NUM_THREADS);
    int openmp_t_ind = 0; // openmp loop index

    /*
    Section II.A: Create JRS online
    */
    BezierCurve traj(q0, qd0, qdd0);

    try {
        #pragma omp parallel for shared(traj) private(openmp_t_ind) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
        for(openmp_t_ind = 0; openmp_t_ind < NUM_TIME_STEPS; openmp_t_ind++) {
            traj.makePolyZono(openmp_t_ind);
        }
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error creating JRS! Check previous error message!");
        return -1;
    }

    /*
    Section II.B: Compute link PZs and nominal torque PZs
    */
    KinematicsDynamics kd(&traj);
    Eigen::Matrix<double, 3, 3 + 3> link_independent_generators[NUM_TIME_STEPS * NUM_JOINTS];

    try {
        #pragma omp parallel for shared(kd, link_independent_generators) private(openmp_t_ind) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
        for(openmp_t_ind = 0; openmp_t_ind < NUM_TIME_STEPS; openmp_t_ind++) {
            // compute link PZs through forward kinematics
            kd.fk(openmp_t_ind);

            // reduce non-only-k-dependent generators so that slice takes less time
            for (int i = 0; i < NUM_JOINTS; i++) {
                link_independent_generators[openmp_t_ind * NUM_JOINTS + i] = kd.links(i, openmp_t_ind).reduce_link_PZ();
            }

            // compute nominal torque
            kd.rnea_nominal(openmp_t_ind);

            // compute interval torque
            kd.rnea_interval(openmp_t_ind);

            // compute max disturbance (stored in u_nom_int)
            for (int i = 0; i < NUM_FACTORS; i++) {
                kd.u_nom_int(i, openmp_t_ind) = kd.u_nom_int(i, openmp_t_ind) - kd.u_nom(i, openmp_t_ind);
            }

            // reduce non-only-k-dependent generators so that slice takes less time
            for (int i = 0; i < NUM_FACTORS; i++) {
                kd.u_nom(i, openmp_t_ind).reduce();
            }
        }
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error computing link PZs and nominal torque PZs! Check previous error message!");
        return -1;
    }

    /*
    Section II.C: Compute robust input bound
    */
    // the radius of the torque PZs
    Eigen::MatrixXd torque_radius(NUM_FACTORS, NUM_TIME_STEPS);
    torque_radius.setZero();

    try {
        for(int t_ind = 0; t_ind < NUM_TIME_STEPS; t_ind++) {
            // (1) add the bound of robust input (||v||)
            Interval rho_max_temp = Interval(0.0);
            for (int i = 0; i < NUM_FACTORS; i++) {
                // compute norm of disturbance
                MatrixXInt temp = kd.u_nom_int(i, t_ind).toInterval(); // should be a 1-dim Interval
                rho_max_temp += temp(0) * temp(0);

                torque_radius(i, t_ind) = alpha * (M_max - M_min) * eps + 0.5 * max(abs(temp(0).lower()), abs(temp(0).upper()));
            }
            rho_max_temp = sqrt(rho_max_temp);
            
            for (int i = 0; i < NUM_FACTORS; i++) {
                torque_radius(i, t_ind) += 0.5 * rho_max_temp.upper();
            }

            // (2) add the radius of the nominal input PZ (after reducing)
            for (int i = 0; i < NUM_FACTORS; i++) {
                torque_radius(i, t_ind) += kd.u_nom(i, t_ind).independent(0);
            }

            // (3) add friction
            for (int i = 0; i < NUM_FACTORS; i++) {
                torque_radius(i, t_ind) += friction[i];
            }

            // so that torque_radius would be the radius of the total control input PZ from now
        }
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error computing torque PZs! Check previous error message!");
        return -1;
    }

    auto stop1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
    cout << "        CUDA & C++: Time taken by generating reachable sets: " << duration1.count() << " milliseconds" << endl;

/*
Section III:
    Slice reachable sets at some point
*/

    // double factors[NUM_FACTORS] = {0.5, 0.7, 0.7, 0.0, -0.8, -0.6, -0.7};
    // double factors[NUM_FACTORS] = {0,0,0,0,0,0,0};
    double factors[NUM_FACTORS] = {1,1,1,1,1,1,1};
    // double factors[NUM_FACTORS] = {-1,-1,-1,-1,-1,-1,-1};

    Eigen::MatrixXd qd_des_sliced_center(NUM_FACTORS, NUM_TIME_STEPS);
    Eigen::MatrixXd torque_sliced_center(NUM_FACTORS, NUM_TIME_STEPS);
    Eigen::Vector3d link_sliced_center[NUM_TIME_STEPS * NUM_JOINTS];
    Eigen::MatrixXd force_value_center(3,NUM_TIME_STEPS);
    Eigen::MatrixXd force_value_radii(3,NUM_TIME_STEPS);
    Eigen::MatrixXd moment_value_center(3,NUM_TIME_STEPS);
    Eigen::MatrixXd moment_value_radii(3,NUM_TIME_STEPS);

    // storage for sliced desired trajectories
    Eigen::MatrixXd qd_center(NUM_FACTORS,NUM_TIME_STEPS);
    Eigen::MatrixXd qda_center(NUM_FACTORS,NUM_TIME_STEPS);
    Eigen::MatrixXd qdda_center(NUM_FACTORS,NUM_TIME_STEPS);

    Eigen::MatrixXd qd_radius(NUM_FACTORS,NUM_TIME_STEPS);
    Eigen::MatrixXd qda_radius(NUM_FACTORS,NUM_TIME_STEPS);
    Eigen::MatrixXd qdda_radius(NUM_FACTORS,NUM_TIME_STEPS);


    double force_constraint_ub[3*NUM_TIME_STEPS];
    double force_constraint_lb[3*NUM_TIME_STEPS];

    // #pragma omp parallel for shared(kd, factors, torque_sliced_center, link_sliced_center) private(openmp_t_ind) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
    for(openmp_t_ind = 0; openmp_t_ind < NUM_TIME_STEPS; openmp_t_ind++) {

        for (int k = 0; k < NUM_FACTORS; k++) {
            MatrixXInt res = traj.qd_des(k, openmp_t_ind).slice(factors);
            qd_des_sliced_center(k, openmp_t_ind) = getCenter(res(0));
        }

        // slicing desired trajectories for comparison
        for (int k = 0; k < NUM_FACTORS; k++) {
            // slice
            MatrixXInt qd_slice = traj.qd_des(k,openmp_t_ind).slice(factors);
            // cout << qd_slice << endl;
            // get center
            qd_center(k,openmp_t_ind) = getCenter(qd_slice(0));
            // cout << qd_center(k,openmp_t_ind) << " ";
            // get radius
            qd_radius(k,openmp_t_ind) = getRadius(qd_slice(0));
            // cout << qd_slice << endl;

            // slice
            MatrixXInt qda_slice = traj.qda_des(k,openmp_t_ind).slice(factors);
            // get center
            qda_center(k,openmp_t_ind) = getCenter(qda_slice(0));
            // cout << qda_center(k,openmp_t_ind) << " ";
            // get radius
            qda_radius(k,openmp_t_ind) = getRadius(qda_slice(0));

            // slice
            MatrixXInt qdda_slice = traj.qdda_des(k,openmp_t_ind).slice(factors);
            // get center
            qdda_center(k,openmp_t_ind) = getCenter(qdda_slice(0));
            // cout << qdda_center(k,openmp_t_ind) << endl;
            // get radius
            qdda_radius(k,openmp_t_ind) = getRadius(qdda_slice(0));

        }

        for (int k = 0; k < NUM_FACTORS; k++) {
            MatrixXInt res = kd.u_nom(k, openmp_t_ind).slice(factors);
            torque_sliced_center(k, openmp_t_ind) = getCenter(res(0));
        }

        for (int l = 0; l < NUM_JOINTS; l++) {
            MatrixXInt res = kd.links(l, openmp_t_ind).slice(factors);
            link_sliced_center[openmp_t_ind * NUM_JOINTS + l] = getCenter(res);
        }

        // ask Bohao about slicing joint by joint
        // slice by each dimension and output the center and radii of each into the wrench value file
        for (int m = 0; m < 3; m++) {
            MatrixXInt res1 = kd.f_c_int(openmp_t_ind)(m,0).slice(factors);
            force_value_center(m,openmp_t_ind) = getCenter(res1(0));
            force_value_radii(m,openmp_t_ind) = getRadius(res1(0));
            MatrixXInt res2 = kd.n_c_int(openmp_t_ind)(m,0).slice(factors);
            moment_value_center(m,openmp_t_ind) = getCenter(res2(0));
            moment_value_radii(m,openmp_t_ind) = getRadius(res2(0));
        }
        

        // calculate the constraints
        // Extract the force PZs, slice, and get the centers and radii
        MatrixXInt f_c_x = kd.f_c_int(openmp_t_ind)(0,0).slice(factors);
        double f_c_x_center = getCenter(f_c_x(0));
        double f_c_x_radius = getRadius(f_c_x(0));

        MatrixXInt f_c_y = kd.f_c_int(openmp_t_ind)(1,0).slice(factors);
        double f_c_y_center = getCenter(f_c_y(0));
        double f_c_y_radius = getRadius(f_c_y(0));

        MatrixXInt f_c_z = kd.f_c_int(openmp_t_ind)(2,0).slice(factors);
        double f_c_z_center = getCenter(f_c_z(0));
        double f_c_z_radius = getRadius(f_c_z(0));
        // f_c_x_radius_2 = f_c_x_radius*f_c_x_radius;
        // f_c_x_center_2 = f_c_x_center*f_c_x_center;

        // Extract the moment PZs
        MatrixXInt n_c_x = kd.n_c_int(openmp_t_ind)(0,0).slice(factors);
        double n_c_x_center = getCenter(n_c_x(0));
        double n_c_x_radius = getRadius(n_c_x(0));

        MatrixXInt n_c_y = kd.n_c_int(openmp_t_ind)(1,0).slice(factors);
        double n_c_y_center = getCenter(n_c_y(0));
        double n_c_y_radius = getRadius(n_c_y(0));

        MatrixXInt n_c_z = kd.n_c_int(openmp_t_ind)(2,0).slice(factors);
        double n_c_z_center = getCenter(n_c_z(0));
        double n_c_z_radius = getRadius(n_c_z(0));

        int idx_offset = 0;
        // Separation constraint
        force_constraint_ub[openmp_t_ind+idx_offset] = -1*f_c_z_center + f_c_z_radius;
        force_constraint_lb[openmp_t_ind+idx_offset] = -1*f_c_z_center - f_c_z_radius;

        idx_offset += NUM_TIME_STEPS;
        // slipping constraint
        if ( (f_c_x_center >= 0) && (f_c_y_center >= 0) && (f_c_z_center >= 0) ){
            // Note: double check that the center/radius is a number that can be squared
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs
        }
        // condition 2: y negative
        else if ( (f_c_x_center >= 0) && (f_c_y_center <= 0) && (f_c_z_center >= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

        }
        // condition 3: z negative
        else if ( (f_c_x_center >= 0) && (f_c_y_center >= 0) && (f_c_z_center <= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs
        }
        // condition 4: y and z negative
        else if ( (f_c_x_center >= 0) && (f_c_y_center <= 0) && (f_c_z_center <= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

        }
        // condition 5: x negative
        else if ( (f_c_x_center <= 0) && (f_c_y_center >= 0) && (f_c_z_center >= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs
        }
        // condition 6: x and y negative
        else if ( (f_c_x_center <= 0) && (f_c_y_center <= 0) && (f_c_z_center >= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs
        }
        // condition 7: x and z negative
        else if ( (f_c_x_center <= 0) && (f_c_y_center >= 0) && (f_c_z_center <= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs
        }
        // condition 8: x and y and z negative
        else if ( (f_c_x_center <= 0) && (f_c_y_center <= 0) && (f_c_z_center <= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs
        }

        idx_offset += NUM_TIME_STEPS;
        // tipping constraint

        // compute the numerator of the ZMP point equation
        Eigen::MatrixXd norm_vec(3,1);
        norm_vec << 0,0,1;
        PZsparse ZMP_top = cross(norm_vec,kd.n_c_int(openmp_t_ind));
        // extract the x, y and z components, slice by the parameters, then get the centers and radii of independent generators
        // x-component
        MatrixXInt ZMP_top_x = ZMP_top(0,0).slice(factors); // ->?
        double ZMP_top_x_center = getCenter(ZMP_top_x(0));
        double ZMP_top_x_radius = getRadius(ZMP_top_x(0));
        // y-component
        MatrixXInt ZMP_top_y = ZMP_top(1,0).slice(factors); // ->?
        double ZMP_top_y_center = getCenter(ZMP_top_y(0));
        double ZMP_top_y_radius = getRadius(ZMP_top_y(0));
        // z-component (for verification, this should be zero always.)
        MatrixXInt ZMP_top_z = ZMP_top(2,0).slice(factors); // ->?
        double ZMP_top_z_center = getCenter(ZMP_top_z(0));
        double ZMP_top_z_radius = getRadius(ZMP_top_z(0));

        // compute the denominator of the ZMP point equation
        MatrixXInt ZMP_bottom = kd.f_c_int(openmp_t_ind)(2,0).slice(factors);
        double ZMP_bottom_center = getCenter(ZMP_bottom(0));
        double ZMP_bottom_radius = getRadius(ZMP_bottom(0));

        // condition 1: all positive
        if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center >= 0) ){
            // Note: double check that the center/radius is a number that can be squared
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

        }
        // condition 2: y negative
        else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center >= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

        }
        // condition 3: z negative
        else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center <= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs
        }
        // condition 4: y and z negative
        else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center <= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs
        }
        // condition 5: x negative
        else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center >= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs
        }
        // condition 6: x and y negative
        else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center >= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs
        }
        // condition 7: x and z negative
        else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center <= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs
        }
        // condition 8: x and y and z negative
        else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center <= 0) ) {
            force_constraint_ub[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
            force_constraint_lb[openmp_t_ind+idx_offset] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs
        }
    }

/*
Section IV:
    Prepare output
*/

    // output FRS and other information, you can comment them if they are unnecessary

    // outputting joint position center
    std::ofstream outputstream2(outputfilename2);
    outputstream2 << std::setprecision(10);
    for (int i = 0; i < NUM_TIME_STEPS; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            for (int l = 0; l < 3; l++) {
                outputstream2 << link_sliced_center[i * NUM_JOINTS + j](l) << ' ';
            }
            outputstream2 << '\n';
        }
        outputstream2 << '\n';
    }
    outputstream2.close();

    // outputting joint position radii 
    std::ofstream outputstream3(outputfilename3);
    outputstream3 << std::setprecision(10);
    for (int i = 0; i < NUM_TIME_STEPS; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 3 + 3; l++) {
                    outputstream3 << link_independent_generators[i * NUM_JOINTS + j](k, l) << ' ';
                }
                outputstream3 << '\n';
            }
            outputstream3 << '\n';
        }
        outputstream3 << '\n';
    }
    outputstream3.close();

    // outputting the control input radius
    std::ofstream outputstream4(outputfilename4);
    outputstream4 << std::setprecision(10);
    for (int i = 0; i < NUM_TIME_STEPS; i++) {
        for (int j = 0; j < NUM_FACTORS; j++) {
            // outputstream4 << torque_radius(j, i) << ' '; // this is radius of final control input
            outputstream4 << kd.u_nom(j, i).independent(0) << ' '; // this is radius nominal torque
        }
        outputstream4 << '\n';
    }
    outputstream4.close();

    // outputting the constraint values (Really only the torque centers)
    std::ofstream outputstream5(outputfilename5);
    outputstream5 << std::setprecision(10);
    for (int i = 0; i < NUM_TIME_STEPS; i++) {
        for (int j = 0; j < NUM_FACTORS; j++) {
            outputstream5 << torque_sliced_center(j, i) << ' ';
        }
        outputstream5 << '\n';
    }
    outputstream5.close();

    // outputting wrench center values and radii
    std::ofstream outputstream6(outputfilename6);
    outputstream6 << std::setprecision(10);
    for (int i = 0; i < NUM_TIME_STEPS; i++) {
        for (int j = 0; j < 3; j++) {
            outputstream6 << force_value_center(j, i) << ' ';
        }
        for (int j = 0; j < 3; j++) {
            outputstream6 << moment_value_center(j, i) << ' '; // + 3 etc?
        }
        for (int j = 0; j < 3; j++) {
            outputstream6 << force_value_radii(j, i) << ' ';
        }
        for (int j = 0; j < 3; j++) {
            outputstream6 << moment_value_radii(j, i) << ' ';
        }
        outputstream6 << '\n';
    }
    outputstream6.close();

    // outputting contact constraint values
    std::ofstream outputstream7(outputfilename7);
    outputstream7 << std::setprecision(10);
    for (int i = 0; i < 3*NUM_TIME_STEPS; i++) {
        outputstream7 << force_constraint_ub[i] << ' ' << force_constraint_lb[i] << ' ';
        outputstream7 << '\n';
    }
    outputstream7.close();

    // outputting sliced desired trajectories
    std::ofstream outputstream8(outputfilename8);
    outputstream8 << std::setprecision(10);
    for (int j = 0; j < NUM_TIME_STEPS; j++){
        for (int i = 0; i < NUM_FACTORS; i++) {
            outputstream8 << qd_center(i,j) << ' ' << qd_radius(i,j) << ' ';
        }
        outputstream8 << '\n';
    }
    for (int j = 0; j < NUM_TIME_STEPS; j++){
        for (int i = 0; i < NUM_FACTORS; i++) {
            outputstream8 << qda_center(i,j) << ' ' << qda_radius(i,j) << ' ';
        }
        outputstream8 << '\n';
    }
    for (int j = 0; j < NUM_TIME_STEPS; j++){
        for (int i = 0; i < NUM_FACTORS; i++) {
            outputstream8 << qdda_center(i,j) << ' ' << qdda_radius(i,j) << ' ';
        }
        outputstream8 << '\n';
    }
    outputstream8.close();

    // for(int i=0;i<NUM_TIME_STEPS;i++){
    //     cout << qd_center(1,i) << endl;
    // }

}