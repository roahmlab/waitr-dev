#include "NLPclass.h"
#include "ReachsetsPath.h"
#include <cmath>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <memory>

namespace py = pybind11;


const std::string inputfilename = pathname + "armour.in";
const std::string outputfilename1 = pathname + "armour.out";
const std::string outputfilename2 = pathname + "armour_joint_position_center.out";
const std::string outputfilename3 = pathname + "armour_joint_position_radius.out";
const std::string outputfilename4 = pathname + "armour_control_input_radius.out";
const std::string outputfilename5 = pathname + "armour_constraints.out";


// struct ForceConstraints
// {
//     double u_s = 0.609382421; // 0.5; // static coefficient of friction between tray and object
//     double surf_rad =  0.058 / 2; // 0.0762; // RADIUS of contact area between tray and object (area assumed to be circular)
//     // NOTE: might want to change this to be input to the C++ code from matlab?
// };

class pzsparse {
    private:

        std::ofstream outputstream1;

        double q0[NUM_FACTORS] = {0.0};
        double qd0[NUM_FACTORS] = {0.0};
        double qdd0[NUM_FACTORS] = {0.0};
        double q_des[NUM_FACTORS] = {0.0};

        double t_plan = 1;

        int num_obstacles;
        double obstacles[MAX_OBSTACLE_NUM * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3];
        std::shared_ptr<Obstacles> O_ptr{nullptr};
        // Obstacles O;


        BezierCurve traj;
        PZsparse p[NUM_TIME_STEPS * NUM_FACTORS * 3];
        PZsparse u_nom[NUM_TIME_STEPS * NUM_FACTORS];
        double v_norm[NUM_TIME_STEPS * NUM_FACTORS];
        double jointPositionRadius[NUM_TIME_STEPS * NUM_FACTORS * 3];

        /** FORCE CONSTRAINTS */
        double u_s = 0.609382421; // 0.5; // static coefficient of friction between tray and object
        double surf_rad =  0.058 / 2; // 0.0762; // RADIUS of contact area between tray and object (area assumed to be circular)

        void set_obstacles(py::array_t<double> obstacle_vec){
            auto obstacle_ = obstacle_vec.unchecked<2>();
            num_obstacles = obstacle_.shape(0);
            int row = 0;
            int col = 0;
            int obs_dim = (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3;
            cout << "Obstacles set to:" << endl;
            for (int i = 0; i < num_obstacles * obs_dim; i++) {
                row = i / obs_dim;
                col = i % obs_dim;
                obstacles[i] = obstacle_(row, col);
            }
            for (int i = 0; i <= row; i++){
                cout << "obstacle" << i+1 << ": [ ";
                for (int j = 0; j <= col; j++){
                    cout << obstacles[i*obs_dim + j] << ' ';
                }
                cout << "] \n";
            }
            cout << '\n' << endl;
            std::cout << "allocating obstacles..." << std::endl;
            O_ptr = std::make_unique<Obstacles>();
            O_ptr->initialize(obstacles, num_obstacles);
            // O.initialize(obstacles, num_obstacles);
            std::cout << "Obstacles allocated!" << std::endl;
        }

        void set_goal(py::array_t<double> qdes_vec){
            auto qdes_ = qdes_vec.unchecked<1>();
            cout << "Goal set to: \n [";
            for (int i = 0; i < NUM_FACTORS; i++) {
                q_des[i] = qdes_(i);
                cout << q_des[i] << ' ';
            }
            cout << "]" << endl;
        }

        void set_state(py::array_t<double> q0_vec, py::array_t<double> qd0_vec, py::array_t<double> qdd0_vec){
            auto q0_ = q0_vec.unchecked<1>();
            auto qd0_ = qd0_vec.unchecked<1>();
            auto qdd0_ = qdd0_vec.unchecked<1>();
            cout << "States set to:" << endl;
            cout << "[q0,     qd0,     qdd0]" << endl;
            for (int i = 0; i < NUM_FACTORS; i++){
                q0[i] = q0_(i);
                qd0[i] = qd0_(i);
                qdd0[i] = qdd0_(i);
                cout << q0[i] << ' ' << qd0[i] << ' ' << qdd0[i] << endl;
            }
        }

        // void write_reachset(SmartPtr<armtd_NLP> mynlp){
        //     cout << "Saving reachsets..." << endl;
        //     for (int link = 0; link < 7; link++){
        //         std::ofstream file(reachset+std::to_string(link+1)+".txt");
        //         file << std::setprecision(8);
        //         for (int point = 0; point < 8; point++){
        //             int axis1 = static_cast<int>(point / 4);
        //             int axis2 = static_cast<int>(point / 2);
        //             int axis3 = point;
        //             int axis[] = {axis1, axis2, axis3};
        //             for (int i = 0; i < NUM_TIME_STEPS; i++) {
        //                 for (int j = 0; j < 3; j++) {
        //                     file << mynlp->checkJointsPosition[(i * NUM_FACTORS + link) * 3 + j] + (link_radius[link][0]+jointPositionRadius[(i * NUM_FACTORS + link) * 3 + j])*std::pow(-1, axis[j])<< ',';
        //                 }
        //                 file << '\n';
        //             }
        //         }
        //         file.close();
        //     }
        //     // write the origin
        //     std::ofstream file(reachset+"0.txt");
        //     for (int point = 0; point<8; point++){
        //         int axis1 = static_cast<int>(point / 4);
        //         int axis2 = static_cast<int>(point / 2);
        //         int axis3 = point;
        //         int axis[] = {axis1, axis2, axis3};
        //         for (int j = 0; j < 3; j++) {
        //             file << (0.04)*std::pow(-1, axis[j])<< ',';
        //         }
        //         file << '\n';
        //     }
        // }

    public:

        pzsparse(py::array_t<double> obs_vec): num_obstacles(0){
            outputstream1 = std::ofstream(outputfilename1);

            set_obstacles(obs_vec);
        }

        ~pzsparse()
        {
            outputstream1.close();
        }

        const int getNumObstacles(){
            return num_obstacles;
        }

        py::array_t<double> optimize(py::array_t<double> q0_vec, py::array_t<double> qd0_vec, py::array_t<double> qdd0_vec,
                                    py::array_t<double> qgoal_vec) {


            set_goal(qgoal_vec);
            set_state(q0_vec, qd0_vec, qdd0_vec);

            auto start1 = std::chrono::high_resolution_clock::now();

            // Create JRS online
            traj = BezierCurve(q0, qd0, qdd0);
            omp_set_num_threads(NUM_THREADS);
            int openmp_t_ind = 0; // openmp loop index

            try {
                #pragma omp parallel for shared(traj) private(openmp_t_ind) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
                for(openmp_t_ind = 0; openmp_t_ind < NUM_TIME_STEPS; openmp_t_ind++) {
                    traj.makePolyZono(openmp_t_ind);
                }
            }
            catch (int errorCode) {
                WARNING_PRINT("        CUDA & C++: Error creating JRS! Check previous error message!");
                throw;
            }

            // Compute link PZs and nominal torque PZs
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
                        // kd.u_nom_int(i, openmp_t_ind) = 0;
                    }

                    // reduce non-only-k-dependent generators so that slice takes less time
                    for (int i = 0; i < NUM_FACTORS; i++) {
                        kd.u_nom(i, openmp_t_ind).reduce();
                    }
                }
            }
            catch (int errorCode) {
                WARNING_PRINT("        CUDA & C++: Error computing link PZs and nominal torque PZs! Check previous error message!");
                throw;
            }

            // Compute robust input bound
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
                        // torque_radius(i, t_ind) = 0;
                    }
                    rho_max_temp = sqrt(rho_max_temp);

                    for (int i = 0; i < NUM_FACTORS; i++) {
                        torque_radius(i, t_ind) += 0.5 * rho_max_temp.upper();
                        // torque_radius(i, t_ind) += 0;
                    }

                    // (2) add the radius of the nominal input PZ (after reducing)
                    for (int i = 0; i < NUM_FACTORS; i++) {
                        torque_radius(i, t_ind) += kd.u_nom(i, t_ind).independent(0);
                        // torque_radius(i, t_ind) += 0;
                    }

                    // (3) add friction
                    for (int i = 0; i < NUM_FACTORS; i++) {
                        torque_radius(i, t_ind) += friction[i];
                        // torque_radius(i, t_ind) += 0;
                    }

                    // so that torque_radius would be the radius of the total control input PZ from now
                }
            }
            catch (int errorCode) {
                WARNING_PRINT("        CUDA & C++: Error computing torque PZs! Check previous error message!");
                throw;
            }

            // Buffer obstacles and initialize collision checking hyperplanes
            try {
                O_ptr->initializeHyperPlane(link_independent_generators);
                // O.initializeHyperPlane(link_independent_generators);
            }
            catch (int errorCode) {
                WARNING_PRINT("        CUDA & C++: Error initializing collision checking hyperplanes! Check previous error message!");
                throw;
            }

            auto stop1 = std::chrono::high_resolution_clock::now();
            auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
            cout << "        CUDA & C++: Time taken by generating reachable sets: " << duration1.count() << " milliseconds" << endl;

            // Solve optimization
            auto start2 = std::chrono::high_resolution_clock::now();

            SmartPtr<armtd_NLP> mynlp = new armtd_NLP();
            try {
                mynlp->set_parameters(q_des, t_plan, &traj, &kd, &torque_radius, O_ptr.get(), u_s, surf_rad);
            }
            catch (int errorCode) {
                WARNING_PRINT("        CUDA & C++: Error initializing Ipopt! Check previous error message!");
                throw;
            }

            SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

            app->Options()->SetNumericValue("tol", IPOPT_OPTIMIZATION_TOLERANCE);
            app->Options()->SetNumericValue("max_cpu_time", IPOPT_MAX_CPU_TIME);
            app->Options()->SetIntegerValue("print_level", IPOPT_PRINT_LEVEL);
            app->Options()->SetStringValue("mu_strategy", IPOPT_MU_STRATEGY);
            app->Options()->SetStringValue("linear_solver", IPOPT_LINEAR_SOLVER);
            app->Options()->SetStringValue("hessian_approximation", "limited-memory");

            // For gradient checking
            // app->Options()->SetStringValue("output_file", "ipopt.out");
            // app->Options()->SetStringValue("derivative_test", "first-order");
            // app->Options()->SetNumericValue("derivative_test_perturbation", 1e-8);
            // app->Options()->SetNumericValue("derivative_test_tol", 1e-6);

            // Initialize the IpoptApplication and process the options
            ApplicationReturnStatus status;
            status = app->Initialize();
            if( status != Solve_Succeeded ) {
                WARNING_PRINT("Error during initialization!");
                outputstream1 << -1 << '\n';
                outputstream1.close();
                throw;
            }

            try {
                // Ask Ipopt to solve the problem
                status = app->OptimizeTNLP(mynlp);
            }
            catch (int errorCode) {
                WARNING_PRINT("        CUDA & C++: Error solving optimization problem! Check previous error message!");
                throw;
            }

            auto stop2 = std::chrono::high_resolution_clock::now();
            auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - start2);

            if (status == Maximum_CpuTime_Exceeded) {
                cout << "        CUDA & C++: Ipopt maximum CPU time exceeded!\n";
            }

            if (status == Invalid_Option) {
                cout << "        CUDA & C++: Cannot find HSL library! Need to put libcoinhsl.so in proper path!\n";
            }
            else {
                cout << "        CUDA & C++: Time taken by Ipopt: " << duration2.count() << " milliseconds" << endl;
            }

            auto start3 = std::chrono::high_resolution_clock::now();

            // set precision to 10 decimal digits
            outputstream1 << std::setprecision(10);

            // output k_opt
            auto k_opt = py::array_t<double>(NUM_FACTORS);
            double *k_opt_ptr = static_cast<double *>(k_opt.request().ptr);

            cout << "k_opt: \n [ ";
            if (mynlp->feasible) {
                for (int i = 0; i < NUM_FACTORS; i++) {
                    cout << mynlp->solution[i] << ' ';
                    k_opt_ptr[i] = mynlp->solution[i] * k_range[i];
                }
            }
            else {
                for (int i = 0; i < NUM_FACTORS; i++) {
                    k_opt_ptr[i] = 0;
                }
                cout << "No feasible solution. ";
            }
            cout << "]" << endl;

            // output time cost (in milliseconds) in C++
            // outputstream1 << duration1.count() + duration2.count();
            // outputstream1.close();

            // output FRS and other information, you can comment them if they are unnecessary
            cout << "saving reach sets ..." << endl;
            std::ofstream outputstream2(outputfilename2);
            outputstream2 << std::setprecision(10);
            for (int i = 0; i < NUM_TIME_STEPS; i++) {
                for (int j = 0; j < NUM_JOINTS; j++) {
                    for (int l = 0; l < 3; l++) {
                        outputstream2 << mynlp->link_sliced_center[i * NUM_JOINTS + j](l) << ' ';
                    }
                    outputstream2 << '\n';
                }
                outputstream2 << '\n';
            }
            outputstream2.close();

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

            // std::ofstream outputstream4(outputfilename4);
            // outputstream4 << std::setprecision(10);
            // for (int i = 0; i < NUM_TIME_STEPS; i++) {
            //     for (int j = 0; j < NUM_FACTORS; j++) {
            //         outputstream4 << torque_radius(j, i) << ' '; // this is radius of final control input
            //     }
            //     outputstream4 << '\n';
            // }
            // outputstream4.close();

            // std::ofstream outputstream5(outputfilename5);
            // outputstream5 << std::setprecision(6);
            // for (int i = 0; i < mynlp->constraint_number; i++) {
            //     outputstream5 << mynlp->g_copy[i] << '\n';
            // }
            // outputstream5.close();

            auto stop3 = std::chrono::high_resolution_clock::now();
            auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(stop3 - start3);
            auto duration4 = std::chrono::duration_cast<std::chrono::milliseconds>(stop3 - start1);

            cout << "        CUDA & C++: Time taken by saving information: " << duration3.count() << " milliseconds" << endl;
            cout << "        CUDA & C++: Time taken overall: " << duration4.count() << " milliseconds" << endl;

            return k_opt;
        }

        py::array_t<double> getDesTraj(py::array_t<double> q0, py::array_t<double> qd0, py::array_t<double> qdd0, py::array_t<double> k, double t){
            // return desired trajectory: [q, qd, qdd]
            int size = NUM_FACTORS * 3;
            auto traj_d = py::array_t<double>(size);
            double *traj_d_ptr = static_cast<double *>(traj_d.request().ptr);

            auto q0_ = q0.unchecked<1>();
            auto qd0_ = qd0.unchecked<1>();
            auto qdd0_ = qdd0.unchecked<1>();
            auto k_ = k.unchecked<1>();

            for (int i = 0; i < NUM_FACTORS; i++){
                traj_d_ptr[3*i] = q_des_func(q0_(i), qd0_(i), qdd0_(i), k_(i), t);
                traj_d_ptr[3*i+1] = qd_des_func(q0_(i), qd0_(i), qdd0_(i), k_(i), t);
                traj_d_ptr[3*i+2] = qdd_des_func(q0_(i), qd0_(i), qdd0_(i), k_(i), t);
            }

            return traj_d;
        }

        void free()
        {
            O_ptr.reset();
        }

};

PYBIND11_MODULE(armour_main_pybind, m) {
    py::class_<pzsparse>(m, "pzsparse")
        .def(py::init<py::array_t<double> &>())
        .def("getNumObstacles", &pzsparse::getNumObstacles)
        .def("optimize", &pzsparse::optimize)
        .def("getDesTraj", &pzsparse::getDesTraj);
        // .def("free", &pzsparse::free);
}
