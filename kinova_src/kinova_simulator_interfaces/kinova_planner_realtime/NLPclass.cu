#ifndef NLP_CLASS_CU
#define NLP_CLASS_CU

#include "NLPclass.h"

double wrap_to_pi(const double angle) {
    double wrapped_angle = angle;
    while (wrapped_angle < -M_PI) {
        wrapped_angle += 2*M_PI;
    }
    while (wrapped_angle > M_PI) {
        wrapped_angle -= 2*M_PI;
    }
    return wrapped_angle;
}

// constructor
armtd_NLP::armtd_NLP()
{
}


// destructor
armtd_NLP::~armtd_NLP()
{
    delete[] g_copy;
}


bool armtd_NLP::set_parameters(
    Eigen::VectorXd& q_des_input,
    double t_plan_input,
    const BezierCurve* desired_trajectory_input,
    KinematicsDynamics* kinematics_dynamics_result_input,
    const Eigen::MatrixXd* torque_radius_input,
    Obstacles* obstacles_input,
    double u_s_input,
    double surf_rad_input
 ) 
 {
    q_des = q_des_input;
    t_plan = t_plan_input;
    desired_trajectory = desired_trajectory_input;
    kinematics_dynamics_result = kinematics_dynamics_result_input;
    torque_radius = torque_radius_input;
    obstacles = obstacles_input;
    u_s = u_s_input;
    surf_rad = surf_rad_input;

    constraint_number = NUM_FACTORS * NUM_TIME_STEPS +
                        3*NUM_TIME_STEPS +
                        NUM_JOINTS * NUM_TIME_STEPS * obstacles->num_obstacles + 
                        NUM_FACTORS * 4;

    g_copy = new Number[constraint_number];

    return true;
}


bool armtd_NLP::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
    // The problem described NUM_FACTORS variables, x[NUM_FACTORS] through x[NUM_FACTORS] for each joint
    n = NUM_FACTORS;

    // number of inequality constraint
    m = constraint_number;

    nnz_jac_g = m * n;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return true;
}
// [TNLP_get_nlp_info]

// [TNLP_get_bounds_info]
// returns the variable bounds
bool armtd_NLP::get_bounds_info(
   Index   n,
   Number* x_l,
   Number* x_u,
   Index   m,
   Number* g_l,
   Number* g_u
)
{
    // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
    // If desired, we could assert to make sure they are what we think they are.
    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in get_bounds_info!");
    }
    if(m != constraint_number){
        WARNING_PRINT("*** Error wrong value of m in get_bounds_info!");
    }

    // lower bounds
    for( Index i = 0; i < n; i++ ) {
        x_l[i] = -1.0;
    }

    // upper bounds  
    for( Index i = 0; i < n; i++ ) {
        x_u[i] = 1.0;
    }

    // control input constraints
    for( Index i = 0; i < NUM_TIME_STEPS; i++ ) {
        for( Index j = 0; j < NUM_FACTORS; j++ ) {
            g_l[i * NUM_FACTORS + j] = -torque_limits[j] + (*torque_radius)(j, i);
            g_u[i * NUM_FACTORS + j] = torque_limits[j] - (*torque_radius)(j, i);
        }
    }    
    Index offset = NUM_FACTORS * NUM_TIME_STEPS;

    //     separation constraint
    // upper bound should be zero and lower bound should be -inf
    for( Index i = offset; i < offset + NUM_TIME_STEPS; i++){
        g_l[i] = -1e19;
        g_u[i] = 0;
    }
    offset += NUM_TIME_STEPS;

    //     slipping constraint
    // upper bound should be zero and lower bound should be -inf for the reformulated constraint (not the normal friction law?)
    for( Index i = offset; i < offset + NUM_TIME_STEPS; i++){
        g_l[i] = -1e19;
        g_u[i] = 0;
    }
    offset += NUM_TIME_STEPS;

    //     tipping constraint
    // upper bound should be zero and lower bound should be -inf for the reformulated constraint
    for( Index i = offset; i < offset + NUM_TIME_STEPS; i++){
        g_l[i] = -1e19;
        g_u[i] = 0;
    }
    offset += NUM_TIME_STEPS;

    // collision avoidance constraints
    for( Index i = offset; i < offset + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles; i++ ) {
        g_l[i] = -1e19;
        g_u[i] = 0;
    }
    offset += NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles;

    // state limit constraints
    //     minimum joint position
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        g_l[i] = state_limits_lb[i - offset] + qe;
        g_u[i] = state_limits_ub[i - offset] - qe;
    }
    offset += NUM_FACTORS;

    //     maximum joint position
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        g_l[i] = state_limits_lb[i - offset] + qe;
        g_u[i] = state_limits_ub[i - offset] - qe;
    }
    offset += NUM_FACTORS;

    //     minimum joint velocity
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        g_l[i] = -speed_limits[i - offset] + qde;
        g_u[i] = speed_limits[i - offset] - qde;
    }
    offset += NUM_FACTORS;

    //     maximum joint velocity
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        g_l[i] = -speed_limits[i - offset] + qde;
        g_u[i] = speed_limits[i - offset] - qde;
    }

    return true;
}
// [TNLP_get_bounds_info]

// [TNLP_get_starting_point]
// returns the initial point for the problem
bool armtd_NLP::get_starting_point(
    Index   n,
    bool    init_x,
    Number* x,
    bool    init_z,
    Number* z_L,
    Number* z_U,
    Index   m,
    bool    init_lambda,
    Number* lambda
)
{
    // Here, we assume we only have starting values for x, if you code
    // your own NLP, you can provide starting values for the dual variables
    // if you wish
    if(init_x == false || init_z == true || init_lambda == true){
        WARNING_PRINT("*** Error wrong value of init in get_starting_point!");
    }

    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in get_starting_point!");
    }

    for( Index i = 0; i < n; i++ ) {
        // initialize to zero
        // x[i] = 0.0;

        // try to avoid local minimum
        x[i] = min(max((q_des[i] - desired_trajectory->q0[i]) / k_range[i], -0.25), 0.25);
    }

    return true;
}
// [TNLP_get_starting_point]

void armtd_NLP::compute(
    bool new_x,
    const Number* x
){

    // check if a new x is passed in
    if (new_x){
        // timing
        // auto start_compute = std::chrono::high_resolution_clock::now();

        // update values

        // compute the constraint values

        // Contact Force Constraints
        Index i;
        #pragma omp parallel for shared(kinematics_dynamics_result, x, link_sliced_center) private(i) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
        for(i = 0; i<NUM_TIME_STEPS; i++){

            for (int m = 0; m < 3; m++) {
                MatrixXInt res1 = kinematics_dynamics_result -> f_c_int(i)(m,0).slice(x);
                force_value_center(m,i) = getCenter(res1(0));
                force_value_radii(m,i) = getRadius(res1(0));
                MatrixXInt res2 = kinematics_dynamics_result -> n_c_int(i)(m,0).slice(x);
                moment_value_center(m,i) = getCenter(res2(0));
                moment_value_radii(m,i) = getRadius(res2(0));
            }

            // Extract the force PZs, slice, and get the centers and radii
            MatrixXInt f_c_x = kinematics_dynamics_result -> f_c_int(i)(0,0).slice(x);
            Number f_c_x_center = getCenter(f_c_x(0));
            Number f_c_x_radius = getRadius(f_c_x(0));

            MatrixXInt f_c_y = kinematics_dynamics_result -> f_c_int(i)(1,0).slice(x);
            Number f_c_y_center = getCenter(f_c_y(0));
            Number f_c_y_radius = getRadius(f_c_y(0));

            MatrixXInt f_c_z = kinematics_dynamics_result -> f_c_int(i)(2,0).slice(x);
            Number f_c_z_center = getCenter(f_c_z(0));
            Number f_c_z_radius = getRadius(f_c_z(0));

            // Extract the moment PZs
            MatrixXInt n_c_x = kinematics_dynamics_result -> n_c_int(i)(0,0).slice(x);
            Number n_c_x_center = getCenter(n_c_x(0));
            Number n_c_x_radius = getRadius(n_c_x(0));

            MatrixXInt n_c_y = kinematics_dynamics_result -> n_c_int(i)(1,0).slice(x);
            Number n_c_y_center = getCenter(n_c_y(0));
            Number n_c_y_radius = getRadius(n_c_y(0));

            MatrixXInt n_c_z = kinematics_dynamics_result -> n_c_int(i)(2,0).slice(x);
            Number n_c_z_center = getCenter(n_c_z(0));
            Number n_c_z_radius = getRadius(n_c_z(0));

            // compute the numerator of the ZMP point equation
            Eigen::MatrixXd norm_vec(3,1);
            norm_vec << 0,0,1;
            PZsparse ZMP_top = cross(norm_vec,kinematics_dynamics_result->n_c_int(i));
            // extract the x, y and z components, slice by the parameters, then get the centers and radii of independent generators
            // x-component
            MatrixXInt ZMP_top_x = ZMP_top(0,0).slice(x); // ->?
            Number ZMP_top_x_center = getCenter(ZMP_top_x(0));
            Number ZMP_top_x_radius = getRadius(ZMP_top_x(0));
            // y-component
            MatrixXInt ZMP_top_y = ZMP_top(1,0).slice(x); // ->?
            Number ZMP_top_y_center = getCenter(ZMP_top_y(0));
            Number ZMP_top_y_radius = getRadius(ZMP_top_y(0));
            // z-component (for verification, this should be zero always.)
            MatrixXInt ZMP_top_z = ZMP_top(2,0).slice(x); // ->?
            Number ZMP_top_z_center = getCenter(ZMP_top_z(0));
            Number ZMP_top_z_radius = getRadius(ZMP_top_z(0));

            // compute the denominator of the ZMP point equation
            MatrixXInt ZMP_bottom = kinematics_dynamics_result->f_c_int(i)(2,0).slice(x);
            Number ZMP_bottom_center = getCenter(ZMP_bottom(0));
            Number ZMP_bottom_radius = getRadius(ZMP_bottom(0));


            // constraints

            // Separation constraint
            force_constraint_ub[i] = -1*f_c_z_center + f_c_z_radius;
            force_constraint_lb[i] = -1*f_c_z_center - f_c_z_radius;

            Index idx_offset2 = NUM_TIME_STEPS;

            // slipping constraint
            if ( (f_c_x_center >= 0) && (f_c_y_center >= 0) && (f_c_z_center >= 0) ){
                // Note: double check that the center/radius is a number that can be squared
                force_constraint_ub[i+idx_offset2] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

            }
            // condition 2: y negative
            else if ( (f_c_x_center >= 0) && (f_c_y_center <= 0) && (f_c_z_center >= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

            }
            // condition 3: z negative
            else if ( (f_c_x_center >= 0) && (f_c_y_center >= 0) && (f_c_z_center <= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

            }
            // condition 4: y and z negative
            else if ( (f_c_x_center >= 0) && (f_c_y_center <= 0) && (f_c_z_center <= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

            }
            // condition 5: x negative
            else if ( (f_c_x_center <= 0) && (f_c_y_center >= 0) && (f_c_z_center >= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

            }
            // condition 6: x and y negative
            else if ( (f_c_x_center <= 0) && (f_c_y_center <= 0) && (f_c_z_center >= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

            }
            // condition 7: x and z negative
            else if ( (f_c_x_center <= 0) && (f_c_y_center >= 0) && (f_c_z_center <= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

            }
            // condition 8: x and y and z negative
            else if ( (f_c_x_center <= 0) && (f_c_y_center <= 0) && (f_c_z_center <= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(f_c_x_center,2) - 2*f_c_x_radius*f_c_x_center + pow(f_c_x_radius,2) + pow(f_c_y_center,2) - 2*f_c_y_radius*f_c_y_center + pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) + 2*f_c_z_radius*f_c_z_center - pow(f_c_z_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(f_c_x_center,2) + 2*f_c_x_radius*f_c_x_center - pow(f_c_x_radius,2) + pow(f_c_y_center,2) + 2*f_c_y_radius*f_c_y_center - pow(f_c_y_radius,2) - pow(u_s,2) * ( pow(f_c_z_center,2) - 2*f_c_z_radius*f_c_z_center + pow(f_c_z_radius,2)); // checked signs

            }

            idx_offset2 += NUM_TIME_STEPS;
            // tipping constraint

            // condition 1: all positive
            if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center >= 0) ){
                // Note: double check that the center/radius is a number that can be squared
                force_constraint_ub[i+idx_offset2] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

            }
            // condition 2: y negative
            else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center >= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

            }
            // condition 3: z negative
            else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center <= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

            }
            // condition 4: y and z negative
            else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center <= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

            }
            // condition 5: x negative
            else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center >= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

            }
            // condition 6: x and y negative
            else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center >= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

            }
            // condition 7: x and z negative
            else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center <= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

            }
            // condition 8: x and y and z negative
            else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center <= 0) ) {
                force_constraint_ub[i+idx_offset2] = pow(ZMP_top_x_center,2) - 2*ZMP_top_x_radius*ZMP_top_x_center + pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) - 2*ZMP_top_y_radius*ZMP_top_y_center + pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) + 2*ZMP_bottom_radius*ZMP_bottom_center - pow(ZMP_bottom_radius,2)); // checked signs
                force_constraint_lb[i+idx_offset2] = pow(ZMP_top_x_center,2) + 2*ZMP_top_x_radius*ZMP_top_x_center - pow(ZMP_top_x_radius,2) + pow(ZMP_top_y_center,2) + 2*ZMP_top_y_radius*ZMP_top_y_center - pow(ZMP_top_y_radius,2) - pow(surf_rad,2) * ( pow(ZMP_bottom_center,2) - 2*ZMP_bottom_radius*ZMP_bottom_center + pow(ZMP_bottom_radius,2)); // checked signs

            }


            // calculate the gradient of the constraints

            // to do: check the index offsets and clean up code and add public variable to store the gradient values
            // to do: remove duplicate code from other functions and properly access and assign these variables in those functions

            // Contact Force Constraints

            // gradients
            // storage for the gradients
            Number f_c_x_grad[NUM_FACTORS];
            Number f_c_y_grad[NUM_FACTORS];
            Number f_c_z_grad[NUM_FACTORS];
            Number ZMP_top_x_grad[NUM_FACTORS];
            Number ZMP_top_y_grad[NUM_FACTORS];
            Number ZMP_bottom_grad[NUM_FACTORS];
            // calculate the gradients
            kinematics_dynamics_result->f_c_int(i)(0,0).slice(f_c_x_grad, x);
            kinematics_dynamics_result->f_c_int(i)(1,0).slice(f_c_y_grad, x);
            kinematics_dynamics_result->f_c_int(i)(2,0).slice(f_c_z_grad, x);
            ZMP_top(0,0).slice(ZMP_top_x_grad, x);
            ZMP_top(1,0).slice(ZMP_top_y_grad, x);
            kinematics_dynamics_result->f_c_int(i)(2,0).slice(ZMP_bottom_grad, x); // same as f_c_z_grad?

            // Separation constraint gradient
            for (int j = 0;j<NUM_FACTORS;j++) {
                force_constraint_gradient[i*NUM_FACTORS+j] = -1*f_c_z_grad[j];
            }

            Index grad_idx_offset = NUM_TIME_STEPS*NUM_FACTORS;
            // Slipping Constraint
            // calculate constraint gradient, depends on the signs of the centers like constraint itself does.
            if ( (f_c_x_center >= 0) && (f_c_y_center >= 0) && (f_c_z_center >= 0) ){
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*f_c_x_center*f_c_x_grad[j] + 2*f_c_x_radius*f_c_x_grad[j] + 2*f_c_y_center*f_c_y_grad[j] + 2*f_c_y_radius*f_c_y_grad[j] - pow(u_s,2) * ( 2*f_c_z_center*f_c_z_grad[j] - 2*f_c_z_radius*f_c_z_grad[j] );
                }
            }
            // condition 2: y negative
            else if ( (f_c_x_center >= 0) && (f_c_y_center <= 0) && (f_c_z_center >= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*f_c_x_center*f_c_x_grad[j] + 2*f_c_x_radius*f_c_x_grad[j] + 2*f_c_y_center*f_c_y_grad[j] - 2*f_c_y_radius*f_c_y_grad[j] - pow(u_s,2) * ( 2*f_c_z_center*f_c_z_grad[j] - 2*f_c_z_radius*f_c_z_grad[j] );
                }
            }
            // condition 3: z negative
            else if ( (f_c_x_center >= 0) && (f_c_y_center >= 0) && (f_c_z_center <= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*f_c_x_center*f_c_x_grad[j] + 2*f_c_x_radius*f_c_x_grad[j] + 2*f_c_y_center*f_c_y_grad[j] + 2*f_c_y_radius*f_c_y_grad[j] - pow(u_s,2) * ( 2*f_c_z_center*f_c_z_grad[j] + 2*f_c_z_radius*f_c_z_grad[j] );
                }
            }
            // condition 4: y and z negative
            else if ( (f_c_x_center >= 0) && (f_c_y_center <= 0) && (f_c_z_center <= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*f_c_x_center*f_c_x_grad[j] + 2*f_c_x_radius*f_c_x_grad[j] + 2*f_c_y_center*f_c_y_grad[j] - 2*f_c_y_radius*f_c_y_grad[j] - pow(u_s,2) * ( 2*f_c_z_center*f_c_z_grad[j] + 2*f_c_z_radius*f_c_z_grad[j] );
                }
            }
            // condition 5: x negative
            else if ( (f_c_x_center <= 0) && (f_c_y_center >= 0) && (f_c_z_center >= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*f_c_x_center*f_c_x_grad[j] - 2*f_c_x_radius*f_c_x_grad[j] + 2*f_c_y_center*f_c_y_grad[j] + 2*f_c_y_radius*f_c_y_grad[j] - pow(u_s,2) * ( 2*f_c_z_center*f_c_z_grad[j] - 2*f_c_z_radius*f_c_z_grad[j] );
                }
            }
            // condition 6: x and y negative
            else if ( (f_c_x_center <= 0) && (f_c_y_center <= 0) && (f_c_z_center >= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*f_c_x_center*f_c_x_grad[j] - 2*f_c_x_radius*f_c_x_grad[j] + 2*f_c_y_center*f_c_y_grad[j] - 2*f_c_y_radius*f_c_y_grad[j] - pow(u_s,2) * ( 2*f_c_z_center*f_c_z_grad[j] - 2*f_c_z_radius*f_c_z_grad[j] );
                }
            }
            // condition 7: x and z negative
            else if ( (f_c_x_center <= 0) && (f_c_y_center >= 0) && (f_c_z_center <= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*f_c_x_center*f_c_x_grad[j] - 2*f_c_x_radius*f_c_x_grad[j] + 2*f_c_y_center*f_c_y_grad[j] + 2*f_c_y_radius*f_c_y_grad[j] - pow(u_s,2) * ( 2*f_c_z_center*f_c_z_grad[j] + 2*f_c_z_radius*f_c_z_grad[j] );
                }
            }
            // condition 8: x and y and z negative
            else if ( (f_c_x_center <= 0) && (f_c_y_center <= 0) && (f_c_z_center <= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*f_c_x_center*f_c_x_grad[j] - 2*f_c_x_radius*f_c_x_grad[j] + 2*f_c_y_center*f_c_y_grad[j] - 2*f_c_y_radius*f_c_y_grad[j] - u_s*u_s * ( 2*f_c_z_center*f_c_z_grad[j] + 2*f_c_z_radius*f_c_z_grad[j] );
                }
            }

            //    tipping constraint

            grad_idx_offset += NUM_TIME_STEPS*NUM_FACTORS;
            // calculate constraint gradient
            if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center >= 0) ){
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*ZMP_top_x_center*ZMP_top_x_grad[j] + 2*ZMP_top_x_radius*ZMP_top_x_grad[j] + 2*ZMP_top_y_center*ZMP_top_y_grad[j] + 2*ZMP_top_y_radius*ZMP_top_y_grad[j] - pow(surf_rad,2) * ( 2*ZMP_bottom_center*ZMP_bottom_grad[j] - 2*ZMP_bottom_radius*ZMP_bottom_grad[j]);
                }
            }
            // condition 2: y negative
            else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center >= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*ZMP_top_x_center*ZMP_top_x_grad[j] + 2*ZMP_top_x_radius*ZMP_top_x_grad[j] + 2*ZMP_top_y_center*ZMP_top_y_grad[j] - 2*ZMP_top_y_radius*ZMP_top_y_grad[j] - pow(surf_rad,2) * ( 2*ZMP_bottom_center*ZMP_bottom_grad[j] - 2*ZMP_bottom_radius*ZMP_bottom_grad[j]);
                }
            }
            // condition 3: z negative
            else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center <= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*ZMP_top_x_center*ZMP_top_x_grad[j] + 2*ZMP_top_x_radius*ZMP_top_x_grad[j] + 2*ZMP_top_y_center*ZMP_top_y_grad[j] + 2*ZMP_top_y_radius*ZMP_top_y_grad[j] - pow(surf_rad,2) * ( 2*ZMP_bottom_center*ZMP_bottom_grad[j] + 2*ZMP_bottom_radius*ZMP_bottom_grad[j]);
                }
            }
            // condition 4: y and z negative
            else if ( (ZMP_top_x_center >= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center <= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*ZMP_top_x_center*ZMP_top_x_grad[j] + 2*ZMP_top_x_radius*ZMP_top_x_grad[j] + 2*ZMP_top_y_center*ZMP_top_y_grad[j] - 2*ZMP_top_y_radius*ZMP_top_y_grad[j] - pow(surf_rad,2) * ( 2*ZMP_bottom_center*ZMP_bottom_grad[j] + 2*ZMP_bottom_radius*ZMP_bottom_grad[j]);
                }
            }
            // condition 5: x negative
            else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center >= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*ZMP_top_x_center*ZMP_top_x_grad[j] - 2*ZMP_top_x_radius*ZMP_top_x_grad[j] + 2*ZMP_top_y_center*ZMP_top_y_grad[j] + 2*ZMP_top_y_radius*ZMP_top_y_grad[j] - pow(surf_rad,2) * ( 2*ZMP_bottom_center*ZMP_bottom_grad[j] - 2*ZMP_bottom_radius*ZMP_bottom_grad[j]);
                }
            }
            // condition 6: x and y negative
            else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center >= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*ZMP_top_x_center*ZMP_top_x_grad[j] - 2*ZMP_top_x_radius*ZMP_top_x_grad[j] + 2*ZMP_top_y_center*ZMP_top_y_grad[j] - 2*ZMP_top_y_radius*ZMP_top_y_grad[j] - pow(surf_rad,2) * ( 2*ZMP_bottom_center*ZMP_bottom_grad[j] - 2*ZMP_bottom_radius*ZMP_bottom_grad[j]);
                }
            }
            // condition 7: x and z negative
            else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center >= 0) && (ZMP_bottom_center <= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*ZMP_top_x_center*ZMP_top_x_grad[j] - 2*ZMP_top_x_radius*ZMP_top_x_grad[j] + 2*ZMP_top_y_center*ZMP_top_y_grad[j] + 2*ZMP_top_y_radius*ZMP_top_y_grad[j] - pow(surf_rad,2) * ( 2*ZMP_bottom_center*ZMP_bottom_grad[j] + 2*ZMP_bottom_radius*ZMP_bottom_grad[j]);
                }
            }
            // condition 8: x and y and z negative
            else if ( (ZMP_top_x_center <= 0) && (ZMP_top_y_center <= 0) && (ZMP_bottom_center <= 0) ) {
                for (int j=0;j<NUM_FACTORS;j++) {
                    force_constraint_gradient[i*NUM_FACTORS+grad_idx_offset+j] = 2*ZMP_top_x_center*ZMP_top_x_grad[j] - 2*ZMP_top_x_radius*ZMP_top_x_grad[j] + 2*ZMP_top_y_center*ZMP_top_y_grad[j] - 2*ZMP_top_y_radius*ZMP_top_y_grad[j] - pow(surf_rad,2) * ( 2*ZMP_bottom_center*ZMP_bottom_grad[j] + 2*ZMP_bottom_radius*ZMP_bottom_grad[j]);
                }
            }

            
        }
        // auto stop_compute = std::chrono::high_resolution_clock::now();
        // auto duration_compute = std::chrono::duration_cast<std::chrono::milliseconds>(stop_compute - start_compute);
        // cout << "        Time Taken to Calculate Compute Function: " << duration_compute.count() << " milliseconds" << endl;

    }
    else{
        // do not update values
    }

}

// [TNLP_eval_f]
// returns the value of the objective function
bool armtd_NLP::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
    if(n != NUM_FACTORS){
       WARNING_PRINT("*** Error wrong value of n in eval_f!");
    }

    // call compute function to update values if necessary
    // cout << "Computing new_x from f function" << endl;
    // auto start_new_x = std::chrono::high_resolution_clock::now();

    compute(new_x,x);

    // auto stop_new_x = std::chrono::high_resolution_clock::now();
    // auto duration_new_x = std::chrono::duration_cast<std::chrono::milliseconds>(stop_new_x - start_new_x);
    // cout << "        Time Taken to Calculate new_x from f grad: " << duration_new_x.count() << " milliseconds" << endl;

    // auto start_nom = std::chrono::high_resolution_clock::now();

    // evaluate cost function
    // obj_value = sum((q_plan - q_des).^2);

    // calculate desired joint angles at planning time
    obj_value = 0; 
    Eigen::VectorXd q_plan(7);
    for(Index i = 0; i < n; i++){
        q_plan(i) = q_des_func(desired_trajectory->q0[i], desired_trajectory->qd0[i], desired_trajectory->qdd0[i], k_range[i] * x[i], t_plan); // Bohao question: why pass in t_plan here instead of duration?
    }
    // evaluate the cost
    obj_value = pow(wrap_to_pi(q_des[0] - q_plan[0]), 2) +
                pow(wrap_to_pi(q_des[2] - q_plan[2]), 2) + 
                pow(wrap_to_pi(q_des[4] - q_plan[4]), 2) + 
                pow(wrap_to_pi(q_des[6] - q_plan[6]), 2) + 
                pow(q_des[1] - q_plan[1], 2) + 
                pow(q_des[3] - q_plan[3], 2) + 
                pow(q_des[5] - q_plan[5], 2);
    // scale the cost
    obj_value *= COST_FUNCTION_OPTIMALITY_SCALE; // needs to change in the gradient as well

    // // modified cost: minimize only the first n joints
    // obj_value = pow(wrap_to_pi(q_des[0] - q_plan[0]), 2) +
    //             pow(wrap_to_pi(q_des[2] - q_plan[2]), 2) + 
    //             pow(q_des[1] - q_plan[1], 2) + 
    //             pow(q_des[3] - q_plan[3], 2);
    // // scale the cost
    // obj_value *= COST_FUNCTION_OPTIMALITY_SCALE; // needs to change in the gradient as well

    // auto stop_nom = std::chrono::high_resolution_clock::now();
    // auto duration_nom = std::chrono::duration<long, std::nano>(stop_nom - start_nom);
    // cout << "        Time for Evaluating Cost Nominal Term: " << duration_nom.count() << " nanoseconds" << endl;

    // auto start_f = std::chrono::high_resolution_clock::now();


    // // new cost term
    // // loop to pull out slip constraint values
    // for(Index i=0; i<NUM_TIME_STEPS;i++){
    //     // offset by NUM_TIME_STEPS to move past the separation constraint
    //     obj_value += force_constraint_ub[i+NUM_TIME_STEPS];
    // }

    // auto stop_f = std::chrono::high_resolution_clock::now();
    // auto duration_f = std::chrono::duration<long, std::nano>(stop_f - start_f);
    // cout << "        Time for Evaluating Cost Slip Term: " << duration_f.count() << " nanoseconds" << endl;

    // for(Index i = 0; i < NUM_TIME_STEPS; i++){
    //     obj_value += cost_slip_ub[i];
    // }

    return true;
}
// [TNLP_eval_f]

// [TNLP_eval_grad_f]
// return the gradient of the objective function grad_{x} f(x)
bool armtd_NLP::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in eval_grad_f!");
    }

    // call compute to update if new_x
    // cout << "Computing new_x from f grad function" << endl;
    // auto start_new_x = std::chrono::high_resolution_clock::now();

    compute(new_x,x);

    // auto stop_new_x = std::chrono::high_resolution_clock::now();
    // auto duration_new_x = std::chrono::duration_cast<std::chrono::milliseconds>(stop_new_x - start_new_x);
    // cout << "        Time Taken to Calculate new_x from f grad: " << duration_new_x.count() << " milliseconds" << endl;


    // compute gradient of cost function
    for(Index i = 0; i < n; i++){

        // values is 7x1
        // sum for each time step resulting in 7x1
        // add each element to corresponding element of grad_f?

        double q_plan = q_des_func(desired_trajectory->q0[i], desired_trajectory->qd0[i], desired_trajectory->qdd0[i], k_range[i] * x[i], t_plan); // Bohao question: why pass in t_plan here instead of duration?
        double dk_q_plan = pow(t_plan,3) * (6 * pow(t_plan,2) - 15 * t_plan + 10);
        if (i % 2 == 0) {
            grad_f[i] = (2 * wrap_to_pi(q_plan - q_des[i]) * dk_q_plan * k_range[i]);
        }
        else {
            grad_f[i] = (2 * (q_plan - q_des[i]) * dk_q_plan * k_range[i]);
        }
        grad_f[i] *= COST_FUNCTION_OPTIMALITY_SCALE;
    }

    // // modified compute gradient of cost function
    // for(Index i = 0; i < n; i++){

    //     // values is 7x1
    //     // sum for each time step resulting in 7x1
    //     // add each element to corresponding element of grad_f?

    //     double q_plan = q_des_func(desired_trajectory->q0[i], desired_trajectory->qd0[i], desired_trajectory->qdd0[i], k_range[i] * x[i], t_plan); // Bohao question: why pass in t_plan here instead of duration?
    //     double dk_q_plan = pow(t_plan,3) * (6 * pow(t_plan,2) - 15 * t_plan + 10);
    //     if ((i == 4) || (i == 5) || (i == 6)) {
    //         grad_f[i] = 0;
    //     }
    //     else if (i % 2 == 0) {
    //         grad_f[i] = (2 * wrap_to_pi(q_plan - q_des[i]) * dk_q_plan * k_range[i]);
    //     }
    //     else {
    //         grad_f[i] = (2 * (q_plan - q_des[i]) * dk_q_plan * k_range[i]);
    //     }
    //     grad_f[i] *= COST_FUNCTION_OPTIMALITY_SCALE;
    // }


    // auto start_grad = std::chrono::high_resolution_clock::now();

    // new cost term gradient
    // Index offset = NUM_TIME_STEPS * NUM_FACTORS; // offset to pass over separation constraint
    // for(Index i=0; i<NUM_TIME_STEPS;i++){
    //     // sum cost_grad_slip and grad_f column-wise
    //     for(Index j = 0; j < n; j++){
    //         // grad_f[j] += cost_grad_slip[i];
    //         grad_f[j] += force_constraint_gradient[i*NUM_FACTORS+offset+j];
    //     }
    // }

    // auto stop_grad = std::chrono::high_resolution_clock::now();
    // auto duration_grad = std::chrono::duration<long, std::nano>(stop_grad - start_grad);
    // cout << "        Time for Evaluating Cost Gradient Slip Term: " << duration_grad.count() << " nanoseconds" << endl;

    return true;
}
// [TNLP_eval_grad_f]

// [TNLP_eval_g]
// return the value of the constraints: g(x)
bool armtd_NLP::eval_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Number*       g
)
{
    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in eval_g!");
    }
    if(m != constraint_number){
        WARNING_PRINT("*** Error wrong value of m in eval_g!");
    }

    // cout << "Computing new_x from g function" << endl;
    compute(new_x,x);

    // auto start_g = std::chrono::high_resolution_clock::now();

    Index i;
    #pragma omp parallel for shared(kinematics_dynamics_result, x, g, link_sliced_center) private(i) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
    for(i = 0; i < NUM_TIME_STEPS; i++) {
        for (int k = 0; k < NUM_FACTORS; k++) {
            MatrixXInt res = kinematics_dynamics_result->u_nom(k, i).slice(x);
            g[i * NUM_FACTORS + k] = getCenter(res(0));
        }

        for (int l = 0; l < NUM_JOINTS; l++) {
            MatrixXInt res = kinematics_dynamics_result->links(l, i).slice(x);
            link_sliced_center[i * NUM_JOINTS + l] = getCenter(res);
        }
    }

    // Part 2. force constraints
    // offset by the number of input constraints NUM_TIME_STEPS*NUM_FACTORS
    for(Index i=0; i<3*NUM_TIME_STEPS;i++){
        g[i+NUM_TIME_STEPS*NUM_FACTORS] = force_constraint_ub[i];
    }

    // // For loop to iterate
    // for(int j=0; j<NUM_FACTORS;j++){
    //     cout << x[j] << ", ";
    // }
    // cout << "\n";
    // int out_index = NUM_FACTORS*NUM_TIME_STEPS + NUM_TIME_STEPS;
    // for(int i=0; i<NUM_TIME_STEPS;i++){
    //     cout << g[i+out_index] << "\n";
    // }

    // Part 3. check collision between joint position reachable set and obstacles (in gpu)
    obstacles->linkFRSConstraints(link_sliced_center, nullptr, g + NUM_FACTORS*NUM_TIME_STEPS + 3*NUM_TIME_STEPS, nullptr);

    // Part 4. (position & velocity) state limit constraints
    desired_trajectory->returnJointPositionExtremum(g + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles + NUM_FACTORS*NUM_TIME_STEPS + 3*NUM_TIME_STEPS, x);
    desired_trajectory->returnJointVelocityExtremum(g + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles + NUM_FACTORS * 2 + NUM_FACTORS*NUM_TIME_STEPS + 3*NUM_TIME_STEPS, x);

    // auto stop_g = std::chrono::high_resolution_clock::now();
    // auto duration_g = std::chrono::duration_cast<std::chrono::milliseconds>(stop_g - start_g);
    // cout << "        Time Taken to Calculate Eval g Function: " << duration_g.count() << " milliseconds" << endl;

    return true;
}
// [TNLP_eval_g]


// [TNLP_eval_jac_g]
// return the structure or values of the Jacobian
bool armtd_NLP::eval_jac_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Index         nele_jac,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in eval_g!");
    }
    if(m != constraint_number){
        WARNING_PRINT("*** Error wrong value of m in eval_g!");
    }

    // cout << "Computing new_x from g grad function" << endl;
    compute(new_x,x); // could this move into the else of the if statement below?

    // auto start_g_grad = std::chrono::high_resolution_clock::now();

    if( values == NULL ) {
       // return the structure of the Jacobian
       // this particular Jacobian is dense
        for(Index i = 0; i < m; i++){
            for(Index j = 0; j < n; j++){
                iRow[i * n + j] = i;
                jCol[i * n + j] = j;
            }
        }
    }
    else {
        Index i;
        #pragma omp parallel for shared(kinematics_dynamics_result, x, values, link_sliced_center, dk_link_sliced_center) private(i) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
        for(i = 0; i < NUM_TIME_STEPS; i++) {
            for (int k = 0; k < NUM_FACTORS; k++) {
                kinematics_dynamics_result->u_nom(k, i).slice(values + (i * NUM_FACTORS + k) * NUM_FACTORS, x);
            }

            for (int l = 0; l < NUM_JOINTS; l++) {
                link_sliced_center[i * NUM_JOINTS + l] = getCenter(kinematics_dynamics_result->links(l, i).slice(x));
                kinematics_dynamics_result->links(l, i).slice(dk_link_sliced_center + (i * NUM_JOINTS + l) * NUM_FACTORS, x);
            }
        }

        
        // Part 2. force constraints
        // offset by number of input constraint gradient terms: NUM_TIME_STEPS*NUM_FACTORS*NUM_FACTORS
        for(Index i=0; i<3*NUM_TIME_STEPS*NUM_FACTORS;i++){
            values[i+NUM_TIME_STEPS*NUM_FACTORS*NUM_FACTORS] = force_constraint_gradient[i];
        }

        // Part 3. check collision between joint position reachable set and obstacles (in gpu)
        obstacles->linkFRSConstraints(link_sliced_center, dk_link_sliced_center, nullptr, values + (NUM_TIME_STEPS * NUM_FACTORS + 3*NUM_TIME_STEPS) * NUM_FACTORS);

        // Part 4. (position & velocity) state limit constraints
        desired_trajectory->returnJointPositionExtremumGradient(values + (NUM_TIME_STEPS * NUM_FACTORS + 3 * NUM_TIME_STEPS + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles) * NUM_FACTORS, x);
        desired_trajectory->returnJointVelocityExtremumGradient(values + (NUM_TIME_STEPS * NUM_FACTORS + 3 * NUM_TIME_STEPS + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles + NUM_FACTORS * 2) * NUM_FACTORS, x);
    }

    // auto stop_g_grad = std::chrono::high_resolution_clock::now();
    // auto duration_g_grad = std::chrono::duration_cast<std::chrono::milliseconds>(stop_g_grad - start_g_grad);
    // cout << "        Time Taken to Calculate Eval g Grad Function: " << duration_g_grad.count() << " milliseconds" << endl;

    return true;
}
// [TNLP_eval_jac_g]


// [TNLP_eval_h]
//return the structure or values of the Hessian
bool armtd_NLP::eval_h(
   Index         n,
   const Number* x,
   bool          new_x,
   Number        obj_factor,
   Index         m,
   const Number* lambda,
   bool          new_lambda,
   Index         nele_hess,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
    return false;
}
// [TNLP_eval_h]


// [TNLP_finalize_solution]
void armtd_NLP::finalize_solution(
    SolverReturn               status,
    Index                      n,
    const Number*              x,
    const Number*              z_L,
    const Number*              z_U,
    Index                      m,
    const Number*              g,
    const Number*              lambda,
    Number                     obj_value,
    const IpoptData*           ip_data,
    IpoptCalculatedQuantities* ip_cq
)
{
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution.

    // added for reconstructing sliced reachable sets from hardware data
    // comment out for simulation use
    Number q_des_copy[NUM_JOINTS];
    for(int i=0;i<NUM_JOINTS;i++){
        q_des_copy[i] = q_des[i];
    }
    compute(true, q_des_copy);

    cout << "        CUDA & C++: Ipopt: desired joint angles: " << q_des.transpose() << endl;
    cout << "        CUDA & C++: Ipopt: final cost function value: " << obj_value / COST_FUNCTION_OPTIMALITY_SCALE << endl;

    // store the solution
    for( Index i = 0; i < n; i++ ) {
        solution[i] = (double)x[i];
    }

    // check constraint violation manually for Maximum_CpuTime_Exceeded case
    memcpy(g_copy, g, m * sizeof(Number));

    feasible = true;

    // control input constraints
    for( Index i = 0; i < NUM_TIME_STEPS; i++ ) {
        for( Index j = 0; j < NUM_FACTORS; j++ ) {
            if (g_copy[i * NUM_FACTORS + j] < -torque_limits[j] + (*torque_radius)(j, i) - TORQUE_INPUT_CONSTRAINT_VIOLATION_THRESHOLD || 
                g_copy[i * NUM_FACTORS + j] > torque_limits[j] - (*torque_radius)(j, i) + TORQUE_INPUT_CONSTRAINT_VIOLATION_THRESHOLD) {
                feasible = false;
                cout << "        CUDA & C++: Ipopt: Control torque of joint " << j << " at time interval " << i << " exceeds limit!\n";
                cout << "                        value: " << g_copy[i * NUM_FACTORS + j] << "\n";
                cout << "                        range: [ " << -torque_limits[j] + (*torque_radius)(j, i) << ", "
                                                            << torque_limits[j] - (*torque_radius)(j, i) << " ]\n";
                return;
            }
        }
    }    

    // collision avoidance constraints
    Index offset = NUM_FACTORS * NUM_TIME_STEPS;
    for( Index i = offset; i < offset + NUM_TIME_STEPS; i++) {
        // separation constraint
        // bool sep_constraint = (g_copy[i] > SEPARATION_CONSTRAINT_VIOLATION_THRESHOLD);
        // cout << sep_constraint << endl;
        if(g_copy[i] > SEPARATION_CONSTRAINT_VIOLATION_THRESHOLD) {
            feasible = false;
            double t_violation = i - offset;
            cout << "        CUDA & C++: Ipopt: Separation constraint violated at time interval: " << t_violation << " with value: " << g_copy[i] << "\n";
            return;
        }
        // slipping constraint
        if(g_copy[i+NUM_TIME_STEPS] > SLIPPING_CONSTRAINT_VIOLATION_THRESHOLD){
            feasible = false;
            double t_violation = i - offset;
            cout << "        CUDA & C++: Ipopt: Slipping constraint violated at time interval: " << t_violation << " with value: " << g_copy[i+NUM_TIME_STEPS] << " \n";
            return;
        }
        // tipping constraint
        if(g_copy[i+2*NUM_TIME_STEPS] > TIPPING_CONSTRAINT_VIOLATION_THRESHOLD){
            feasible = false;
            double t_violation = i - offset;
            cout << "        CUDA & C++: Ipopt: Tipping constraint violated at time interval: " << t_violation << " with value: " << g_copy[i+2*NUM_TIME_STEPS] << " \n";
            return;
        }
    }
    offset +=  NUM_TIME_STEPS*3;



    for( Index i = 0; i < NUM_JOINTS; i++ ) {
        for( Index j = 0; j < NUM_TIME_STEPS; j++ ) {
            for( Index h = 0; h < obstacles->num_obstacles; h++ ) {
                if (g_copy[(i * NUM_TIME_STEPS + j) * obstacles->num_obstacles + h + offset] > COLLISION_AVOIDANCE_CONSTRAINT_VIOLATION_THRESHOLD) {
                    feasible = false;
                    cout << "        CUDA & C++: Ipopt: Collision between link " << i + 1 << " and obstacle " << h << " at time interval " << j << "!\n";
                    cout << "                        value: " << g_copy[(i * NUM_TIME_STEPS + j) * obstacles->num_obstacles + h + offset] << "\n";
                    return;
                }
            }
        }
    }
    offset += NUM_JOINTS * NUM_TIME_STEPS * obstacles->num_obstacles;

    // state limit constraints
    //     minimum joint position
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        if (g_copy[i] < state_limits_lb[i - offset] + qe - STATE_LIMITS_VIOLATION_THRESHOLD || g_copy[i] > state_limits_ub[i - offset] - qe + STATE_LIMITS_VIOLATION_THRESHOLD) {
            feasible = false;
            cout << "        CUDA & C++: Ipopt: joint " << i - offset << " exceeds position limit when it reaches minimum!\n";
            cout << "                        value: " << g_copy[i] << "\n";
            cout << "                        range: [ " << state_limits_lb[i - offset] + qe << ", "
                                                        << state_limits_ub[i - offset] - qe << " ]\n";
            return;
        }
    }
    offset += NUM_FACTORS;

    //     maximum joint position
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        if (g_copy[i] < state_limits_lb[i - offset] + qe - STATE_LIMITS_VIOLATION_THRESHOLD || g_copy[i] > state_limits_ub[i - offset] - qe + STATE_LIMITS_VIOLATION_THRESHOLD) {
            feasible = false;
            cout << "        CUDA & C++: Ipopt: joint " << i - offset << " exceeds position limit when it reaches maximum!\n";
            cout << "                        value: " << g_copy[i] << "\n";
            cout << "                        range: [ " << state_limits_lb[i - offset] + qe << ", "
                                                        << state_limits_ub[i - offset] - qe << " ]\n";
            return;
        }
    }
    offset += NUM_FACTORS;

    //     minimum joint velocity
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        if (g_copy[i] < -speed_limits[i - offset] + qde - VELOCITY_LIMITS_VIOLATION_THRESHOLD || g_copy[i] > speed_limits[i - offset] - qde + VELOCITY_LIMITS_VIOLATION_THRESHOLD) {
            feasible = false;
            cout << "        CUDA & C++: Ipopt: joint " << i - offset << " exceeds velocity limit when it reaches minimum!\n";
            cout << "                        value: " << g_copy[i] << "\n";
            cout << "                        range: [ " << -speed_limits[i - offset] + qde << ", "
                                                        << speed_limits[i - offset] - qde << " ]\n";
            return;
        }
    }
    offset += NUM_FACTORS;

    //     maximum joint velocity
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        if (g_copy[i] < -speed_limits[i - offset] + qde - VELOCITY_LIMITS_VIOLATION_THRESHOLD || g_copy[i] > speed_limits[i - offset] - qde + VELOCITY_LIMITS_VIOLATION_THRESHOLD) {
            feasible = false;
            cout << "        CUDA & C++: Ipopt: joint " << i - offset << " exceeds velocity limit when it reaches maximum!\n";
            cout << "                        value: " << g_copy[i] << "\n";
            cout << "                        range: [ " << -speed_limits[i - offset] + qde << ", "
                                                        << speed_limits[i - offset] - qde << " ]\n";
            return;
        }
    }
}
// [TNLP_finalize_solution]


#endif
