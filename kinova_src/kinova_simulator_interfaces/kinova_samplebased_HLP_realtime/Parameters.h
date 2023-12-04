#ifndef PARAMETERS_H
#define PARAMETERS_H

// #include "FetchInfo.h"
// #include "KinovaInfo.h"
// #include "KinovaWithoutGripperInfo.h"
// #include "KinovaForceInfo.h"
#include "KinovaFixedGripperInfo.h"
// #include "KinovaChampagneEdgeCondensed.h"

// #define DEBUG_MODE true

// Parameters for PZsparse.h:
    // monomials with a coefficient smaller than this number will be reduced (should be close to V_m magnitude)
    #define SIMPLIFY_THRESHOLD 2e-4

// Parameters for Trajectories.h:
    // Duration of the Bezier Curve
    #define DURATION 2.0

    // number of time steps (This should be an EVEN number!!!)
    #define NUM_TIME_STEPS 128

    // range of parameters (size of generators in PZ).
    const double k_range[NUM_FACTORS] = { M_PI / 72, M_PI / 72, M_PI / 72, M_PI / 72, M_PI / 72, M_PI / 72, M_PI / 72 }; // (radian)

// Parameters for CollisionChecking.h:
    // maximum number of obstacles (used for memory pre-allocation)
    #define MAX_OBSTACLE_NUM 10

    // number of generators of obstacle zonotopes
    #define MAX_OBSTACLE_GENERATOR_NUM 3

// Parameters for NLPclass.h:
    // number of threads to parallelize in CPU
    // As a rule of thumb, you SHOULD NOT need to exceed the number of available processors in your system!!!
    // In Linux, run nproc in command line to find out number of processors
    #define NUM_THREADS 32

    // threshold for collision avoidance constraint considered to be violated (unit: meter)
    #define COLLISION_AVOIDANCE_CONSTRAINT_VIOLATION_THRESHOLD 1e-4

    // threshold for input constraint considered to be violated (unit: Newton * meter)
    #define TORQUE_INPUT_CONSTRAINT_VIOLATION_THRESHOLD 1e-4

    // threshold for separation constraint (unit: Newton)
    #define SEPARATION_CONSTRAINT_VIOLATION_THRESHOLD 1e-6

    // threshold for slipping constraint violation (unit: Newton) // squared?
    #define SLIPPING_CONSTRAINT_VIOLATION_THRESHOLD 1e-6

    // threshold for tipping constraint violation (unit: meter) // squared?
    #define TIPPING_CONSTRAINT_VIOLATION_THRESHOLD 1e-6

    // threshold for state limits violation (unit: radian)
    #define STATE_LIMITS_VIOLATION_THRESHOLD 1e-4

    // threshold for state limits violation (unit: radian / sec)
    #define VELOCITY_LIMITS_VIOLATION_THRESHOLD 1e-3

    // scale the cost function value so that it could converge faster (be careful with it!)
    #define COST_FUNCTION_OPTIMALITY_SCALE 1.0

// Parameters for armour_main.cpp
    #define IPOPT_OPTIMIZATION_TOLERANCE 1e-4

    #define IPOPT_MAX_CPU_TIME 12.0 // 14.0

    #define IPOPT_PRINT_LEVEL 0

    #define IPOPT_MU_STRATEGY "adaptive"

    #define IPOPT_LINEAR_SOLVER "ma97"

#endif