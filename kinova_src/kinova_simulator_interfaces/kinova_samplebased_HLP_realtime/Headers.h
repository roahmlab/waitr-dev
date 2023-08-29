#ifndef HEADER_H
#define HEADER_H

#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include "device_launch_parameters.h"
#include <cstdio>
#include <omp.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <memory>
#include <fstream>
#include <cstring>
#include <boost/numeric/interval.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cassert>
#include <vector>
#include <cstdint>

#define WARNING_PRINT printf
#define PRINT printf

// // cuda stuff
// #ifndef DEVICE_INDEPENDENT
// #define DEVICE_INDEPENDENT __host__ __device__
// #endif

// // Segment<T> is a template class that emulates the memory layout of T
// // but strips off T's type info and manually add it later on.
// // The purpose is solely to bybass constructor.
// template <typename T> class alignas(alignof(T)) Segment {
// protected:
//     char __data[sizeof(T)];

// public:
//     using Type = T;

// template <typename... Args>
//     DEVICE_INDEPENDENT void init(const Args &...args) {
//     new (__data) T(args...);
//     }

//     DEVICE_INDEPENDENT T *operator->(void) {
//     return reinterpret_cast<T *>(__data);
//     }

//     DEVICE_INDEPENDENT const T *operator->(void) const {
//     return reinterpret_cast<const T *>(__data);
//     }

//     DEVICE_INDEPENDENT inline T &reference(void) {
//     return *reinterpret_cast<T *>(__data);
//     }

//     DEVICE_INDEPENDENT inline const T &reference(void) const {
//     return *reinterpret_cast<const T *>(__data);
//     }

//     DEVICE_INDEPENDENT operator T &(void) { return reference(); }

//     DEVICE_INDEPENDENT operator const T &(void) const { return reference(); }
// };

// intervals
namespace bn = boost::numeric;
namespace bi = bn::interval_lib;

using Interval = bn::interval<
        double, 
        bi::policies<
            bi::save_state<bi::rounded_transc_std<double> >,
            bi::checking_base<double>
        > 
    >;

// interval matrices
namespace Eigen {
    namespace internal {
        template<typename X, typename S, typename P>
        struct is_convertible<X, bn::interval<S,P> > {
            enum { value = is_convertible<X,S>::value };
        };

        template<typename S, typename P1, typename P2>
        struct is_convertible<bn::interval<S,P1>, bn::interval<S,P2> > {
            enum { value = true };
        };
    }
}

typedef Eigen::Matrix<Interval, Eigen::Dynamic, Eigen::Dynamic> MatrixXInt;

using std::vector;
using std::cout;
using std::endl;
using std::sort;
using std::swap;
using std::min;
using std::max;

#endif