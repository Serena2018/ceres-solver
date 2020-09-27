#ifndef NORMALIZE_ANGLE_H_
#define NORMALIZE_ANGLE_H_

#include <cmath>
#include <ceres/ceres.h>


namespace ceres_study{

    //normalize the angle in radians between [-pi and pi]
    template <typename T>
    inline T NormalizeAngle(const T& angle_radians)
    {
        T two_pi(2.0*M_PI);
        return angle_radians 
        - two_pi*ceres::floor((angle_radians + T(M_PI)) / two_pi);
    }

} //namespace ceres_study
#endif


