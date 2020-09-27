#ifndef TYPES_H_
#define TYPES_H_
#include <fstream>
#include <eigen3/Eigen/Core>
#include "normalize_angle.h"

namespace ceres_study
{
    //The state for each vertex in the pose graph
    struct Pose2d{
        double x;
        double y;
        double yaw_radians;
        //The name of the data type in the g2o file format.
        static std::string name(){
            return "VERTEX_SE2";
        }
    };

    std::istream& operator>>(std::istream& input, Pose2d& pose){
        input >> pose.x >> pose.y >> pose.yaw_radians;
        //Normalize the angle between -pi to pi
        pose.yaw_radians = NormalizeAngle(pose.yaw_radians);
        return input;
    }
    struct Constraint2d{
        int id_begin;
        int id_end;
        double x;
        double y;
        double yaw_radians;

        //The inverse of the corvariance matrix for the measurement
        //The order of the entries are x, y, yaw.
        Eigen::Matrix3d information;
        static std::string name(){
            return "EDGE_SE2";
        }
    };

    std::istream& operator>>(std::istream& input, Constraint2d& constraint){
        input >> constraint.id_begin >> constraint.id_end >> constraint.x>>
        constraint.y >> constraint.yaw_radians >>
        constraint.information(0,0) >> constraint.information(0,1) >> constraint.information(0,2) >> 
        constraint.information(1,1) >> constraint.information(1,2) >> constraint.information(2,2);

        //set the lower triangular part of the information matrix.
        constraint.information(1,0) = constraint.information(0,1);
        constraint.information(2,0) = constraint.information(0,2);
        constraint.information(2,1) = constraint.information(1,2);

        //Normalize the angle between -pi to pi
        constraint.yaw_radians =  NormalizeAngle(constraint.yaw_radians);
        return input;
    }

}//namespace ceres_study



#endif

