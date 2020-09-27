#ifndef TYPES_H_
#define TYPES_H_

#include <istream>
#include <map>
#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
namespace ceres_study{
struct Pose3d{
Eigen::Vector3d p;
Eigen::Quaterniond q;
//The name of the data type in the g2o file format
static std::string name(){
    return "VERTEX_SE3:QUAT";
}
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
//运算符重载
//Pose3D同时包括位移和四元数表示的朝向
std::istream& operator>>(std::istream& input, Pose3d& pose){
    input >> pose.p.x() >>pose.p.y() >> pose.p.z() >> pose.q.x() >> pose.q.y()
    >>pose.q.z() >> pose.q.w();
    //Normalization the quaternion to account for precision loss due to serialization
    pose.q.normalize();
    return input;
}

typedef std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d>>> MapofPoses;
//The constraint between two vertices in the pose graph. The constraint is the
//transformation from vertex id_begin to vertex id_end;

struct Constraint3d{
    int id_begin;
    int id_end;
    //The transformation that represent the pose of the end frame E w.r.t the
    //begin frame B. In other words, it transforms a vector in the E frame to
    //the B frame.
    Pose3d t_be;//从end frame转换到begin frame
    //The inverse of the covariance matrix for the measurement. 
    //The order of the entries is x y z delta orientation.
    Eigen::Matrix<double, 6, 6> information;
    static std::string name(){
        return "EDGE_SE3:QUAT";
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::istream& operator>>(std::istream& input, Constraint3d& constraint){
    Pose3d& t_be = constraint.t_be;
    input>>constraint.id_begin >> constraint.id_end>>t_be;
    for(int i = 0; i<6 && input.good(); ++i){
        for(int j = i; j<6 && input.good(); ++j){
            input >> constraint.information(i,j);
            if(i!=j){
                constraint.information(j,i) = constraint.information(i,j);
            }
        }
    }
    
    return input;
    }
    typedef std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d>>
    VectorOfConstraints;

}//namespace ceres_study
#endif