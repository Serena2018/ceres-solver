#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "angle_local_parameterization.h"
#include <ceres/ceres.h>
#include "../common/read_g2o.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "pose_graph_2d_error_term.h"
#include "types.h"

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");

using namespace std;
namespace ceres_study{

void BuildOptimizationProblem(const std::vector<Constraint2d>& constraints,
                                std::map<int, Pose2d>* poses,
                                ceres::Problem* problem){

    std::cout<<"Number of poses: "<<poses->size()<<'\n';
    std::cout<<"Number of constraints: "<<constraints.size()<<'\n';

    CHECK(poses != NULL);
    CHECK(problem != NULL);
    if(constraints.empty()){
        LOG(INFO) << "No constraints, no problem to optimize.";
        return;
    }

    ceres::LossFunction* loss_function = NULL;
    //The class LocalParameterization defines the function Plus and its
    //Jacobian which is needed to compute the Jacobian of f w.r.t delta
    ceres::LocalParameterization* angle_local_parameterization = 
        AngleLocalParameterization::Create();

    for(std::vector<Constraint2d>::const_iterator constraints_iter = 
        constraints.begin();
        constraints_iter != constraints.end(); ++constraints_iter){
            const Constraint2d& constraint  = *constraints_iter;
            //在constrain中得到id_begin和id_end,表示这两个点之间的约束，
            //在constraint中存储的是x_ab, y_ab, yaw_radians_ab, 
            //然后需要根据id_begin和id_end分别找到两个pose中存储的x_a, y_a, yaw_radiant_a,
            // x_b, y_b, yaw_radiant_b,这样就可以直到两者
            //（x_a-x_b）-x_ab就可以知道x方向的residual了。
            std::map<int, Pose2d>::iterator pose_begin_iter = 
                poses->find(constraint.id_begin);
                //CHECK dies with a fatal error if condition is not ture.
            CHECK(pose_begin_iter != poses->end())
                <<"Pose with ID: "<<constraint.id_begin << "not found.";
            std::map<int, Pose2d>::iterator pose_end_iter = 
                poses->find(constraint.id_end);
                //CHECK dies with a fatal error if condition is not ture.
            CHECK(pose_end_iter != poses->end())
                <<"Pose with ID: "<<constraint.id_end << "not found.";
            //Eigen中实现Cholesky分解，A = L*D*L.transpose()
            const Eigen::Matrix3d sqrt_infomation = 
                constraint.information.llt().matrixL();
            
            //ceres will take ownership of the pointer.
            ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(
                constraint.x, constraint.y, constraint.yaw_radians, sqrt_infomation);
            //convinence methods for adding residuals with a small number of parameters.
            //This is the common case. Instead of specifying the parameter block arguments 
            //as a vector, list them as pointers.
            problem->AddResidualBlock(
                cost_function, loss_function, &pose_begin_iter->second.x,
                &pose_begin_iter->second.y, &pose_begin_iter->second.yaw_radians,
                &pose_end_iter->second.x, &pose_end_iter->second.y,
                &pose_end_iter->second.yaw_radians);

            problem->SetParameterization(&pose_begin_iter->second.yaw_radians,
                                        angle_local_parameterization);
            problem->SetParameterization(&pose_end_iter->second.yaw_radians,
                                            angle_local_parameterization);
            
        }
        std::map<int, Pose2d>::iterator pose_start_iter = poses->begin();
        CHECK(pose_start_iter != poses->end()) <<"There are no poses.";
        //SetParameterBlockConstrain() Hold the indicated parameter block constraint during optimization
        problem->SetParameterBlockConstant(&pose_start_iter->second.x);
        problem->SetParameterBlockConstant(&pose_start_iter->second.y);
        problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
}

bool SolveOptimizationProblem(ceres::Problem* problem){
    CHECK(problem != NULL);
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    std::cout<<summary.FullReport()<<'\n';

    return summary.IsSolutionUsable();
    std::cout<<"come here 2"<<std::endl;
}

//output the poses to the file with format: ID x y yaw_radians
bool OutputPoses(const std::string& filename,
                const std::map<int, Pose2d>& poses){
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if(!outfile){
        std::cerr<<"Error opening the file: "<<filename<<'\n';
        return false;
    }
    for(std::map<int, Pose2d>::const_iterator poses_iter = poses.begin();
        poses_iter != poses.end(); ++poses_iter){
            const std::map<int, Pose2d>::value_type& pair = *poses_iter;
            // if(pair.second.x > 1000 || pair.second.y>1000 || pair.second.x < -1000 || pair.second.y<-1000)
            // {
            //     continue;
            // }

            outfile<<pair.first <<' '<<pair.second.x<<' '<< pair.second.y<<' '
            <<pair.second.yaw_radians<<'\n';
    }
    std::cout<<"poses have been written into outfile"<<std::endl;
    return true;
                    
}

}//namespace ceres_study

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    CHECK(FLAGS_input != "") << "Need to specify the filename to read.";
    //define two containers to contain poses and constraints
    std::map<int, ceres_study::Pose2d> poses;
    std::vector<ceres_study::Constraint2d> constraints;

    //read data from .g2o file.
    CHECK(ceres_study::ReadG2oFile(FLAGS_input, &poses, &constraints))
    <<"Error reading the file: "<<FLAGS_input;

    std::cout<<"Number of poses: "<<poses.size()<<'\n';
    std::cout<<"Number of constraints: "<<constraints.size()<<'\n';

    //output original poses 
    CHECK(ceres_study::OutputPoses("./dataset/poses2d_original.txt", poses))
    <<"Error outputting to poses_original.txt";

    //build the real problem into an optimization problem.
    ceres::Problem problem;
    ceres_study::BuildOptimizationProblem(constraints, &poses, &problem);
    //use the solver built in ceres to solve the peoblem defined above.
    CHECK(ceres_study::SolveOptimizationProblem(&problem))
    <<"The solve was not sucessful, exiting.";
    //come here, the original poses have been optimized, and output it.
    CHECK(ceres_study::OutputPoses("./dataset/poses2d_optimized.txt", poses))
    <<"Error outputting to poses_optimized.txt";
    return 0;
}