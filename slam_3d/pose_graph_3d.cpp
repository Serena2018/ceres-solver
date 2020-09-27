#include <iostream>
#include <fstream>
#include <string>

#include <ceres/ceres.h>
// #include <common/read_g2o.h>
#include "../common/read_g2o.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "pose_graph_3d_error_term.h"
#include "types.h"

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");

void BuildOptimizationProblem(const ceres_study::VectorOfConstraints& constraints, ceres_study::MapofPoses* poses,
                                ceres::Problem* problem){
    CHECK(poses != NULL);
    CHECK(problem !=NULL);

    if(constraints.empty())
    {
        LOG(INFO) <<"No constraints, no problem to optimize.";
        return;
    }
    ceres::LossFunction* loss_function = NULL;
    ceres::LocalParameterization* quaternion_local_parameterization = 
    new ceres::EigenQuaternionParameterization;

    for(ceres_study::VectorOfConstraints::const_iterator constraints_iter = 
            constraints.begin();
            constraints_iter != constraints.end(); ++constraints_iter){
            const ceres_study::Constraint3d& constraint = *constraints_iter;
            ceres_study::MapofPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
            CHECK(pose_begin_iter != poses->end())
            <<"Pose with ID: "<<constraint.id_begin<<" not found.";

            ceres_study::MapofPoses::iterator pose_end_iter = poses->find(constraint.id_end);
            CHECK(pose_end_iter != poses->end())
            <<"Pose with ID: "<<constraint.id_end <<" not found.";

            const Eigen::Matrix<double, 6,6> sqrt_information = constraint.information.llt().matrixL();
            ceres::CostFunction* cost_function = ceres_study::PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

            problem->AddResidualBlock(cost_function, loss_function,
                                       pose_begin_iter->second.p.data(),
                                       pose_begin_iter->second.q.coeffs().data(),
                                       pose_end_iter->second.p.data(),
                                       pose_end_iter->second.q.coeffs().data());
            problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                        quaternion_local_parameterization);
            problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                         quaternion_local_parameterization);
        }
        ceres_study::MapofPoses::iterator pose_start_iter = poses->begin();
        CHECK(pose_start_iter != poses->end()) <<"There are no poses.";
        problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
        problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

bool SolveOptimizationProblem(ceres::Problem* problem){
    CHECK(problem != NULL);

    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout<<summary.FullReport()<<'\n';

    return summary.IsSolutionUsable();
}


bool OutputPoses(const std::string& filename, const ceres_study::MapofPoses& poses){
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if(!outfile){
        LOG(ERROR) <<"Error opening the file: "<<filename;
        return false;
    }
    for(std::map<int, ceres_study::Pose3d, std::less<int>, 
        Eigen::aligned_allocator<std::pair<const int, ceres_study::Pose3d>>>::
        const_iterator poses_iter = poses.begin();
        poses_iter != poses.end(); ++poses_iter){
        const std::map<int, ceres_study::Pose3d, std::less<int>,
        Eigen::aligned_allocator<std::pair<const int, ceres_study::Pose3d>>>::
        value_type& pair = *poses_iter;

    // outfile<<pair.first<<" "<<pair.second.p.transpose()<<" "
    //         <<pair.second.q.x()<<" "<<pair.second.q.y()<<" "
    //         <<pair.second.q.z()<<" "<<pair.second.q.w()<<'\n';
    outfile<<pair.first<<" "<<pair.second.p.x()<<" "
          <<pair.second.p.y()<<" "<<pair.second.p.z()<<" "
          <<pair.second.q.x()<<" "<<pair.second.q.y()<<" "
          <<pair.second.q.z()<<" "<<pair.second.q.w()<<'\n';
    }
    std::cout<<"poses have been written into outfile"<<std::endl;
    return true;

}
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    CHECK(FLAGS_input != "")<<"Need to specify the filename to read";

    ceres_study::MapofPoses poses;
    ceres_study::VectorOfConstraints constraints;
    CHECK(ceres_study::ReadG2oFile(FLAGS_input, &poses, &constraints))
        <<"Error reading the file"<<FLAGS_input;

    std::cout<<"Number of poses: "<<poses.size()<<'\n';
    std::cout<<"Number of constraints: "<<constraints.size()<<'\n';
    std::cout<<"test pose_3d"<<std::endl;

    CHECK(OutputPoses("./dataset/poses3d_original.txt",poses))
        <<"Error outputting to poses_original.txt";

    ceres::Problem problem;
    BuildOptimizationProblem(constraints, &poses, &problem);

    CHECK(SolveOptimizationProblem(&problem))
    <<"The solve was not successful, exiting.";

    CHECK(OutputPoses("./dataset/poses3d_optimized.txt", poses))
    <<"Error outputting to poses_optimized.txt";

    return 0;
}