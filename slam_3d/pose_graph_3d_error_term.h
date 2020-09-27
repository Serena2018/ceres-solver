#ifndef POSE_GRAPH_3D_ERROR_TERM_H_
#define POSE_GRAPH_3D_ERROR_TERM_H_

#include <eigen3/Eigen/Core>
#include <ceres/ceres.h>
#include "types.h"
namespace ceres_study
{

    class PoseGraph3dErrorTerm
    {
    public:
    //t_ab_measured表示位移的差值，还有信息矩阵，要关注，信息矩阵是如何被使用的。
        PoseGraph3dErrorTerm(const Pose3d& t_ab_measured,
                             const Eigen::Matrix<double, 6, 6> &sqrt_infomation) : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_infomation) {}
        //两个节点的位移和朝向数据
        template <typename T>
        bool operator()(const T *const p_a_ptr, const T *const q_a_ptr,
                        const T *const p_b_ptr, const T *const q_b_ptr,
                        T *residuals_ptr) const{
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>p_a(p_a_ptr);//定义中间变量
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>>p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);
        //compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();//q_a的共轭
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse*q_b;

        //Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3,1>p_ab_estimated = q_a_inverse*(p_b-p_a);
        //compute the error between the two orientation estimates.
        //一个四元数的逆与另外一个四元数的乘积，可以计算两个四元数的差，这个差表示角位移差。
        //四元数的共轭是保持常数部分不变，把向量部分取负，四元数的逆就是让四元数的共轭除以四元数的模长
        //如果使用的事=是单位四元数，那么四元数的逆和共轭就是相同的。
        Eigen::Quaternion<T> delta_q = 
            t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

        //compute the residuals_ptr
        Eigen::Map<Eigen::Matrix<T, 6, 1>>residuals(residuals_ptr);
        residuals.template block<3,1>(0,0) = 
            p_ab_estimated - t_ab_measured_.p.template cast<T>();
        //刚开始我写成block<3,1>(0,0)发现优化后的轨迹与优化之前的一模一样，找了很长时间才找到错误在这里
        //residual是6x1的向量，前三个维度表示平移的残差，后三个维度表示旋转的残差，先是用四元数表示的
        //然后转换成旋转向量的模式。
        residuals.template block<3,1>(3,0) = T(2.0)*delta_q.vec();

        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
        return true;
        }

    static ceres::CostFunction* Create(
        const Pose3d& t_ab_measured,
        const Eigen::Matrix<double, 6, 6>& sqrt_information){
        return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6,3,4,3,4>(
            new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
    //The measurement for the position of B relative to A in the A frame
        const Pose3d t_ab_measured_;
    //The square root of the measurement information matrix
        const Eigen::Matrix<double, 6, 6> sqrt_information_;
    };

} //namespace ceres_study
#endif