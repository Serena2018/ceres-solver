#include <iostream>
#include <ceres/ceres.h>

using namespace std;
//第一部分：构建代价函数， 重载（）符号，仿函数小技巧
struct CostFunctor{
    template <typename T>
    bool operator()(const T* const x, T* residual) const{
        residual[0] = T(10.0) - x[0];
        return true;
    }
};
//f(x) = 10-x
struct NumericDiffCostFunctor{
    bool operator()(const double* const x, double* residual) const{
        residual[0] = 10.0 - x[0];
        return true;
    }
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    double initial_x = 5.0;
    double x = initial_x;
//第二部分：构建寻优问题。
//ceres::Problem problem;
ceres::Problem problem2;
//使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，就是残差的维度，
//第二个1是输入维度，就是待寻优参数x的维度
    // ceres::CostFunction* cost_function = 
    //     new ceres::AutoDiffCostFunction<CostFunctor,1,1>(new CostFunctor);
    //向问题中添加误差项。
    //problem.AddResidualBlock(cost_function, NULL, &x);

    ceres::CostFunction* cost_function2 = 
        new ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL,1,1>(
            new NumericDiffCostFunctor
        );
    problem2.AddResidualBlock(cost_function2, NULL, &x);


    //第三部分：配置运行求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;//配置增量方式的求解方法
    options.minimizer_progress_to_stdout = true; //输出到cout
    ceres::Solver::Summary summary;//优化信息
    ceres::Solve(options, &problem2, &summary);//求解

    std::cout<<summary.BriefReport() << "\n";

    std::cout<<"x: "<<initial_x<<"->"<<x<<"\n";
    return 0;

    cout<<"test"<<endl;
    
}
