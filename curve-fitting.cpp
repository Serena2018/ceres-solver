#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>

using namespace std;
using namespace cv;

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(double x, double y):_x(x), _y(y){}
    template <typename T>
    bool operator()(const T* const abc, T* residual) const
    {
        //代价函数 y = exp(a*x*x + b*x + c)
        residual[0] = _y-ceres::exp(abc[0]*_x*_x + abc[1]*_x + abc[2]);
        return true;
    }
    //结构体，成员变量
    const double _x, _y;
};
int main()
{

    double a = 3, b = 2, c = 1;
    double w = 1;
    cv::RNG rng;
    double abc[3] = {0,0,0};

    //生成待拟合曲线的数据散点，存储在vector中， x_data, y_data.
    vector<double> x_data, y_data;
    for(int i = 0; i<1000; i++)
    {
        double x = i/1000.0;
        x_data.push_back(x);
        y_data.push_back(exp(a*x*x + b*x + c) + rng.gaussian(w));
    }
    ceres::Problem problem;
    for(int i = 0; i<1000; i++)
    {
        //AddResidualBlock(cost_function, loss_function)
        //自动对代价方程进行求导，
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            new ceres::CauchyLoss(0.5),
            abc
        );
    }
    //配置求解器并求解，输出结果
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary summery;
    ceres::Solve(options, &problem, &summery);
    cout<<"a="<<abc[0]<<endl;
    cout<<"b="<<abc[1]<<endl;
    cout<<"c="<<abc[2]<<endl;
    return 0;
}