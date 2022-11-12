//
// Created by csl on 11/12/22.
//

#include "ceres/ceres.h"
#include "artwork/logger/logger.h"

struct DemoFunctor {

public:
    DemoFunctor() = default;

    static auto Create() {
        return new ceres::DynamicAutoDiffCostFunction<DemoFunctor>(new DemoFunctor());
    }

    template<typename T>
    bool operator()(T const *const *parameters, T *residuals) const {
        // get params
        residuals[0] = parameters[0][0] - T(3.0);
        return true;
    }
};

int main(int argc, char **argv) {
    {
        ceres::Problem problem;
        auto costFunc = DemoFunctor::Create();
        costFunc->AddParameterBlock(1);
        costFunc->SetNumResiduals(1);
        double x = 0.0;
        problem.AddResidualBlock(costFunc, nullptr, &x);

        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 1;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;

        LOG_VAR(x)
        ceres::Solve(options, &problem, &summary);
        LOG_INFO(summary.BriefReport())
        LOG_VAR(x)
    }
    {
        ceres::Problem problem;
        auto costFunc = DemoFunctor::Create();
        costFunc->AddParameterBlock(1);
        costFunc->SetNumResiduals(1);
        double x = 0.0;
        problem.AddResidualBlock(costFunc, nullptr, &x);
        problem.SetParameterLowerBound(&x, 0, -2.0);
        problem.SetParameterUpperBound(&x, 0, 2.0);

        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 1;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;

        LOG_VAR(x)
        ceres::Solve(options, &problem, &summary);
        LOG_INFO(summary.BriefReport())
        LOG_VAR(x)
    }


    return 0;
}