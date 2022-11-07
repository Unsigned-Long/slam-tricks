//
// Created by csl on 11/5/22.
//

#ifndef ST20_G2O_TEST_CERES_H
#define ST20_G2O_TEST_CERES_H

#include "sim_data.h"
#include "artwork/timer/timer.h"

namespace ns_st20 {
    /// @brief Local parametrization for ceres that can be used with Sophus Lie
    /// group implementations.
    template<class Groupd>
    class LieLocalParameterization : public ceres::LocalParameterization {
    public:
        ~LieLocalParameterization() override = default;

        using Tangentd = typename Groupd::Tangent;

        /// @brief plus operation for Ceres
        bool Plus(double const *T_raw, double const *delta_raw,
                  double *T_plus_delta_raw) const override {
            Eigen::Map<Groupd const> const T(T_raw);
            Eigen::Map<Tangentd const> const delta(delta_raw);
            Eigen::Map<Groupd> T_plus_delta(T_plus_delta_raw);
            T_plus_delta = T * Groupd::exp(delta);
            return true;
        }

        ///@brief Jacobian of plus operation for Ceres
        bool ComputeJacobian(double const *T_raw, double *jacobian_raw) const override {
            Eigen::Map<Groupd const> T(T_raw);
            Eigen::Map<Eigen::Matrix<double, Groupd::num_parameters, Groupd::DoF, Eigen::RowMajor>>
                    jacobian(jacobian_raw);
            jacobian = T.Dx_this_mul_exp_x_at_0();
            return true;
        }

        ///@brief Global size
        [[nodiscard]] int GlobalSize() const override { return Groupd::num_parameters; }

        ///@brief Local size
        [[nodiscard]] int LocalSize() const override { return Groupd::DoF; }
    };

    struct ProjectFactor {
    private:
        const Eigen::Vector2d feature;

    public:
        explicit ProjectFactor(Eigen::Vector2d feature)
                : feature(std::move(feature)) {}

        static auto Create(const Eigen::Vector2d &feature) {
            return new ceres::DynamicAutoDiffCostFunction<ProjectFactor>(new ProjectFactor(feature));
        }

    public:
        /**
         * [ SO3 | POS | landmark ]
         */
        template<typename T>
        bool operator()(T const *const *parameters, T *residuals) const {
            // get params
            Eigen::Map<const Sophus::SO3<T>> SO3_CtoW(parameters[0]);
            Eigen::Map<const Sophus::Vector3<T>> POS_CtoW(parameters[1]);
            Eigen::Map<const Sophus::Vector3<T>> landmark(parameters[2]);

            // trans
            Sophus::SE3<T> SE3_CtoW(SO3_CtoW, POS_CtoW);
            Sophus::Vector3<T> pInC = SE3_CtoW.inverse() * landmark;
            Sophus::Vector2<T> normP(pInC(0) / pInC(2), pInC(1) / pInC(2));

            // compute residuals
            Eigen::Map<Sophus::Vector2<T>> residualsMap(residuals);
            residualsMap = normP - feature.template cast<T>();

            return true;
        }
    };

    struct VisualCallBack : public ceres::IterationCallback {
        DataManager *_dataManager;

        explicit VisualCallBack(DataManager &dataManager)
                : _dataManager(&dataManager) {}

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {
            _dataManager->RemoveScene();
            _dataManager->DrawScene();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return ceres::SOLVER_CONTINUE;
        }

    };

    static void SolveWithCeresDynamicAutoDiff(DataManager &dataManager, bool countTime) {
        if (!countTime) {
            LOG_INFO("press any key to start the process...")
            std::cin.get();
        }
        ns_timer::Timer timer;
        timer.re_start();

        ceres::LocalParameterization *localParameterization = new LieLocalParameterization<Sophus::SO3d>();

        ceres::Problem problem;
        for (auto &landmark: dataManager.landmarks) {
            for (const auto &[cameraIdx, feature]: landmark.features) {
                auto costFunc = ProjectFactor::Create(feature);
                costFunc->AddParameterBlock(4);
                costFunc->AddParameterBlock(3);
                costFunc->AddParameterBlock(3);
                costFunc->SetNumResiduals(2);

                auto &camera = dataManager.cameraPoses.at(cameraIdx);

                problem.AddResidualBlock(
                        costFunc, nullptr, {camera.SO3.data(), camera.POS.data(), landmark.landmark.data()}
                );

                // local param
                problem.AddParameterBlock(camera.SO3.data(), 4, localParameterization);

                // set the front and back camera pose const
                if (cameraIdx == 0 || cameraIdx == dataManager.cameraPoses.size() - 1) {
                    problem.SetParameterBlockConstant(camera.SO3.data());
                    problem.SetParameterBlockConstant(camera.POS.data());
                }
            }
        }
        ceres::Solver::Options options;

        if (!countTime) {
            auto *callBack = new VisualCallBack(dataManager);
            options.callbacks.push_back(callBack);
            options.update_state_every_iteration = true;
            options.minimizer_progress_to_stdout = true;
        }

        // single thread
        options.num_threads = 1;
        // liner solver
        options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);
        LOG_PROCESS(timer.last_elapsed("SolveWithCeresDynamicAutoDiff"))

        LOG_PLAINTEXT(summary.BriefReport())
    }
}


#endif //ST20_G2O_TEST_CERES_H
