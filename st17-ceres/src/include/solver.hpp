//
// Created by csl on 10/18/22.
//

#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

#include <utility>
#include "sophus/se3.hpp"
#include "ceres/ceres.h"
#include "Eigen/Dense"
#include "thread"

namespace ns_st17 {
    std::mutex mt;

    struct CorrPair {
        Eigen::Vector3d point;
        Eigen::Vector2d feature;

        CorrPair(Eigen::Vector3d point, Eigen::Vector2d feature)
                : point(std::move(point)), feature(std::move(feature)) {}

        CorrPair() = default;
    };

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
            Eigen::Map<Eigen::Matrix<double, Groupd::num_parameters, Groupd::DoF,
                    Eigen::RowMajor>>
                    jacobian(jacobian_raw);
            jacobian = T.Dx_this_mul_exp_x_at_0();
            return true;
        }

        ///@brief Global size
        [[nodiscard]] int GlobalSize() const override { return Groupd::num_parameters; }

        ///@brief Local size
        [[nodiscard]] int LocalSize() const override { return Groupd::DoF; }
    };

    struct PnPFunctor {
    private:
        CorrPair _corrPair;

    public:
        explicit PnPFunctor(CorrPair corrPair) : _corrPair(std::move(corrPair)) {}

        static auto Create(const CorrPair &corrPair) {
            return new ceres::DynamicAutoDiffCostFunction<PnPFunctor>(new PnPFunctor(corrPair));
        }

        template<typename T>
        bool operator()(T const *const *parameters, T *residuals) const {
            // get params
            Eigen::Map<const Sophus::SO3<T>> SO3_CtoW(parameters[0]);
            Eigen::Map<const Sophus::Vector3<T>> POS_CtoW(parameters[1]);

            // trans
            Sophus::SE3<T> SE3_CtoW(SO3_CtoW, POS_CtoW);
            Sophus::Vector3<T> pInC = SE3_CtoW.inverse() * _corrPair.point.template cast<T>();
            Sophus::Vector2<T> normP(pInC(0) / pInC(2), pInC(1) / pInC(2));

            // compute residuals
            Eigen::Map<Sophus::Vector2<T>> residualsMap(residuals);
            residualsMap = normP - _corrPair.feature.template cast<T>();

            return true;
        }
    };

    struct VisualCallBack : public ceres::IterationCallback {
        const Sophus::SO3d *_curSO3;
        const Sophus::Vector3d *_curPOS;
        Scene *_scene;

        VisualCallBack(const Sophus::SO3d &SO3, const Sophus::Vector3d &POS, Scene *scene)
                : _curSO3(&SO3), _curPOS(&POS), _scene(scene) {}

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {
            std::lock_guard<std::mutex> lock(mt);
            _scene->AddCamera(
                    std::to_string(std::chrono::system_clock::now().time_since_epoch().count()),
                    Posed(*_curSO3, *_curPOS), 1.0, 0.0, 0.0, 2.0, 0.5
            );
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return ceres::SOLVER_CONTINUE;
        }

    };

    static Sophus::SE3d SolvePnPWithDynamicAutoDiff(const std::vector<CorrPair> &data,
                                                    const Sophus::SO3d &initSO3,
                                                    const Sophus::Vector3d &initPOS,
                                                    Scene *scene) {
        Sophus::SO3d SO3_CtoW = initSO3;
        Sophus::Vector3d POS_CtoW = initPOS;
        ceres::LocalParameterization *localParameterization = new LieLocalParameterization<Sophus::SO3d>();

        ceres::Problem problem;
        for (const auto &item: data) {
            auto costFunc = PnPFunctor::Create(item);
            costFunc->AddParameterBlock(4);
            costFunc->AddParameterBlock(3);
            costFunc->SetNumResiduals(2);

            problem.AddResidualBlock(costFunc, nullptr, {SO3_CtoW.data(), POS_CtoW.data()});

            // local param
            problem.AddParameterBlock(SO3_CtoW.data(), 4, localParameterization);
        }
        ceres::Solver::Options options;

        auto *callBack = new VisualCallBack(SO3_CtoW, POS_CtoW, scene);
        options.callbacks.push_back(callBack);
        options.update_state_every_iteration = true;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 5;
        options.linear_solver_type = ceres::DENSE_QR;

        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);
        LOG_INFO(summary.BriefReport())

        return {SO3_CtoW, POS_CtoW};
    }
}

#endif //PNP_SOLVER_HPP
