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

    class LieR3LocalParameterization : public ceres::LocalParameterization {
    public:
        ~LieR3LocalParameterization() override = default;

        bool Plus(const double *x, const double *delta, double *x_plus_delta) const override {
            Eigen::Map<Eigen::Vector3d const> xVec(x);
            Sophus::SO3d SO3 = Sophus::SO3d::exp(xVec);

            Eigen::Map<Eigen::Vector3d const> deltaVec(delta);
            Sophus::SO3d deltaSO3 = Sophus::SO3d::exp(deltaVec);

            Eigen::Map<Eigen::Vector3d> plusDeltaVec(x_plus_delta);
            plusDeltaVec = (SO3 * deltaSO3).log();

            return true;
        }

        bool ComputeJacobian(const double *x, double *j) const override {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian(j);
            jacobian = Eigen::Matrix3d::Identity();
            return true;
        }

        [[nodiscard]] int GlobalSize() const override {
            return 3;
        }

        [[nodiscard]] int LocalSize() const override {
            return 3;
        }
    };

    struct PnPDynamicAutoDiffFunctor {
    private:
        CorrPair _corrPair;

    public:
        explicit PnPDynamicAutoDiffFunctor(CorrPair corrPair) : _corrPair(std::move(corrPair)) {}

        static auto Create(const CorrPair &corrPair) {
            return new ceres::DynamicAutoDiffCostFunction<PnPDynamicAutoDiffFunctor>(
                    new PnPDynamicAutoDiffFunctor(corrPair));
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

    struct PnPAutoDiffFunctor {
    private:
        CorrPair _corrPair;

    public:
        explicit PnPAutoDiffFunctor(CorrPair corrPair) : _corrPair(std::move(corrPair)) {}

        static auto Create(const CorrPair &corrPair) {
            return new ceres::AutoDiffCostFunction<PnPAutoDiffFunctor, 2, 4, 3>(
                    new PnPAutoDiffFunctor(corrPair));
        }

        template<typename T>
        bool operator()(const T *const SO3_CtoW_DATA, const T *const POS_CtoW_DATA, T *residuals) const {
            // get params
            Eigen::Map<const Sophus::SO3<T>> SO3_CtoW(SO3_CtoW_DATA);
            Eigen::Map<const Sophus::Vector3<T>> POS_CtoW(POS_CtoW_DATA);

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

    struct PnPSizedCostFunction : public ceres::SizedCostFunction<2, 3, 3> {
    private:
        CorrPair _corrPair;

    public:
        explicit PnPSizedCostFunction(CorrPair corrPair) : _corrPair(std::move(corrPair)) {}

        static auto Create(const CorrPair &corrPair) {
            return new PnPSizedCostFunction(corrPair);
        }

        bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
            // get params
            Sophus::Vector3d so3_CtoW(parameters[0]);
            Sophus::SO3d SO3_CtoW = Sophus::SO3d::exp(so3_CtoW);
            Eigen::Map<const Sophus::Vector3d> POS_CtoW(parameters[1]);

            // trans
            Sophus::SE3d SE3_CtoW(SO3_CtoW, POS_CtoW);
            Sophus::Vector3d pInC = SE3_CtoW.inverse() * _corrPair.point;
            Sophus::Vector2d normP(pInC(0) / pInC(2), pInC(1) / pInC(2));

            // compute residuals
            Eigen::Map<Sophus::Vector2d> residualsMap(residuals);
            residualsMap = normP - _corrPair.feature;

            if (jacobians != nullptr) {
                const double X_C = pInC(0), Y_C = pInC(1), Z_C = pInC(2), Z_C_INV = 1.0 / Z_C;
                Sophus::SO3d SO3_WtoC = SO3_CtoW.inverse();

                using Mat23d = Eigen::Matrix<double, 2, 3>;
                using Mat26d = Eigen::Matrix<double, 2, 6>;

                Mat23d pn_pc;
                pn_pc(0, 0) = Z_C_INV, pn_pc(0, 1) = 0.0, pn_pc(0, 2) = -X_C * Z_C_INV * Z_C_INV;
                pn_pc(1, 0) = 0.0, pn_pc(1, 1) = Z_C_INV, pn_pc(1, 2) = -Y_C * Z_C_INV * Z_C_INV;

                // the SO3
                Mat23d e_R =
                        pn_pc * (-SO3_WtoC.matrix() * Sophus::SO3d::hat(_corrPair.point)) * (-SO3_CtoW.matrix());

                // the POS
                Mat23d e_t = pn_pc * (-SO3_WtoC.matrix());

                // organize
                Mat26d j;
                j.block<2, 3>(0, 0) = e_R;
                j.block<2, 3>(0, 3) = e_t;
                for (int r = 0; r != 2; ++r) {
                    for (int c = 0; c != 6; ++c) {
                        jacobians[r][c] = j(r, c);
                    }
                }
            }

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
            auto costFunc = PnPDynamicAutoDiffFunctor::Create(item);
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

    static Sophus::SE3d SolvePnPWithAutoDiff(const std::vector<CorrPair> &data,
                                             const Sophus::SO3d &initSO3,
                                             const Sophus::Vector3d &initPOS,
                                             Scene *scene) {
        Sophus::SO3d SO3_CtoW = initSO3;
        Sophus::Vector3d POS_CtoW = initPOS;
        ceres::LocalParameterization *localParameterization = new LieLocalParameterization<Sophus::SO3d>();

        ceres::Problem problem;
        for (const auto &item: data) {
            auto costFunc = PnPAutoDiffFunctor::Create(item);

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

    static Sophus::SE3d SolvePnPWithSizedCostFunction(const std::vector<CorrPair> &data,
                                                      const Sophus::SO3d &initSO3,
                                                      const Sophus::Vector3d &initPOS,
                                                      Scene *scene) {
        Sophus::Vector3d SO3_CtoW = initSO3.log();
        Sophus::Vector3d POS_CtoW = initPOS;
        ceres::LocalParameterization *localParameterization = new LieR3LocalParameterization();

        ceres::Problem problem;
        for (const auto &item: data) {
            auto costFunc = PnPSizedCostFunction::Create(item);

            problem.AddResidualBlock(costFunc, nullptr, {SO3_CtoW.data(), POS_CtoW.data()});

            // local param
            problem.AddParameterBlock(SO3_CtoW.data(), 3, localParameterization);

        }
        ceres::Solver::Options options;

//        auto *callBack = new VisualCallBack(SO3_CtoW, POS_CtoW, scene);
//        options.callbacks.push_back(callBack);
//        options.update_state_every_iteration = true;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 5;
        options.linear_solver_type = ceres::DENSE_QR;

        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);
        LOG_INFO(summary.BriefReport())

        return {Sophus::SO3d::exp(SO3_CtoW), POS_CtoW};
    }

    static Sophus::SE3d SelfGaussNewton(const std::vector<CorrPair> &data,
                                        const Sophus::SO3d &initSO3,
                                        const Sophus::Vector3d &initPOS,
                                        Scene *scene) {
        Sophus::SO3d SO3_CtoW = initSO3;
        Sophus::Vector3d POS_CtoW = initPOS;

        std::shared_ptr<VisualCallBack> callBack = std::make_shared<VisualCallBack>(SO3_CtoW, POS_CtoW, scene);

        for (int i = 0; i != 10; ++i) {
            Eigen::Matrix<double, 6, 6> hMat = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> gMat = Eigen::Matrix<double, 6, 1>::Zero();

            for (const auto &corrPair: data) {
                // compute residuals
                auto pInW = corrPair.point;
                Sophus::SE3d SE3_CtoW(SO3_CtoW, POS_CtoW);
                auto pInC = SE3_CtoW.inverse() * pInW;
                Sophus::Vector2d normP(pInC(0) / pInC(2), pInC(1) / pInC(2));
                Eigen::Vector2d residuals = normP - corrPair.feature;

                // compute jacobians
                const double X_C = pInC(0), Y_C = pInC(1), Z_C = pInC(2), Z_C_INV = 1.0 / Z_C;
                Sophus::SO3d SO3_WtoC = SO3_CtoW.inverse();

                using Mat23d = Eigen::Matrix<double, 2, 3>;
                using Mat26d = Eigen::Matrix<double, 2, 6>;

                Mat23d pn_pc;
                pn_pc(0, 0) = Z_C_INV, pn_pc(0, 1) = 0.0, pn_pc(0, 2) = -X_C * Z_C_INV * Z_C_INV;
                pn_pc(1, 0) = 0.0, pn_pc(1, 1) = Z_C_INV, pn_pc(1, 2) = -Y_C * Z_C_INV * Z_C_INV;

                // the SO3
                Mat23d e_R = pn_pc * (-SO3_WtoC.matrix() * Sophus::SO3d::hat(pInW)) * (-SO3_CtoW.matrix());

                // the POS
                Mat23d e_t = pn_pc * (-SO3_WtoC.matrix());

                Mat26d j;
                j.block<2, 3>(0, 0) = e_R;
                j.block<2, 3>(0, 3) = e_t;

                hMat += j.transpose() * j;
                gMat += -j.transpose() * residuals;
            }

            Eigen::Matrix<double, 6, 1> delta = hMat.ldlt().solve(gMat);
            Eigen::Vector3d deltaSO3(delta.data());
            Eigen::Vector3d deltaPOS(delta.data() + 3);

            SO3_CtoW = SO3_CtoW * Sophus::SO3d::exp(deltaSO3);
            POS_CtoW = POS_CtoW + deltaPOS;

            callBack->operator()(ceres::IterationSummary());

            double change = deltaSO3.norm() + deltaPOS.norm();
            LOG_INFO("iterator: ", i, ", change: ", change)
            if (change < 1E-8) {
                break;
            }
        }

        return {SO3_CtoW, POS_CtoW};
    }
}

#endif //PNP_SOLVER_HPP
