//
// Created by csl on 11/5/22.
//

#ifndef ST20_G2O_TEST_G2O_H
#define ST20_G2O_TEST_G2O_H

#include "sim_data.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/linear_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/sba/edge_project_xyz.h"

namespace ns_st20 {

    struct VertexCamera : public g2o::BaseVertex<6, OptPose> {
        bool read(istream &is) override {
            return false;
        }

        bool write(ostream &os) const override {
            return false;
        }

        [[nodiscard]] Eigen::Vector2d Project(const Eigen::Vector3d &landmark) const {
            auto WtoC = _estimate.inverse();
            Eigen::Vector3d pInC = WtoC.SO3 * landmark + WtoC.POS;
            Eigen::Vector2d pInCameraPlane(pInC(0) / pInC(2), pInC(1) / pInC(2));
            return pInCameraPlane;
        }

    protected:
        void oplusImpl(const number_t *v) override {
            _estimate.SO3 = _estimate.SO3 * Sophus::SO3d::exp(Eigen::Vector3d(v[0], v[1], v[2]));
            _estimate.POS = _estimate.POS + Eigen::Vector3d(v[3], v[4], v[5]);
        }

        void setToOriginImpl() override {
            _estimate.POS = Eigen::Vector3d::Zero();
            _estimate.SO3 = Sophus::SO3d();
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct VertexLandmark : public g2o::BaseVertex<3, Eigen::Vector3d> {
        bool read(istream &is) override {
            return true;
        }

        bool write(ostream &os) const override {
            return true;
        }

    protected:
        void oplusImpl(const number_t *v) override {
            Eigen::Map<const Eigen::Vector3d> update(v);
            _estimate += update;
        }

        void setToOriginImpl() override {
            _estimate = Eigen::Vector3d::Zero();
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct EdgeProject : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCamera, VertexLandmark> {

        void computeError() override {
            auto cameraPoseVertex = dynamic_cast<VertexCamera *>(_vertices[0]);
            auto landmarkVertex = dynamic_cast<VertexLandmark *>(_vertices[1]);
            auto pInCameraPlane = cameraPoseVertex->Project(landmarkVertex->estimate());
            _error = pInCameraPlane - _measurement;
        }

        bool read(istream &is) override {
            return true;
        }

        bool write(ostream &os) const override {
            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    static void SolveWithG2O(DataManager &dataManager, bool countTime) {
        using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
        using LinearSolverType = g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
        );

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);

        std::vector<VertexCamera *> cameraVertexVec;
        std::vector<VertexLandmark *> landmarkVertexVec;
        for (int i = 0; i < dataManager.cameraPoses.size(); ++i) {
            const auto &camera = dataManager.cameraPoses.at(i);
            auto cameraVertex = new VertexCamera();
            cameraVertex->setId(i);
            cameraVertex->setEstimate(camera);
            optimizer.addVertex(cameraVertex);
            cameraVertexVec.push_back(cameraVertex);
        }
        for (int i = 0; i < dataManager.landmarks.size(); ++i) {
            const auto &landmark = dataManager.landmarks.at(i);
            auto *landmarkVertex = new VertexLandmark();
            landmarkVertex->setId(i + static_cast<int>(dataManager.cameraPoses.size()));
            landmarkVertex->setEstimate(landmark.landmark);
            landmarkVertex->setMarginalized(true);
            optimizer.addVertex(landmarkVertex);
            landmarkVertexVec.push_back(landmarkVertex);
            for (auto [cameraIdx, point]: landmark.features) {
                auto *e = new EdgeProject;
                e->setVertex(0, cameraVertexVec.at(cameraIdx));
                e->setVertex(1, landmarkVertex);
                e->setMeasurement(point);
                e->setInformation(Eigen::Matrix2d::Identity());
                optimizer.addEdge(e);
            }
        }

        optimizer.initializeOptimization();
        optimizer.optimize(40);

        for (int i = 0; i < dataManager.landmarks.size(); ++i) {
            auto &landmark = dataManager.landmarks.at(i);
            landmark.landmark = landmarkVertexVec[i]->estimate();
        }

        for (int i = 0; i < dataManager.cameraPoses.size(); ++i) {
            auto camera = dataManager.cameraPoses.at(i);
            camera = cameraVertexVec.at(i)->estimate();
        }

    }
}

#endif //ST20_G2O_TEST_G2O_H
