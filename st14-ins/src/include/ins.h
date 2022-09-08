//
// Created by csl on 9/8/22.
//

#ifndef INS_INS_H
#define INS_INS_H

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ellipsoid.hpp"

namespace ns_ins {

    class INSSolver {
    private:

        static constexpr double EARTH_ANGLE_RATE = 7.292E-5;

        const RefEllipsoid ellipsoid;

        Eigen::Quaterniond attBody2Nav_k;
        Eigen::Quaterniond attBody2Nav_k_1;

        Position position_k;
        Position position_k_1;

        Velocity velocityNav_k;
        Velocity velocityNav_k_1;

        double timeStamp_k;
        double timeStamp_k_1;
        double durTime;

        Eigen::Vector3d angleGen_k;
        Eigen::Vector3d angleGen_k_1;

        Eigen::Vector3d veloGen_k;
        Eigen::Vector3d veloGen_k_1;

    public:
        INSSolver(const RefEllipsoid &ellipsoid, double initTimeStamp,
                  const Eigen::Quaterniond &initAttBody2Nav,
                  const Position &initPosition,
                  const Velocity &initVelocity);

        bool update(const Eigen::Vector3d &curAngleGen, const Eigen::Vector3d &curVeloGen, double timeStamp);

        const Eigen::Quaterniond &getAttBody2NavK() const;

        const Position &getPositionK() const;

        const Velocity &getVelocityNavK() const;

    protected:
        bool updateAttitude();

        bool updateVelocity();

        bool updatePosition();

    private:
        static Eigen::Vector3d angleRateEar2Inr(const Position &position);

        static Eigen::Vector3d angleRateNav2Ear(const Velocity &velocity,
                                                const Position &position,
                                                const RefEllipsoid &ellipsoid);

        static Eigen::Vector3d predict(const Eigen::Vector3d &vecFir, const Eigen::Vector3d &vecSed);

        static Eigen::Matrix3d toAntiMat(const Eigen::Vector3d &vec);

        static Eigen::Vector3d localGravity(const Position &position);
    };

}

#endif //INS_INS_H
