//
// Created by csl on 10/3/22.
//

#ifndef LIC_CALIB_ODOMETER_POSE_H
#define LIC_CALIB_ODOMETER_POSE_H

#include <ostream>
#include "sophus/se3.hpp"
#include "Eigen/Dense"

namespace ns_st17 {
#define INVALID_TIME_STAMP (-1.0)

    template<class ScalarType>
    struct Pose {
    public:

        using Scale = ScalarType;
        using Rotation = Sophus::SO3<Scale>;
        using Translation = Sophus::Vector3<Scale>;

        Rotation so3;
        Translation t;
        double timeStamp;

        Pose(const Rotation &so3, const Translation &t, double timeStamp = INVALID_TIME_STAMP)
                : so3(so3), t(t), timeStamp(timeStamp) {}

        explicit Pose(double timeStamp = INVALID_TIME_STAMP)
                : so3(), t(Translation::Zero()), timeStamp(timeStamp) {}

        Eigen::Quaternion<ScalarType> q() const {
            return so3.unit_quaternion();
        }

        Sophus::Matrix3 <ScalarType> R() const {
            return q().toRotationMatrix();
        }

        Sophus::SE3 <ScalarType> se3() const {
            return Sophus::SE3<ScalarType>(so3, t);
        }

        Sophus::Matrix4 <ScalarType> T() const {
            Sophus::Matrix4<ScalarType> T = Sophus::Matrix4<ScalarType>::Identity();
            T.template block<3, 3>(0, 0) = R();
            T.template block<3, 1>(0, 3) = t;
            return T;
        }

        static Pose fromT(const Sophus::Matrix4 <ScalarType> &T, double timeStamp = INVALID_TIME_STAMP) {
            Sophus::Matrix3<ScalarType> rotMat = T.template block<3, 3>(0, 0);
            rotMat = adjustRotationMatrix(rotMat);

            Pose pose(timeStamp);
            pose.so3 = Rotation(rotMat);
            pose.t = T.template block<3, 1>(0, 3);
            return pose;
        }

        static Pose
        fromRt(const Sophus::Matrix3 <ScalarType> &R,
               const Sophus::Vector3 <ScalarType> &t,
               double timeStamp = INVALID_TIME_STAMP) {
            Sophus::Matrix3<ScalarType> rotMat = adjustRotationMatrix(R);

            Pose pose(timeStamp);
            pose.so3 = Rotation(rotMat);
            pose.t = t;
            return pose;
        }

        static Pose fromSE3(const Sophus::SE3 <ScalarType> &se3, double timeStamp = INVALID_TIME_STAMP) {
            Pose pose(timeStamp);
            pose.so3 = se3.so3();
            pose.t = se3.translation();
            return pose;
        }

        friend std::ostream &operator<<(std::ostream &os, const Pose &pose) {
            auto q = pose.so3.unit_quaternion();
            os << " timeStamp: " << pose.timeStamp << " qx: " << q.x() << " qy: " << q.y()
               << " qz: " << q.z() << ", qw: " << q.w() << " t: " << pose.t.transpose();
            return os;
        }

    protected:
        static Sophus::Matrix3 <ScalarType> adjustRotationMatrix(const Sophus::Matrix3 <ScalarType> &rotMat) {
            // adjust
            Eigen::JacobiSVD<Sophus::Matrix3<ScalarType>> svd(rotMat, Eigen::ComputeFullV | Eigen::ComputeFullU);
            const Sophus::Matrix3<ScalarType> &vMatrix = svd.matrixV();
            const Sophus::Matrix3<ScalarType> &uMatrix = svd.matrixU();
            Sophus::Matrix3<ScalarType> adjustedRotMat = uMatrix * vMatrix.transpose();
            return adjustedRotMat;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using Posed = Pose<double>;
    using Posef = Pose<float>;
}

#endif //LIC_CALIB_ODOMETER_POSE_H
