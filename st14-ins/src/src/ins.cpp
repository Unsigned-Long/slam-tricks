//
// Created by csl on 9/8/22.
//
#include "ins.h"

ns_ins::INSSolver::INSSolver(const RefEllipsoid &ellipsoid, double initTimeStamp,
                             const Eigen::Quaterniond &initAttBody2Nav,
                             const Position &initPosition, const Velocity &initVelocity)
        : ellipsoid(ellipsoid), timeStamp_k(initTimeStamp), timeStamp_k_1(initTimeStamp),
          attBody2Nav_k(initAttBody2Nav), attBody2Nav_k_1(initAttBody2Nav),
          position_k(initPosition), position_k_1(initPosition),
          velocityNav_k(initVelocity), velocityNav_k_1(initVelocity), durTime(0.0) {}


bool ns_ins::INSSolver::update(const Eigen::Vector3d &curAngleGen,
                               const Eigen::Vector3d &curVeloGen,
                               double curTimeStamp) {

    this->angleGen_k_1 = this->angleGen_k;
    this->angleGen_k = curAngleGen;

    this->veloGen_k_1 = this->veloGen_k;
    this->veloGen_k = curVeloGen;

    this->timeStamp_k_1 = this->timeStamp_k;
    this->timeStamp_k = curTimeStamp;

    this->durTime = this->timeStamp_k - this->timeStamp_k_1;

    this->updateAttitude();
    this->updateVelocity();
    this->updatePosition();

    return true;
}

bool ns_ins::INSSolver::updateAttitude() {
    Eigen::Vector3d phi_k = angleGen_k + 1.0 / 12.0 * angleGen_k_1.cross(angleGen_k);

    Eigen::Quaterniond attBody_k_k_1 = Eigen::Quaterniond(
            Eigen::AngleAxisd(phi_k.norm(), phi_k.normalized())
    );

    Eigen::Vector3d zeta_k = (angleRateEar2Inr(position_k_1) +
                              angleRateNav2Ear(velocityNav_k_1, position_k_1, ellipsoid))
                             * durTime;

    Eigen::Quaterniond attNav_k_k_1 = Eigen::Quaterniond(
            Eigen::AngleAxisd(zeta_k.norm(), zeta_k.normalized())
    );

    attBody2Nav_k = attNav_k_k_1 * attBody2Nav_k_1 * attBody_k_k_1;

    return true;
}

bool ns_ins::INSSolver::updateVelocity() {
    // TODO: optimal
    Eigen::Vector3d locGravity = localGravity(position_k_1);

    Eigen::Vector3d angelRateE2I = angleRateEar2Inr(position_k_1);
    Eigen::Vector3d angelRateN2E = angleRateNav2Ear(velocityNav_k_1, position_k_1, ellipsoid);

    Eigen::Vector3d coriolisIntegral =
            (locGravity - (2.0 * angelRateE2I + angelRateN2E).cross(velocityNav_k_1.toEigenVec())) * durTime;

    Eigen::Vector3d zeta = (angelRateE2I + angelRateN2E) * durTime;

    Eigen::Vector3d deltaV =
            veloGen_k + 0.5 * angleGen_k.cross(veloGen_k) +
            1.0 / 12.0 * (angleGen_k_1.cross(veloGen_k) + veloGen_k_1.cross(angleGen_k));

    Eigen::Vector3d forceIntegral =
            (Eigen::Matrix3d::Identity() - (0.5 * toAntiMat(zeta))) * attBody2Nav_k_1 * deltaV;

    velocityNav_k = Velocity::fromEigenVec(velocityNav_k_1.toEigenVec() + coriolisIntegral + forceIntegral);

    return true;
}

bool ns_ins::INSSolver::updatePosition() {

    double vDownAvg = 0.5 * (velocityNav_k.vDown + velocityNav_k_1.vDown);
    double vEastAvg = 0.5 * (velocityNav_k.vEast + velocityNav_k_1.vEast);
    double vNorthAvg = 0.5 * (velocityNav_k.vNorth + velocityNav_k_1.vNorth);

    position_k.height = position_k_1.height - vDownAvg * durTime;
    double hAvg = 0.5 * (position_k.height + position_k_1.height);

    double RN = ellipsoid.N(position_k_1.lat), RM = ellipsoid.M(position_k_1.lat);

    position_k.lat = position_k_1.lat + vNorthAvg * durTime / (RM + hAvg);
    double latAvg = 0.5 * (position_k.lat + position_k_1.lat);

    position_k.lon = position_k_1.lon + vEastAvg * durTime / ((RN + hAvg) * std::cos(latAvg));

    return true;
}

Eigen::Vector3d ns_ins::INSSolver::predict(const Eigen::Vector3d &vecFir, const Eigen::Vector3d &vecSed) {
    return vecSed * 2 - vecFir;
}

Eigen::Matrix3d ns_ins::INSSolver::toAntiMat(const Eigen::Vector3d &vec) {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();

    mat(0, 1) = -vec(2), mat(0, 2) = vec(1);
    mat(1, 0) = vec(2), mat(1, 2) = -vec(0);
    mat(2, 0) = -vec(1), mat(2, 1) = vec(0);

    return mat;
}

Eigen::Vector3d ns_ins::INSSolver::angleRateEar2Inr(const Position &position) {
    return Eigen::Vector3d(EARTH_ANGLE_RATE * std::cos(position.lat),
                           0.0,
                           -EARTH_ANGLE_RATE * std::sin(position.lat));
}

Eigen::Vector3d ns_ins::INSSolver::angleRateNav2Ear(const Velocity &velocity,
                                                    const Position &position,
                                                    const RefEllipsoid &ellipsoid) {

    double v1 = velocity.vEast / (ellipsoid.N(position.lat) + position.height);
    double v2 = -velocity.vNorth / (ellipsoid.M(position.lat) + position.height);
    double v3 = -velocity.vEast * std::tan(position.lat) / (ellipsoid.N(position.lat) + position.height);

    return Eigen::Vector3d(v1, v2, v3);
}

Eigen::Vector3d ns_ins::INSSolver::localGravity(const Position &position) {
    // TODO finish this function
    return Eigen::Vector3d();
}

const Eigen::Quaterniond &ns_ins::INSSolver::getAttBody2NavK() const {
    return attBody2Nav_k;
}

const Position &ns_ins::INSSolver::getPositionK() const {
    return position_k;
}

const Velocity &ns_ins::INSSolver::getVelocityNavK() const {
    return velocityNav_k;
}
