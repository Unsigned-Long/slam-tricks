//
// Created by csl on 9/8/22.
//

#ifndef INS_ELLIPSOID_HPP
#define INS_ELLIPSOID_HPP

#include <cmath>
#include <iomanip>
#include <iostream>
#include <tuple>

struct RefEllipsoid {
public:
    double a;
    double b;
    double c;
    double f;
    double efir;
    double efir2;
    double esed;
    double esed2;

public:
    RefEllipsoid(double longRadius, double oblateness) : a(longRadius), f(oblateness) {
        b = (1.0 - f) * a;
        c = a * a / b;
        efir = std::sqrt(a * a - b * b) / a;
        efir2 = efir * efir;
        esed = std::sqrt(a * a - b * b) / b;
        esed2 = esed * esed;
    }

    double W(double latitude) const {
        return std::sqrt(1.0 - efir2 * std::pow(std::sin(latitude), 2));
    }

    double V(double latitude) const {
        return std::sqrt(1.0 + esed2 * std::pow(std::cos(latitude), 2));
    }

    double N(double latitude) const {
        return a / W(latitude);
    }

    double M(double latitude) const {
        return a * (1 - efir2) / std::pow(W(latitude), 3);
    }
};

static const RefEllipsoid WGS1984(6378137.0, 1.0 / 298.257223563);

static const RefEllipsoid CGCS2000(6378137.0, 1.0 / 298.257222101);

static const RefEllipsoid GRS1980(6378137.0, 1.0 / 298.257222101);

static const RefEllipsoid Krasovsky(6378245.0, 1.0 / 298.2997381);

static const RefEllipsoid Clarke1866(6378206.4, 1.0 / 294.9786982);

static const RefEllipsoid Clarke1880(6378245.0, 1.0 / 293.46);

static const RefEllipsoid Bessel1841(6377397.155, 1.0 / 299.1528434);

struct Position {
    double lat;
    double lon;
    double height;

    Position(double lat, double lon, double height) : lat(lat), lon(lon), height(height) {}

    Eigen::Vector3d toEigenVec() const {
        return Eigen::Vector3d(lat, lon, height);
    }

    static Position fromEigenVec(const Eigen::Vector3d &vec) {
        return Position(vec(0), vec(1), vec(2));
    }

    friend std::ostream &operator<<(std::ostream &os, const Position &position) {
        os << "lat: " << position.lat << " lon: " << position.lon << " height: " << position.height;
        return os;
    }
};

struct Velocity {
    double vNorth;
    double vEast;
    double vDown;

    Velocity(double vNorth, double vEast, double vDown) : vNorth(vNorth), vEast(vEast), vDown(vDown) {}

    Eigen::Vector3d toEigenVec() const {
        return Eigen::Vector3d(vNorth, vEast, vDown);
    }

    static Velocity fromEigenVec(const Eigen::Vector3d &vec) {
        return Velocity(vec(0), vec(1), vec(2));
    }

    friend std::ostream &operator<<(std::ostream &os, const Velocity &velocity) {
        os << "vNorth: " << velocity.vNorth << " vEast: " << velocity.vEast << " vDown: " << velocity.vDown;
        return os;
    }
};

#endif //INS_ELLIPSOID_HPP
