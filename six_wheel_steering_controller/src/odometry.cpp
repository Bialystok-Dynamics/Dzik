#include "six_wheel_steering_controller/odometry.h"
#include "six_wheel_steering_controller/math/vector.h"
#include "six_wheel_steering_controller/math/point.h"
#include <cmath>

namespace six_wheel_steering_controller {
    math::Vector getVectorFromWheelUnit(const WheelUnit &wheelUnit) {
        const auto &info = wheelUnit.getInfo();

        return math::Vector::fromLengthAndAngle(info.speed, info.angle);
    }

    Odometry::Odometry(std::shared_ptr<Drive> drive) :
            _drive{std::move(drive)} {
        double dy_2 = _drive->getY_AxisSpacing() / 2;
        double df = _drive->getDistanceToFront();
        double dr = _drive->getDistanceToRear();

        pw1 = math::Point(df, dy_2);
        pw2 = math::Point(df, -dy_2);
        pw3 = math::Point(0, dy_2);
        pw4 = math::Point(0, -dy_2);
        pw5 = math::Point(-dr, dy_2);
        pw6 = math::Point(-dr, -dy_2);
    }

    OdometryInfo six_wheel_steering_controller::Odometry::getOdometry(double dt) const {
        static const double ySpacing = _drive->getY_AxisSpacing();
        static const double ySpacing_2 = ySpacing / 2;
        static const double df = _drive->getDistanceToFront();
        static const double dr = _drive->getDistanceToRear();

        // vwx - wheel 'x' velocity vector
        const auto vw1 = getVectorFromWheelUnit(*_drive->getFrontLeft());
        const auto vw2 = getVectorFromWheelUnit(*_drive->getFrontRight());
        const auto vw3 = getVectorFromWheelUnit(*_drive->getMidLeft());
        const auto vw4 = getVectorFromWheelUnit(*_drive->getMidRight());
        const auto vw5 = getVectorFromWheelUnit(*_drive->getRearLeft());
        const auto vw6 = getVectorFromWheelUnit(*_drive->getRearRight());

        pw1 += vw1 * dt;
        pw2 += vw2 * dt;
        pw3 += vw3 * dt;
        pw4 += vw4 * dt;
        pw5 += vw5 * dt;
        pw6 += vw6 * dt;

        // mean rotational velocity
        double vzrm = (-vw1 + vw2 - vw3 + vw4 - vw5 + vw6).x / ySpacing / 3;
        // mean y-axis velocity
        double vym = (vw3 + vw4).y / 2;
        // mean x-axis velocity
        double vxm = (vw1 + vw2 + vw3 + vw4 + vw5 + vw6).x / 6;

        double vzrmVariance = 0.;


        vzrmVariance += std::pow(vzrm - (-vw1.x + vxm) / ySpacing_2, 2);
        vzrmVariance += std::pow(vzrm - (vw2.x - vxm) / ySpacing_2, 2);
        vzrmVariance += std::pow(vzrm - (-vw3.x + vxm) / ySpacing_2, 2);
        vzrmVariance += std::pow(vzrm - (vw4.x - vxm) / ySpacing_2, 2);
        vzrmVariance += std::pow(vzrm - (-vw5.x + vxm) / ySpacing_2, 2);
        vzrmVariance += std::pow(vzrm - (vw6.x - vxm) / ySpacing_2, 2);
        vzrmVariance /= 6;

        double vymVariance = 0.;

        vymVariance += std::pow(vym - (vw1.y - vzrm * df), 2);
        vymVariance += std::pow(vym - (vw2.y - vzrm * df), 2);
        vymVariance += std::pow(vym - vw3.y, 2);
        vymVariance += std::pow(vym - vw4.y, 2);
        vymVariance += std::pow(vym - (vw5.y + vzrm * dr), 2);
        vymVariance += std::pow(vym - (vw6.y + vzrm * dr), 2);
        vymVariance /= 6;

        double vxmVariance = 0.;

        vxmVariance += std::pow(vxm - (vw1.x + vzrm * ySpacing_2), 2);
        vxmVariance += std::pow(vxm - (vw2.x - vzrm * ySpacing_2), 2);
        vxmVariance += std::pow(vxm - (vw3.x + vzrm * ySpacing_2), 2);
        vxmVariance += std::pow(vxm - (vw4.x - vzrm * ySpacing_2), 2);
        vxmVariance += std::pow(vxm - (vw5.x + vzrm * ySpacing_2), 2);
        vxmVariance += std::pow(vxm - (vw6.x - vzrm * ySpacing_2), 2);
        vxmVariance /= 6;

        auto velocityVector = math::Vector(vxm, vym).rotate(baseLinkYaw);
        baseLinkPosition += velocityVector*dt;
        baseLinkYaw += vzrm*dt;

        OdometryInfo ret;

        ret.x = baseLinkPosition.x;
        ret.y = baseLinkPosition.y;
        ret.yaw = baseLinkYaw;

        ret.vX = vxm;
        ret.vY = vym;
        ret.vYaw = vzrm;

        ret.velocityCovariance[0] = vxmVariance;
        ret.velocityCovariance[7] = vymVariance;
        ret.velocityCovariance[14] = vzrmVariance;
        return ret;
    }

};
