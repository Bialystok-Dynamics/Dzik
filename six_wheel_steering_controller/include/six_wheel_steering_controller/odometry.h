#ifndef SIX_WHEEL_STEERING_CONTROLLER_ODOMETRY_H
#define SIX_WHEEL_STEERING_CONTROLLER_ODOMETRY_H

#include "common.h"
#include "drive.h"
#include "math/point.h"

namespace six_wheel_steering_controller {

    class Odometry {

    public:
        explicit Odometry(std::shared_ptr<Drive> drive);

        ~Odometry() = default;

        OdometryInfo getOdometry(double dt) const;


    private:
        std::shared_ptr<Drive> _drive;

        mutable math::Point pw1;
        mutable math::Point pw2;
        mutable math::Point pw3;
        mutable math::Point pw4;
        mutable math::Point pw5;
        mutable math::Point pw6;

        mutable math::Point baseLinkPosition{};
        mutable double baseLinkYaw =0.;
    };

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_ODOMETRY_H
