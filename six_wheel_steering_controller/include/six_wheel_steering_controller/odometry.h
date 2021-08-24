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

        ///
        /// \param dt time since last call [s]
        /// \return odometry info
        OdometryInfo getOdometry(double dt) const;


    private:
        std::shared_ptr<Drive> _drive;

        mutable math::Point _pw1;
        mutable math::Point _pw2;
        mutable math::Point _pw3;
        mutable math::Point _pw4;
        mutable math::Point _pw5;
        mutable math::Point _pw6;

        mutable math::Point _baseLinkPosition{};
        mutable double _baseLinkYaw =0.;

    };

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_ODOMETRY_H
