#ifndef SIX_WHEEL_STEERING_CONTROLLER_ODOMETRY_H
#define SIX_WHEEL_STEERING_CONTROLLER_ODOMETRY_H

#include "common.h"
#include "drive.h"

namespace six_wheel_steering_controller {

    class Odometry {

    public:
        explicit Odometry(std::shared_ptr<Drive> drive);

        OdometryInfo getOdometry(double dt) const;

    private:
        std::shared_ptr<Drive> _drive;
    };

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_ODOMETRY_H
