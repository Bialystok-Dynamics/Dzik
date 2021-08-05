#include "six_wheel_steering_controller/odometry.h"

six_wheel_steering_controller::Odometry::Odometry(std::shared_ptr<Drive> drive) :
        _drive{std::move(drive)} {

}

six_wheel_steering_controller::OdometryInfo six_wheel_steering_controller::Odometry::getOdometry(double dt) const {
    return {};
}
