#include "six_wheel_steering_controller/odometry.h"

six_wheel_steering_controller::Odometry::Odometry(std::shared_ptr<Drive> drive) :
        _drive{std::move(drive)} {

}
