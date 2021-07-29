#include "six_wheel_steering_controller/kinematics.h"

#include <utility>

namespace six_wheel_steering_controller{

    Kinematics::Kinematics(std::shared_ptr<Drive> drive): _drive{std::move(drive)} {

    }

}
