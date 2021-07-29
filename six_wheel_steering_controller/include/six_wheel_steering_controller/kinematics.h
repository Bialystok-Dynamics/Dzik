#ifndef SIX_WHEEL_STEERING_CONTROLLER_KINEMATICS_H
#define SIX_WHEEL_STEERING_CONTROLLER_KINEMATICS_H

#include "common.h"
#include "drive.h"

namespace six_wheel_steering_controller {

    class Kinematics {

    public:
        explicit Kinematics(std::shared_ptr<Drive> drive);
        void setVelocity(Twist velocity);

    private:
        std::shared_ptr<Drive> _drive;
    };

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_KINEMATICS_H
