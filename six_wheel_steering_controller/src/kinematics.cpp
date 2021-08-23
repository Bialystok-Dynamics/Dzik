#include "six_wheel_steering_controller/kinematics.h"
#include "six_wheel_steering_controller/math/vector.h"
#include <cmath>

namespace six_wheel_steering_controller {

    Kinematics::Kinematics(std::shared_ptr<Drive> drive) : _drive{std::move(drive)} {

    }

    void Kinematics::setVelocity(Twist velocity) {
        // planar
        auto planarComponent = math::Vector(velocity.linX, velocity.linY);

        // turning
        const double ySpacing2 = _drive->getY_AxisSpacing() / 2;
        auto frontR = math::Vector(_drive->getDistanceToFront(), -ySpacing2).rotate(M_PI_2) *
                      velocity.angZ + planarComponent;
        auto frontL = math::Vector(_drive->getDistanceToFront(), ySpacing2).rotate(M_PI_2) *
                      velocity.angZ + planarComponent;

        auto midR = math::Vector(0, -ySpacing2).rotate(M_PI_2) *
                    velocity.angZ + planarComponent;
        auto midL = math::Vector(0, ySpacing2).rotate(M_PI_2) *
                    velocity.angZ + planarComponent;

        auto rearR = math::Vector(-_drive->getDistanceToRear(), -ySpacing2).rotate(M_PI_2) *
                     velocity.angZ + planarComponent;
        auto rearL = math::Vector(-_drive->getDistanceToRear(), ySpacing2).rotate(M_PI_2) *
                     velocity.angZ + planarComponent;


        auto getWheelUnitInfo = [](const math::Vector v) { return WheelUnitInfo{v.angle(), v.length()}; };

        _drive->setCommand({
                                   getWheelUnitInfo(frontR),
                                   getWheelUnitInfo(frontL),
                                   getWheelUnitInfo(midR),
                                   getWheelUnitInfo(midL),
                                   getWheelUnitInfo(rearR),
                                   getWheelUnitInfo(rearL)
                           });

    }

}
