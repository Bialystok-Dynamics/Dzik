#ifndef SIX_WHEEL_STEERING_CONTROLLER_DRIVE_H
#define SIX_WHEEL_STEERING_CONTROLLER_DRIVE_H

#include "common.h"
#include <memory>
#include "wheel_unit.h"

namespace six_wheel_steering_controller {

    class Mode;

    class Drive {
    public:
        friend Mode;

        void setMode(std::unique_ptr<Mode> mode);
        void setCommand(DriveInfo command);

    private:

        double _spacing;
        double _distanceToFront;
        double _distanceToRear;
        std::unique_ptr<Mode> _mode;

        std::unique_ptr<WheelUnit> _frontRight;
        std::unique_ptr<WheelUnit> _frontLeft;
        std::unique_ptr<WheelUnit> _midRight;
        std::unique_ptr<WheelUnit> _midLeft;
        std::unique_ptr<WheelUnit> _rearRight;
        std::unique_ptr<WheelUnit> _rearLeft;

    public:
        const std::unique_ptr<WheelUnit> &getFrontRight() const;

        void setFrontRight(std::unique_ptr<WheelUnit> frontRight);

        const std::unique_ptr<WheelUnit> &getFrontLeft() const;

        void setFrontLeft(std::unique_ptr<WheelUnit> frontLeft);

        const std::unique_ptr<WheelUnit> &getMidRight() const;

        void setMidRight(std::unique_ptr<WheelUnit> midRight);

        const std::unique_ptr<WheelUnit> &getMidLeft() const;

        void setMidLeft(std::unique_ptr<WheelUnit> midLeft);

        const std::unique_ptr<WheelUnit> &getRearRight() const;

        void setRearRight(std::unique_ptr<WheelUnit> rearRight);

        const std::unique_ptr<WheelUnit> &getRearLeft() const;

        void setRearLeft(std::unique_ptr<WheelUnit> rearLeft);

        double getSpacing() const;

        void setSpacing(double spacing);

        double getDistanceToFront() const;

        void setDistanceToFront(double distanceToFront);

        double getDistanceToRear() const;

        void setDistanceToRear(double distanceToRear);
    };

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_DRIVE_H
