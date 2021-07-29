#ifndef SIX_WHEEL_STEERING_CONTROLLER_WHEEL_UNIT_H
#define SIX_WHEEL_STEERING_CONTROLLER_WHEEL_UNIT_H

#include "common.h"

namespace six_wheel_steering_controller{

    class WheelUnit {
    public:

        WheelUnitInfo getInfo() const; // todo: implement
        void setCommand(WheelUnitInfo command) const;


    private:
        double _wheelRadius=0;
        double _angleMin=0;
        double _angleMax=0;
        double _rotationalSpeedMax=0;

    public:
        double getWheelRadius() const;

        void setWheelRadius(double wheelRadius);

        double getAngleMin() const;

        void setAngleMin(double angleMin);

        double getAngleMax() const;

        void setAngleMax(double angleMax);

        double getRotationalSpeedMax() const;

        void setRotationalSpeedMax(double rotationalSpeedMax);
    };
}



#endif //SIX_WHEEL_STEERING_CONTROLLER_WHEEL_UNIT_H
