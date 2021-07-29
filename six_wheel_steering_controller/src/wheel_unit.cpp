#include <six_wheel_steering_controller/wheel_unit.h>
#include <stdexcept>

namespace six_wheel_steering_controller{

    double WheelUnit::getWheelRadius() const {
        return _wheelRadius;
    }

    void WheelUnit::setWheelRadius(double wheelRadius) {
        if(wheelRadius <0)
            throw std::runtime_error("Wheel radius have to be greater or equal to 0");
        _wheelRadius = wheelRadius;
    }

    double WheelUnit::getAngleMin() const {
        return _angleMin;
    }

    void WheelUnit::setAngleMin(double angleMin) {
        _angleMin = angleMin;
    }

    double WheelUnit::getAngleMax() const {
        return _angleMax;
    }

    void WheelUnit::setAngleMax(double angleMax) {
        _angleMax = angleMax;
    }

    double WheelUnit::getRotationalSpeedMax() const {
        return _rotationalSpeedMax;
    }

    void WheelUnit::setRotationalSpeedMax(double rotationalSpeedMax) {
        if(rotationalSpeedMax < 0)
            throw std::runtime_error("Maximal rotational speed have to be greater or equal to 0");
        _rotationalSpeedMax = rotationalSpeedMax;
    }
}

