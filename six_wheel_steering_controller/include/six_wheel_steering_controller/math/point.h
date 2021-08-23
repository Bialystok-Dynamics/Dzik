#ifndef SIX_WHEEL_STEERING_CONTROLLER_POINT_H
#define SIX_WHEEL_STEERING_CONTROLLER_POINT_H

#include "vector.h"

namespace six_wheel_steering_controller {

    namespace math{
        class Point {
        public:
            double x=0.;
            double y=0.;

            explicit Point(double x=0, double y=0): x(x), y(y)
            {}

            Point operator+(const Vector& vector) const;

            Point operator-(const Vector& vector) const;

            Point& operator+=(const Vector& vector);

            Point& operator-=(const Vector& vector);
        };
    }


}


#endif //SIX_WHEEL_STEERING_CONTROLLER_POINT_H
