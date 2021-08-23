#include "six_wheel_steering_controller/math/point.h"

namespace six_wheel_steering_controller {
    namespace math {
        Point Point::operator+(const Vector &vector) const {
            return Point(x + vector.x, y + vector.y);
        }

        Point Point::operator-(const Vector &vector) const {
            return Point(x-vector.x, y-vector.y);
        }

        Point &Point::operator+=(const Vector &vector) {
            x+=vector.x;
            y+=vector.y;
            return *this;
        }

        Point &Point::operator-=(const Vector &vector) {
            x-=vector.x;
            y-=vector.y;
            return *this;
        }

    }
}
