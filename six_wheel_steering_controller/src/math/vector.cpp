#include "six_wheel_steering_controller/math/vector.h"
#include <cmath>
#include <stdexcept>

namespace six_wheel_steering_controller {
    namespace math {
        Vector::Vector(double x, double y)
                : x(x), y(y) {
        }

        Vector
        Vector::operator+(const Vector &v) const {
            return Vector(x + v.x, y + v.y);
        }

        Vector
        Vector::operator-(const Vector &v) const {
            return Vector(x - v.x, y - v.y);
        }

        Vector Vector::operator-() const {
            return Vector(-x, -y);
        }

        Vector Vector::operator*(double k) const {
            return Vector(x * k, y * k);
        }

        Vector &Vector::operator*=(double k) {
            x *= k;
            y *= k;
            return *this;
        }

        Vector &
        Vector::operator+=(const Vector &v) {
            x += v.x;
            y += v.y;
            return *this;
        }

        Vector &
        Vector::operator-=(const Vector &v) {
            x -= v.x;
            y -= v.y;
            return *this;
        }

        double Vector::length() const {
            return std::hypot(x, y);
        }

        Vector Vector::fromLengthAndAngle(double length, double angle) {
            return Vector(length * std::cos(angle), length * std::sin(angle));
        }

        Vector &Vector::rotate(double angle) {
            const double x1 = x;
            const double y1 = y;
            x = std::cos(angle) * x1 - std::sin(angle) * y1;
            y = std::sin(angle) * x1 + std::cos(angle) * y1;

            return *this;
        }

        double Vector::angle() const {
            return std::atan2(y,x);
        }

        Vector &Vector::normalize() {
            if(x==0&&y==0)
                throw std::domain_error("Cannto normalize vector with length equal to 0");
            const double l = length();
            x/=l;
            y/=l;
            return *this;
        }
    }
}
