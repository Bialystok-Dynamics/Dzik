#ifndef SIX_WHEEL_STEERING_CONTROLLER_VECTOR_H
#define SIX_WHEEL_STEERING_CONTROLLER_VECTOR_H


namespace six_wheel_steering_controller {
    namespace math {

        class Vector {

        public:
            double x;
            double y;

            explicit Vector(double x = 0, double y = 0);

            Vector operator+(const Vector &v) const;

            Vector operator-(const Vector &v) const;

            Vector operator-() const;

            Vector operator*(double k) const;

            Vector &operator*=(double k);

            Vector &operator+=(const Vector &v);

            Vector &operator-=(const Vector &v);

            double length() const;
            double angle() const;

            Vector &rotate(double angle);
            Vector &normalize();

            static Vector fromLengthAndAngle(double length, double angle);
        };
    }
}


#endif //SIX_WHEEL_STEERING_CONTROLLER_VECTOR_H
