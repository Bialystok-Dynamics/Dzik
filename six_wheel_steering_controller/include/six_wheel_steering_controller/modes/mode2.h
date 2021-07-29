#ifndef SIX_WHEEL_STEERING_CONTROLLER_MODE2_H
#define SIX_WHEEL_STEERING_CONTROLLER_MODE2_H

#include "../mode.h"
#include "../drive.h"

namespace six_wheel_steering_controller {

    namespace modes{
        class Mode2 : Mode{


        protected:
            void doSetCommand(DriveInfo command) override;

        private:
            double _maxAngleError = 1e-3;
        public:
            double getMaxAngleError() const;

            void setMaxAngleError(double maxAngleError);

        private:

            void checkAndRun(const std::unique_ptr<WheelUnit> &wheelUnit, WheelUnitInfo command) const;
        };

    }

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_MODE2_H
