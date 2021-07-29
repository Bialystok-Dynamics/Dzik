#ifndef SIX_WHEEL_STEERING_CONTROLLER_MODE_H
#define SIX_WHEEL_STEERING_CONTROLLER_MODE_H

#include "common.h"

namespace six_wheel_steering_controller{
    class Drive;

    class Mode {

    public:
        Drive *getDrive() const;
        void setDrive(Drive *drive);

        void setCommand(DriveInfo command);

    protected:
        virtual void doSetCommand(DriveInfo command)=0;
        Drive* _drive= nullptr;
    };
}


#endif //SIX_WHEEL_STEERING_CONTROLLER_MODE_H
