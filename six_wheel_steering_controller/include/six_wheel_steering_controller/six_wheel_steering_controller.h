#ifndef SIX_WHEEL_STEERING_CONTROLLER_SIX_WHEEL_STEERING_CONTROLLER_H
#define SIX_WHEEL_STEERING_CONTROLLER_SIX_WHEEL_STEERING_CONTROLLER_H


#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace six_wheel_steering_controller{

    class SixWheelSteeringController: public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
            hardware_interface::PositionJointInterface>{
    public:
        void starting(const ros::Time &time) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void stopping(const ros::Time &time) override;

        void waiting(const ros::Time &time) override;

        void aborting(const ros::Time &time) override;

        bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &handle, ros::NodeHandle &nodeHandle) override;

    };
}

#endif //SIX_WHEEL_STEERING_CONTROLLER_SIX_WHEEL_STEERING_CONTROLLER_H
