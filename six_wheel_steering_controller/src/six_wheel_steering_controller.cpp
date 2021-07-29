#include <six_wheel_steering_controller/six_wheel_steering_controller.h>
#include <pluginlib/class_list_macros.hpp>


namespace six_wheel_steering_controller {

    void SixWheelSteeringController::update(const ros::Time &time, const ros::Duration &period) {

    }

    void SixWheelSteeringController::stopping(const ros::Time &time1) {
        ControllerBase::stopping(time1);
    }

    void SixWheelSteeringController::waiting(const ros::Time &time1) {
        ControllerBase::waiting(time1);
    }

    void SixWheelSteeringController::aborting(const ros::Time &time1) {
        ControllerBase::aborting(time1);
    }

    bool SixWheelSteeringController::init(hardware_interface::RobotHW *hw,
                                          ros::NodeHandle &handle,
                                          ros::NodeHandle &nodeHandle) {
        return MultiInterfaceController::init(hw, handle, nodeHandle);
    }

    void six_wheel_steering_controller::SixWheelSteeringController::starting(const ros::Time &time) {
        ControllerBase::starting(time);
    }
}

PLUGINLIB_EXPORT_CLASS(six_wheel_steering_controller::SixWheelSteeringController, controller_interface::ControllerBase)
