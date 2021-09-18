#include <argo_mini_hardware_interface/six_wheel_handler.h>

namespace argo_mini_hardware_interface {
    bool SixWheelHandler::registerHandles(ros::NodeHandle &privateNh, hardware_interface::RobotHW *hw,
                                          std::string logNamespace) {

        name_ = logNamespace;
        std::string jointNameCache;
        hardware_interface::JointStateHandle stateHandleCache;

#define TRY_GET_JOINT_NAME(jointNameParam)\
                if(!privateNh.getParam(jointNameParam, jointNameCache)){      \
                    ROS_ERROR_STREAM_NAMED(name_, "Couldn't get " jointNameParam " param. Aborting..."); \
                    return false;\
                }

#define TRY_REGISTER_POSITION_HANDLE(jointNameParam, field) \
                TRY_GET_JOINT_NAME(jointNameParam) \
                stateHandleCache = hardware_interface::JointStateHandle(jointNameCache, &positions.field, &velocities.field, &efforts.field); \
                jointStateInterface.registerHandle(stateHandleCache); \
                positionJointInterface.registerHandle(hardware_interface::JointHandle(stateHandleCache,&commands.field)); \
                ROS_INFO_STREAM_NAMED(name_, "Registered handle for position steered joint: "<< stateHandleCache.getName());

#define TRY_REGISTER_VELOCITY_HANDLE(jointNameParam, field) \
                TRY_GET_JOINT_NAME(jointNameParam) \
                stateHandleCache = hardware_interface::JointStateHandle(jointNameCache, &positions.field, &velocities.field, &efforts.field); \
                jointStateInterface.registerHandle(stateHandleCache); \
                velocityJointInterface.registerHandle(hardware_interface::JointHandle(stateHandleCache,&commands.field)); \
                ROS_INFO_STREAM_NAMED(name_, "Registered handle for velocity steered joint: "<< stateHandleCache.getName());

        if (!privateNh.getParam("argo_mini/wheel_joints/command_coeff", wheelCommandCoefficient_)) {
            ROS_ERROR_STREAM_NAMED(name_, "Couldn't get argo_mini/wheel_joints/command_coeff param. Aborting ...");
            return false;
        }

        if (!privateNh.getParam("argo_mini/steering_joints/command_coeff", steerCommandCoefficient_)) {
            ROS_ERROR_STREAM_NAMED(name_, "Couldn't get argo_mini/steering_joints/command_coeff param. Aborting ...");
            return false;
        }

        ROS_INFO_STREAM_NAMED(name_, "Protocol parameters:\n"
                << "\t- wheel command coefficient: " << wheelCommandCoefficient_ << " (rad/s)^-1\n"
                << "\t- steer command coefficient: " << steerCommandCoefficient_ << " rad^-1"
        );

        TRY_REGISTER_POSITION_HANDLE("argo_mini/steering_joints/front_left", frontLeftSteer)
        TRY_REGISTER_POSITION_HANDLE("argo_mini/steering_joints/front_right", frontRightSteer)
        TRY_REGISTER_POSITION_HANDLE("argo_mini/steering_joints/mid_left", midLeftSteer)
        TRY_REGISTER_POSITION_HANDLE("argo_mini/steering_joints/mid_right", midRightSteer)
        TRY_REGISTER_POSITION_HANDLE("argo_mini/steering_joints/rear_left", rearLeftSteer)
        TRY_REGISTER_POSITION_HANDLE("argo_mini/steering_joints/rear_right", rearRightSteer)

        TRY_REGISTER_VELOCITY_HANDLE("argo_mini/wheel_joints/front_left", frontLeftWheel)
        TRY_REGISTER_VELOCITY_HANDLE("argo_mini/wheel_joints/front_right", frontRightWheel)
        TRY_REGISTER_VELOCITY_HANDLE("argo_mini/wheel_joints/mid_left", midLeftWheel)
        TRY_REGISTER_VELOCITY_HANDLE("argo_mini/wheel_joints/mid_right", midRightWheel)
        TRY_REGISTER_VELOCITY_HANDLE("argo_mini/wheel_joints/rear_left", rearLeftWheel)
        TRY_REGISTER_VELOCITY_HANDLE("argo_mini/wheel_joints/rear_right", rearRightWheel)

        hw->registerInterface(&jointStateInterface);
        hw->registerInterface(&positionJointInterface);
        hw->registerInterface(&velocityJointInterface);

        return true;
    }

    std::vector<uint8_t> argo_mini_hardware_interface::SixWheelHandler::getData() const {
        auto getWheelCommand = [this](double velocity) {
            auto ret = (int) (velocity * wheelCommandCoefficient_);
            return (uint8_t) (ret > 100 ? 100 : (ret < -100 ? -100 : ret));
        };

        auto getSteerCommand = [this](double angle) {
            auto ret = (int) (angle * steerCommandCoefficient_);
            return (uint8_t) (ret > 100 ? 100 : (ret < -100 ? -100 : ret));
        };

        ROS_DEBUG_STREAM_THROTTLE_NAMED(0.1, "received_commands",
                                        "Received following commands:"
                                        "\n-frontLeftSteer: " << commands.frontLeftSteer <<
                                                              "\n-frontLeftWheel: " << commands.frontLeftWheel <<
                                                              "\n-frontRightSteer: " << commands.frontRightSteer <<
                                                              "\n-frontRightWheel: " << commands.frontRightWheel <<
                                                              "\n-midLeftSteer: " << commands.midLeftSteer <<
                                                              "\n-midLeftWheel: " << commands.midLeftWheel <<
                                                              "\n-midRightSteer: " << commands.midRightSteer <<
                                                              "\n-midRightWheel: " << commands.midRightWheel <<
                                                              "\n-rearLeftSteer: " << commands.rearLeftSteer <<
                                                              "\n-rearLeftWheel: " << commands.rearLeftWheel <<
                                                              "\n-rearRightSteer: " << commands.rearRightSteer <<
                                                              "\n-rearRightWheel: " << commands.rearRightWheel
        );

        return std::vector<uint8_t>{0x9B,
                                    getWheelCommand(commands.frontLeftWheel),
                                    getSteerCommand(commands.frontLeftSteer),
                                    getWheelCommand(commands.frontRightWheel),
                                    getSteerCommand(commands.frontRightSteer),
                                    getWheelCommand(commands.midLeftWheel),
                                    getSteerCommand(commands.midLeftSteer),
                                    getWheelCommand(commands.midRightWheel),
                                    getSteerCommand(commands.midRightSteer),
                                    getWheelCommand(commands.rearLeftWheel),
                                    getSteerCommand(commands.rearLeftSteer),
                                    getWheelCommand(commands.rearRightWheel),
                                    getSteerCommand(commands.rearRightSteer),
                                    0x65};
    }

}
