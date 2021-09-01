#include <argo_mini_hardware_interface/argo_mini_hardware_interface.h>
#include "ostream"

namespace argo_mini_hardware_interface {

    bool HardwareInterface::init(ros::NodeHandle &rootNh, ros::NodeHandle &privateNh) {

        std::string completeNs = privateNh.getNamespace();
        auto id = completeNs.find_last_of('/');
        name_ = completeNs.substr(id + 1);

        if (!initSerial(privateNh)) return false;

        if (!registerHandles(privateNh)) return false;

        return true;
    }

    void HardwareInterface::read(const ros::Time &time1, const ros::Duration &duration) {
    }

    void HardwareInterface::write(const ros::Time &time, const ros::Duration &duration) {
        if (serialTimer_.timeForWrite(time)) {
            const auto &data = getDataToWrite();
            std::stringstream ss;
            std::copy(data.begin(), data.end(), std::ostream_iterator<int>(ss << std::hex, " "));
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
            ROS_DEBUG_STREAM_THROTTLE_NAMED(0.1, "serial_comm", "Sending frame: " << ss.str());
            serial_->write(data);
        }
    }

    bool HardwareInterface::initSerial(ros::NodeHandle &privateNh) {
        std::string serialPath = "argo_mini/serial/";
        double freq;
        int baudrate;
        std::string serialDev;

        if (!privateNh.getParam(serialPath + "wheel_command_coeff", wheelCommandCoefficient_)) {
            ROS_INFO_STREAM_NAMED(name_, "Couldn't get serial/wheel_command_coeff param. Aborting ...");
            return false;
        }

        if (!privateNh.getParam(serialPath + "steer_command_coeff", steerCommandCoefficient_)) {
            ROS_INFO_STREAM_NAMED(name_, "Couldn't get serial/steer_command_coeff param. Aborting ...");
            return false;
        }

        if (!privateNh.param(serialPath + "write_frequency", freq, 50.))
            ROS_INFO_STREAM_NAMED(name_, "Couldn't get serial/write_frequency param. Defaulting to 50Hz.");
        if (freq <= 0.) {
            ROS_ERROR_STREAM_NAMED(name_, "serial/write_frequency have to be >0. Aborting ...");
            return false;
        }

        if (!privateNh.param(serialPath + "baudrate", baudrate, 115200))
            ROS_INFO_STREAM_NAMED(name_, "Couldn't get serial/baudrate param. Defaulting to 115200.");

        if (!privateNh.getParam(serialPath + "device", serialDev)) {
            ROS_ERROR_STREAM_NAMED(name_, "Couldn't get serial/device param. Aborting...");
            return false;
        }

        serialTimer_ = SerialTimer(freq, ros::Time::now());

        try {
            serial_ = std::make_unique<serial::Serial>(
                    serialDev, (uint32_t) baudrate, serial::Timeout::simpleTimeout(50));
        } catch (const serial::PortNotOpenedException &e) {
            ROS_ERROR_STREAM_NAMED(name_, "Serial port opening exception: " << e.what());
            return false;
        } catch (const std::invalid_argument &e) {
            ROS_ERROR_STREAM_NAMED(name_, "Serial invalid argument: " << e.what());
            return false;
        } catch (const std::exception &e) {
            ROS_ERROR_STREAM_NAMED(name_, "Serial exception: " << e.what());
            return false;
        }

        // if successful
        ROS_INFO_STREAM_NAMED(name_, "Serial parameters:\n"
                << "- device: " << serial_->getPort() << "\n"
                << "- baudrate: " << serial_->getBaudrate() << "\n"
                << "- write frequency: " << freq << " Hz\n"
                << "- wheel command coefficient: " << wheelCommandCoefficient_ << " (rad/s)^-1\n"
                << "- steer command coefficient: " << steerCommandCoefficient_ << " rad^-1"
        );
        return true;
    }

    std::vector<uint8_t> HardwareInterface::getDataToWrite() const {
        auto getWheelCommand = [this](double velocity) {
            auto ret = (int)(velocity * wheelCommandCoefficient_);
            return (uint8_t) (ret > 100 ? 100 : (ret < -100 ? -100 : ret));
        };

        auto getSteerCommand = [this](double angle) {
            auto ret = (int)(angle * steerCommandCoefficient_);
            return (uint8_t) (ret > 100 ? 100 : (ret < -100 ? -100 : ret));
        };

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

    bool HardwareInterface::registerHandles(ros::NodeHandle &privateNh) {
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
                ROS_INFO_STREAM_NAMED(name_, "Registered handles for position steered joint: "<< stateHandleCache.getName());

#define TRY_REGISTER_VELOCITY_HANDLE(jointNameParam, field) \
                TRY_GET_JOINT_NAME(jointNameParam) \
                stateHandleCache = hardware_interface::JointStateHandle(jointNameCache, &positions.field, &velocities.field, &efforts.field); \
                jointStateInterface.registerHandle(stateHandleCache); \
                velocityJointInterface.registerHandle(hardware_interface::JointHandle(stateHandleCache,&commands.field)); \
                ROS_INFO_STREAM_NAMED(name_, "Registered handles for velocity steered joint: "<< stateHandleCache.getName());

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

        registerInterface(&jointStateInterface);
        registerInterface(&positionJointInterface);
        registerInterface(&velocityJointInterface);

        return true;
    }
}

