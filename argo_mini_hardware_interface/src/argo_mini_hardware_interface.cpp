#include <argo_mini_hardware_interface/argo_mini_hardware_interface.h>


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
        if(serialTimer_.timeForWrite(time)){
            // TODO: do write serial
        }
    }

    bool HardwareInterface::initSerial(ros::NodeHandle &privateNh) {
        std::string serialPath = "argo_mini/serial/";
        double freq;
        int baudrate;
        std::string serialDev;

        if (!privateNh.param(serialPath + "write_frequency", freq, 50.))
            ROS_INFO_STREAM_NAMED(name_, "Couldn't get serial/write_frequency param. Defaulting to 50Hz.");
        if(freq <=0.){
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
        // TODO: init serial

        // if successful
        ROS_INFO_STREAM_NAMED(name_, "Serial parameters:\n"
                                     "- device: " << serialDev << "\n"
                                     "- baudrate: " << baudrate << "\n"
                                     "- write frequency: " << freq << "Hz"
                                     );
        return true;
    }

    bool HardwareInterface::registerHandles(ros::NodeHandle &privateNh) {
        std::string jointNameCache;
        hardware_interface::JointStateHandle handleCache;

#define TRY_GET_JOINT_NAME(jointNameParam)\
                if(!privateNh.getParam(jointNameParam, jointNameCache)){      \
                    ROS_ERROR_STREAM_NAMED(name_, "Couldn't get " jointNameParam " param. Aborting..."); \
                    return false;\
                }

#define TRY_REGISTER_POSITION_HANDLE(jointNameParam, field) \
                TRY_GET_JOINT_NAME(jointNameParam) \
                handleCache = hardware_interface::JointStateHandle(jointNameCache, &positions.field, &velocities.field, &efforts.field); \
                jointStateInterface.registerHandle(handleCache); \
                positionJointInterface.registerHandle(hardware_interface::JointHandle(handleCache,&commands.field)); \
                ROS_INFO_STREAM_NAMED(name_, "Registered handles for position steered joint: "<< handleCache.getName());

#define TRY_REGISTER_VELOCITY_HANDLE(jointNameParam, field) \
                TRY_GET_JOINT_NAME(jointNameParam) \
                handleCache = hardware_interface::JointStateHandle(jointNameCache, &positions.field, &velocities.field, &efforts.field); \
                jointStateInterface.registerHandle(handleCache); \
                velocityJointInterface.registerHandle(hardware_interface::JointHandle(handleCache,&commands.field)); \
                ROS_INFO_STREAM_NAMED(name_, "Registered handles for velocity steered joint: "<< handleCache.getName());

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

