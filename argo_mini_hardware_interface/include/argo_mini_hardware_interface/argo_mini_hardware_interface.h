#ifndef ARGO_MINI_HARDWARE_INTERFACE_ARGO_MINI_HARDWARE_INTERFACE_H
#define ARGO_MINI_HARDWARE_INTERFACE_ARGO_MINI_HARDWARE_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <utility>

namespace argo_mini_hardware_interface {

    class HardwareInterface : public hardware_interface::RobotHW {

    public:
        ~HardwareInterface() override = default;

        bool init(ros::NodeHandle &rootNh, ros::NodeHandle &privateNh) override;

        void read(const ros::Time &time, const ros::Duration &duration) override;

        void write(const ros::Time &time, const ros::Duration &duration) override;

    private:
        bool initSerial(ros::NodeHandle &privateNh);

        bool registerHandles(ros::NodeHandle &privateNh);

        std::string name_;

        hardware_interface::JointStateInterface jointStateInterface;
        hardware_interface::PositionJointInterface positionJointInterface;
        hardware_interface::VelocityJointInterface velocityJointInterface;

        struct HardwareInfo {
            double frontLeftSteer;
            double frontRightSteer;
            double midLeftSteer;
            double midRightSteer;
            double rearLeftSteer;
            double rearRightSteer;

            double frontLeftWheel;
            double frontRightWheel;
            double midLeftWheel;
            double midRightWheel;
            double rearLeftWheel;
            double rearRightWheel;
        } commands, velocities, positions, efforts;


        class SerialTimer {
        public:
            SerialTimer(double frequency, const ros::Time &initTime) :
                    period_(1/frequency), lastWrite_(initTime) {}

            SerialTimer() = default;

            bool timeForWrite(const ros::Time &now) {
                if ((now - lastWrite_) >= period_) {
                    lastWrite_ = now;
                    return true;
                } else return false;
            }

        private:
            ros::Duration period_;
            ros::Time lastWrite_;
        } serialTimer_;
    };

}


#endif //ARGO_MINI_HARDWARE_INTERFACE_ARGO_MINI_HARDWARE_INTERFACE_H
