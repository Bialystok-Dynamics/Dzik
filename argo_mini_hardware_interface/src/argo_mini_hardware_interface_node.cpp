#include "ros/ros.h"
#include <argo_mini_hardware_interface/argo_mini_hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>


namespace AMHI = argo_mini_hardware_interface;

class TimerCallback {
public:
    TimerCallback(AMHI::HardwareInterface *robot, controller_manager::ControllerManager *cm) :
            robot_{robot}, controllerManager_{cm} {}

    void operator()(const ros::SteadyTimerEvent &e) {
        assert(robot_ && controllerManager_);
        const auto &currentTime = ros::Time(e.current_real.toSec());
        const auto &period = ros::Duration((e.current_real - e.last_real).toSec());
        robot_->read(currentTime, period);
        controllerManager_->update(currentTime, period);
        robot_->write(currentTime, period);
    }

private:
    AMHI::HardwareInterface *robot_;
    controller_manager::ControllerManager *controllerManager_;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "argo_mini_hardware_interface_node");

    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    AMHI::HardwareInterface robot;

    if (!robot.init(nh, privateNh)) return EXIT_FAILURE;
    controller_manager::ControllerManager cm(&robot, nh);

    auto timer = nh.createSteadyTimer(ros::WallRate(50).expectedCycleTime(), TimerCallback(&robot, &cm));

    ros::CallbackQueue q;
    nh.setCallbackQueue(&q);
    ros::AsyncSpinner spinner(2, &q);

    spinner.start();
    ros::waitForShutdown();
}