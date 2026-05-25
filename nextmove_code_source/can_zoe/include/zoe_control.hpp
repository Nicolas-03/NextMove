#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "can_msgs/msg/frame.hpp"

#include "can_zoe_ze40_msgs/msg/speed.hpp"
#include "can_zoe_ze40_msgs/msg/imu.hpp"
#include "can_zoe_ze40_msgs/msg/odom.hpp"
#include "can_zoe_ze40_msgs/msg/steering.hpp"
#include "can_zoe_ze40_msgs/msg/activation_condition.hpp"
#include "can_zoe_ze40_msgs/msg/prnd_cmd_info.hpp"
#include "can_zoe_ze40_msgs/msg/batt_info.hpp"
#include "can_zoe_ze40_msgs/msg/torque_info.hpp"
#include "can_zoe_ze40_msgs/msg/selector_a_info.hpp"
#include "can_zoe_ze40_msgs/msg/selector_b_info.hpp"

#include "can_zoe_ze40_msgs/msg/robotization_activation.hpp"

#include "can_zoe_ze40_msgs/msg/cmd_options.hpp"
#include "can_zoe_ze40_msgs/msg/cmd_acceleration.hpp"
#include "can_zoe_ze40_msgs/msg/cmd_brake.hpp"
#include "can_zoe_ze40_msgs/msg/cmd_steering.hpp"
#include "can_zoe_ze40_msgs/msg/speed_control.hpp"

#include "ze40_can.h"
//#include "ze40_can.c"

#include <boost/thread.hpp>

class ZoeControl : public rclcpp::Node
{
    public:
        ZoeControl();
        void threadLock();
        bool run_th;
    private:
        void cmdSendAccel (const can_zoe_ze40_msgs::msg::CmdAcceleration::SharedPtr cmdAccel);
        void cmdSendBrake (const can_zoe_ze40_msgs::msg::CmdBrake::SharedPtr cmdBrake);
        void cmdSendSteering (const can_zoe_ze40_msgs::msg::CmdSteering::SharedPtr cmdSteering);
        void cmdSendOptions (const can_zoe_ze40_msgs::msg::CmdOptions::SharedPtr cmdOption);
        void cmdActivationRobotization (const can_zoe_ze40_msgs::msg::RobotizationActivation::SharedPtr cmdActivation);
        
        bool publishAccel;
        bool publishBrake;
        bool publishSteer;
        bool publishAccelRequest;
        bool publishBrakeRequest;
        bool publishSteerRequest;

        // Subscribers
        rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdAcceleration>::SharedPtr subCmdAccel;
        rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdBrake>::SharedPtr subCmdBrake;
        rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdSteering>::SharedPtr subCmdSteering;
        rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdOptions>::SharedPtr subCmdOptions;
        rclcpp::Subscription<can_zoe_ze40_msgs::msg::RobotizationActivation>::SharedPtr subCmdActivationRobot;

        // Publisher for the raw can RX messages
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pubCanRx;


        size_t count_;

        uint8_t counterCmdAcceleration = 0;
        uint8_t counterCmdBrake = 0;
        uint8_t counterCmdSteer = 0;
        uint8_t counterCmdPRND = 0;
        uint8_t counterCmdOptions = 0;
        uint8_t counterCmdActivation = 0;


        rclcpp::Time t_accel, t_brake, t_steer, t_ref, t_exec_end;

        float accelValue;
        float brakeValue;
        float steerValue;
        float steerSpeedValue;
        uint8_t steercoopValue;
        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        std_msgs::msg::Header accelValueHeader;
        std_msgs::msg::Header brakeValueHeader;
        std_msgs::msg::Header steerValueHeader;
};