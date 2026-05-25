#pragma once
#include "can_encoder.hpp"
#include "ze40_vehicle_command.hpp"

class CanZE40Encoder : public CanEncoder
{
    public:
        CanZE40Encoder();
        ~CanZE40Encoder() = default;

        void receiveBrakeCommand(const can_zoe_msgs::msg::CmdBrake::SharedPtr cmdBrake);
        void receiveAccelerationCommand(const can_zoe_msgs::msg::CmdAcceleration::SharedPtr cmdAccel);
        void receiveSteerCommand(const can_zoe_msgs::msg::CmdSteering::SharedPtr cmdSteer);
        void receiveOptionsCommand(can_zoe_ze40_msgs::msg::CmdOptions::SharedPtr cmdOptions);
        void receiveActivationRobotizationCommand(const can_zoe_ze40_msgs::msg::RobotizationActivation::SharedPtr cmdRA);
        
    private:
        rclcpp::Subscription<can_zoe_msgs::msg::CmdAcceleration>::SharedPtr             subCmdAccel_;
        rclcpp::Subscription<can_zoe_msgs::msg::CmdBrake>::SharedPtr                    subCmdBrake_;
        rclcpp::Subscription<can_zoe_msgs::msg::CmdSteering>::SharedPtr                 subCmdSteering_;
        rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdOptions>::SharedPtr             subCmdOptions_;
        rclcpp::Subscription<can_zoe_ze40_msgs::msg::RobotizationActivation>::SharedPtr subCmdActivationRobot_;
};