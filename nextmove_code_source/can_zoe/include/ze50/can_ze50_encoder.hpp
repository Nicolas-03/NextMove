#pragma once
#include "can_encoder.hpp"
#include "ze50_vehicle_command.hpp"

class CanZE50Encoder : public CanEncoder
{
    public:
        CanZE50Encoder();
        ~CanZE50Encoder() = default;

        void receiveBrakeCommand(const can_zoe_msgs::msg::CmdBrake::SharedPtr cmdBrake);
        void receiveAccelerationCommand(const can_zoe_msgs::msg::CmdAcceleration::SharedPtr cmdAccel);
        void receiveSteerCommand(const can_zoe_msgs::msg::CmdSteering::SharedPtr cmdSteer);
        void receiveRNDBCommand(const can_zoe_msgs::msg::CmdRNDB::SharedPtr cmdRNDB);
    private:
        rclcpp::Subscription<can_zoe_msgs::msg::CmdAcceleration>::SharedPtr        subCmdAccel_;
        rclcpp::Subscription<can_zoe_msgs::msg::CmdBrake>::SharedPtr        subCmdBrake_;
        rclcpp::Subscription<can_zoe_msgs::msg::CmdRNDB>::SharedPtr         subCmdRNDB_;
        rclcpp::Subscription<can_zoe_msgs::msg::CmdSteering>::SharedPtr     subCmdSteering_;
};