#pragma once
#include "vehicle_command.hpp"
#include "can_ze50.hpp"
#include "can_zoe_msgs/msg/cmd_steering.hpp"
#include "can_zoe_msgs/msg/cmd_acceleration.hpp"
#include "can_zoe_msgs/msg/cmd_brake.hpp"
#include "can_zoe_msgs/msg/cmd_rndb.hpp"

enum Command {
    ACCELERATION,
    BRAKE,
    STEER,
    RNDB,
    COUNT_CMD
};

class BrakeCommand : public VehicleCommand
{
    public:
        BrakeCommand() : VehicleCommand("Brake") {};
        ~BrakeCommand() = default;
        status encodeMsg(can_msgs::msg::Frame &canMsg) const;
        void receiveBrakeCommand(const can_zoe_msgs::msg::CmdBrake::SharedPtr cmdBrake);
    private:
        float   brakeValue_;
};

class AccelerationCommand : public VehicleCommand
{
    public:
        AccelerationCommand() : VehicleCommand("Acceleration") {};
        ~AccelerationCommand() = default;
        status  encodeMsg(can_msgs::msg::Frame &canMsg) const;
        void    receiveAccelerationCommand(const can_zoe_msgs::msg::CmdAcceleration::SharedPtr cmdAccel);
    private:
        float   accelerationValue_;
};

class SteerCommand : public VehicleCommand
{
    public:
        SteerCommand() : VehicleCommand("Steer") {};
        ~SteerCommand() = default;
        status encodeMsg(can_msgs::msg::Frame &canMsg) const;
        void    receiveSteerCommand(const can_zoe_msgs::msg::CmdSteering::SharedPtr cmdSteer);
    private:
        float   steerAngle_;
        float   steerRotationSpeed_;
        uint8_t steerCoopValue_; // where is this set? UB
};

class RNDBCommand : public VehicleCommand
{
    public:
        RNDBCommand() : VehicleCommand("RNDB") {};
        ~RNDBCommand() = default;
        status encodeMsg(can_msgs::msg::Frame &canMsg) const;
        void receiveRNDBCommand(const can_zoe_msgs::msg::CmdRNDB::SharedPtr cmdRNDB);
    private: 
        uint8_t rndbValue_; // todo get more info?
        uint8_t rndbRetry_;
};

