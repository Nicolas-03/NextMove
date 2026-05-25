#include "can_encoder.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

CanEncoder::CanEncoder(const char *nodeName, const int numberOfCmds) : Node(nodeName), countersCmd_(numberOfCmds), t_valuesAtTime_(numberOfCmds)
{
    pubCanRx_ = this->create_publisher<can_msgs::msg::Frame>("/can_rx", 100);
    clock_ = this->get_clock();
    t_exec_end_ = clock_->now();
    for (size_t i = 0; i < t_valuesAtTime_.size(); i++)
		this->t_valuesAtTime_[i] = clock_->now();
	timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CanEncoder::control_timer_callback, this));
}

int CanEncoder::_computeCRC16Checksum(int counterCmd, std::array<unsigned char, 8> &canData) const
{
    counterCmd = counterCmd == 16 ? 0 : counterCmd;
    uint16_t chk = canData[0] + canData[1] + canData[2] + canData[3] + canData[4] + canData[5] + canData[6] + counterCmd;
    uint8_t chk_mod16 = chk & 0x000F; //define this
    canData[7] = (counterCmd << 4) + chk_mod16;
    return (++counterCmd);
}

const rclcpp::Duration CanEncoder::_delta_time(const rclcpp::Time &t1, const rclcpp::Time &t2) const
{
    return  t1 - t2;
}

void CanEncoder::timing_check()
{
    rclcpp::Time t_ref_ = clock_->now();
    if (_delta_time(t_ref_, t_exec_end_).seconds() > 0.015)
    {
        for (size_t cmdIdx = 0; cmdIdx < vehicleCommands_.size(); cmdIdx++)
        {
		    RCLCPP_WARN_STREAM_SKIPFIRST(this->get_logger(), "Global Can order Timing issue ");
            this->vehicleCommands_[cmdIdx]->setPublishStatusToFalse();
            this->vehicleCommands_[cmdIdx]->setPublishRequestToFalse();
        }
        return;
    }
    for (size_t cmdIdx = 0; cmdIdx < vehicleCommands_.size(); cmdIdx++)
    {
        if (_delta_time(t_ref_, t_valuesAtTime_[cmdIdx]).seconds() < 0.050)
        {
            this->vehicleCommands_[cmdIdx]->publishEqualsRequest();
            continue;
        }
        if (this->vehicleCommands_[cmdIdx]->isPublishRequested())
            RCLCPP_WARN_STREAM_SKIPFIRST(this->get_logger(), vehicleCommands_[cmdIdx]->getTypeName() << " Can order Timing issue ");
        this->vehicleCommands_[cmdIdx]->setPublishStatusToFalse();
        this->vehicleCommands_[cmdIdx]->setPublishRequestToFalse();
    }

}

void CanEncoder::control_timer_callback()
{
    timing_check();
    can_msgs::msg::Frame canMsg{};
    for (size_t i = 0; i < vehicleCommands_.size(); i++) // Travelling commands (steer, brake...)
    {
        canMsg = can_msgs::msg::Frame{};
        if (vehicleCommands_[i]->isSetToPublish() == false)
			continue;
        if (vehicleCommands_[i]->encodeMsg(canMsg) == SUCCESS)// get error
        {
            countersCmd_[i] = _computeCRC16Checksum(countersCmd_[i], canMsg.data);
            pubCanRx_->publish(canMsg);
        }
        else
            RCLCPP_WARN_STREAM_SKIPFIRST(this->get_logger(), vehicleCommands_[i]->getTypeName() << " CAN data packing error");
        vehicleCommands_[i]->setPublishStatusToFalse();
        vehicleCommands_[i]->setPublishRequestToFalse();
    }
	t_exec_end_ = clock_->now();
}
