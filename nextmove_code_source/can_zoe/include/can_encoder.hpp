#pragma once
#include <memory>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "vehicle_command.hpp"

class CanEncoder : public rclcpp::Node
{
    protected:
        std::unordered_map<int, std::shared_ptr<IVehicleCommand>> vehicleCommands_;
        std::vector<int>                                    countersCmd_;
        std::vector<rclcpp::Time>                           t_valuesAtTime_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr  pubCanRx_;
        rclcpp::Time                                        t_exec_end_;
        rclcpp::TimerBase::SharedPtr                        timer_;
        void timing_check();
        int _computeCRC16Checksum(int counterCmd, std::array<unsigned char, 8> &canData) const;
        // double _delta_time(const rclcpp::Time &t1, const rclcpp::Time &t2) const;
        const rclcpp::Duration _delta_time(const rclcpp::Time &t1, const rclcpp::Time &t2) const;
        rclcpp::Clock::SharedPtr clock_;
    public:
        CanEncoder(const char* nodeName, const int numberOfOptions);
        ~CanEncoder() = default;
        void    control_timer_callback();
};