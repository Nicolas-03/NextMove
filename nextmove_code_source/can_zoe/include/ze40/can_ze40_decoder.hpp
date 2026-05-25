#pragma once 

#include "rclcpp/rclcpp.hpp"
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

#include "can_decoder.hpp"
#include "ze40/can_ze40.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ZE40CanDecoder : public rclcpp::Node, public virtual ICanDecoder
{
    private: 
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subCanTx_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::Speed>::SharedPtr pubSpeed_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::IMU>::SharedPtr pubIMU_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::Odom>::SharedPtr pubOdom_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::Steering>::SharedPtr pubSteering_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::ActivationCondition>::SharedPtr pubCond_A_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::ActivationCondition>::SharedPtr pubCond_B_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::PRNDCmdInfo>::SharedPtr pubPRNDStatus_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::BattInfo>::SharedPtr pubBattInfo_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::TorqueInfo>::SharedPtr pubTorqueInfo_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::SelectorAInfo>::SharedPtr pubSelectorInfo_a_;
        rclcpp::Publisher<can_zoe_ze40_msgs::msg::SelectorBInfo>::SharedPtr pubSelectorInfo_b_;
        void decodeCanMsg(const can_msgs::msg::Frame::SharedPtr canMsg) const;

    public:
        ZE40CanDecoder();
        ~ZE40CanDecoder() = default;
      
};
