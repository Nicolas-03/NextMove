#ifndef ZE50CANDECODER_H
#define ZE50CANDECODER_H

#include "rclcpp/rclcpp.hpp"
#include "can_zoe_msgs/msg/frame.hpp"
#include "can_zoe_msgs/msg/activation_robotisation.hpp"
#include "can_zoe_msgs/msg/cmd_acceleration.hpp"
#include "can_zoe_msgs/msg/cmd_brake.hpp"
#include "can_zoe_msgs/msg/cmd_options.hpp"
#include "can_zoe_msgs/msg/cmd_rndb.hpp"
#include "can_zoe_msgs/msg/cmd_steering.hpp"
#include "can_zoe_msgs/msg/rear_wheels_steering.hpp"
#include "can_zoe_msgs/msg/front_wheels_steering.hpp"
#include "can_zoe_msgs/msg/accessories.hpp"
#include "can_zoe_msgs/msg/accel_err_info.hpp"
#include "can_zoe_msgs/msg/accel_info.hpp"
#include "can_zoe_msgs/msg/brake_err_info.hpp"
#include "can_zoe_msgs/msg/brake_info.hpp"
#include "can_zoe_msgs/msg/steering_err_info.hpp"
#include "can_zoe_msgs/msg/steering_info.hpp"
#include "can_zoe_msgs/msg/rndb_err_info.hpp"
#include "can_zoe_msgs/msg/rndb_info.hpp"

#include "can_zoe_msgs/msg/condition_activation.hpp"
#include "can_zoe_msgs/msg/control_panel.hpp"

#include "can_msgs/msg/frame.hpp"
#include "can_ze50.hpp"
#include "can_decoder.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ZE50CanDecoder : public rclcpp::Node, public virtual ICanDecoder
{
  private: 
      rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subCanTx_;
      rclcpp::Publisher<can_zoe_msgs::msg::ConditionActivation>::SharedPtr pub_cdt_act_;
      rclcpp::Publisher<can_zoe_msgs::msg::ActivationRobotisation>::SharedPtr pub_robot_act_;
      rclcpp::Publisher<can_zoe_msgs::msg::ControlPanel>::SharedPtr pub_panel_;
      rclcpp::Publisher<can_zoe_msgs::msg::RearWheelsSteering>::SharedPtr pub_rear_wheels_steering_;
      rclcpp::Publisher<can_zoe_msgs::msg::FrontWheelsSteering>::SharedPtr pub_front_wheels_steering_;
      rclcpp::Publisher<can_zoe_msgs::msg::Accessories>::SharedPtr pub_accessories_;
      rclcpp::Publisher<can_zoe_msgs::msg::SteeringInfo>::SharedPtr pub_steer_info_;
      rclcpp::Publisher<can_zoe_msgs::msg::SteeringErrInfo>::SharedPtr pub_steer_err_info_;
      rclcpp::Publisher<can_zoe_msgs::msg::AccelInfo>::SharedPtr pub_accel_info_;
      rclcpp::Publisher<can_zoe_msgs::msg::AccelErrInfo>::SharedPtr pub_accel_err_info_;
      rclcpp::Publisher<can_zoe_msgs::msg::BrakeInfo>::SharedPtr pub_brake_info_;
      rclcpp::Publisher<can_zoe_msgs::msg::BrakeErrInfo>::SharedPtr pub_brake_err_info_;
      rclcpp::Publisher<can_zoe_msgs::msg::RNDBInfo>::SharedPtr pub_rndb_info_;
      rclcpp::Publisher<can_zoe_msgs::msg::RNDBErrInfo>::SharedPtr pub_rndb_err_info_;
      void decodeCanMsg(const can_msgs::msg::Frame::SharedPtr canMsg) const;
      
  public:
      ZE50CanDecoder();
      ~ZE50CanDecoder() = default;
      
};

#endif
