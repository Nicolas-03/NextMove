#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "can_zoe_msgs/msg/cmd_steering.hpp"

using namespace std::chrono_literals;
using CmdSteering = can_zoe_msgs::msg::CmdSteering; // Remplacez par votre namespace

class CmdSteeringRepublisher : public rclcpp::Node
{
public:
  CmdSteeringRepublisher()
  : Node("cmd_steering_republisher")
  {
    // Abonnement au topic /cmd_steering_request
    subscription_ = this->create_subscription<CmdSteering>(
      "/cmd_steering_request", 10,
      [this](const CmdSteering::SharedPtr msg) {
        last_msg_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Message reçu : angle=%f, v=%f", last_msg_.angle, last_msg_.rotation_speed);
      });

    // Publication sur /cmd_steering à 100 Hz
    publisher_ = this->create_publisher<CmdSteering>("/cmd_steering", 10);

    // Timer à 100 Hz (10 ms)
    timer_ = this->create_wall_timer(
      10ms, [this]() {
        if (last_msg_set_) {
          publisher_->publish(last_msg_);
          RCLCPP_DEBUG(this->get_logger(), "Message republié : angle=%f, v=%f", last_msg_.angle, last_msg_.rotation_speed);
        } else {
          RCLCPP_WARN(this->get_logger(), "Aucun message reçu sur /cmd_steering_request");
        }
      });

    // Initialisation
    last_msg_set_ = true;
  }

private:
  rclcpp::Subscription<CmdSteering>::SharedPtr subscription_;
  rclcpp::Publisher<CmdSteering>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  CmdSteering last_msg_;
  bool last_msg_set_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdSteeringRepublisher>());
  rclcpp::shutdown();
  return 0;
}