/*
Created on 14/12/2022

@author: Maxime Duquesne (CRIStAL UMR CNRS 9189, Université de Lille)

@license: CeCILL 2.1 (https://cecill.info/licences/Licence_CeCILL_V2.1-en.html) compatible with the GNU, Affero, GPL and EUPL licences.

@brief: zoe steer control.
*/

#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "can_zoe_ze40_msgs/msg/cmd_steering.hpp"

using std::placeholders::_1;

class SteerTeleop : public rclcpp::Node
{
  public:
    SteerTeleop()
    : Node("steer_teleop")
    {
      // Sub
      subCmdAngular = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 100, std::bind(&SteerTeleop::getCmdAngular, this, _1));
      
      // Pub
      pubCmdSteer = this->create_publisher<can_zoe_ze40_msgs::msg::CmdSteering>("/cmd_steering", 100);
      
    }

  private:
    // callback
    void getCmdAngular(const geometry_msgs::msg::Twist::SharedPtr cmdAngular)
    {
      can_zoe_ze40_msgs::msg::CmdSteering cmdSteer;
      
      angle = cmdAngular->angular.z * 270;
      
      cmdSteer.angle = angle;
      cmdSteer.rotation_speed = 350.0;
      cmdSteer.can_publish = true;
      
      // Publish Steer
      pubCmdSteer->publish(cmdSteer);
    }
    // variables
    
    // Sub
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdAngular;
    
    // Pub
    rclcpp::Publisher<can_zoe_ze40_msgs::msg::CmdSteering>::SharedPtr pubCmdSteer;
    
    double angle = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteerTeleop>());
  rclcpp::shutdown();
  return 0;
}
