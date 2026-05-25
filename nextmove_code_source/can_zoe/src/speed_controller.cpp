/*
Created on 14/12/2022

@author: Maxime Duquesne (CRIStAL UMR CNRS 9189, Université de Lille)

@license: CeCILL 2.1 (https://cecill.info/licences/Licence_CeCILL_V2.1-en.html) compatible with the GNU, Affero, GPL and EUPL licences.

@brief: zoe speed control.
*/

#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "std_msgs/msg/uint8.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "can_msgs/msg/frame.hpp"
#include "can_zoe_msgs/msg/rear_wheels_steering.hpp"
// #include "can_zoe_msgs/msg/imu.hpp" // not needed
// #include "can_zoe_msgs/msg/odom.hpp" // not needed
// #include "can_zoe_msgs/msg/steering_info.hpp" // functionnal but not needed
// #include "can_zoe_msgs/msg/activation_condition.hpp" // not needed
// #include "can_zoe_msgs/msg/prnd_cmd_info.hpp" // not needed
// #include "can_zoe_msgs/msg/batt_info.hpp" // not needed
// #include "can_zoe_msgs/msg/torque_info.hpp" // not needed
// #include "can_zoe_msgs/msg/selector_a_info.hpp" // not needed
// #include "can_zoe_msgs/msg/selector_b_info.hpp" // not needed
// #include "can_zoe_msgs/msg/robotization_activation.hpp" // not needed
// #include "can_zoe_msgs/msg/cmd_options.hpp" // not needed
#include "can_zoe_msgs/msg/cmd_acceleration.hpp"
#include "can_zoe_msgs/msg/cmd_brake.hpp"
// #include "can_zoe_msgs/msg/cmd_steering.hpp" // not needed
// #include "can_zoe_msgs/msg/speed_control.hpp" // not needed

#include "ze50/can_ze50.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SpeedController : public rclcpp::Node
{
  public:
    SpeedController()
    : Node("speed_controller"), count_(0)
    {
      // Sub
      subCmdVel = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 100, std::bind(&SpeedController::cmdSpeedControl, this, _1));
      subRearWheelsSteering = this->create_subscription<can_zoe_msgs::msg::RearWheelsSteering>("/can/rear_wheels_steering", 100, std::bind(&SpeedController::getCurrentSpeed, this, _1));
      
      // Pub
      pubSpeedAccel = this->create_publisher<can_zoe_msgs::msg::CmdAcceleration>("/cmd_accel", 100);
      pubSpeedBrake = this->create_publisher<can_zoe_msgs::msg::CmdBrake>("/cmd_brake", 100);
      
    }

  private:
    // callback
    void getCurrentSpeed (const can_zoe_msgs::msg::RearWheelsSteering::SharedPtr msgSpeed)
    {
      double rear_left_wheel_steering = msgSpeed->vit_roue_arg;
      double rear_right_wheel_steering = msgSpeed->vit_roue_ard;

      // wheels direction (0=stop 1=forward 2=backward)
      uint8_t rear_left_wheel_direction = msgSpeed->sens_roue_arg;
      uint8_t rear_right_wheel_direction = msgSpeed->sens_roue_ard;

      if (rear_left_wheel_direction == 0 && rear_right_wheel_direction == 0) {
        currentSpeed = 0;
      }

      if (rear_left_wheel_direction == 1 && rear_right_wheel_direction == 1) {
        float average_wheel_speed = (rear_left_wheel_steering + rear_right_wheel_steering) / 2;
        currentSpeed = (average_wheel_speed * wheel_circumference) * 0.0006; // cm/min to km/h conversion 
      }

    }
    void cmdSpeedControl (const geometry_msgs::msg::Twist::SharedPtr msgSpeedControl)
    {
      int error = 100;
      rclcpp::Time tnow = this->get_clock()->now();
      // Time gesture
      dt =  tnow - rclcpp::Duration(t_old.seconds(), t_old.nanoseconds());
      
      err = (msgSpeedControl->linear.x * 3.6) - currentSpeed; //desired speed - current vehicle speed;
      
      // PID
      
      P = Kp * err; 
      I = I + Ki * err * dt.seconds();  // à vérifier (niveau temps)
      D = Kd * err / dt.seconds();
      
      double U = P + I + D;
      
      // Next we update the CAN data 
      
      if( U < 0 ){
        // if deccel
        U = abs(U); 
        if (U > 30.0) U = 30.0;
        cmdAccelSPC.accel = 0.0;
        cmdBrakeSPC.brake = U;
        cmdAccelSPC.can_publish = true;
        cmdBrakeSPC.can_publish = true;
        
        // Publish accel and brake
        pubSpeedAccel->publish(cmdAccelSPC);
        pubSpeedBrake->publish(cmdBrakeSPC);
      } else // if accel
      { 
          if (U > 30.0) U = 30.0;
          cmdBrakeSPC.brake = 0.0;
          cmdAccelSPC.accel = U;
          cmdAccelSPC.can_publish = true;
          cmdBrakeSPC.can_publish = true;
          
          // Publish accel and brake
          pubSpeedAccel->publish(cmdAccelSPC);
          pubSpeedBrake->publish(cmdBrakeSPC);
        }
        t_old = tnow;

      (void)error; // disable "unused variable" warning
    }
    // variables
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Sub
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel;
    rclcpp::Subscription<can_zoe_msgs::msg::RearWheelsSteering>::SharedPtr subRearWheelsSteering;
    
    // Pub
    rclcpp::Publisher<can_zoe_msgs::msg::CmdAcceleration>::SharedPtr pubSpeedAccel;
    rclcpp::Publisher<can_zoe_msgs::msg::CmdBrake>::SharedPtr pubSpeedBrake;
    
    
    double Kp, Kd, Ki, P, I, D, err;
    float currentSpeed;
    const float wheel_circumference = 194.78; // wheel circumference in centimeters
    uint8_t counterCmdAcceleration = 0;
    uint8_t counterCmdBrake = 0;
    size_t count_;
    rclcpp::Time dt = rclcpp::Time();
    rclcpp::Time t_old = rclcpp::Time();
    can_msgs::msg::Frame canMsgAccel;
    can_msgs::msg::Frame canBrakeMsg;
    
    can_zoe_msgs::msg::CmdAcceleration cmdAccelSPC = can_zoe_msgs::msg::CmdAcceleration();
    can_zoe_msgs::msg::CmdBrake cmdBrakeSPC = can_zoe_msgs::msg::CmdBrake();
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedController>());
  rclcpp::shutdown();
  return 0;
}
