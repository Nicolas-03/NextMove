/*
Created on 14/12/2022

Author : Maxime Duquesne

This software is to control the Renault Zoe by a separated UI with can_zoe_ze40_msgs version 2 with thread.

LICENCE : to define

*/
#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

#include "can_zoe_ze40_msgs/msg/robotization_activation.hpp"

#include "can_zoe_ze40_msgs/msg/cmd_options.hpp"
#include "can_zoe_ze40_msgs/msg/cmd_acceleration.hpp"
#include "can_zoe_ze40_msgs/msg/cmd_brake.hpp"
#include "can_zoe_ze40_msgs/msg/cmd_steering.hpp"
#include "can_zoe_ze40_msgs/msg/speed_control.hpp"

#include "ze40/can_ze40.h"

#include <boost/thread.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

double time_compare (rclcpp::Time t1, rclcpp::Time t2)
{
  double dt;
  dt =  t1.seconds() - t2.seconds() + (t1.nanoseconds() - t2.nanoseconds())/1000000000;
  return dt;
}

class ZoeControl : public rclcpp::Node
{
  public:
    ZoeControl()
    : Node("zoe_control"), count_(0)
    {
      // Sub
      subCmdAccel = this->create_subscription<can_zoe_ze40_msgs::msg::CmdAcceleration>("/cmd_accel", 100, std::bind(&ZoeControl::cmdSendAccel, this, _1));
      subCmdBrake = this->create_subscription<can_zoe_ze40_msgs::msg::CmdBrake>("/cmd_brake", 100, std::bind(&ZoeControl::cmdSendBrake, this, _1));
      subCmdSteering = this->create_subscription<can_zoe_ze40_msgs::msg::CmdSteering>("/cmd_steering", 100, std::bind(&ZoeControl::cmdSendSteering, this, _1));
      subCmdOptions = this->create_subscription<can_zoe_ze40_msgs::msg::CmdOptions>("/cmd_options", 100, std::bind(&ZoeControl::cmdSendOptions, this, _1));
      subCmdActivationRobot = this->create_subscription<can_zoe_ze40_msgs::msg::RobotizationActivation>("/cmd_activationrobot", 100, std::bind(&ZoeControl::cmdActivationRobotization, this, _1));
      
      // Pub
      pubCanRx = this->create_publisher<can_msgs::msg::Frame>("/can_rx", 100);
      
      
      
      timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ZoeControl::threadLock, this));
      printf("init finished\n");
    }
    void threadLock()
    {
      printf("threadLock\n");

        t_ref = this->get_clock()->now();
        double dt = time_compare (t_ref, t_exec_end);
        
        if(dt > 0.015) // if dt of loop exec was > 15ms 
        {
          // add ros warning
          publishAccel = false;
          publishBrake = false;
          publishSteer = false;
        } else {
          if((time_compare (this->get_clock()->now(), t_accel)) > 0.050)
          {
            publishAccel = false;
            // ros warning
          } else {
            if(publishAccelRequest == true)
            {
              publishAccel = true;
            } else {
              publishAccel = false;
            }
          }
        if((time_compare (this->get_clock()->now(), t_brake)) > 0.050){
          publishBrake = false;
          //ros warning
        } else {
          if(publishBrakeRequest == true){
            publishBrake = true;
          }else {
            publishBrake = false;
          }
        }
        if((time_compare (this->get_clock()->now(), t_steer)) > 0.050){
          publishSteer = false;
          printf("publishSteer1:%d\n", publishSteer);

          // ros warning
        } else {
          if(publishSteerRequest == true){
            printf("publishSteer2:%d\n", publishSteer);
            publishSteer = true;
          } else {
            printf("publishSteer3:%d\n", publishSteer);

            publishSteer = false;
          }
        }
        int error = 100;
        // Command publication according to booleans (we have to send true or false)
        if(publishAccel == true) {
          // Encodre Accel Msg on CAN
          ze40_can_id_200_cons_accel_autonome_t canCmdAcceleration;
          canCmdAcceleration.cons_accelateur_autonome = ze40_can_id_200_cons_accel_autonome_cons_accelateur_autonome_encode(accelValue);
          
          can_msgs::msg::Frame canMsg;
          canMsg.header = accelValueHeader;
          canMsg.id = ZE40_CAN_ID_200_CONS_ACCEL_AUTONOME_FRAME_ID;
          canMsg.dlc = ZE40_CAN_ID_200_CONS_ACCEL_AUTONOME_LENGTH;
          canMsg.is_rtr = false;
          canMsg.is_extended = false;
          canMsg.is_error = false;
          
          uint8_t canData[8];
          error = ze40_can_id_200_cons_accel_autonome_pack(canData, &canCmdAcceleration, (size_t)ZE40_CAN_ID_200_CONS_ACCEL_AUTONOME_LENGTH);
          for(int i = 0; i < canMsg.dlc; i++){
            canMsg.data[i] = (unsigned char)canData[i];
          }
          // Checksum CRC16 computation
          if (counterCmdAcceleration==16) counterCmdAcceleration = 0;
          uint16_t chk = canMsg.data[0] + canMsg.data[1] + canMsg.data[2] + canMsg.data[3] + canMsg.data[4] + canMsg.data[5] + canMsg.data[6] + counterCmdAcceleration;
          uint8_t chk_mod16 = chk & 0x000F;
          canMsg.data[7] = (counterCmdAcceleration<<4) + chk_mod16;

          ++counterCmdAcceleration;
          pubCanRx->publish(canMsg);
          publishAccel = false;
        }
        
        if(publishBrake == true)
        {
          // Encode Brake MSG on CAN
          ze40_can_id_300_cons_frein_autonome_t canCmdBrake;
          canCmdBrake.cons_frein_autonome = ze40_can_id_300_cons_frein_autonome_cons_frein_autonome_encode(brakeValue);

          can_msgs::msg::Frame canBrakeMsg;
          canBrakeMsg.header = brakeValueHeader;
          canBrakeMsg.id = ZE40_CAN_ID_300_CONS_FREIN_AUTONOME_FRAME_ID;
          canBrakeMsg.dlc = ZE40_CAN_ID_300_CONS_FREIN_AUTONOME_LENGTH;
          canBrakeMsg.is_rtr = false;
          canBrakeMsg.is_extended = false;
          canBrakeMsg.is_error = false;

          uint8_t canData[8];
          error = ze40_can_id_300_cons_frein_autonome_pack(canData, &canCmdBrake, (size_t)ZE40_CAN_ID_300_CONS_FREIN_AUTONOME_LENGTH);
          for(int i = 0; i < canBrakeMsg.dlc; i++){
              canBrakeMsg.data[i] = (unsigned char)canData[i];
          }
          // Checksum CRC16 computation
          if (counterCmdBrake==16) counterCmdBrake = 0;
          uint16_t chk = canBrakeMsg.data[0] + canBrakeMsg.data[1] + canBrakeMsg.data[2] + canBrakeMsg.data[3] + canBrakeMsg.data[4] + canBrakeMsg.data[5] + canBrakeMsg.data[6] + counterCmdBrake;
          uint8_t chk_mod16 = chk & 0x000F;
          canBrakeMsg.data[7] = (counterCmdBrake<<4) + chk_mod16;

          ++counterCmdBrake;
          pubCanRx->publish(canBrakeMsg);
          publishBrake = false;
        }
        
        if(publishSteer == true)
        {
           //Encode Steer on CAN
          printf("publishing steer\n");

          ze40_can_id_100_cons_dae_autonome_t canCmdSteering;
          canCmdSteering.cons_dae_autonome = ze40_can_id_100_cons_dae_autonome_cons_dae_autonome_encode(steerValue);
          canCmdSteering.cons_vitesse_dae = ze40_can_id_100_cons_dae_autonome_cons_vitesse_dae_encode(steerSpeedValue);
          canCmdSteering.reactivation_mode_cooperatif_dae = ze40_can_id_100_cons_dae_autonome_reactivation_mode_cooperatif_dae_encode(steercoopValue);
          
          can_msgs::msg::Frame canSteerMsg;
          canSteerMsg.header = steerValueHeader;
          canSteerMsg.id = ZE40_CAN_ID_100_CONS_DAE_AUTONOME_FRAME_ID;
          canSteerMsg.dlc = ZE40_CAN_ID_100_CONS_DAE_AUTONOME_LENGTH;
          canSteerMsg.is_rtr = false;
          canSteerMsg.is_extended = false;
          canSteerMsg.is_error = false;
          
          uint8_t canData[8];
          error = ze40_can_id_100_cons_dae_autonome_pack(canData, &canCmdSteering, (size_t)ZE40_CAN_ID_100_CONS_DAE_AUTONOME_LENGTH);
          for(int i = 0; i < canSteerMsg.dlc; i++){
            canSteerMsg.data[i] = (unsigned char)canData[i];
          }
          
          // Checksum CRC16 computation
          if (counterCmdSteering==16) counterCmdSteering = 0;
          uint16_t chk = canSteerMsg.data[0] + canSteerMsg.data[1] + canSteerMsg.data[2] + canSteerMsg.data[3] + canSteerMsg.data[4] + canSteerMsg.data[5] + canSteerMsg.data[6] + counterCmdSteering;
          uint8_t chk_mod16 = chk & 0x000F;
          canSteerMsg.data[7] = (counterCmdSteering<<4) + chk_mod16;

          ++counterCmdSteering;

          pubCanRx->publish(canSteerMsg);
          publishSteer = false;
        }
        
        (void)error; // disable "unused variable" warning
        }
      
      t_exec_end = this->get_clock()->now();
      rclcpp::Duration diff = t_exec_end - t_ref;
    }
    
    bool run_th;

  private:
    
    
    
    void cmdSendAccel (const can_zoe_ze40_msgs::msg::CmdAcceleration::SharedPtr cmdAccel)
    {
      t_accel = this->get_clock()->now();
      
      accelValueHeader = cmdAccel->header;
      accelValue = cmdAccel->acceleration;
      publishAccelRequest = cmdAccel->can_publish;
    }
    
    void cmdSendBrake (const can_zoe_ze40_msgs::msg::CmdBrake::SharedPtr cmdBrake)
    {
      t_brake = this->get_clock()->now();
      
      brakeValueHeader = cmdBrake->header;
      brakeValue = cmdBrake->deceleration;
      publishBrakeRequest = cmdBrake->can_publish;
    }
    
    void cmdSendSteering (const can_zoe_ze40_msgs::msg::CmdSteering::SharedPtr cmdSteering)
    {
      
      t_steer = this->get_clock()->now();
      
      steerValueHeader = cmdSteering->header;
      steerValue = cmdSteering->angle;
      steerSpeedValue = cmdSteering->rotation_speed;
      steercoopValue = cmdSteering->cooperative_mode;
      publishSteerRequest = cmdSteering->can_publish;
    }
    
    void cmdSendOptions (const can_zoe_ze40_msgs::msg::CmdOptions::SharedPtr cmdOption)
    {
      int error = 100; 
      ze40_can_id_500_cons_accessoire_robotise_t canMsgOptions;

      canMsgOptions.clignotant_droit = ze40_can_id_500_cons_accessoire_robotise_clignotant_droit_encode(cmdOption->clignotant_droit);
      canMsgOptions.clignotant_gauche = ze40_can_id_500_cons_accessoire_robotise_clignotant_gauche_encode(cmdOption->clignotant_gauche);
      canMsgOptions.clignotant_warning = ze40_can_id_500_cons_accessoire_robotise_clignotant_warning_encode(cmdOption->clignotant_warning);
      canMsgOptions.feux_antibrouillard_ar = ze40_can_id_500_cons_accessoire_robotise_feux_antibrouillard_ar_encode(cmdOption->feux_antibrouillard_ar);
      canMsgOptions.feux_antibrouillard_av = ze40_can_id_500_cons_accessoire_robotise_feux_antibrouillard_av_encode(cmdOption->feux_antibrouillard_av);
      canMsgOptions.feux_code = ze40_can_id_500_cons_accessoire_robotise_feux_code_encode(cmdOption->feux_code);
      canMsgOptions.feux_route = ze40_can_id_500_cons_accessoire_robotise_feux_route_encode(cmdOption->feux_route);
      canMsgOptions.feux_veilleuse = ze40_can_id_500_cons_accessoire_robotise_feux_veilleuse_encode(cmdOption->feux_veilleuse);
      canMsgOptions.klaxon = ze40_can_id_500_cons_accessoire_robotise_klaxon_encode(cmdOption->klaxon);

      can_msgs::msg::Frame canOptionsMsg;
      canOptionsMsg.header = cmdOption->header;
      canOptionsMsg.id = ZE40_CAN_ID_500_CONS_ACCESSOIRE_ROBOTISE_FRAME_ID;
      canOptionsMsg.dlc = ZE40_CAN_ID_500_CONS_ACCESSOIRE_ROBOTISE_LENGTH;
      canOptionsMsg.is_rtr = false;
      canOptionsMsg.is_extended = false;
      canOptionsMsg.is_error = false;

      uint8_t canData[8];
      error = ze40_can_id_500_cons_accessoire_robotise_pack(canData, &canMsgOptions, (size_t)ZE40_CAN_ID_500_CONS_ACCESSOIRE_ROBOTISE_LENGTH);
      for(int i = 0; i < canOptionsMsg.dlc; i++){
        canOptionsMsg.data[i] = (unsigned char)canData[i];
      }

      // Checksum CRC16 computation
      if (counterCmdOptions==16) counterCmdOptions = 0;
      unsigned short chk = canOptionsMsg.data[0] + canOptionsMsg.data[1] + canOptionsMsg.data[2] + canOptionsMsg.data[3] + canOptionsMsg.data[4] + canOptionsMsg.data[5] + canOptionsMsg.data[6] + counterCmdOptions;
      unsigned char chk_mod16 = chk & 0x000F;
      canOptionsMsg.data[7] = (counterCmdOptions<<4) + chk_mod16;

      ++counterCmdOptions;

      // Command publication
      pubCanRx->publish(canOptionsMsg);
      (void)error;
    }
    
    void cmdActivationRobotization (const can_zoe_ze40_msgs::msg::RobotizationActivation::SharedPtr cmdActivation)
    {
      int error = 100;
      ze40_can_id_050_activation_robotisation_t canMsgActivation;

      canMsgActivation.mode_desire = ze40_can_id_050_activation_robotisation_mode_desire_encode(cmdActivation->mode_auto);
      canMsgActivation.activation_accel_robotise_can = ze40_can_id_050_activation_robotisation_activation_accel_robotise_can_encode(cmdActivation->acceleration);
      canMsgActivation.activation_dae_robotise_can = ze40_can_id_050_activation_robotisation_activation_dae_robotise_can_encode(cmdActivation->steering);
      canMsgActivation.activation_frein_robotise_can = ze40_can_id_050_activation_robotisation_activation_frein_robotise_can_encode(cmdActivation->brake);
      canMsgActivation.activation_prnd_robotise_can = ze40_can_id_050_activation_robotisation_activation_prnd_robotise_can_encode(cmdActivation->prnd);

      can_msgs::msg::Frame canMsg;
      canMsg.header = cmdActivation->header;
      canMsg.id = ZE40_CAN_ID_050_ACTIVATION_ROBOTISATION_FRAME_ID;
      canMsg.dlc = ZE40_CAN_ID_050_ACTIVATION_ROBOTISATION_LENGTH;
      canMsg.is_rtr = false;
      canMsg.is_extended = false;
      canMsg.is_error = false;

      uint8_t canData[8];
      error = ze40_can_id_050_activation_robotisation_pack(canData, &canMsgActivation, (size_t)ZE40_CAN_ID_050_ACTIVATION_ROBOTISATION_LENGTH);
      for(int i = 0; i < canMsg.dlc; i++){
        canMsg.data[i] = (unsigned char)canData[i];
      }

      // Checksum CRC16 computation
      if (counterCmdActivation==16) counterCmdActivation = 0;
      unsigned short chk = canMsg.data[0] + canMsg.data[1] + canMsg.data[2] + canMsg.data[3] + canMsg.data[4] + canMsg.data[5] + canMsg.data[6] + counterCmdActivation;
      unsigned char chk_mod16 = chk & 0x000F;
      canMsg.data[7] = (counterCmdActivation<<4) + chk_mod16;

      ++counterCmdActivation;

      //command publication
      pubCanRx->publish(canMsg);

      (void)error;
    }
    // variables
    
    bool publishAccel;
    bool publishBrake;
    bool publishSteer;
    bool publishAccelRequest;
    bool publishBrakeRequest;
    bool publishSteerRequest;
    
    // Subscribers
    rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdAcceleration>::SharedPtr subCmdAccel;
    rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdBrake>::SharedPtr subCmdBrake;
    rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdSteering>::SharedPtr subCmdSteering;
    rclcpp::Subscription<can_zoe_ze40_msgs::msg::CmdOptions>::SharedPtr subCmdOptions;
    rclcpp::Subscription<can_zoe_ze40_msgs::msg::RobotizationActivation>::SharedPtr subCmdActivationRobot;

    // Publisher for the raw can RX messages
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pubCanRx;
    
    
    size_t count_;
    
    uint8_t counterCmdAcceleration = 0;
    uint8_t counterCmdBrake = 0;
    uint8_t counterCmdSteering = 0;
    uint8_t counterCmdPRND = 0;
    uint8_t counterCmdOptions = 0;
    uint8_t counterCmdActivation = 0;
    
    
    rclcpp::Time t_accel, t_brake, t_steer, t_ref, t_exec_end;
    
    float accelValue;
    float brakeValue;
    float steerValue;
    float steerSpeedValue;
    uint8_t steercoopValue;
    // Timer
      rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Header accelValueHeader;
    std_msgs::msg::Header brakeValueHeader;
    std_msgs::msg::Header steerValueHeader;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZoeControl>());
  rclcpp::shutdown();
  return 0;
}
