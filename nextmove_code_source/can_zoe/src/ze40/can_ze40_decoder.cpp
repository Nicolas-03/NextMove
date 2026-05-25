#include "ze40/can_ze40_decoder.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

ZE40CanDecoder::ZE40CanDecoder() : Node("can_decoder")
{
    subCanTx_ = this->create_subscription<can_msgs::msg::Frame>("/can_tx", 100, std::bind(&ICanDecoder::decodeCanMsg, this, _1));
    pubSpeed_ = this->create_publisher<can_zoe_ze40_msgs::msg::Speed>("/can/speed", 100);
    pubIMU_ = this->create_publisher<can_zoe_ze40_msgs::msg::IMU>("/can/imu", 100);
    pubOdom_ = this->create_publisher<can_zoe_ze40_msgs::msg::Odom>("/can/odom", 100);
    pubSteering_ = this->create_publisher<can_zoe_ze40_msgs::msg::Steering>("/can/steering", 100);
    pubCond_A_ = this->create_publisher<can_zoe_ze40_msgs::msg::ActivationCondition>("/can/conditions_a", 100);
    pubCond_B_ = this->create_publisher<can_zoe_ze40_msgs::msg::ActivationCondition>("/can/conditions_b", 100);
    pubPRNDStatus_ = this->create_publisher<can_zoe_ze40_msgs::msg::PRNDCmdInfo>("/can/prnd", 100);
    pubBattInfo_ = this->create_publisher<can_zoe_ze40_msgs::msg::BattInfo>("/can/battery_info", 100);
    pubTorqueInfo_ = this->create_publisher<can_zoe_ze40_msgs::msg::TorqueInfo>("/can/torque_info", 100);
    pubSelectorInfo_a_ = this->create_publisher<can_zoe_ze40_msgs::msg::SelectorAInfo>("/can/selector_a_info", 100);
    pubSelectorInfo_b_ = this->create_publisher<can_zoe_ze40_msgs::msg::SelectorBInfo>("/can/selector_b_info", 100);
}

void ZE40CanDecoder::decodeCanMsg(const can_msgs::msg::Frame::SharedPtr canMsg) const
{
      // Decoding error initialization
      int error = 100;
      // ‘const _data_type’ {aka ‘const boost::array<unsigned char, 8>’} to ‘const uint8_t*’ {aka ‘const unsigned char*’}
      // uint8_t canData[canMsg->data.size()]; // variable length arrays are forbiden 
      uint8_t *canData = new uint8_t[canMsg->data.size()]; // using a pointor instead of a vla 
      for(int i = 0; (unsigned)i < canMsg->data.size(); i++){
        canData[i] = (std::uint8_t)canMsg->data[i];
      }
      
      // Depanding on the CAN message Id
      switch(canMsg->id){
          // If the speed message is received
          case ZE40_CAN_ID_700_INFO_RENAULT_VITESSE_FRAME_ID:
              ze40_can_id_700_info_renault_vitesse_t canMsgSpeed;
              // Message decoding
              error = ze40_can_id_700_info_renault_vitesse_unpack(&canMsgSpeed, canData, (size_t)canMsg->dlc);
              // If the decoding is done correctly, the ROS message is completed and published
              if(!error){
                  can_zoe_ze40_msgs::msg::Speed msgSpeed = can_zoe_ze40_msgs::msg::Speed();
                  msgSpeed.header = canMsg->header;
                  msgSpeed.wheel_speed_fl = ze40_can_id_700_info_renault_vitesse_vitesse_roue_avg_decode(canMsgSpeed.vitesse_roue_avg);
                  msgSpeed.wheel_speed_fr = ze40_can_id_700_info_renault_vitesse_vitesse_roue_avd_decode(canMsgSpeed.vitesse_roue_avd);
                  msgSpeed.wheel_speed_rl = ze40_can_id_700_info_renault_vitesse_vitesse_roue_arg_decode(canMsgSpeed.vitesse_roue_arg);
                  msgSpeed.wheel_speed_rr = ze40_can_id_700_info_renault_vitesse_vitesse_roue_ard_decode(canMsgSpeed.vitesse_roue_ard);
                  msgSpeed.speed = ze40_can_id_700_info_renault_vitesse_vitesse_vehicule_decode(canMsgSpeed.vitesse_vehicule);
                  pubSpeed_->publish(msgSpeed);
              }
              break;
          // If the Odometry message is received
          case ZE40_CAN_ID_701_INFO_RENAULT_ODOMETRIE_FRAME_ID:
              ze40_can_id_701_info_renault_odometrie_t canMsgOdom;
              // Message decoding
              error = ze40_can_id_701_info_renault_odometrie_unpack(&canMsgOdom, canData, (size_t)canMsg->dlc);
              // If the decoding is done correctly, the ROS message is completed and published
              if(!error){
                  can_zoe_ze40_msgs::msg::Odom msgOdom = can_zoe_ze40_msgs::msg::Odom();
                  msgOdom.header = canMsg->header;
                  msgOdom.odom_fl = ze40_can_id_701_info_renault_odometrie_odometrie_roue_avg_decode(canMsgOdom.odometrie_roue_avg);
               msgOdom.odom_fr = ze40_can_id_701_info_renault_odometrie_odometrie_roue_avd_decode(canMsgOdom.odometrie_roue_avd);
                  msgOdom.odom_rl = ze40_can_id_701_info_renault_odometrie_odometrie_roue_arg_decode(canMsgOdom.odometrie_roue_arg);
                  msgOdom.odom_rr = ze40_can_id_701_info_renault_odometrie_odometrie_roue_ard_decode(canMsgOdom.odometrie_roue_ard);
                  pubOdom_->publish(msgOdom);
              } 
              break;
          // If the IMU message is received
          case ZE40_CAN_ID_702_INFO_RENAULT_ESP_FRAME_ID:
              ze40_can_id_702_info_renault_esp_t canMsgImu;
              // Message decoding
              error = ze40_can_id_702_info_renault_esp_unpack(&canMsgImu, canData, (size_t)canMsg->dlc);
              // If the decoding is done correctly, the ROS message is completed and published
              if(!error){
                  can_zoe_ze40_msgs::msg::IMU msgImu = can_zoe_ze40_msgs::msg::IMU();
                  msgImu.header = canMsg->header;
                  msgImu.acc_lon = ze40_can_id_702_info_renault_esp_accel_longitudinale_decode(canMsgImu.accel_longitudinale);
                  msgImu.acc_lat = ze40_can_id_702_info_renault_esp_accel_laterale_decode(canMsgImu.accel_laterale);
                  msgImu.yaw_rate = ze40_can_id_702_info_renault_esp_vitesse_du_lacet_decode(canMsgImu.vitesse_du_lacet);
                  pubIMU_->publish(msgImu);
              }
              break;
          // If the Steering wheel message is received
          case ZE40_CAN_ID_703_INFO_RENAULT_VOLANT_FRAME_ID:
              ze40_can_id_703_info_renault_volant_t canMsgSteering;
              // Message decoding
              error = ze40_can_id_703_info_renault_volant_unpack(&canMsgSteering, canData, (size_t)canMsg->dlc);
              // If the decoding is done correctly, the ROS message is completed and published
              if(!error){
                  can_zoe_ze40_msgs::msg::Steering msgSteering = can_zoe_ze40_msgs::msg::Steering();
                  msgSteering.header = canMsg->header;
                  msgSteering.angle = ze40_can_id_703_info_renault_volant_volant_angle_decode(canMsgSteering.volant_angle);
                  msgSteering.angle_offset = ze40_can_id_703_info_renault_volant_volant_angle_offset_decode(canMsgSteering.volant_angle_offset);
                  msgSteering.rotation_speed = ze40_can_id_703_info_renault_volant_volant_vitesse_rotation_decode(canMsgSteering.volant_vitesse_rotation);
                  pubSteering_->publish(msgSteering);
              }
              break;
          //if conditions message is received (A)
          case ZE40_CAN_ID_060_CONDITION_ACTIVATION_G_A_FRAME_ID:
              ze40_can_id_060_condition_activation_g_a_t canMsgCond_A;
              // Message decoding
              error = ze40_can_id_060_condition_activation_g_a_unpack(&canMsgCond_A, canData, (size_t)canMsg->dlc);
              // If decoding is OK, Publishing over ros.
              if(!error){
                  can_zoe_ze40_msgs::msg::ActivationCondition msgCond_A = can_zoe_ze40_msgs::msg::ActivationCondition();
                  msgCond_A.header = canMsg->header;
                  msgCond_A.vehicle = ze40_can_id_060_condition_activation_g_a_cond_activ_gen_veh_robotisation_a_decode(canMsgCond_A.cond_activ_gen_veh_robotisation_a);
                  msgCond_A.door_fl = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_porte_avg_a_decode(canMsgCond_A.cause_non_activ_gen_porte_avg_a);
                  msgCond_A.door_fr = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_porte_avd_a_decode(canMsgCond_A.cause_non_activ_gen_porte_avd_a);
                  msgCond_A.door_rl = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_porte_ard_a_decode(canMsgCond_A.cause_non_activ_gen_porte_ard_a);
                  msgCond_A.door_rr = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_porte_arg_a_decode(canMsgCond_A.cause_non_activ_gen_porte_arg_a);
                  msgCond_A.door_r = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_porte_hayon_a_decode(canMsgCond_A.cause_non_activ_gen_porte_hayon_a);
                  msgCond_A.driver_safety = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_secu_conducte_a_decode(canMsgCond_A.cause_non_activ_gen_secu_conducte_a);
                  msgCond_A.passenger_safety = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_secu_passager_a_decode(canMsgCond_A.cause_non_activ_gen_secu_passager_a);
                  msgCond_A.handbrake = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_frein_main_a_decode(canMsgCond_A.cause_non_activ_gen_frein_main_a);
                  msgCond_A.prnd = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_prnd_a_decode(canMsgCond_A.cause_non_activ_gen_prnd_a);
                  msgCond_A.emergency_stop = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_arret_urgence_a_decode(canMsgCond_A.cause_non_activ_gen_arret_urgence_a);
                  msgCond_A.can_intersys = ze40_can_id_060_condition_activation_g_a_cause_non_activ_gen_can_intersys_a_decode(canMsgCond_A.cause_non_activ_gen_can_intersys_a);
                  pubCond_A_->publish(msgCond_A);
              }
              break;
          // if conditions message is received (B) 
          case ZE40_CAN_ID_061_CONDITION_ACTIVATION_G_B_FRAME_ID:
              ze40_can_id_061_condition_activation_g_b_t canMsgCond_B;
              // Message decoding
              error = ze40_can_id_061_condition_activation_g_b_unpack(&canMsgCond_B, canData, (size_t)canMsg->dlc);
              // If decoding is OK, Publishing data over ROS.
              if(!error){
                  can_zoe_ze40_msgs::msg::ActivationCondition msgCond_B = can_zoe_ze40_msgs::msg::ActivationCondition();
                  msgCond_B.header = canMsg->header;
                  msgCond_B.vehicle = ze40_can_id_061_condition_activation_g_b_cond_activ_gen_veh_robotisation_b_decode(canMsgCond_B.cond_activ_gen_veh_robotisation_b);
                  msgCond_B.door_fl = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_porte_avg_b_decode(canMsgCond_B.cause_non_activ_gen_porte_avg_b);
                  msgCond_B.door_fr = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_porte_avd_b_decode(canMsgCond_B.cause_non_activ_gen_porte_avd_b);
                  msgCond_B.door_rl = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_porte_ard_b_decode(canMsgCond_B.cause_non_activ_gen_porte_ard_b);
                  msgCond_B.door_rr = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_porte_arg_b_decode(canMsgCond_B.cause_non_activ_gen_porte_arg_b);
                  msgCond_B.door_r = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_porte_hayon_b_decode(canMsgCond_B.cause_non_activ_gen_porte_hayon_b);
                  msgCond_B.driver_safety = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_secu_conducte_b_decode(canMsgCond_B.cause_non_activ_gen_secu_conducte_b);
                  msgCond_B.passenger_safety = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_secu_passager_b_decode(canMsgCond_B.cause_non_activ_gen_secu_passager_b);
                  msgCond_B.handbrake = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_frein_main_b_decode(canMsgCond_B.cause_non_activ_gen_frein_main_b);
                  msgCond_B.prnd = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_prnd_b_decode(canMsgCond_B.cause_non_activ_gen_prnd_b);
                  msgCond_B.emergency_stop = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_arret_urgence_b_decode(canMsgCond_B.cause_non_activ_gen_arret_urgence_b);
                  msgCond_B.can_intersys = ze40_can_id_061_condition_activation_g_b_cause_non_activ_gen_can_intersys_b_decode(canMsgCond_B.cause_non_activ_gen_can_intersys_b);
                  pubCond_B_->publish(msgCond_B);
              }
              break;
          // if prnd message is received
          case ZE40_CAN_ID_410_PRND_INFO_FH_FRAME_ID:
              ze40_can_id_410_prnd_info_fh_t canMsgPRNDInfo;
              // Message decoding
              error = ze40_can_id_410_prnd_info_fh_unpack(&canMsgPRNDInfo, canData, (size_t)canMsg->dlc);
              //if decoding is OK, publishing data over ROS.
              if(!error){
                  can_zoe_ze40_msgs::msg::PRNDCmdInfo msgPRNDInfo = can_zoe_ze40_msgs::msg::PRNDCmdInfo();
                  msgPRNDInfo.header = canMsg->header;
                  msgPRNDInfo.prnd_enable = ze40_can_id_410_prnd_info_fh_prnd_engage_decode(canMsgPRNDInfo.prnd_engage);
                  msgPRNDInfo.mode_auto = ze40_can_id_410_prnd_info_fh_mode_prnd_en_cours_decode(canMsgPRNDInfo.mode_prnd_en_cours);
                  pubPRNDStatus_->publish(msgPRNDInfo);
              }
              break;
          // If Accel info msg is received
          case ZE40_CAN_ID_210_ACCEL_INFO_FH_FRAME_ID:
              ze40_can_id_210_accel_info_fh_t canMsgAccelinfo;
              // msg decoding
              error = ze40_can_id_210_accel_info_fh_unpack(&canMsgAccelinfo, canData, (size_t)canMsg->dlc);
              if(!error){
                  //RCLCPP_INFO("ACCEL_MODE is %f", ze40_can_id_210_accel_info_fh_mode_accel_en_cours_decode(canMsgAccelinfo.mode_accel_en_cours));    
              }
              break;
          
          case ZE40_CAN_ID_110_DAE_INFO_FH_FRAME_ID:
              ze40_can_id_110_dae_info_fh_t canMsgDAEInfo;
              // msg decoding
              error = ze40_can_id_110_dae_info_fh_unpack(&canMsgDAEInfo, canData, (size_t)canMsg->dlc);
              // if no error when decoding
              if(!error){
                  //RCLCPP_INFO("DAE_MODE is %f", ze40_can_id_110_dae_info_fh_mode_dae_en_cours_decode(canMsgDAEInfo.mode_dae_en_cours));
              }
              break;
          
          case ZE40_CAN_ID_310_FREIN_INFO_FH_FRAME_ID:
              ze40_can_id_310_frein_info_fh_t canMsgFreinInfo;
              //msg decoding
              error = ze40_can_id_310_frein_info_fh_unpack(&canMsgFreinInfo, canData, (size_t)canMsg->dlc);
              if(!error){
                  //RCLCPP_INFO("BRAKE MODE is %f", ze40_can_id_310_frein_info_fh_mode_frein_en_cours_decode(canMsgFreinInfo.mode_frein_en_cours));
              }
              break;
          
          case ZE40_CAN_ID_070_PUPITRE_A_FRAME_ID:
              ze40_can_id_070_pupitre_a_t canMsgPupitreAInfo;
              //msg decoding
              error = ze40_can_id_070_pupitre_a_unpack(&canMsgPupitreAInfo, canData, (size_t)canMsg->dlc);
              if(!error){
                  can_zoe_ze40_msgs::msg::SelectorAInfo msgSelectorAInfo = can_zoe_ze40_msgs::msg::SelectorAInfo();
                  msgSelectorAInfo.header = canMsg->header;
                  msgSelectorAInfo.accel_led = ze40_can_id_070_pupitre_a_pupitre_led_accel_decode(canMsgPupitreAInfo.pupitre_led_accel);
                  msgSelectorAInfo.brake_led = ze40_can_id_070_pupitre_a_pupitre_led_frein_decode(canMsgPupitreAInfo.pupitre_led_frein);
                  msgSelectorAInfo.emergency_stop_a = ze40_can_id_070_pupitre_a_pupitre_arret_urgence_a_decode(canMsgPupitreAInfo.pupitre_arret_urgence_a);
                  msgSelectorAInfo.buzzer_accel_brake = ze40_can_id_070_pupitre_a_pupitre_buzzer_dmd_accel_frein_decode(canMsgPupitreAInfo.pupitre_buzzer_dmd_accel_frein);
                  pubSelectorInfo_a_->publish(msgSelectorAInfo);
              }
              break;
          
            case ZE40_CAN_ID_071_PUPITRE_B_FRAME_ID:
              ze40_can_id_071_pupitre_b_t canMsgPupitreBInfo;
              //msg decoding
              error = ze40_can_id_071_pupitre_b_unpack(&canMsgPupitreBInfo, canData, (size_t)canMsg->dlc);
              if(!error){
                  can_zoe_ze40_msgs::msg::SelectorBInfo msgSelectorBInfo = can_zoe_ze40_msgs::msg::SelectorBInfo();
                  msgSelectorBInfo.header = canMsg->header;
                  msgSelectorBInfo.mode = ze40_can_id_071_pupitre_b_pupitre_rotacteur_mode_decode(canMsgPupitreBInfo.pupitre_rotacteur_mode);
                  msgSelectorBInfo.led_default = ze40_can_id_071_pupitre_b_pupitre_led_defaut_decode(canMsgPupitreBInfo.pupitre_led_defaut);
                  msgSelectorBInfo.led_dae = ze40_can_id_071_pupitre_b_pupitre_led_dae_decode(canMsgPupitreBInfo.pupitre_led_dae);
                  msgSelectorBInfo.inter_dae = ze40_can_id_071_pupitre_b_pupitre_interr_dae_decode(canMsgPupitreBInfo.pupitre_interr_dae);
                  msgSelectorBInfo.inter_brake = ze40_can_id_071_pupitre_b_pupitre_interr_frein_decode(canMsgPupitreBInfo.pupitre_interr_frein);
                  msgSelectorBInfo.inter_accel = ze40_can_id_071_pupitre_b_pupitre_interr_accel_decode(canMsgPupitreBInfo.pupitre_interr_accel);
                  msgSelectorBInfo.emergency_stop_b = ze40_can_id_071_pupitre_b_pupitre_arret_urgence_b_decode(canMsgPupitreBInfo.pupitre_arret_urgence_b);
                  msgSelectorBInfo.buzzer_dae = ze40_can_id_071_pupitre_b_pupitre_buzzer_dmd_dae_decode(canMsgPupitreBInfo.pupitre_buzzer_dmd_dae);
                  pubSelectorInfo_b_->publish(msgSelectorBInfo);
              }
              break;
          
            case ZE40_CAN_ID_707_INFO_RENAULT_BATTERIE400_V_FRAME_ID:
                ze40_can_id_707_info_renault_batterie400_v_t canMsgBattInfo;
              //msg decoding
                error = ze40_can_id_707_info_renault_batterie400_v_unpack(&canMsgBattInfo, canData, (size_t)canMsg->dlc);
                if(!error)
                {
                    can_zoe_ze40_msgs::msg::BattInfo msgBattInfo = can_zoe_ze40_msgs::msg::BattInfo();
                    msgBattInfo.header = canMsg->header;
                    msgBattInfo.voltage = ze40_can_id_707_info_renault_batterie400_v_batt400_tension_decode(canMsgBattInfo.batt400_tension);
                    msgBattInfo.soc = ze40_can_id_707_info_renault_batterie400_v_batt400_soc_decode(canMsgBattInfo.batt400_soc);
                    msgBattInfo.autonomie_km = ze40_can_id_707_info_renault_batterie400_v_batt400_autonomie_km_decode(canMsgBattInfo.batt400_autonomie_km);
                    msgBattInfo.autonomie_kwh = ze40_can_id_707_info_renault_batterie400_v_batt400_autonomie_k_wh_decode(canMsgBattInfo.batt400_autonomie_k_wh);
                    pubBattInfo_->publish(msgBattInfo);
                }
                break;
          
            case ZE40_CAN_ID_708_INFO_RENAULT_COUPLE_FRAME_ID:
            ze40_can_id_708_info_renault_couple_t canMsgTorqueInfo;
            //msg decoding
            error = ze40_can_id_708_info_renault_couple_unpack(&canMsgTorqueInfo, canData, (size_t)canMsg->dlc);
            if(!error){
                can_zoe_ze40_msgs::msg::TorqueInfo msgTorqueInfo = can_zoe_ze40_msgs::msg::TorqueInfo();
                msgTorqueInfo.header = canMsg->header;
                msgTorqueInfo.meanefftorque = ze40_can_id_708_info_renault_couple_mean_eff_torque_decode(canMsgTorqueInfo.mean_eff_torque);
                msgTorqueInfo.requestedtorqueafterproc = ze40_can_id_708_info_renault_couple_requested_torque_after_proc_decode(canMsgTorqueInfo.requested_torque_after_proc);
                msgTorqueInfo.driverbrakewheeltq_req = ze40_can_id_708_info_renault_couple_driver_brake_wheel_tq_req_decode(canMsgTorqueInfo.driver_brake_wheel_tq_req);
                msgTorqueInfo.elecbrakewheelstorquerequest = ze40_can_id_708_info_renault_couple_elec_brake_wheels_torque_request_decode(canMsgTorqueInfo.elec_brake_wheels_torque_request);
                pubTorqueInfo_->publish(msgTorqueInfo);
            }
            break; 
          }
       
      if(error){
          // If the CAN message is not in any cases of the switch
          if(error == 100){
              //RCLCPP_DEBUG("CAN ID not supported for decoding!\n");
          }
         // Return the decoding error if an error occur
          else{
              //RCLCPP_ERROR("Error '" << error << "' during CAN decoding!\n");
          }
      }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZE40CanDecoder>());
  rclcpp::shutdown();
  return 0;
}
