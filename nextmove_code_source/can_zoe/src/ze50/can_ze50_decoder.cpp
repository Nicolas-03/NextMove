#include "ze50/can_ze50_decoder.hpp"

ZE50CanDecoder::ZE50CanDecoder()
: Node("can_ze50_decoder")
    {
    subCanTx_ = this->create_subscription<can_msgs::msg::Frame>("/can_tx", 100, std::bind(&ZE50CanDecoder::decodeCanMsg, this, _1));
    pub_cdt_act_ = this->create_publisher<can_zoe_msgs::msg::ConditionActivation>("/can/condition_activation", 100);
    pub_robot_act_ = this->create_publisher<can_zoe_msgs::msg::ActivationRobotisation>("/can/robotisation_activation", 100);
    pub_panel_ = this->create_publisher<can_zoe_msgs::msg::ControlPanel>("/can/control_panel", 100);
    pub_rear_wheels_steering_ = this->create_publisher<can_zoe_msgs::msg::RearWheelsSteering>("/can/rear_wheels_steering", 100);
    pub_front_wheels_steering_ = this->create_publisher<can_zoe_msgs::msg::FrontWheelsSteering>("/can/front_wheels_accel_brake", 100);
    pub_accessories_ = this->create_publisher<can_zoe_msgs::msg::Accessories>("/can/accessories", 100);
    pub_steer_info_ = this->create_publisher<can_zoe_msgs::msg::SteeringInfo>("/can/steering_info", 100);
    pub_steer_err_info_ = this->create_publisher<can_zoe_msgs::msg::SteeringErrInfo>("/can/steering_err_info", 100);
    pub_accel_info_ = this->create_publisher<can_zoe_msgs::msg::AccelInfo>("/can/acel_info", 100);
    pub_accel_err_info_ = this->create_publisher<can_zoe_msgs::msg::AccelErrInfo>("/can/accel_err_info", 100);
    pub_brake_info_ = this->create_publisher<can_zoe_msgs::msg::BrakeInfo>("/can/brake_info", 100);
    pub_brake_err_info_ = this->create_publisher<can_zoe_msgs::msg::BrakeErrInfo>("/can/brake_err_info", 100);
    pub_rndb_info_ = this->create_publisher<can_zoe_msgs::msg::RNDBInfo>("/can/rndb_info", 100);
    pub_rndb_err_info_ = this->create_publisher<can_zoe_msgs::msg::RNDBErrInfo>("/can/rndb_err_info", 100);
    
}

void ZE50CanDecoder::decodeCanMsg(const can_msgs::msg::Frame::SharedPtr canMsg) const
{
    uint8_t *canData = new uint8_t[canMsg->data.size()]; // TODO Handle leak!!!
    try
    {
          // Decoding error initialization
        int error = 100;
        // Add error handling
        
        for(size_t i = 0; i < canMsg->data.size(); i++){
          canData[i] = (std::uint8_t)canMsg->data[i];
        }
    
    
          // Depanding on the CAN message Id
        switch(canMsg->id)
        {        // ----------------------------- Start Switch CAN ID Case ----------------------------------------------- 
            case ZE50_CAN_ID_050_ACTIVATION_ROBOTISATION_FRAME_ID: {
                ze50_can_id_050_activation_robotisation_t canMsgRobotisationActivation;

                error = ze50_can_id_050_activation_robotisation_unpack(&canMsgRobotisationActivation, canData, (size_t)canMsg->dlc);
                if (error >= 0)
                {
                    can_zoe_msgs::msg::ActivationRobotisation msgRobotisationActivation = can_zoe_msgs::msg::ActivationRobotisation();
                    msgRobotisationActivation.header = canMsg->header;
                    msgRobotisationActivation.mode = ze50_can_id_050_activation_robotisation_mode_desire_decode(canMsgRobotisationActivation.mode_desire);
                    msgRobotisationActivation.steering = ze50_can_id_050_activation_robotisation_activation_dae_robotise_can_decode(canMsgRobotisationActivation.activation_dae_robotise_can);
                    msgRobotisationActivation.acceleration = ze50_can_id_050_activation_robotisation_activation_accel_robotise_can_decode(canMsgRobotisationActivation.activation_accel_robotise_can);
                    msgRobotisationActivation.brake = ze50_can_id_050_activation_robotisation_activation_frein_robotise_can_decode(canMsgRobotisationActivation.activation_frein_robotise_can);
                    msgRobotisationActivation.prnd = ze50_can_id_050_activation_robotisation_activation_rndb_robotise_can_decode(canMsgRobotisationActivation.activation_rndb_robotise_can);
                    msgRobotisationActivation.blinker = ze50_can_id_050_activation_robotisation_activation_cligno_robotise_can_decode(canMsgRobotisationActivation.activation_cligno_robotise_can);
                    pub_robot_act_->publish(msgRobotisationActivation);
                }
            break;
            }
            // ---------------------------------  CAN ID 060 - Condition Activation ---------------------------------
            case ZE50_CAN_ID_060_CONDITION_ACTIVATION_FULL_FRAME_ID: {
                ze50_can_id_060_condition_activation_full_t canMsgConditionActivation;
                // Message decoding
                error = ze50_can_id_060_condition_activation_full_unpack(&canMsgConditionActivation, canData, (size_t)canMsg->dlc);
                // If the decoding is done correctly, the ROS message is completed and published
                if(error >= 0){
                    can_zoe_msgs::msg::ConditionActivation msgConditionActivation = can_zoe_msgs::msg::ConditionActivation();
                    msgConditionActivation.header = canMsg->header;
                    msgConditionActivation.vehicle = ze50_can_id_060_condition_activation_full_cond_activ_gen_veh_robotisation_decode(canMsgConditionActivation.cond_activ_gen_veh_robotisation);
                    msgConditionActivation.door_fl = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_porte_avg_decode(canMsgConditionActivation.cause_non_activ_gen_porte_avg);
                    msgConditionActivation.door_fr = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_porte_avd_decode(canMsgConditionActivation.cause_non_activ_gen_porte_avd);
                    msgConditionActivation.door_rl = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_porte_arg_decode(canMsgConditionActivation.cause_non_activ_gen_porte_arg);
                    msgConditionActivation.door_rr = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_porte_ard_decode(canMsgConditionActivation.cause_non_activ_gen_porte_ard);
                    msgConditionActivation.rndb = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_rndb_decode(canMsgConditionActivation.cause_non_activ_gen_rndb);
                    msgConditionActivation.handbrake = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_frein_main_decode(canMsgConditionActivation.cause_non_activ_gen_frein_main);
                    msgConditionActivation.emergency_stop = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_arret_urgence_decode(canMsgConditionActivation.cause_non_activ_gen_arret_urgence);
                    msgConditionActivation.trunk = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_porte_hayon_decode(canMsgConditionActivation.cause_non_activ_gen_porte_hayon);
                    msgConditionActivation.belt_fl = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_secu_conducte_decode(canMsgConditionActivation.cause_non_activ_gen_secu_conducte);
                    msgConditionActivation.belt_fr = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_secu_passager_decode(canMsgConditionActivation.cause_non_activ_gen_secu_passager);
                    msgConditionActivation.can_intersys = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_can_intersys_decode(canMsgConditionActivation.cause_non_activ_gen_can_intersys);
                    msgConditionActivation.emergency_stop_uca = ze50_can_id_060_condition_activation_full_cause_non_activ_gen_arret_urgen_ext_decode(canMsgConditionActivation.cause_non_activ_gen_arret_urgen_ext);
                    pub_cdt_act_->publish(msgConditionActivation);
                }
            break;
            }
            
            // --------------------------------- CAN ID 070 - Pupitre ---------------------------------
            case ZE50_CAN_ID_070_PUPITRE_FULL_FRAME_ID: {
                ze50_can_id_070_pupitre_full_t canMsgPupitre;
                error = ze50_can_id_070_pupitre_full_unpack(&canMsgPupitre, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::ControlPanel msgControlPanel;
                    msgControlPanel.header = canMsg->header;
                    msgControlPanel.switch_rndb = ze50_can_id_070_pupitre_full_pupitre_interr_rndb_decode(canMsgPupitre.pupitre_interr_rndb);
                    msgControlPanel.switch_brake = ze50_can_id_070_pupitre_full_pupitre_interr_frein_decode(canMsgPupitre.pupitre_interr_frein);
                    msgControlPanel.switch_accel = ze50_can_id_070_pupitre_full_pupitre_interr_accel_decode(canMsgPupitre.pupitre_interr_accel);
                    msgControlPanel.switch_dae = ze50_can_id_070_pupitre_full_pupitre_interr_dae_decode(canMsgPupitre.pupitre_interr_dae);
                    msgControlPanel.selector_mode = ze50_can_id_070_pupitre_full_pupitre_rotacteur_mode_decode(canMsgPupitre.pupitre_rotacteur_mode);
                    msgControlPanel.led_rndb = ze50_can_id_070_pupitre_full_pupitre_led_rndb_decode(canMsgPupitre.pupitre_led_rndb);
                    msgControlPanel.led_dae = ze50_can_id_070_pupitre_full_pupitre_led_dae_decode(canMsgPupitre.pupitre_led_dae);
                    msgControlPanel.led_brake = ze50_can_id_070_pupitre_full_pupitre_led_frein_decode(canMsgPupitre.pupitre_led_frein);
                    msgControlPanel.led_accel = ze50_can_id_070_pupitre_full_pupitre_led_accel_decode(canMsgPupitre.pupitre_led_accel);
                    msgControlPanel.led_default = ze50_can_id_070_pupitre_full_pupitre_led_defaut_decode(canMsgPupitre.pupitre_led_defaut);
                    pub_panel_->publish(msgControlPanel);
                }
            break;
            }
            
            // --------------------------------- CAN ID 110 - DAE Info FH ---------------------------------
            case ZE50_CAN_ID_110_DAE_INFO_FH_FRAME_ID: {
                ze50_can_id_110_dae_info_fh_t canMsgDaeInfo;
                error = ze50_can_id_110_dae_info_fh_unpack(&canMsgDaeInfo, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::SteeringInfo msgSteeringInfo;
                    msgSteeringInfo.header = canMsg->header;
                    msgSteeringInfo.position_dae = ze50_can_id_110_dae_info_fh_position_dae_decode(canMsgDaeInfo.position_dae);
                    msgSteeringInfo.ecart_pos_dae_cons_pos = ze50_can_id_110_dae_info_fh_ecart_pos_dae_cons_pos_decode(canMsgDaeInfo.ecart_pos_dae_cons_pos);
                    msgSteeringInfo.couple_volant_mesure_dae = ze50_can_id_110_dae_info_fh_couple_volant_mesure_dae_decode(canMsgDaeInfo.couple_volant_mesure_dae);
                    msgSteeringInfo.couple_volant_genere_dae = ze50_can_id_110_dae_info_fh_couple_volant_genere_dae_decode(canMsgDaeInfo.couple_volant_genere_dae);
                    msgSteeringInfo.dae_robotisable_plage_ok = ze50_can_id_110_dae_info_fh_dae_robotisable_plage_ok_decode(canMsgDaeInfo.dae_robotisable_plage_ok);
                    msgSteeringInfo.mode_dae_en_cours = ze50_can_id_110_dae_info_fh_mode_dae_en_cours_decode(canMsgDaeInfo.mode_dae_en_cours);
                    pub_steer_info_->publish(msgSteeringInfo);
                }
            break;
            }
            
            // --------------------------------- CAN ID 111 - DAE Info FH (suite) ---------------------------------
            case ZE50_CAN_ID_111_DAE_INFO_FH_FRAME_ID: {
                ze50_can_id_111_dae_info_fh_t canMsgDaeInfo2;
                error = ze50_can_id_111_dae_info_fh_unpack(&canMsgDaeInfo2, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::SteeringErrInfo msgSteeringErrInfo;
                    msgSteeringErrInfo.header = canMsg->header;
                    msgSteeringErrInfo.butee_maxi_dae_g_autorise = ze50_can_id_111_dae_info_fh_butee_maxi_dae_g_autorise_decode(canMsgDaeInfo2.butee_maxi_dae_g_autorise);
                    msgSteeringErrInfo.butee_maxi_dae_d_autorise = ze50_can_id_111_dae_info_fh_butee_maxi_dae_d_autorise_decode(canMsgDaeInfo2.butee_maxi_dae_d_autorise);
                    msgSteeringErrInfo.erreur_perte_can_intersystem_dae = ze50_can_id_111_dae_info_fh_erreur_perte_can_intersystem_dae_decode(canMsgDaeInfo2.erreur_perte_can_intersystem_dae);
                    msgSteeringErrInfo.erreur_capteur_dae_incoh_sign1_2 = ze50_can_id_111_dae_info_fh_erreur_capteur_dae_incoh_sign1_2_decode(canMsgDaeInfo2.erreur_capteur_dae_incoh_sign1_2);
                    msgSteeringErrInfo.erreur_capt_dae_origine_couple_1 = ze50_can_id_111_dae_info_fh_erreur_capt_dae_origine_couple_1_decode(canMsgDaeInfo2.erreur_capt_dae_origine_couple_1);
                    msgSteeringErrInfo.erreur_butee_dae_droite_atteinte = ze50_can_id_111_dae_info_fh_erreur_butee_dae_droite_atteinte_decode(canMsgDaeInfo2.erreur_butee_dae_droite_atteinte);
                    msgSteeringErrInfo.erreur_butee_dae_gauche_atteinte = ze50_can_id_111_dae_info_fh_erreur_butee_dae_gauche_atteinte_decode(canMsgDaeInfo2.erreur_butee_dae_gauche_atteinte);
                    
                    msgSteeringErrInfo.cause_desact_dae_reprise_manu_coop = ze50_can_id_111_dae_info_fh_cause_desact_dae_reprise_manu_coop_decode(canMsgDaeInfo2.cause_desact_dae_reprise_manu_coop);
                    msgSteeringErrInfo.cause_desact_dae_pb_capteur_ori = ze50_can_id_111_dae_info_fh_cause_desact_dae_pb_capteur_ori_decode(canMsgDaeInfo2.cause_desact_dae_pb_capteur_ori);
                    msgSteeringErrInfo.cause_desact_dae_pb_can_rob_cpt = ze50_can_id_111_dae_info_fh_cause_desact_dae_pb_can_rob_cpt_decode(canMsgDaeInfo2.cause_desact_dae_pb_can_rob_cpt);
                    msgSteeringErrInfo.cause_desact_dae_pb_can_rob_chk = ze50_can_id_111_dae_info_fh_cause_desact_dae_pb_can_rob_chk_decode(canMsgDaeInfo2.cause_desact_dae_pb_can_rob_chk);
                    msgSteeringErrInfo.cause_desact_dae_pb_can_rot_timeout = ze50_can_id_111_dae_info_fh_cause_desact_dae_pb_can_rot_timeout_decode(canMsgDaeInfo2.cause_desact_dae_pb_can_rot_timeout);
                    msgSteeringErrInfo.cause_desact_dae_reprise_externe = ze50_can_id_111_dae_info_fh_cause_desact_dae_reprise_externe_decode(canMsgDaeInfo2.cause_desact_dae_reprise_externe);
                    msgSteeringErrInfo.cause_desact_dae_reprise_manu = ze50_can_id_111_dae_info_fh_cause_desact_dae_reprise_manu_decode(canMsgDaeInfo2.cause_desact_dae_reprise_manu);
                    msgSteeringErrInfo.dae_dmd_pilotage_led_dft = ze50_can_id_111_dae_info_fh_dae_dmd_pilotage_led_dft_decode(canMsgDaeInfo2.dae_dmd_pilotage_led_dft);
                    msgSteeringErrInfo.dae_dmd_pilotage_buzzer = ze50_can_id_111_dae_info_fh_dae_dmd_pilotage_buzzer_decode(canMsgDaeInfo2.dae_dmd_pilotage_buzzer);
                    msgSteeringErrInfo.dae_dmd_pilotage_led = ze50_can_id_111_dae_info_fh_dae_dmd_pilotage_led_decode(canMsgDaeInfo2.dae_dmd_pilotage_led);
                    pub_steer_err_info_->publish(msgSteeringErrInfo);
                }
            break;
            }
            
            // --------------------------------- CAN ID 210 - Accel Info FH ---------------------------------
            case ZE50_CAN_ID_210_ACCEL_INFO_FH_FRAME_ID: {
                ze50_can_id_210_accel_info_fh_t canMsgAccelInfo;
                error = ze50_can_id_210_accel_info_fh_unpack(&canMsgAccelInfo, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::AccelInfo msgAccelInfo;
                    msgAccelInfo.header = canMsg->header;
                    msgAccelInfo.position_accel_piste_1_mesure = ze50_can_id_210_accel_info_fh_position_accel_piste_1_mesure_decode(canMsgAccelInfo.position_accel_piste_1_mesure);
                    msgAccelInfo.position_accel_piste_2_mesure = ze50_can_id_210_accel_info_fh_position_accel_piste_2_mesure_decode(canMsgAccelInfo.position_accel_piste_2_mesure);
                    msgAccelInfo.position_accel_piste_1_genere = ze50_can_id_210_accel_info_fh_position_accel_piste_1_genere_decode(canMsgAccelInfo.position_accel_piste_1_genere);
                    msgAccelInfo.mode_accel_en_cours = ze50_can_id_210_accel_info_fh_mode_accel_en_cours_decode(canMsgAccelInfo.mode_accel_en_cours);
                    pub_accel_info_->publish(msgAccelInfo);
                }
            break;
            }
            
            // --------------------------------- CAN ID 211 - Accel Info FH (suite) ---------------------------------
            case ZE50_CAN_ID_211_ACCEL_INFO_FH_FRAME_ID: {
                ze50_can_id_211_accel_info_fh_t canMsgAccelInfo2;
                error = ze50_can_id_211_accel_info_fh_unpack(&canMsgAccelInfo2, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::AccelErrInfo msgAccelErrInfo;
                    msgAccelErrInfo.header = canMsg->header;
                    msgAccelErrInfo.erreur_perte_can_intersystem_acce = ze50_can_id_211_accel_info_fh_erreur_perte_can_intersystem_acce_decode(canMsgAccelInfo2.erreur_perte_can_intersystem_acce);
                    msgAccelErrInfo.erreur_capteur_accel_incoh_ped1_2 = ze50_can_id_211_accel_info_fh_erreur_capteur_accel_incoh_ped1_2_decode(canMsgAccelInfo2.erreur_capteur_accel_incoh_ped1_2);
                    msgAccelErrInfo.erreur_capteur_accel_origine = ze50_can_id_211_accel_info_fh_erreur_capteur_accel_origine_decode(canMsgAccelInfo2.erreur_capteur_accel_origine);
                    msgAccelErrInfo.cause_desact_accel_reprise_manu_coop = ze50_can_id_211_accel_info_fh_cause_desact_accel_reprise_manu_coop_decode(canMsgAccelInfo2.cause_desact_accel_reprise_manu_coop);
                    msgAccelErrInfo.cause_desact_accel_pb_capteur_ori = ze50_can_id_211_accel_info_fh_cause_desact_accel_pb_capteur_ori_decode(canMsgAccelInfo2.cause_desact_accel_pb_capteur_ori);
                    msgAccelErrInfo.cause_desact_accel_pb_can_rot_cpt = ze50_can_id_211_accel_info_fh_cause_desact_accel_pb_can_rot_cpt_decode(canMsgAccelInfo2.cause_desact_accel_pb_can_rot_cpt);
                    msgAccelErrInfo.cause_desact_accel_pb_can_rob_chk = ze50_can_id_211_accel_info_fh_cause_desact_accel_pb_can_rob_chk_decode(canMsgAccelInfo2.cause_desact_accel_pb_can_rob_chk);
                    msgAccelErrInfo.cause_desact_accel_pb_can_rob_timeou = ze50_can_id_211_accel_info_fh_cause_desact_accel_pb_can_rob_timeou_decode(canMsgAccelInfo2.cause_desact_accel_pb_can_rob_timeou);
                    msgAccelErrInfo.cause_desact_accel_reprise_externe = ze50_can_id_211_accel_info_fh_cause_desact_accel_reprise_externe_decode(canMsgAccelInfo2.cause_desact_accel_reprise_externe);
                    msgAccelErrInfo.cause_desact_accel_reprise_manu = ze50_can_id_211_accel_info_fh_cause_desact_accel_reprise_manu_decode(canMsgAccelInfo2.cause_desact_accel_reprise_manu);
                    msgAccelErrInfo.accel_dmd_pilotage_led_dft = ze50_can_id_211_accel_info_fh_accel_dmd_pilotage_led_dft_decode(canMsgAccelInfo2.accel_dmd_pilotage_led_dft);
                    msgAccelErrInfo.accel_dmd_pilotage_buzzer = ze50_can_id_211_accel_info_fh_accel_dmd_pilotage_buzzer_decode(canMsgAccelInfo2.accel_dmd_pilotage_buzzer);
                    msgAccelErrInfo.accel_dmd_pilotage_led = ze50_can_id_211_accel_info_fh_accel_dmd_pilotage_led_decode(canMsgAccelInfo2.accel_dmd_pilotage_led);
                    pub_accel_err_info_->publish(msgAccelErrInfo);
                }
            break;
            }
            
            // --------------------------------- CAN ID 310 - Frein Info FH ---------------------------------
            case ZE50_CAN_ID_310_FREIN_INFO_FH_FRAME_ID: {
                ze50_can_id_310_frein_info_fh_t canMsgFreinInfo;
                error = ze50_can_id_310_frein_info_fh_unpack(&canMsgFreinInfo, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::BrakeInfo msgBrakeInfo;
                    msgBrakeInfo.header = canMsg->header;
                    msgBrakeInfo.position_frein_piste_1_mesure = ze50_can_id_310_frein_info_fh_position_frein_piste_1_mesure_decode(canMsgFreinInfo.position_frein_piste_1_mesure);
                    msgBrakeInfo.position_frein_piste_2_mesure = ze50_can_id_310_frein_info_fh_position_frein_piste_2_mesure_decode(canMsgFreinInfo.position_frein_piste_2_mesure);
                    msgBrakeInfo.position_frein_piste_1_genere = ze50_can_id_310_frein_info_fh_position_frein_piste_1_genere_decode(canMsgFreinInfo.position_frein_piste_1_genere);
                    msgBrakeInfo.pression_frein_mesure = ze50_can_id_310_frein_info_fh_pression_frein_mesure_decode(canMsgFreinInfo.pression_frein_mesure);
                    msgBrakeInfo.mode_frein_en_cours = ze50_can_id_310_frein_info_fh_mode_frein_en_cours_decode(canMsgFreinInfo.mode_frein_en_cours);
                    pub_brake_info_->publish(msgBrakeInfo);
                }
            break;
            }
            
            // --------------------------------- CAN ID 311 - Frein Info FH (suite) ---------------------------------
            case ZE50_CAN_ID_311_FREIN_INFO_FH_FRAME_ID: {
                ze50_can_id_311_frein_info_fh_t canMsgFreinInfo2;
                error = ze50_can_id_311_frein_info_fh_unpack(&canMsgFreinInfo2, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::BrakeErrInfo msgBrakeErrInfo;
                    msgBrakeErrInfo.header = canMsg->header;
                    msgBrakeErrInfo.erreur_perte_can_intersystem_frei = ze50_can_id_311_frein_info_fh_erreur_perte_can_intersystem_frei_decode(canMsgFreinInfo2.erreur_perte_can_intersystem_frei);
                    msgBrakeErrInfo.erreur_capteur_frein_incoh_ped1_2 = ze50_can_id_311_frein_info_fh_erreur_capteur_frein_incoh_ped1_2_decode(canMsgFreinInfo2.erreur_capteur_frein_incoh_ped1_2);
                    msgBrakeErrInfo.erreur_capteur_frein_origine = ze50_can_id_311_frein_info_fh_erreur_capteur_frein_origine_decode(canMsgFreinInfo2.erreur_capteur_frein_origine);
                    msgBrakeErrInfo.cause_desact_frein_reprise_manu_coop = ze50_can_id_311_frein_info_fh_cause_desact_frein_reprise_manu_coop_decode(canMsgFreinInfo2.cause_desact_frein_reprise_manu_coop);
                    msgBrakeErrInfo.cause_desact_frein_pb_capteur_ori = ze50_can_id_311_frein_info_fh_cause_desact_frein_pb_capteur_ori_decode(canMsgFreinInfo2.cause_desact_frein_pb_capteur_ori);
                    msgBrakeErrInfo.cause_desact_frein_pb_can_rot_cpt = ze50_can_id_311_frein_info_fh_cause_desact_frein_pb_can_rot_cpt_decode(canMsgFreinInfo2.cause_desact_frein_pb_can_rot_cpt);
                    msgBrakeErrInfo.cause_desact_frein_pb_can_rob_chk = ze50_can_id_311_frein_info_fh_cause_desact_frein_pb_can_rob_chk_decode(canMsgFreinInfo2.cause_desact_frein_pb_can_rob_chk);
                    msgBrakeErrInfo.cause_desact_frein_pb_can_rob_timeou = ze50_can_id_311_frein_info_fh_cause_desact_frein_pb_can_rob_timeou_decode(canMsgFreinInfo2.cause_desact_frein_pb_can_rob_timeou);
                    msgBrakeErrInfo.cause_desact_frein_reprise_externe = ze50_can_id_311_frein_info_fh_cause_desact_frein_reprise_externe_decode(canMsgFreinInfo2.cause_desact_frein_reprise_externe);
                    msgBrakeErrInfo.cause_desact_frein_reprise_manu = ze50_can_id_311_frein_info_fh_cause_desact_frein_reprise_manu_decode(canMsgFreinInfo2.cause_desact_frein_reprise_manu);
                    msgBrakeErrInfo.frein_dmd_pilotage_led = ze50_can_id_311_frein_info_fh_frein_dmd_pilotage_led_dft_decode(canMsgFreinInfo2.frein_dmd_pilotage_led_dft);
                    msgBrakeErrInfo.frein_dmd_pilotage_buzzer = ze50_can_id_311_frein_info_fh_frein_dmd_pilotage_buzzer_decode(canMsgFreinInfo2.frein_dmd_pilotage_buzzer);
                    msgBrakeErrInfo.frein_dmd_pilotage_led_dft = ze50_can_id_311_frein_info_fh_frein_dmd_pilotage_led_decode(canMsgFreinInfo2.frein_dmd_pilotage_led);
                    pub_brake_err_info_->publish(msgBrakeErrInfo);
                }
            break;
            }
            
            // --------------------------------- CAN ID 410 - RNDB Info FH ---------------------------------
            case ZE50_CAN_ID_410_RNDB_INFO_FH_FRAME_ID: {
                ze50_can_id_410_rndb_info_fh_t canMsgRndbInfo;
                error = ze50_can_id_410_rndb_info_fh_unpack(&canMsgRndbInfo, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::RNDBInfo msgRndbInfo;
                    msgRndbInfo.header = canMsg->header;
                    msgRndbInfo.rndb_engage_genere = ze50_can_id_410_rndb_info_fh_rndb_engage_genere_decode(canMsgRndbInfo.rndb_engage_genere);
                    msgRndbInfo.rndb_engage_mesure = ze50_can_id_410_rndb_info_fh_rndb_engage_mesure_decode(canMsgRndbInfo.rndb_engage_mesure);
                    msgRndbInfo.rndb_changement_invalide_renault = ze50_can_id_410_rndb_info_fh_rndb_changement_invalide_renault_decode(canMsgRndbInfo.rndb_changement_invalide_renault);
                    msgRndbInfo.rndb_changement_invalide_pied_frein = ze50_can_id_410_rndb_info_fh_rndb_changement_invalide_pied_frein_decode(canMsgRndbInfo.rndb_changement_invalide_pied_frein);
                    msgRndbInfo.rndb_changement_valide = ze50_can_id_410_rndb_info_fh_rndb_changement_valide_decode(canMsgRndbInfo.rndb_changement_valide);
                    msgRndbInfo.mode_rndb_en_cours = ze50_can_id_410_rndb_info_fh_mode_rndb_en_cours_decode(canMsgRndbInfo.mode_rndb_en_cours);
                    pub_rndb_info_->publish(msgRndbInfo);
                }
            break;
            }
            
            // --------------------------------- CAN ID 411 - RNDB Info FH (suite) ---------------------------------
            case ZE50_CAN_ID_411_RNDB_INFO_FH_FRAME_ID: {
                ze50_can_id_411_rndb_info_fh_t canMsgRndbInfo2;
                error = ze50_can_id_411_rndb_info_fh_unpack(&canMsgRndbInfo2, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::RNDBErrInfo msgRNDBErrInfo;
                    msgRNDBErrInfo.header = canMsg->header;
                    msgRNDBErrInfo.erreur_perte_can_intersystem_rndb = ze50_can_id_411_rndb_info_fh_erreur_perte_can_intersystem_rndb_decode(canMsgRndbInfo2.erreur_perte_can_intersystem_rndb);
                    msgRNDBErrInfo.erreur_capteur_rndb_incoh_ped1_2 = ze50_can_id_411_rndb_info_fh_erreur_capteur_rndb_incoh_ped1_2_decode(canMsgRndbInfo2.erreur_capteur_rndb_incoh_ped1_2);
                    msgRNDBErrInfo.erreur_capteur_rndb_origine = ze50_can_id_411_rndb_info_fh_erreur_capteur_rndb_origine_decode(canMsgRndbInfo2.erreur_capteur_rndb_origine);
                    msgRNDBErrInfo.cause_desact_rndb_reprise_manu_coop = ze50_can_id_411_rndb_info_fh_cause_desact_rndb_reprise_manu_coop_decode(canMsgRndbInfo2.cause_desact_rndb_reprise_manu_coop);
                    msgRNDBErrInfo.cause_desact_rndb_pb_capteur_ori = ze50_can_id_411_rndb_info_fh_cause_desact_rndb_pb_capteur_ori_decode(canMsgRndbInfo2.cause_desact_rndb_pb_capteur_ori);
                    msgRNDBErrInfo.cause_desact_rndb_pb_can_rot_cpt = ze50_can_id_411_rndb_info_fh_cause_desact_rndb_pb_can_rot_cpt_decode(canMsgRndbInfo2.cause_desact_rndb_pb_can_rot_cpt);
                    msgRNDBErrInfo.cause_desact_rndb_pb_can_rob_chk = ze50_can_id_411_rndb_info_fh_cause_desact_rndb_pb_can_rob_chk_decode(canMsgRndbInfo2.cause_desact_rndb_pb_can_rob_chk);
                    msgRNDBErrInfo.cause_desact_rndb_pb_can_rob_timeou = ze50_can_id_411_rndb_info_fh_cause_desact_rndb_pb_can_rob_timeou_decode(canMsgRndbInfo2.cause_desact_rndb_pb_can_rob_timeou);
                    msgRNDBErrInfo.cause_desact_rndb_reprise_externe = ze50_can_id_411_rndb_info_fh_cause_desact_rndb_reprise_externe_decode(canMsgRndbInfo2.cause_desact_rndb_reprise_externe);
                    msgRNDBErrInfo.cause_desact_rndb_reprise_manu = ze50_can_id_411_rndb_info_fh_cause_desact_rndb_reprise_manu_decode(canMsgRndbInfo2.cause_desact_rndb_reprise_manu);
                    msgRNDBErrInfo.rndb_dmd_pilotage_led_dft = ze50_can_id_411_rndb_info_fh_rndb_dmd_pilotage_led_dft_decode(canMsgRndbInfo2.rndb_dmd_pilotage_led_dft);
                    msgRNDBErrInfo.rndb_dmd_pilotage_buzzer = ze50_can_id_411_rndb_info_fh_rndb_dmd_pilotage_buzzer_decode(canMsgRndbInfo2.rndb_dmd_pilotage_buzzer);
                    msgRNDBErrInfo.rndb_dmd_pilotage_led = ze50_can_id_411_rndb_info_fh_rndb_dmd_pilotage_led_decode(canMsgRndbInfo2.rndb_dmd_pilotage_led);
                    pub_rndb_err_info_->publish(msgRNDBErrInfo);
                }
            break;
            }
            
            // --------------------------------- CAN ID 650 - Optimisation 1 ---------------------------------
            case ZE50_CAN_ID_650_OPTIMISATION_1_FRAME_ID: {
                ze50_can_id_650_optimisation_1_t canMsgOptimisation1;
                error = ze50_can_id_650_optimisation_1_unpack(&canMsgOptimisation1, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::RearWheelsSteering msgRearWheelsSteering;
                    msgRearWheelsSteering.header = canMsg->header;
                    msgRearWheelsSteering.vit_roue_arg = ze50_can_id_650_optimisation_1_vit_roue_arg_decode(canMsgOptimisation1.vit_roue_arg);
                    msgRearWheelsSteering.vit_roue_ard = ze50_can_id_650_optimisation_1_vit_roue_ard_decode(canMsgOptimisation1.vit_roue_ard);
                    msgRearWheelsSteering.angle_volant = ze50_can_id_650_optimisation_1_angle_volant_decode(canMsgOptimisation1.angle_volant);
                    msgRearWheelsSteering.sens_roue_arg = ze50_can_id_650_optimisation_1_sens_roue_arg_decode(canMsgOptimisation1.sens_roue_arg);
                    msgRearWheelsSteering.sens_roue_ard = ze50_can_id_650_optimisation_1_sens_roue_ard_decode(canMsgOptimisation1.sens_roue_ard);
                    pub_rear_wheels_steering_->publish(msgRearWheelsSteering);
                }
            break;
            }
            
            // --------------------------------- CAN ID 651 - Optimisation 2 ---------------------------------
            case ZE50_CAN_ID_651_OPTIMISATION_2_FRAME_ID: {
                ze50_can_id_651_optimisation_2_t canMsgFrontWheelsSteering;
                error = ze50_can_id_651_optimisation_2_unpack(&canMsgFrontWheelsSteering, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::FrontWheelsSteering msgFrontWheelsSteering;
                    msgFrontWheelsSteering.header = canMsg->header;
                    msgFrontWheelsSteering.vit_roue_avg = ze50_can_id_651_optimisation_2_vit_roue_avg_decode(canMsgFrontWheelsSteering.vit_roue_avg);
                    msgFrontWheelsSteering.vit_roue_avd = ze50_can_id_651_optimisation_2_vit_roue_avd_decode(canMsgFrontWheelsSteering.vit_roue_avd);
                    msgFrontWheelsSteering.pedale_accel_enfoncement = ze50_can_id_651_optimisation_2_pedale_accel_enfoncement_decode(canMsgFrontWheelsSteering.pedale_accel_enfoncement);
                    msgFrontWheelsSteering.pedale_frein_enfoncement = ze50_can_id_651_optimisation_2_pression_frein_renault_decode(canMsgFrontWheelsSteering.pression_frein_renault);
                    pub_front_wheels_steering_->publish(msgFrontWheelsSteering);
                }
            break;
            }
            
            // --------------------------------- CAN ID 652 - Optimisation 3 ---------------------------------
            case ZE50_CAN_ID_652_OPTIMISATION_3_FRAME_ID: {
                ze50_can_id_652_optimisation_3_t canMsgAccessories;
                error = ze50_can_id_652_optimisation_3_unpack(&canMsgAccessories, canData, (size_t)canMsg->dlc);
                if(error >= 0){
                    can_zoe_msgs::msg::Accessories msgAccessories;
                    msgAccessories.header = canMsg->header;
                    msgAccessories.feux_warning_commodo = ze50_can_id_652_optimisation_3_feux_warning_commodo_decode(canMsgAccessories.feux_warning_commodo);
                    msgAccessories.feux_clignotant_d_commodo = ze50_can_id_652_optimisation_3_feux_clignotant_d_commodo_decode(canMsgAccessories.feux_clignotant_d_commodo);
                    msgAccessories.feux_clignotant_g_commodo = ze50_can_id_652_optimisation_3_feux_clignotant_g_commodo_decode(canMsgAccessories.feux_clignotant_g_commodo);
                    msgAccessories.feux_brouillard_ar_commodo = ze50_can_id_652_optimisation_3_feux_brouillard_ar_commodo_decode(canMsgAccessories.feux_brouillard_ar_commodo);
                    msgAccessories.feux_route_commodo = ze50_can_id_652_optimisation_3_feux_route_commodo_decode(canMsgAccessories.feux_route_commodo);
                    msgAccessories.feux_codes_commodo = ze50_can_id_652_optimisation_3_feux_codes_commodo_decode(canMsgAccessories.feux_codes_commodo);
                    msgAccessories.feux_veilleuses_commodo = ze50_can_id_652_optimisation_3_feux_veilleuses_commodo_decode(canMsgAccessories.feux_veilleuses_commodo);
                    msgAccessories.feux_warning_etat = ze50_can_id_652_optimisation_3_feux_warning_etat_decode(canMsgAccessories.feux_warning_etat);
                    msgAccessories.feux_clignotant_d_etat = ze50_can_id_652_optimisation_3_feux_clignotant_d_etat_decode(canMsgAccessories.feux_clignotant_d_etat);
                    msgAccessories.feux_clignotant_g_etat = ze50_can_id_652_optimisation_3_feux_clignotant_g_etat_decode(canMsgAccessories.feux_clignotant_g_etat);
                    msgAccessories.feux_brouillard_ar_etat = ze50_can_id_652_optimisation_3_feux_brouillard_ar_etat_decode(canMsgAccessories.feux_brouillard_ar_etat);
                    msgAccessories.feux_route_etat = ze50_can_id_652_optimisation_3_feux_route_etat_decode(canMsgAccessories.feux_route_etat);
                    msgAccessories.feux_codes_etat = ze50_can_id_652_optimisation_3_feux_codes_etat_decode(canMsgAccessories.feux_codes_etat);
                    msgAccessories.feux_veilleuses_etat = ze50_can_id_652_optimisation_3_feux_veilleuses_etat_decode(canMsgAccessories.feux_veilleuses_etat);
                    msgAccessories.ouvrant_coffre_ar_etat = ze50_can_id_652_optimisation_3_ouvrant_coffre_ar_etat_decode(canMsgAccessories.ouvrant_coffre_ar_etat);
                    msgAccessories.ouvrant_portiere_ard_etat = ze50_can_id_652_optimisation_3_ouvrant_portiere_ard_etat_decode(canMsgAccessories.ouvrant_portiere_ard_etat);
                    msgAccessories.ouvrant_portiere_arg_etat = ze50_can_id_652_optimisation_3_ouvrant_portiere_arg_etat_decode(canMsgAccessories.ouvrant_portiere_arg_etat);
                    msgAccessories.ouvrant_portiere_avd_etat = ze50_can_id_652_optimisation_3_ouvrant_portiere_avd_etat_decode(canMsgAccessories.ouvrant_portiere_avd_etat);
                    msgAccessories.ouvrant_portiere_avg_etat = ze50_can_id_652_optimisation_3_ouvrant_portiere_avg_etat_decode(canMsgAccessories.ouvrant_portiere_avg_etat);
                    msgAccessories.ceinture_securite_arc = ze50_can_id_652_optimisation_3_ceinture_securite_arc_decode(canMsgAccessories.ceinture_securite_arc);
                    msgAccessories.ceinture_securite_ard = ze50_can_id_652_optimisation_3_ceinture_securite_ard_decode(canMsgAccessories.ceinture_securite_ard);
                    msgAccessories.ceinture_securite_arg = ze50_can_id_652_optimisation_3_ceinture_securite_arg_decode(canMsgAccessories.ceinture_securite_arg);
                    msgAccessories.ceinture_securite_passager_en_secu = ze50_can_id_652_optimisation_3_ceinture_securite_passager_en_secu_decode(canMsgAccessories.ceinture_securite_passager_en_secu);
                    msgAccessories.ceinture_securite_conducteu_en_secu = ze50_can_id_652_optimisation_3_ceinture_securite_conducteu_en_secu_decode(canMsgAccessories.ceinture_securite_conducteu_en_secu);
                    msgAccessories.essuies_glace_vit_grande_etat = ze50_can_id_652_optimisation_3_essuies_glace_vit_grande_etat_decode(canMsgAccessories.essuies_glace_vit_grande_etat);
                    msgAccessories.essuies_glace_vit_petite_etat = ze50_can_id_652_optimisation_3_essuies_glace_vit_petite_etat_decode(canMsgAccessories.essuies_glace_vit_petite_etat);
                    msgAccessories.essuies_glace_vit_grande_commodo = ze50_can_id_652_optimisation_3_essuies_glace_vit_grande_commodo_decode(canMsgAccessories.essuies_glace_vit_grande_commodo);
                    msgAccessories.essuies_glace_vit_petite_commodo = ze50_can_id_652_optimisation_3_essuies_glace_vit_petite_commodo_decode(canMsgAccessories.essuies_glace_vit_petite_commodo);
                    msgAccessories.etat_frein_a_main = ze50_can_id_652_optimisation_3_etat_frein_a_main_decode(canMsgAccessories.etat_frein_a_main);
                    msgAccessories.etat_pedale_accel = ze50_can_id_652_optimisation_3_etat_pedale_accel_decode(canMsgAccessories.etat_pedale_accel);
                    msgAccessories.etat_pedale_frein = ze50_can_id_652_optimisation_3_etat_pedale_frein_decode(canMsgAccessories.etat_pedale_frein);
                    msgAccessories.position_rndb = ze50_can_id_652_optimisation_3_position_rndb_decode(canMsgAccessories.position_rndb);
                    pub_accessories_->publish(msgAccessories);
                }
            break;
            }
            
            // --------------------------------- CAN ID ? - Default ---------------------------------
            default:
            // If the CAN message is not in any cases of the switch
                if(error == 100){
                    RCLCPP_DEBUG(this->get_logger(), "CAN ID 0x%X not supported for decoding!", canMsg->id);
                }
                // Return the decoding error if an error occur
                else if(error < 0){
                    RCLCPP_ERROR(this->get_logger(), "Error %d during CAN ID 0x%X decoding!", error, canMsg->id);
                }
            break;
          // ---------------------------------------------------------------- End Switch CAN ID Case ----------------------------------------------------------------          
       
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
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "CAN decoding error: %s", e.what());
    }
    delete[] canData;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZE50CanDecoder>());
  rclcpp::shutdown();
  return 0;
}

