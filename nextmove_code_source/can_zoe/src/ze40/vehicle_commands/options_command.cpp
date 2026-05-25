

void cmdSendOptions(const can_zoe_ze40_msgs::msg::CmdOptions::SharedPtr cmdOption)
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
}
    
