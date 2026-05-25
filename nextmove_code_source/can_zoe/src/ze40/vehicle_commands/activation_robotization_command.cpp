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