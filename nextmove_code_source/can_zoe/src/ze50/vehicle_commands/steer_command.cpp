#include "ze50/ze50_vehicle_command.hpp"

void SteerCommand::receiveSteerCommand(const can_zoe_msgs::msg::CmdSteering::SharedPtr cmdSteer)
{
	header_ = cmdSteer->header;
	steerAngle_ = cmdSteer->angle;
	steerRotationSpeed_ = cmdSteer->rotation_speed;
	requestToPublish_ = cmdSteer->can_publish;
}

status SteerCommand::encodeMsg(can_msgs::msg::Frame &canMsg) const
{
	ze50_can_id_100_cons_dae_autonome_t canCmdDae;
	canCmdDae.cons_dae_autonome = ze50_can_id_100_cons_dae_autonome_cons_dae_autonome_encode(double(steerAngle_));
	canCmdDae.cons_vitesse_dae = ze50_can_id_100_cons_dae_autonome_cons_vitesse_dae_encode(double(steerRotationSpeed_));
	canCmdDae.reactivation_mode_cooperatif_dae = ze50_can_id_100_cons_dae_autonome_reactivation_mode_cooperatif_dae_encode(double(steerCoopValue_));

	canMsg.header = header_;
	canMsg.id = ZE50_CAN_ID_100_CONS_DAE_AUTONOME_FRAME_ID;
	canMsg.dlc = 8u;
	canMsg.is_rtr = false;
	canMsg.is_extended = false;
	canMsg.is_error = false;

	uint8_t canData[8];
	if (ze50_can_id_100_cons_dae_autonome_pack(canData, &canCmdDae, 8) == -EINVAL)
	{
		// RCLCPP_WARN_STREAM_SKIPFIRST(this->get_logger(), "Steering wheel CAN data packing error, exiting callback");
		return FAILURE;
	}

	for(int i = 0; i < canMsg.dlc; i++){
		canMsg.data[i] = (unsigned char)canData[i];
	}
    return SUCCESS;
}