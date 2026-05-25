#include "ze50/ze50_vehicle_command.hpp"

void BrakeCommand::receiveBrakeCommand(const can_zoe_msgs::msg::CmdBrake::SharedPtr cmdBrake)
{
	header_ = cmdBrake->header;
    brakeValue_ = cmdBrake->brake;
	requestToPublish_ = cmdBrake->can_publish;
}

status BrakeCommand::encodeMsg(can_msgs::msg::Frame &canMsg) const
{
    ze50_can_id_300_cons_frein_autonome_t canCmdBrake;
	canCmdBrake.cons_frein_autonome = ze50_can_id_300_cons_frein_autonome_cons_frein_autonome_encode(double(brakeValue_));

	canMsg.header = header_;
	canMsg.id = ZE50_CAN_ID_300_CONS_FREIN_AUTONOME_FRAME_ID;
	canMsg.dlc = 8u;
	canMsg.is_rtr = false;
	canMsg.is_extended = false;
	canMsg.is_error = false;

	uint8_t canData[8];
	if (ze50_can_id_300_cons_frein_autonome_pack(canData, &canCmdBrake, 8) == -EINVAL)
	{
		// RCLCPP_WARN_STREAM_SKIPFIRST(this->get_logger(), "Brake CAN data packing error, exiting callback");
		return FAILURE;
	}

	for(int i = 0; i < canMsg.dlc; i++){
		canMsg.data[i] = (unsigned char)canData[i];
	}
    return SUCCESS;
}
