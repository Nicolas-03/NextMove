#include "ze50/ze50_vehicle_command.hpp"

void RNDBCommand::receiveRNDBCommand(const can_zoe_msgs::msg::CmdRNDB::SharedPtr cmdRNDB)
{
	header_ = cmdRNDB->header;
	rndbValue_ = cmdRNDB->rndb;
	rndbRetry_ = cmdRNDB->rndb_retry;
	requestToPublish_ = cmdRNDB->can_publish;
}

status RNDBCommand::encodeMsg(can_msgs::msg::Frame &canMsg) const
{
	// Encodre RNDB Msg on CAN
	ze50_can_id_400_cons_rndb_robotise_t canCmdRndb;
	canCmdRndb.cons_rndb_robotise = ze50_can_id_400_cons_rndb_robotise_cons_rndb_robotise_encode(double(rndbValue_));
	canCmdRndb.rndb_redemande_changement_invalid = ze50_can_id_400_cons_rndb_robotise_rndb_redemande_changement_invalid_encode(double(rndbRetry_));

	canMsg.header = header_;
	canMsg.id = ZE50_CAN_ID_400_CONS_RNDB_ROBOTISE_FRAME_ID;
	canMsg.dlc = 8u;
	canMsg.is_rtr = false;
	canMsg.is_extended = false;
	canMsg.is_error = false;

	uint8_t canData[8];
	if (ze50_can_id_400_cons_rndb_robotise_pack(canData, &canCmdRndb, 8) == -EINVAL)
	{
		// RCLCPP_WARN_STREAM_SKIPFIRST(this->get_logger(), "RNDB CAN data packing error, exiting callback");
		return FAILURE;
	}

	for(int i = 0; i < canMsg.dlc; i++){
		canMsg.data[i] = (unsigned char)canData[i];
	}
	return SUCCESS;
}

