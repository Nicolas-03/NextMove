#include "ze50/ze50_vehicle_command.hpp"

void AccelerationCommand::receiveAccelerationCommand(const can_zoe_msgs::msg::CmdAcceleration::SharedPtr cmdAccel)
{
    header_ = cmdAccel->header;
    accelerationValue_ = cmdAccel->accel;
    requestToPublish_ = cmdAccel->can_publish;
}

status AccelerationCommand::encodeMsg(can_msgs::msg::Frame &canMsg) const
{
    ze50_can_id_200_cons_accel_autonome_t canCmdAcceleration;
	canCmdAcceleration.cons_accelateur_autonome = ze50_can_id_200_cons_accel_autonome_cons_accelateur_autonome_encode(double(accelerationValue_));

	canMsg.header = header_;
	canMsg.id = ZE50_CAN_ID_200_CONS_ACCEL_AUTONOME_FRAME_ID;
	canMsg.dlc = 8u;
	canMsg.is_rtr = false;
	canMsg.is_extended = false;
	canMsg.is_error = false;

	uint8_t canData[8];
    if (ze50_can_id_200_cons_accel_autonome_pack(canData , &canCmdAcceleration , 8) == -EINVAL) 
	{
		// RCLCPP_WARN_STREAM_SKIPFIRST(this->get_logger(), "Accel CAN data packing error, exiting callback");
		return FAILURE;
	}
    for(int i = 0; i < canMsg.dlc; i++){ // TODO Replace this
			canMsg.data[i] = (unsigned char)canData[i];
		}
    return SUCCESS;
}