#include "ze50/can_ze50_encoder.hpp"

CanZE50Encoder::CanZE50Encoder() : CanEncoder("can_ze50_encoder", COUNT_CMD)
{
	this->vehicleCommands_[BRAKE] = std::make_shared<BrakeCommand>();
	this->vehicleCommands_[RNDB] = std::make_shared<RNDBCommand>();
	this->vehicleCommands_[STEER] = std::make_shared<SteerCommand>();
	this->vehicleCommands_[ACCELERATION] = std::make_shared<AccelerationCommand>();
	subCmdAccel_ = this->create_subscription<can_zoe_msgs::msg::CmdAcceleration>("/cmd_accel", rclcpp::SensorDataQoS(),std::bind(&CanZE50Encoder::receiveAccelerationCommand, this, std::placeholders::_1));
    subCmdBrake_ = this->create_subscription<can_zoe_msgs::msg::CmdBrake>("/cmd_brake", rclcpp::SensorDataQoS(),std::bind(&CanZE50Encoder::receiveBrakeCommand, this, std::placeholders::_1));
    subCmdSteering_ = this->create_subscription<can_zoe_msgs::msg::CmdSteering>("/cmd_steering", rclcpp::SensorDataQoS(),std::bind(&CanZE50Encoder::receiveSteerCommand, this, std::placeholders::_1));
}

void CanZE50Encoder::receiveBrakeCommand(const can_zoe_msgs::msg::CmdBrake::SharedPtr cmdBrake)
{
	this->t_valuesAtTime_[BRAKE] = clock_->now();
	static std::shared_ptr<BrakeCommand> brakeCommandPtr = std::dynamic_pointer_cast<BrakeCommand>(this->vehicleCommands_[BRAKE]);
	// TODO add security
	brakeCommandPtr->receiveBrakeCommand(cmdBrake);
}

void CanZE50Encoder::receiveRNDBCommand(const can_zoe_msgs::msg::CmdRNDB::SharedPtr cmdRNDB)
{
	this->t_valuesAtTime_[RNDB] = clock_->now();
	static std::shared_ptr<RNDBCommand> rndbCommandPtr = std::dynamic_pointer_cast<RNDBCommand>(this->vehicleCommands_[RNDB]);
	rndbCommandPtr->receiveRNDBCommand(cmdRNDB);
}

void CanZE50Encoder::receiveAccelerationCommand(const can_zoe_msgs::msg::CmdAcceleration::SharedPtr cmdAccel)
{
	this->t_valuesAtTime_[ACCELERATION] = clock_->now();
	static std::shared_ptr<AccelerationCommand> accelerationCommandPtr = std::dynamic_pointer_cast<AccelerationCommand>(this->vehicleCommands_[ACCELERATION]);
	accelerationCommandPtr->receiveAccelerationCommand(cmdAccel);
}

void CanZE50Encoder::receiveSteerCommand(const can_zoe_msgs::msg::CmdSteering::SharedPtr cmdSteer)
{
	this->t_valuesAtTime_[STEER] = clock_->now();
	static std::shared_ptr<SteerCommand> steerCommandPtr = std::dynamic_pointer_cast<SteerCommand>(this->vehicleCommands_[STEER]);
	steerCommandPtr->receiveSteerCommand(cmdSteer);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanZE50Encoder>());
  rclcpp::shutdown();
  return 0;
}