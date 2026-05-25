#include "vehicle_command.hpp"

VehicleCommand::VehicleCommand(const std::string commandName) : commandName_(commandName)
{

}

std::string VehicleCommand::getTypeName() const 
{
    return this->commandName_;
}

void VehicleCommand::setPublishRequestToFalse()
{
    this->requestToPublish_ = false;
}

void VehicleCommand::setPublishStatusToFalse()
{
    this->setToPublish_ = false;
}

bool VehicleCommand::isPublishRequested() const
{
    return requestToPublish_; 
}

bool VehicleCommand::isSetToPublish() const
{
    return setToPublish_;
}

void VehicleCommand::publishEqualsRequest()
{
    setToPublish_ = requestToPublish_;
}