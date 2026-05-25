#pragma once

#include "can_msgs/msg/frame.hpp"
#include <rclcpp/rclcpp.hpp>

enum status {
    SUCCESS,
    FAILURE
};

class IVehicleCommand {
    public:
        virtual status encodeMsg(can_msgs::msg::Frame &canMsg) const = 0;
        virtual bool isPublishRequested() const = 0;
        virtual bool isSetToPublish() const = 0;
        virtual void setPublishStatusToFalse() = 0;
        virtual void setPublishRequestToFalse() = 0;
        virtual void publishEqualsRequest() = 0;
        virtual std::string getTypeName() const = 0;
};

class VehicleCommand : public IVehicleCommand
{
    public:
        virtual status encodeMsg(can_msgs::msg::Frame &canMsg) const = 0;
        virtual std::string getTypeName() const;
        bool isPublishRequested() const;
        bool isSetToPublish() const;
        void setPublishStatusToFalse();
        void setPublishRequestToFalse();
        void publishEqualsRequest();
        VehicleCommand(const std::string commandName);
        ~VehicleCommand() = default;

    protected:
        bool                    requestToPublish_;
        bool                    setToPublish_;
        std_msgs::msg::Header   header_;
        const std::string       commandName_;

};