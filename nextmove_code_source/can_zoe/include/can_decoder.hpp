#pragma once

class ICanDecoder{
    public :
        virtual void decodeCanMsg(const can_msgs::msg::Frame::SharedPtr canMsg) const = 0;
};