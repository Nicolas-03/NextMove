#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

geometry_msgs::msg::PointStamped toPointStamped(const geometry_msgs::msg::PoseStamped& pose) {
    geometry_msgs::msg::PointStamped p;
    p.header = pose.header;
    p.point = pose.pose.position;
    return p;
}
