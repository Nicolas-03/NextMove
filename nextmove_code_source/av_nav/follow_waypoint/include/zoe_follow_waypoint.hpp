#pragma once
#include <memory>
#include <string>
#include <cmath>
#include <functional>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "can_zoe_msgs/msg/cmd_steering.hpp"
#include "can_zoe_msgs/msg/steering_info.hpp"
#include "can_zoe_msgs/msg/front_wheels_steering.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "novatel_oem7_msgs/msg/inspva.hpp"

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "zoe_waypoint_interfaces/srv/get_path.hpp"
#include "zoe_waypoint_interfaces/srv/change_path.hpp"

#include "math_utils.hpp"
#include "msg_conversion.hpp"

#define MINIMAL_DISTANCE 10.0

enum MsgType {
    PATH,
    GPS,
    GPS_START,
    ODOMETRY,
    INSPVAX,
    SPEED,
    MSG_COUNT,
};

constexpr double WHEEL_BASE = 2.588; // distance between front and rear wheels in meters
constexpr int TIMER_MS = 10; // seconds, the bus can needs information every 10 ms
constexpr double TIMER_S = (static_cast<double>(TIMER_MS) * 0.001); // seconds, the bus can needs information every 10 ms
constexpr double STEER_ROTATION_SPEED = 350; // deg/s
constexpr double MAX_ROTATION_SPEED = 800; // deg/s
constexpr double WHEEL_TO_STEER_RATIO = 15;


class FollowWaypoint : public rclcpp::Node
{
    public:
        FollowWaypoint();
        ~FollowWaypoint() = default;
        void    timer_callback();
    private:
        // Callbacks
        void                                callback_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void                                callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
        void                                callback_inspvax(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg);
        void                                callback_steering(const can_zoe_msgs::msg::SteeringInfo::SharedPtr msg);
        void                                callback_speed(const can_zoe_msgs::msg::FrontWheelsSteering::SharedPtr msg);
        // Service
        void                                get_path(const char* pathname);
        // Client
        void                                callback_change_path(const std::shared_ptr<zoe_waypoint_interfaces::srv::ChangePath::Request> request,
                                                                    std::shared_ptr<zoe_waypoint_interfaces::srv::ChangePath::Response> response);
        // Security check                                                            
        bool                                has_received_all_required_data();
        // Steer Algorithm
        bool                                is_solution_on_path(geometry_msgs::msg::Point const &A, 
                                                                geometry_msgs::msg::Point const &vecAb, 
                                                                geometry_msgs::msg::Point const &C);                                       
        geometry_msgs::msg::Point           compute_solution(geometry_msgs::msg::Point const &A, geometry_msgs::msg::Point const &B);
        geometry_msgs::msg::PointStamped    find_path_point(size_t pathIdx);
        void                                pure_pursuit_steering_angle_computation(can_zoe_msgs::msg::CmdSteering &cmd_steering);
        bool                                closest_point_on_path(const geometry_msgs::msg::Point &point, size_t &pathIndex);
        // Smoothing
        double                              rate_limiter(const double max_rate, double s0, double s1);
        // Rviz simulation
        void                                simulate_angle(geometry_msgs::msg::PointStamped const &currentPointMap, const double wanted_angle);
        

        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                       pubPath_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr          pubPoint_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr           pubFutPose_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr           pubVolPose_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr          pubFutPoint_;
        rclcpp::Publisher<can_zoe_msgs::msg::CmdSteering>::SharedPtr            pubSteering_;
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr            subGps_ ;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                subOdom_;
        rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr         subInspvax_;
        rclcpp::Subscription<can_zoe_msgs::msg::SteeringInfo>::SharedPtr        subSteer_;
        rclcpp::Subscription<can_zoe_msgs::msg::FrontWheelsSteering>::SharedPtr subSpeed_;
        // Service
        rclcpp::Service<zoe_waypoint_interfaces::srv::ChangePath>::SharedPtr    change_path_service_;
        // Client Service
        rclcpp::Client<zoe_waypoint_interfaces::srv::GetPath>::SharedPtr        get_path_client_;
        // Callback msgs and data
        can_zoe_msgs::msg::SteeringInfo                                         steer_;
        novatel_oem7_msgs::msg::INSPVA                                          inspva_;
        sensor_msgs::msg::NavSatFix                                             gps_start_;
        sensor_msgs::msg::NavSatFix                                             gps_;
        nav_msgs::msg::Odometry                                                 odom_;
        double                                                                  avg_speed_;
        // Timer Loop
        rclcpp::TimerBase::SharedPtr                                            timer_;
        // Securities
        // Algorithm doesn't work if not validated
        bool                                                                    hasReceivedData_[MSG_COUNT] = {false};
        bool                                                                    allReceived = false;
        // Transform
        bool simu_;
        geometry_msgs::msg::TransformStamped                                    map_H_baselink_;
        std::shared_ptr<tf2_ros::TransformBroadcaster>                          tf_broadcaster_;
        std::shared_ptr<tf2_ros::Buffer>                                        tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener>                             tf_listener_;
        // Path
        std::string                                                             path_name_;
        nav_msgs::msg::Path                                                     path_;
        // Parameters
        double                                                                  min_ldd_;
        double                                                                  max_ldd_;
        double                                                                  kdd_;

};