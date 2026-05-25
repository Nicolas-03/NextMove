/**
 * @file zoe_follow_waypoint.cpp
 * @brief Computes and publishes a steering command.
 * @author Elise Colin
 *
 * Publishes a steering command every 10ms based on gps.
 * Angle value is based on a Pure Pursuit algorithm.
 * Needs to be in a 10 meter distance from the path point.
 */
#include "zoe_follow_waypoint.hpp"

FollowWaypoint::FollowWaypoint() : Node("zoe_follow_waypoint")
{
    this->declare_parameter<bool>("simu", false);
    this->get_parameter("simu", simu_);
    this->declare_parameter<double>("min_ldd", 6.5);
    this->get_parameter("min_ldd", min_ldd_);
    this->declare_parameter<double>("max_ldd", 30.0);
    this->get_parameter("min_ldd", max_ldd_);
    this->declare_parameter<double>("kdd", 1.2);
    this->get_parameter("kdd", kdd_);

    pubSteering_ = this->create_publisher<can_zoe_msgs::msg::CmdSteering>("/cmd_steering", 100);
    get_path_client_ = this->create_client<zoe_waypoint_interfaces::srv::GetPath>("get_path");
    change_path_service_ = this->create_service<zoe_waypoint_interfaces::srv::ChangePath>("change_path",
                                                                                            std::bind(&FollowWaypoint::callback_change_path, 
                                                                                                        this,
                                                                                                        std::placeholders::_1,
                                                                                                        std::placeholders::_2));
    subGps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/novatel/oem7/fix", 10,
        std::bind(&FollowWaypoint::callback_gps, this, std::placeholders::_1)
    );
    subOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/novatel/oem7/odom", 10,
        std::bind(&FollowWaypoint::callback_odom, this, std::placeholders::_1) // Unused
    );
    subInspvax_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(
        "/novatel/oem7/inspva", 10,
        std::bind(&FollowWaypoint::callback_inspvax, this, std::placeholders::_1)
    );
    subSteer_ = this->create_subscription<can_zoe_msgs::msg::SteeringInfo>(
        "/can/steering_info", 10,
        std::bind(&FollowWaypoint::callback_steering, this, std::placeholders::_1)
    );
    subSpeed_ = this->create_subscription<can_zoe_msgs::msg::FrontWheelsSteering>(
        "/can/front_wheels_accel_brake", 10,
        std::bind(&FollowWaypoint::callback_speed, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Simulation topics
    pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 100);
    pubPoint_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/point", 100);
    pubFutPoint_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/future_point", 100);
    pubFutPose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/future_pose", 100);
    pubVolPose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/volant_pose", 100);


    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_MS), std::bind(&FollowWaypoint::timer_callback, this));
}

void FollowWaypoint::callback_change_path(const std::shared_ptr<zoe_waypoint_interfaces::srv::ChangePath::Request> request,
                                std::shared_ptr<zoe_waypoint_interfaces::srv::ChangePath::Response> response)
{
    path_name_ = request->pathname;
    RCLCPP_INFO(rclcpp::get_logger("zoe_follow_waypoint"), "Received path changing request: %s.", path_name_.c_str());
    try {
        get_path(path_name_.c_str());
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        response->path_changed = false;
        return;
    }
    response->path_changed = true;
}

void FollowWaypoint::get_path(const char* pathname)
{
    RCLCPP_INFO(rclcpp::get_logger("zoe_follow_waypoint"), "Waiting for map loader service...");
    get_path_client_->wait_for_service();
    auto request = std::make_shared<zoe_waypoint_interfaces::srv::GetPath::Request>();
    request->filename = pathname;

    get_path_client_->async_send_request(
    request,
    [this](rclcpp::Client<zoe_waypoint_interfaces::srv::GetPath>::SharedFuture future)
    {
        auto response = future.get();

        if (!response->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed request");
            return;
        }

        path_ = response->path;
        gps_start_ = response->gps_start;

        hasReceivedData_[PATH] = true;
        hasReceivedData_[GPS_START] = true;
        RCLCPP_INFO(rclcpp::get_logger("zoe_follow_waypoint"), "Path loaded successfully.");
    });
}

void FollowWaypoint::callback_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    hasReceivedData_[GPS] = true;
    gps_ = *msg;
}

void FollowWaypoint::callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_ = *msg;
    if (!hasReceivedData_[ODOMETRY]) {
        hasReceivedData_[ODOMETRY] = true;
    }
}

void FollowWaypoint::callback_inspvax(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg)
{
    inspva_ = *msg;
    tf2Scalar pitch(deg_to_rad(inspva_.pitch));
    tf2Scalar roll(deg_to_rad(inspva_.roll));
    tf2Scalar yaw(deg_to_rad(inspva_.azimuth));
    
    tf2::Quaternion tf2_quat, tf2_quat_from_msg;
    tf2_quat.setRPY(roll, pitch, yaw);
    // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    
    // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
    tf2::convert(msg_quat, tf2_quat_from_msg);
    
    geometry_msgs::msg::Point currentPosition = get_point_with_geodesic_constraints(this->gps_start_, this->gps_);
    map_H_baselink_.header.stamp = this->get_clock()->now();
    map_H_baselink_.header.frame_id = "map";
    map_H_baselink_.child_frame_id = "base_link";
    map_H_baselink_.transform.rotation.x = msg_quat.x;
    map_H_baselink_.transform.rotation.y = msg_quat.y;
    map_H_baselink_.transform.rotation.z = msg_quat.z;
    map_H_baselink_.transform.rotation.w = msg_quat.w;
    map_H_baselink_.transform.translation.x = currentPosition.x;
    map_H_baselink_.transform.translation.y = currentPosition.y;
    map_H_baselink_.transform.translation.z = currentPosition.z;
    tf_broadcaster_->sendTransform(map_H_baselink_);
    hasReceivedData_[INSPVAX] = true;
}

void FollowWaypoint::callback_steering(const can_zoe_msgs::msg::SteeringInfo::SharedPtr msg)
{
    steer_ = *msg;
}

void FollowWaypoint::callback_speed(const can_zoe_msgs::msg::FrontWheelsSteering::SharedPtr msg)
{
// avg_speed is not a "global" average, its the average between the two front wheels in the moment of the callback
    hasReceivedData_[SPEED] = true;   
    double vavg = msg->vit_roue_avg * M_PI * 2  * (0.615  / 2) / 60; //rad s
    double vavd = msg->vit_roue_avd * M_PI * 2  * (0.615  / 2) / 60; //rad s

    avg_speed_ = (vavg + vavd) / 2;
}

bool FollowWaypoint::closest_point_on_path(const geometry_msgs::msg::Point &point, size_t &pathIndex)
{
/**
 * @brief Finds the closest point on the loaded path
 *
 * @param point Current position of the base link frame
 * @param pathIndex Reference of the pathindex
 * @return True if a point below 10 meter distance was found.
 */
    bool valid_point = false;
    double minimal_distance = MINIMAL_DISTANCE;
    for (size_t i = pathIndex; i < path_.poses.size(); i++)
    {
        double distance = norm(path_.poses[i].pose.position.x - point.x, path_.poses[i].pose.position.y - point.y);
        if (distance < minimal_distance)
        {
            valid_point = true;
            minimal_distance = distance;
            pathIndex = i;
        }
    }
    return valid_point;
}


bool FollowWaypoint::has_received_all_required_data()
{
// Algorithm won't start if all required data hasn't been received
    if (this->allReceived)
        return true;
    for (int i = 0; i < MSG_COUNT; i++)
    {
        if (!this->hasReceivedData_[i])
        {
            RCLCPP_WARN(rclcpp::get_logger("zoe_follow_waypoint"), "Haven't received %d.", i);
            return false;
        }
    }
    this->allReceived = true;
    return this->allReceived;
}

bool  FollowWaypoint::is_solution_on_path(geometry_msgs::msg::Point const &A, 
                                            geometry_msgs::msg::Point const &vecAb, 
                                            geometry_msgs::msg::Point const &C)
{
/**
 * @brief Checks if solution is on the path segment
 *
 * @param A Current position of the base link frame
 * @param vecAb Vector path segment
 * @param C Solution
 * @return True if the point exists on AB.
 */
    geometry_msgs::msg::Point vecAc;
    double t = 0;

    vecAc.x = C.x - A.x;
    vecAc.y = C.y - A.y;
    t = dotProduct(vecAb, vecAc) / std::pow(norm(vecAb.x, vecAb.y), 2);
    if (t >= 0 && t <= 1)
        return true;
    return false;
}

geometry_msgs::msg::Point FollowWaypoint::compute_solution(geometry_msgs::msg::Point const &A, geometry_msgs::msg::Point const &B)
{
/**
 * @brief Compute a solution for the AB path Segment
 *
 * @param A First Point
 * @param B Second Point
 * @return The found solution
 * 
 * Finding the solution depends on crossing the lookahead distance and the looked at path point.
 * Function throws if no solution has been found for the given segment.
 */
    geometry_msgs::msg::Point C; //solution
    geometry_msgs::msg::Point vecAb;
    geometry_msgs::msg::Point vecAc;

    vecAb.x = B.x - A.x;
    vecAb.y = B.y - A.y;
    
    double m = vecAb.y / vecAb.x;
    double k = B.y - (m * B.x);

    double ldd = std::clamp(avg_speed_ * kdd_, min_ldd_, max_ldd_); // lookahead distance
    
    double a = (1 + std::pow(m, 2));
    double b = 2 * m * k;
    double c = std::pow(k, 2) - std::pow(ldd, 2);

    double delta = discriminant(a, b, c);
    if (delta < 0)
        throw std::runtime_error("No valid point found on path");
    if (delta == 0)
    {
        C.x = (-b) / (2 * a);
        C.y = linear_function(m, C.x, k);
        if (is_solution_on_path(A, vecAb, C))
            return C;
    }
    {
        C.x = (-b - std::sqrt(delta)) / (2 * a);
        C.y = linear_function(m, C.x, k);
        if (is_solution_on_path(A, vecAb, C))
            return C;
    }
    {
        C.x = (-b + std::sqrt(delta)) / (2 * a); 
        C.y = linear_function(m, C.x, k);
        if (is_solution_on_path(A, vecAb, C))
            return C;
    }
    throw std::runtime_error("No valid point found on path");
}

geometry_msgs::msg::PointStamped FollowWaypoint::find_path_point(size_t pathIdx)
{
/**
 * @brief Looks for a solution each path points
 *
 * @param pathIdx Ideally the index of the closest point on path because search will start from that point
 * @return The found point
 * 
 * The function transforms every path point to the base_link frame since solution depends on its distance from that frame.
 * Currently works only when car goes forward
 * Function throws if no solution has been found starting from pathIdx.
 */
    for (size_t i = pathIdx; i < path_.poses.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pathPoseBaselink = tf_buffer_->transform(path_.poses[i], "base_link");
        geometry_msgs::msg::PointStamped pathPointBaselink = toPointStamped(pathPoseBaselink);
        if (pathPoseBaselink.pose.position.x < 0)
        // Means position is behind the frame
            continue;
        geometry_msgs::msg::PoseStamped     posefut1 = tf_buffer_->transform(path_.poses[(i + 1) % path_.poses.size()], "base_link");
        geometry_msgs::msg::PointStamped    pointfut1 = toPointStamped(posefut1);
        try {
                geometry_msgs::msg::PointStamped    solution;
                solution.point = compute_solution(pathPointBaselink.point, pointfut1.point);
                solution.header.stamp = this->get_clock()->now();
                solution.header.frame_id = "base_link";
                if (simu_)
                    pubFutPoint_->publish(solution);
                return (solution);
            }
        catch (const std::exception & e) {}
    }
    throw std::runtime_error("No valid point found on path with given idx: " + std::to_string(pathIdx) + " for " + std::to_string(path_.poses.size()) + " points.");
}

void FollowWaypoint::simulate_angle(geometry_msgs::msg::PointStamped const &currentPointMap, const double wanted_angle)
{
    // Rviz Visualisation
    geometry_msgs::msg::PointStamped currentPointBaselink = tf_buffer_->transform(currentPointMap, "base_link");
    pubPoint_->publish(currentPointBaselink);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "front_axle";
    pose.pose.position.x = currentPointBaselink.point.x;
    pose.pose.position.y = currentPointBaselink.point.y;
    pose.pose.position.z = currentPointBaselink.point.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, deg_to_rad(wanted_angle / WHEEL_TO_STEER_RATIO));  // yaw only
    pose.pose.orientation = tf2::toMsg(q);
    pubFutPose_->publish(pose);
    q.setRPY(0, 0, deg_to_rad(wanted_angle));
    pose.pose.orientation = tf2::toMsg(q);
    pubVolPose_->publish(pose);
    
}

void FollowWaypoint::pure_pursuit_steering_angle_computation(can_zoe_msgs::msg::CmdSteering &cmd_steering)
{
/**
 * @brief Compute an angle based on pure pursuit
 *
 * @param cmd_steering The message that will be published
 * 
 * Finds a point on the path in range (lookahead distance), then computes a wheel angle.
 * Wheel angle is multiplied by a ratio to get the steering wheel angle.
 */
    constexpr const char* parent_frame = "map";
    constexpr const char* child_frame = "base_link";
    geometry_msgs::msg::PointStamped currentPointMap;
    currentPointMap.point.x = map_H_baselink_.transform.translation.x;
    currentPointMap.point.y = map_H_baselink_.transform.translation.y;
    currentPointMap.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
    currentPointMap.header.frame_id = parent_frame;

    if (!tf_buffer_->canTransform(child_frame, parent_frame, currentPointMap.header.stamp))
        throw std::runtime_error("Transform isn't published yet. ");
    static size_t pathIdx = 0;
    geometry_msgs::msg::PointStamped targetPointBaselink;
    if (!closest_point_on_path(currentPointMap.point, pathIdx))
        throw std::runtime_error("Too far from path. ");
    try {
        // Finding target point from lookahead distance
        targetPointBaselink = find_path_point(pathIdx);
    }
    catch (const std::exception & e)
    {
        pathIdx = 0;
        targetPointBaselink = find_path_point(0);
    }
    // Pure Pursuit
    double ld = norm(targetPointBaselink.point.x, targetPointBaselink.point.y);
    double alpha = atan2(targetPointBaselink.point.y, targetPointBaselink.point.x);
    double rad_angle = std::atan2(2 * WHEEL_BASE * std::sin(alpha), ld);
    double deg_angle = rad_to_deg(rad_angle * WHEEL_TO_STEER_RATIO);

    // Smoothing values
    static double last_angle = 0;
    deg_angle = exponential_smoothing(0.2, last_angle, deg_angle);
    cmd_steering.angle = -deg_angle;
    last_angle = deg_angle;

    // Simulation visualization
    if (!simu_)
        return;
    simulate_angle(currentPointMap, deg_angle);
}

void FollowWaypoint::timer_callback()
{
    if (!has_received_all_required_data())
        return;
    can_zoe_msgs::msg::CmdSteering cmd_steering;
    cmd_steering.rotation_speed = STEER_ROTATION_SPEED;
	cmd_steering.can_publish = true;
    try {
        pure_pursuit_steering_angle_computation(cmd_steering);
        pubSteering_->publish(cmd_steering);
        if (simu_)
        // Publishing path to visualize on rviz
            pubPath_->publish(path_);
    }
    catch (const std::exception & e) {
      RCLCPP_INFO(
        this->get_logger(), e.what());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        std::shared_ptr<FollowWaypoint> fw = std::make_shared<FollowWaypoint>();
        rclcpp::spin(fw);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}