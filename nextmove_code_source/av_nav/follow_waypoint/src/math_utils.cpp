#include "math_utils.hpp"

#include <iostream> //< REMOVE

geometry_msgs::msg::Point convert_polar_to_cartesian(double r, double theta)
{
    geometry_msgs::msg::Point p;
    p.x = std::cos(theta) * r;
    p.y = std::sin(theta) * r;
    p.z = 0;
    return p;
}

geometry_msgs::msg::Point get_point_with_geodesic_constraints(
        sensor_msgs::msg::NavSatFix origin,
        sensor_msgs::msg::NavSatFix goal)
{
    double s12 = 0; // distance
    double azi1 = 0; // azimuth first in degrees
    double azi2 = 0;
    const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(origin.latitude, origin.longitude, goal.latitude, goal.longitude, s12, azi1, azi2);
    azi1 *=  M_PI / 180.0;
    return convert_polar_to_cartesian(s12, azi1);
}

double normalize_angle(double angle)
{
    angle = std::fmod(angle + 180.0, 360.0);
    if (angle < 0)
        angle += 360.0;
    return angle - 180.0;
}