#pragma once
#include <cmath>
#include <GeographicLib/Geodesic.hpp>

#include "geometry_msgs/msg/point.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

geometry_msgs::msg::Point convert_polar_to_cartesian(double r, double theta);
geometry_msgs::msg::Point get_point_with_geodesic_constraints(
        sensor_msgs::msg::NavSatFix origin,
        sensor_msgs::msg::NavSatFix goal);

inline double deg_to_rad(double deg)
{
    return deg * M_PI / 180;
}

inline double rad_to_deg(double rad)
{
    return rad * 180 / M_PI;
}

double normalize_angle(double angle);

inline double linear_function(double m, double x, double b)
{
    return m * x + b;
}

inline double discriminant(double a, double b, double c)
{
    return (std::pow(b, 2) - 4 * a * c);
}
inline double norm(double x, double y)
{
    return std::sqrt(x * x + y * y);
}

inline double dotProduct(const geometry_msgs::msg::Point &u,
                  const geometry_msgs::msg::Point &v)
{
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

inline double exponential_smoothing(const double alpha, const double s0, const double s1)
{
    return alpha * s1 + ((1 - alpha) * s0);
}
