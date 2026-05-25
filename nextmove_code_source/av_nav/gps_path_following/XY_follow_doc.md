# XYFollow - Lateral Path-Following Controller for Autonomous Vehicles

**Author:** Maxime Duquesne  
**Modified by:** Tom Billet (07/01/2025)  
**ROS DIST:** ROS2 HUMBLE
**Purpose:** Steering control for autonomous vehicle based on GPS, path, and INS orientation

---

## 📌 Overview

`XYFollow` is a ROS2 node that performs **lateral control** for a self-driving vehicle. It tracks a predefined path using GPS data and publishes steering angle commands to align the vehicle with the path while correcting for lateral deviations.

---

## 📦 Subscribed Topics

- `/path` (`nav_msgs/Path`)  
- `/gpsstart` (`sensor_msgs/NavSatFix`)  
- `/novatel/oem7/fix` (`sensor_msgs/NavSatFix`)  
- `/novatel/oem7/odom` (`nav_msgs/Odometry`)  
- `/novatel/oem7/inspva` (`novatel_oem7_msgs/INSPVA`)  
- `/can/steering` (`can_zoe_msgs/Steering`)

---

## 📤 Published Topic

- `/cmd_steering` (`can_zoe_msgs/CmdSteering`)

---

## 🧠 Core Functionalities

### 1. Coordinate Transformation
Transforms GPS data into local X/Y coordinates using the `geographiclib` library:
```
(x, y) = distance × [cos(azimuth), sin(azimuth)]
```

### 2. Closest Path Point
Finds the index `i` of the closest path point to the current vehicle position.

### 3. Path Heading
Estimates path heading using multiple future points:
```
angle = acos(X / sqrt(X² + Y²))
```

### 4. Vehicle Orientation
Extracts vehicle yaw (`capz`) from either:
- Odometry quaternion
- INS azimuth (`INSPVA.azimuth`)

### 5. Lateral Deviation Direction
Uses a 2D cross product (vehicle-to-path vs. path direction) to determine the sign of lateral error.

### 6. Steering Control Law
Combines lateral error and heading error:
```
steering_angle = (P_angle × heading_error) + (P_dist_angle × lateral_error)
```
Output is saturated by:
- `max_dist_angle`
- `max_cap_angle`
- `max_diff_angle` (rate limiter)

---

## 🔧 Tunable Parameters

| Parameter               | Description                               | Default |
|------------------------|-------------------------------------------|---------|
| `topic_path`           | Path topic name                           | `/path` |
| `pas`                  | Path resolution in meters                 | `0.2`   |
| `dist_gps_to_wheel`    | Distance from GPS to front axle           | `5.0`   |
| `P_dist_angle`         | Gain for lateral error                    | `15`    |
| `P_angle`              | Gain for heading error                    | `10`    |
| `max_dist_angle`       | Max contribution of lateral error         | `90`    |
| `max_cap_angle`        | Max contribution of heading error         | `300`   |
| `max_diff_angle`       | Max rate of change for steering command   | `300`   |

---

## 🧾 Equation Summary

### Path Offset:
```python
X, Y = calc_goal(gps_start, gps)
```

### Lateral Deviation:
```python
Vpv = (x_i - x) * Yvp - (y_i - y) * Xvp
```

### Steering Control:
```python
dist_angle = dist * sign(Vpv) * P_dist_angle
cap_angle = heading_error * P_angle
steering_angle = cap_angle + dist_angle
```

---

## 🏁 Launching the Node

Make sure your ROS2 workspace is built and sourced:

```bash
ros2 run zoe_gps_waypoint XY_follow.py
```

---

## 🛠 Dependencies

- `rclpy` (ROS2 Python Client Library)
- `geographiclib`
- `numpy`
- Custom message types:
  - `can_zoe_msgs/CmdSteering`
  - `can_zoe_msgs/Steering`
  - `novatel_oem7_msgs/INSPVA`