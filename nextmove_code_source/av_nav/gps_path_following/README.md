# package av_nav

Code for ROS2 Humble

"Autonomous Vehicle Navigation" ROS package.

This repository contains the codes for the autonomous driving with the vehicles of PRETIL platform.  

Checkout the branch corresponding to your ROS version.  

Available:  
- ROS1 noetic
- ROS2 foxy
- ROS2 humble

## Package zoe_gps_waypoint

## build ok but not tested

GPS path follower for the Zoe

## gps_to_XY_point

Make a path with Navfix gps data, and make a map with point separate with the same distance.

## use

``ros2 run zoe_gps_waypoint gps_to_XY_point.py ``

When ready press enter to start the acquisition, and press enter to stop it.

Follow the message to save or show the map.

## Map Loader

``ros2 run zoe_gps_waypoint waypoint_map_loader``
``ros2 service call /load_map_from_pathfile zoe_waypoint_interfaces/srv/LoadMap "{absolutepaths: [test1, test2]}"``

Load the map file create by gps_to_XY_point and creat two topic :

-/path      path to follow (Type Path)
-/gpsstart  starting gps point of the map (Navsatfix)

## XY_follow

``ros2 run zoe_gps_waypoint XY_follow.py``

Follow the path using gps data and a cap.

subscribes to :
-/path                Path
-/gpsstart            NavSatFix
-gps_novatel/fix      NavSatFix
-/novatel/oem7/odom   Odometry

publishes :
-/cmd_steering [[can_zoe_msgs::CmdSteering]](https://gitlab.cristal.univ-lille.fr/pretil/zoe/can_zoe/blob/master/can_zoe_msgs/msg/CmdSteering.msg)

Documentation can be found on <a href="/gps_path_following/XY_follow_doc.md">XY_follow_doc.md</a>.

# TODO
- XY_follow : ? take cap from a topic ?
>>>>>>> source-main
