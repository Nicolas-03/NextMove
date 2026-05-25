# 📃 Wiki for the package av_path_speed_analyzer
## 📋 Table of content
- [📃 Wiki for the package av\_path\_speed\_analyzer](#-wiki-for-the-package-av_path_speed_analyzer)
  - [📋 Table of content](#-table-of-content)
  - [💬 Introduction](#-introduction)
    - [Overview](#overview)
    - [Meta-Data](#meta-data)
  - [⚙️ Installation](#️-installation)
    - [Dependencies](#dependencies)
    - [Package Installation](#package-installation)
  - [📚 Package Description](#-package-description)
    - [Description](#description)
      - [speed\_analyzer.py](#speed_analyzerpy)
        - [📨 Speed Analyzer : Subscriptions Topics \& Published Topics](#-speed-analyzer--subscriptions-topics--published-topics)
      - [profile\_generator.py](#profile_generatorpy)
      - [obstacle\_detect.py](#obstacle_detectpy)
        - [📨 Obs Detect : Subscriptions Topics \& Published Topics](#-obs-detect--subscriptions-topics--published-topics)
    - [🧠 Methods](#-methods)
      - [1️⃣ Method 1 : Via Speed Profile](#1️⃣-method-1--via-speed-profile)
      - [2️⃣ Method 2 : Via Path Angle](#2️⃣-method-2--via-path-angle)
      - [3️⃣ Method 3 : Via Steering Wheel Angle](#3️⃣-method-3--via-steering-wheel-angle)
      - [4️⃣ Method 4 : Via Dynamic Pure Pursuit (without Steering angle calculation)](#4️⃣-method-4--via-dynamic-pure-pursuit-without-steering-angle-calculation)
  - [🏁 Use the package](#-use-the-package)
  - [🚧 Known Issues](#-known-issues)
  - [✅ To Do List](#-to-do-list)

## 💬 Introduction
### Overview
av_path_speed_analyzer is a ROS2 package used for calculating the speed required by a vehicle which follows a predefined trajectory.
Speed is only determined by the trajectory shape, the package doesn't apply road signalisation or adapt to road features such as speed bumps.
This package includes differents methods for speed calculation, this methods are developed in [Methods](#methods) section
### Meta-Data
*Creator* : MALET Pierre, Student @IG2I - Centrale Lille Institut, Class 2028  
*Project* : Autnomous Vehicule Project with Renault Zoé of the @PRETIL from the @CRIStAL laboratory  
*Package Version* : 1.0.0
## ⚙️ Installation
### Dependencies
This package has the following depedencies :  
<font color="blue">a blue name means its a ROS pacakge</font>
<font color="red">a red name means its a python package</font>
- **<font color="blue">ROS2 Humble</font>**  
To develop this package, we use ROS2 Humble Hawksbill. You can find a guide to installing ROS2 Humble in the ROS2 Documentation for Humble ([here](https://docs.ros.org/en/humble/Installation.html))
- **<font color="blue">av_path_speed_analyzer_interface</font>**  
av_path_speed_analyzer_interface contains the differents interfaces used by the package av_path_speed_analyzer.  
av_path_speed_analyzer_interface is available [here](https://gitlab.cristal.univ-lille.fr/pretil/zoe/av_nav/-/tree/humble/av_path_speed_analyzer_interface?ref_type=heads).  
These are the interfaces availables : 1 service named SpeedProfileGen  
*For futher informations on av_path_speed_analyzer_interface, refer to its documentation*
- **<font color="blue">novatel_oem7_driver</font>**  
To calculate the speed, we need the GPS position to find where is the vehicule on the trajectory.
To do this, we use a GNSS Recevier with the ROS Package named novatel_oem7_driver.  
Novatel Package for ROS Humble and its documentation are available [here](https://github.com/novatel/novatel_oem7_driver/tree/humble).
- **<font color="blue">can_zoe_msgs</font>**  
This package contains CAN messages. This package is available [here](https://gitlab.cristal.univ-lille.fr/pretil/zoe/zoe1/code/ros2/can/can_zoe_msgs).
We use to obtain the steering angle that is used in one of methods developed and the Speed of the vehicule.
- **<font color="red">geographiclib</font>**  
This is a **python** package used in one of the node of the package. It implements the geodosics routines.  
To install, see the documentation available [here](https://geographiclib.sourceforge.io/html/python/package.html#installation).

### Package Installation
Once all the dependencies have been installed, you can now add this package to your ROS workspace.  
To do this, you copy/paste this repository into the src repository of your workspace.

## 📚 Package Description
### Description
This package is ROS2 package developed for the autonomous vehicule project made at @CRIStAL laboratory.  
In this package, there is 3 python files corresponding to 3 nodes :
#### speed_analyzer.py  
**<font color="green">As of 07/16/2025, this node was tested and is fully functional</font>**  
This file corresponds to the following node :
```.sh
$ros2 run av_path_speed_analyzer speed_analyzer
```
In this file, multiple methods have been developed and tested. The methods are developed in [Methods](#methods) section
This node have 4 parameters :
```.yaml
    maxSpeed: 30 #maximum Speed (km/h) allowed. Default : 30
    minSpeed: 7  #minimun Speed (km/h) allowed. Default : 7
    maxGForce: 10.0 #max G Force(m/s^2) allwoed. Default : 10.0
    methodSelect: 1 #ID of the method choosen for calculating speed. Default : 1
```
Method IDs are given in Method Section
This node is a client for the service SpeedProfileGen, this service is used to generate a speed profile. This speed Profile is used in one the method develop.
In the node, we do the following algorithm :
- Step 1 : Generation of the Speed Profile
- Step 2 : Get the current GPS position
- Step 3 : Calculate the speed according to the method choosen
- Step 4 : Send the speed to the node speed_controller from can_zoe_decoder package (https://gitlab.cristal.univ-lille.fr/pretil/zoe/zoe1/code/ros2/can/can_zoe_decoder)
##### 📨 Speed Analyzer : Subscriptions Topics & Published Topics
This node has the following subscriptions :
- /path : a Topic containning a Path Message (from nav_msgs package), this message contains the trajectory the vehicule follows
- /gpsstart : a Topic containning a NavSatFix Message from (from sensor_msgs package), this message contains a GPS position
- /novatel/oem7/fix : a Topic containning a NavSatFix Message (from nav_msgs package ), this message contains vehicule's current GPS position
- /novatel/oem7/odom : a Topic containning a Odometry Message (from nav_msgs package), this message contains vehicule's odometry
- /cmd_steering : a Topic containning a CmdSteering Message (from can_zoe_msgs package), this message contains the steering wheels angle which is used in one on the method developped.

The node publishes on the following topic :
- /cmd_vel : a Topic containning a Twist Message (from Geometry_msgs package), this message contains a linear and angular speed in m/s

#### profile_generator.py
**<font color="green">As of 07/16/2025, this node was tested and is fully functional</font>**  
This file corresponds to the following node :
```.sh
$ros2 run av_path_speed_analyzer genProfile
```
This node is the server for the service SpeedGenProfile (named speed_profile_gen).  
This node generated a speed Profile for the method developped in the speed_analyzer.py  
To generate the profile, we loop through every point of the trajectory. For each point, we do a second loop that do the following range : from the point to the point+maxLoop (maxLoop is determined by distance between two points of the trajectory). During this second loop, we calculate the local curvature between 3 points. The local curvature is calculated by doing the following calculations, first we calculate the area of the triangle composed by the 3 choosen points : $$area =\frac{1}{2} *( x_0 * (y_1-y_2) + x_1 * (y2-y_0) + x_2*(y_0 -y_1))$$
with xi,yi corresponding to the coords of each point. Point 0 corresponds to the current point of the loop, Point 1 = current point+1, Point 2 = current point +2
Then we calculate the local curvature with the following formula : $$\kappa = \frac{4*area}{d_{01}*d_{12}*d_{20}}$$ with dXY corresponding to the distance between point X and Point Y
Then, we calculate the curvature $\kappa$ by summing all the local curvature found.
Finally, we choose between the minimu between the maximum Speed and $\sqrt{\frac{maxGForce}{\kappa}}$ 

#### obstacle_detect.py
**<font color="red">As of 07/16/2025, this node has never been tested, it is highly likely to change in the future or to be reworked</font>**  
This file correspond to the following node :
```.sh
$ros2 run av_path_speed_analyzer obs_detect
```
This node is make the vehicule stop if an obstacle is too close to the vehicule.
We consider an obstacle too close if the distance between the vehicule and the vehicule is inferior to the safety distance. The safety distance is determined by application of the french road laws : "The distance between two vehicule must be of 2 seconds minimum" so we found the safety distance by doing : $$dist_{safety} = \frac{v_{current}}{3.6}$$
##### 📨 Obs Detect : Subscriptions Topics & Published Topics
This node has the following subscription :
- /path : a Topic containning a Path Message (from nav_msgs package), this message contains the trajectory the vehicule follows
- /gpsstart : a Topic containning a NavSatFix Message from (from sensor_msgs package), this message contains a GPS position
- /novatel/oem7/odom : a Topic containning a Odometry Message (from nav_msgs package), this message contains vehicule's odometry
- /can/speed : a Topic containning a speed Message (from can_zoe_msgs package), this message contains vehicule's current Speed
- /yolo_3d_result : a Topic containning Detection3DArray Message (from vision_msgs package), this message contains an array with the differents elements detected by the LIDAR+camera

This node publishes on the following topic:
- /cmd_vel : a Topic containning a Twist Message (from Geometry_msgs package), this message contains a linear and angular speed in m/s

### 🧠 Methods
In the node speed_analyzer, we have developped several methods for speed calculations :
#### 1️⃣ Method 1 : Via Speed Profile
The first method developed is based on the speed profile. In this method, we find what is the nearest point of the speed profile (named goal Speed)  relatively to vehicule's current GPS position.
Once we get the goal speed, the new speed is determined based on wether the goal Speed is lower or superior.  
If the goal speed is superior, we choose the minimun Speed between the maximum speed and a new speed determines with the following calculation : $$speed_{new} = speed_{current} + \frac{30}{100}*speed_{Max}$$
If the goal speed is inferior, we choose the maximun speed between the minimu speed and a new speed determines with the following speed : $$speed_{new} = speed_{current} + \frac{50}{100}*speed_{Min}$$

#### 2️⃣ Method 2 : Via Path Angle
This second method finds the speed needed by finding trajectory's angle in the next 7 meters.
To find the trajecory's angle, we do the follwing algorithm :
1) Select 3 points A,B and C. With A corresponding to the point near the current GPS position, B the point following A and C the point following C
2) Calculate the vectors $\vec{AB}$ and $\vec{BC}$ and their norms
3) Calculate the cosinus $\theta$ of the trajectory angle : $$\theta = \frac{\vec{AB}\cdot\vec{BC}}{||\vec{AB}||.||\vec{BC}||}$$
4) Find the trajectory's angle $\alpha$ : $\alpha = \arccos{\theta}$
We repeat this algorithm on 7 meters following the current GPS position (i.e we do loop with a range that goes from the nearest position of the current GPS position in the trajectory to the point in the trajectory that is around 7 meters futher). Each time, we find a new angle with sum it with the others found.  

Once we have find all angles, we calculate the speed with this formula : $$speed = -\frac{speed_{Max}}{90} * \alpha_{total} + speed_{Max}$$

#### 3️⃣ Method 3 : Via Steering Wheel Angle
This third method finds the speed needed by calculating the speed relatively to the steering wing's angle.
To do this, we obtain the steering wheel's angle with the topic "/cmd_steering". Once we have the steering wheel's angle, we calculate the speed with the following formule : $$speed = -\frac{speed_{Max}}{540}* |\alpha_{steering}| + speed_{Max}$$
we divide by 540 because 540 correspond to the maximun steering angle possible.

#### 4️⃣ Method 4 : Via Dynamic Pure Pursuit (without Steering angle calculation)
This fourth method is based on the pure pursuit algorithm. In this method, we do the following algorithm :
1) Calculate the local curvature $\kappa_{local}$ at the current GPS position (via the nearest point in trajectory). To calculate the curvature, we use the same formulas used in the speed Profile Generation
2) Filter the curvature : $$\kappa = \alpha_{C} * \kappa_{local} + (1-\alpha_{C}) * \kappa_{last}$$
with $\kappa_{last}$ corresponding to the curvature of the last position and $\alpha_{C}$ equals 0.1
3) Then we calculate a raw speed : $$speed_{Raw} = K_v / {\kappa + 0.1}$$ with $k_v$ equal 0.36   
Then a target speed : $$speed_{Target} = max(speed_{Min}, min(speed_{Max}, speed_{Raw}))$$
3) Calculate an alpha for filtering the speed : $$\alpha_{Speed} = min(0.2 , max (0.01, 1 -\exp{-5*|\kappa - \kappa_{last}|}))$$
Then filter speed : $$speed_{Target} = \alpha_{Speed} * speed_{Target} + (1-\alpha_{Speed})*speed_{last}$$

## 🏁 Use the package
After you build the package with colcon, you can use this package after sourcing your environnement.
You can launch the node speed_analyzer with the following command :
```.sh
$ros2 run av_path_speed_analyzer speed_analyzer
```
With this command, the node launches with its default values.
You can also do : 
```.sh
$ros2 run av_path_speed_analyzer speed_analyzer --ros-args --params-file src/av_nav/av_path_speed_analyzer/paramSpeedAnalyzer.yaml
```
With this command, you can run the node with differents parameters. The paramaters use will be the one in the paramSpeedAnalyzer.yaml file. Edit paramSpeedAnalyzer.yaml file to use the node with differents parameters.

The node speed_analyzer is using a service, so the node genProfile is also needed to make speed_analyzer runs.  
To launch genProfile do :
```.sh
$ros2 run av_path_speed_analyzer genProfile
```

## 🚧 Known Issues
Sometimes when running the genProfile node, you may have an error indicating there is no executable.
To do avoid thisw error, launch the node by using python (at the root of your worksapce) :
```.sh
$/bin/python3 src/av_nav/av_path_speed_analyzer/av_path_speed_analyzer/profile_generator.py
```
## ✅ To Do List
- Rework Obs_Dectect Node

