#####
# @Title : obstacle_detect.py
# @Author : MALET Pierre @IG2I - Centrale Lille Insitut, Class 2028
# @Date : V1 | Begin : 03 June 2025 - End : Unknow
# @Project : Autnomous Vehcule Project with Renault Zoé @PRETIL from of the @CRIStAL laboratory
# @Description :  This ROS Node analyzes the message from the topic created by the liddar and slows down
#                  the vehicule if an obstacle is in range
#####

# ------------ Imports  ------------ #
import math
import numpy as np

import rclpy
import rclpy.duration
from rclpy.node import Node

import rclpy.time
from vision_msgs.msg import Detection3DArray
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist,PoseStamped
from can_zoe_msgs.msg import Speed
from geographiclib.geodesic import Geodesic
import tf2_ros
import tf2_geometry_msgs

# -------------- Functions --------------- #
def quaternion_from_euler(roll, pitch, yaw):
	"""
	Convert an Euler angle to a quaternion.

	Input
	:param roll: The roll (rotation around x-axis) angle in radians.
	:param pitch: The pitch (rotation around y-axis) angle in radians.
	:param yaw: The yaw (rotation around z-axis) angle in radians.

	Output
	:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
	"""
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]

# -------------- Node -------------- #
class ObsDetector (Node):
    """
    ObsDetector is ROS node. It detects if there is an obstacle on the path of the car
    """
    path = Path()
    detect = Detection3DArray()
    gpsStart = NavSatFix()
    gps = NavSatFix()
    crtSpeed = Speed()
    
    dist_e = 3 #Epsilon for the half width of the road (in France around road witdh around 6m)

    ## Constructor
    def __init__ (self):
        """
        Constructor of a ObsDetector Object, child class of the Node class from rclpy
        """
        super().__init__("obs_detector_node")
        # Sub Flag - Indicate if data were recieved
        self.pathOn = False
        self.detectOn = False
        self.gpsStartOn = False
        self.gpsOn = False
        self.crtSpeedOn = False
        #Subscriptions
        self.sub_path = self.create_subscription(Path,"/path", self.path_cb, 10)
        self.sub_gpsStart = self.create_subscription(NavSatFix,"/gpsstart", self.gpsStart_cb, 10)
        self.sub_gps = self.create_subscription(NavSatFix,"/novatel/oem7/fix",self.gps_cb,10)
        self.sub_crtSpeed = self.create_subscription(Speed, "/can/speed",self.crtSpeed_cb,100)
        self.sub_detect = self.create_subscription(Detection3DArray,"/yolo_3d_result", self.detect_cb, 10) # /!\ Topic Name could alson be /yolo_3d_result_topic
        #Publisher
        self.pub_speed = self.create_publisher(Twist, "/cmd_vel",10)
        #Tf2 buffer for conversion
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)

        self.get_logger().info("ObstacleDectetor : Node initialized")
        #Timer
        timer_per = 0.01 # seconds
        self.timer = self.create_timer(timer_per,self.main)
    
    ##Callback
    def path_cb (self, data):
        """
		Obtain data from the path topic et save them in a attribut named path
		@param : data : data contains in the Path Message sent on the topic /path
		@return : None
		"""
        self.pathOn = True
        self.path = data
    
    def gpsStart_cb (self, data):
        """
		Obtain data from the gps start topic et save them in a attribut named gpsStart
		@param : data : data contains in the NavSatFix Message sent on the topic /gpsstart
		@return : None
		""" 
        self.gpsStartOn = True
        self.gpsStart = data
    
    def gps_cb (self, data):
        """
		Obtain data from the gps topic et save them in a attribut named gps
		@param : data : data contains in the NavSatFix Message sent on the topic /novatel/oem7/fix
		@return : None
		"""
        self.gpsOn = True
        self.gps = data
    def crtSpeed_cb (self, data):
        """
		Obtain data from the currentSpeed topic et save them in a attribut named crtSpeed
		@param : data : data contains in a Speed Message sent on the topic /cmd/speed
		@return : None
		"""
        self.crtSpeedOn = True
        self.crtSpeed = data
    
    def detect_cb (self, data):
        """
		Obtain data from the detection et save them in a attribut named detect
		@param : data : data contains in a Detection3DArray sent on the topic /yolo_3d_result
		@return : None
		"""
        self.detectOn = True
        self.detect = data
    
    ##Method
    def convertDetectIntoPath (self, detection : Detection3DArray , path : Path, buffer : tf2_ros.Buffer):
        """
        Convert positions of the elements detected into the path reference
        @params :
            - detection : Detion3DArray, data containnning the element detected by the lidar+camera
            - path : Path, a path Message that contain an array with the pose of each point belonging to the trajectory
            - buffer : tf2_ros.Buffer, a tf2 Buffer, used to do the tranformation into the same reference
        @return :
            - detectConv : array of PoseStamped, an array that contains the position of each element detected  into the path reference
		"""
        path_frame = path.header.frame_id
        detectConv = []
        for detec in detection.detections :
            for hyp in detec.results :
                poseDetec = PoseStamped()
                poseDetec.header = detec.header
                poseDetec.pose = hyp.pose.pose # Maybe juste get hyp.pose to keep the orientation
                try :
                    transform = buffer.lookup_transform (
                        path_frame,
                        poseDetec.header.frame_id,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    poseDetecConv = tf2_geometry_msgs.do_transform_pose(poseDetec, transform)
                    detectConv.append(poseDetecConv)
                except Exception as e:
                    self.get_logger().info("### Failed to convert to into Path reference")
        return detectConv
    
    def getCurrentPos(self, gps:NavSatFix, gpsStart:NavSatFix):
        """
		Obtain the current GPS postion of the vehicule and convert it into the XY reference used
		when genereating the path
		@param : 
			-gps : NavSatFix, Message containing the GPS position
			-gpsStart : NavSatFix, Message containning the GPS position used to generated the path
		@return :
			- x : float, X coord of the vehicule in the XY reference of the path
			- y : float, Y coord of the vehicule in the XY reference of the path
		"""
		# Transform GPS localization into a a cartesian reference based on gpsStart which was used to create to create to the ref map
        ref = Geodesic.WGS84.Inverse(gpsStart.latitude, gpsStart.longitude, gps.latitude, gps.longitude) 
        dist = ref["s12"]
        azi = math.radians(ref["azi1"])
        x = dist * math.sin(azi)
        y = dist * math.cos(azi)
        return x,y
    
    def calculateDistBetween (self, point, x, y):
        """
        Calculate the euclidean distance between the vehicule and the position of an element detected by the liddar
        @params :
            - point : Pose, coordinates of the element detected
            - x : float, X coord of the car
            - y : float, Y coord of the car
        @return :
            dist : float, euclidean distance between the point and the vehicule
        """
        return math.sqrt(
            math.pow(point.x - x,2) + math.pow(point.y - y,2)
        )
    
    ## Main Method
    def main (self):
        """
        Main method of the ObsDetector Node.
        This node stops the vehicule if there is a obstacle on the path of the vehicule
        and if the obstacle is too close from the vehicule
        To this, the node does the following algorithm :
        1 - Convert the position of the elements detected in the path reference
        2 - Calculate the GPS position of the vehicule in the path reference
        3 - Calulate the safety distance needed
        4 - Save which points are on the road and which are not (for example parked car, pedestrian on the sidewalk)
        5 - Checks if one of the points saved is too close from the vehicule
        6 - Stop the vehicule if a obstacle has been detected
        """
        #Wainting for all topic to be enable
        while (not(self.pathOn and self.gpsStartOn and self.detect and self.crtSpeedOn and self.gpsOn)):
            self.get_logger().info("ObstacleDectetor : Has not recieved all the data needed")
        # Convert Detection Point into the same reference as the path
        detectConv = self.convertDetectIntoPath (self.detect, self.path, self.tf_buffer)
        self.get_logger().info("ObstacleDectetor : Dection Positions converted")
        #Obtain Current GPS position
        gps_x, gps_y = self.getCurrentPos(self.gps,self.gpsStart)
        self.get_logger().info("ObstacleDectetor : Current Position obtained")
        #Calculate security distance
        safeDist = (self.crtSpeed.speed/3.6)*2
        self.get_logger().info("ObstacleDectetor : Safety Distance Found: {0}".format(safeDist))
        #Reject points that are not on the path (ex : parked cars, pedestrian on sidewalks ..)
        #Maybe, instead of using a new list, remove point from the list with detected point?
        obstOnPath = []
        for p in detectConv:
            if not( ((p.x == gps_x) and (p.y<gps_y-self.dist_e or p.y>gps_y+self.dist_e))
                or ((p.y == gps_y) and (p.x<gps_x-self.dist_e or p.x>gps_x+self.dist_e))) :
                obstOnPath.append(p)
        self.get_logger().info("ObstacleDectetor : Rejected elements ")
        # Check if we need to stop
        obsNear = 0 # flag
        for i in obstOnPath:
            distBetween = self.calculateDistBetween (i,gps_x,gps_y)
            if (distBetween < safeDist):
                obsNear = 1 # raise flag
        self.get_logger().info("ObstacleDectetor : Check elements on path")
        speedMsg = Twist
        speedMsg.linear.x = 0.0 if (obsNear) else -1.0
        speedMsg.linear.y = 0.0
        speedMsg.linear.z = 0.0
        speedMsg.angular.x = 0.0
        speedMsg.angular.y = 0.0
        speedMsg.angular.z = 0.0
        self.get_logger().info("ObstacleDectetor : Speed Message Created") 
        self.pub_speed.publish(speedMsg)

# -------------- Main  -------------- #
def main ():
    rclpy.init()
    obsDetec = ObsDetector()
    rclpy.spin(obsDetec)
    obsDetec.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

