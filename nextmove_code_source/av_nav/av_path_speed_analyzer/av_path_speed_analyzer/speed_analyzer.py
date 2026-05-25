#####
# @Title : speed_analyzer
# @Author : MALET Pierre @IG2I - Centrale Lille Insitut, Class 2028
# @Date : V1 | Begin : 26 May 2025 - End : 02 June 2025
#               -> impletments speed calculation via path angle
#		  V2 | Begin : 10 June 2025 - End : 11 June 2025
#               -> implements speed calculation via curvature of the road
#		  V3 | Begin : 17 June 2025 - End : 18 June 2025
#               -> implements speed calculation via cmd steering
#         V4 | Begin : 30 June 2025 - End : 04 July 2025
#               -> implements speed calculation via speed profile
#               -> implements all the methods used in the others versions
#               -> adds maxSpeed, maxGForce, method used as parameters
# @Project : Autnomous Vehcule Project with Renault Zoé of the @PRETIL from the @CRIStAL laboratory
# @Description :  This ROS Node analyzes the path followed by the car to deduce the speed at it must move
#####

# --- Libraries Imports --- #
import os
import math
import rclpy
from rclpy.node import Node
from av_path_speed_analyzer_interface.srv import SpeedProfileGen
from can_zoe_msgs.msg import CmdSteering
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from geographiclib.geodesic import Geodesic

# --- SpeedAnalyzer Node --- #
class SpeedAnalyzer (Node):
    #Variable
    path = Path() #path message containnning the path followed by the vehicule
    gpsStart = NavSatFix() #NavSatFix message, containning the position of GPS start
    gps = NavSatFix() #NavSatFix message, containning the current GPS position 
    odom = Odometry() #Odometry message, containning the odometry of the vehicule
    cmdSteering = CmdSteering()
    lastSpeed = 0 #m/s, Last speed of the vehicule
    distPoint = 0.2 #meter,distance between two point of the path (default 0.2 m)

    ## Variables used for the Dynamic Pure pursuit
    lastKappa = 0
    #Constructor
    def __init__ (self):
        """
        Constructor of SpeedAnalyzer Node, a child class of Node
        This node contains differents methods to find the most suitable speed for the path followed by the vehicule
        This node has subscirbed to the following topics : 
            - /path, for trajectory
            - /gpsstart, for the gps start postions
            - /novatel/oem7/fix, for the current gps position
            - /novatel/oem7/odom, for the odometry of the vehicule
            - /cmd_steering, to get the sttering angle
        This node is also a client side of the service "speed_profile_gen"
        This node published on the topic "/cmd_vel"
        """
        super().__init__("speed_analyzer")
        #Service Client
        self.genProfileClient = self.create_client(SpeedProfileGen, "speed_profile_gen")
        while not (self.genProfileClient.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info("Unable to reach Speed Profile Generator Service")
        self.get_logger().info("Found Speed Profile Generator Service")
        #Parameters
        self.declare_parameter("maxSpeed", 30)
        self.declare_parameter("minSpeed",7)
        self.declare_parameter("maxGForce", 10.0)
        self.declare_parameter("methodSelect", 1)
        #Subscriptions
        self.subPath = self.create_subscription(Path,"/path", self.path_cb,10)
        self.subGpsStart = self.create_subscription(NavSatFix,"/gpsstart", self.gpsStart_cb, 10)
        self.subGps = self.create_subscription(NavSatFix,"/novatel/oem7/fix", self.gps_cb, 10)
        self.subOdom = self.create_subscription(Odometry,"/novatel/oem7/odom",self.odom_cb, 10)
        self.subCmdSteering = self.create_subscription(CmdSteering,"/cmd_steering",self.cmdSteer_cb,10)
        #Publisher
        self.pubCmdVel = self.create_publisher(Twist,"/cmd_vel",10)
        #Flag
        self.subPathFlag = False
        self.subGpsStartFlag = False
        self.subGpsFlag = False
        self.subOdomFlag = False
        self.subCmdSteeringFlag = False
        self.srvGenPathFlag = False

        #Attributs
        self.profile = []
        self.profileStep = 1

        #Timer
        self.get_logger().info("Speed Analyzer Node Initialized")
        self.timer = self.create_timer(0.01, self.mainLoop)
    
    #Methods
    def getEuclideanDistance (self,x1,y1,x2,y2):
        """
        Calculate the Euclidean Distance between two points
        @params :
            - x1 : float, X coord of the 1st point
            - y1 : float, Y cood of the 1st point
            - x2 : float, X coord of the 2nd point
            - y2 : float, Y coord of the 2nd point
        @returns :
            - float, distance between two points
        """
        return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))
    
    def setOffset(self, origin : NavSatFix, current : NavSatFix):
        """
        Convert current GPS position into the local reference uof the path 
        @params :
            - origin : NavSatFix, original position used to generated path
            - current : NavSatFix, current position
        @returns :
            - x : float, position into the local reference
            - y : float, position into the local reference
        """
        geod = Geodesic.WGS84
        g = geod.Inverse ( float (origin.latitude) , float(origin.longitude),
                        float(current.latitude), float(current.longitude))
        hyp = g["s12"]
        azi = g["azi1"]
        azi = math.radians(azi)
        x = math.cos(azi)*hyp
        y = math.sin(azi)*hyp
        return x,y
    
    def getNeareastPoint(self, path : Path, x, y):
        """
        Find the nearest point in the path array to the current position in the local reference
        To find the nearest point, it loops through each points of the path and calculate the distance between the point and the current position
        If the distance is inferior to the current minimal distance, it set the new minimal distance and the save the index of the point ofunnd
        @params :
            -x : float, X coord of the current position
            -y : float, Y coord of the current position
        @returns :
            - i : int, index of the nearest point
            - dist : float, distance between the nearest point and the current positions
        """
        i= -1
        minDist = 20000 #20km
        for p in range(0, len(path.poses)):
            dist = self.getEuclideanDistance(path.poses[p].pose.position.x, path.poses[p].pose.position.y,
                                            x,y)
            if (dist<minDist):
                minDist = dist
                i=p
        return i,dist
    
    #Subscritpion Callback
    def path_cb (self, data):
        """
        Callback function of the subscription to the "/path" topic
        @params:
            -data : Path Message, containning the path's data
        """
        self.subPathFlag = True #rise flag to notify it has recieved the data
        self.path = data
    
    def gpsStart_cb (self,data):
        """
        Callback function of the subscription to the "/gpsstart" topic
        @params:
            -data : NavSatFix Message, containning the gpsStart's data
        """
        self.subGpsStartFlag = True #rise flag to notify it has recieved the data
        self.gpsStart = data

    def gps_cb(self,data):
        """
        Callback function of the subscription to the "/novatel/oem7/fix" topic
        @params:
            -data : NavSatFix Message, containning the gps' data
        """
        self.subGpsFlag = True #rise flag to notify it has recieved the data
        self.gps = data

    def odom_cb(self,data):
        """
        Callback function of the subscription to the "/novatel/oem7/odom" topic
        @params:
            -data : Odometry Message, containning the odometry's data
        """
        self.subOdomFlag = True #rise flag to notify it has recieved the data
        self.odom = data
    
    def cmdSteer_cb (self, data):
        """
        Callback function of the subscription to the "/cmd_steering" topic
        @params:
            -data : Odometry Message, containning the cmdSteering's data
        """
        self.subCmdSteeringFlag = True #rise flag to notify it has recieved the data
        self.cmdSteering = data

    #Client Method
    def requestGenProfile (self, path:Path, maxSpeed : float, maxGForce : float):
        """
        Call the service "speed_profile_gen" to generated the profile
        Creating the request sedn to the service
        @params:
            - path : Path, path followed by the vehicule
            - maxSpeed : float, the maximum speed allowed
            - maxGForce : float, the maximum GForce allowed
        @returns :
            return response of the service
        """
        #Set Basic info for request
        self.reqProfile.path = path
        self.reqProfile.maxspd = maxSpeed
        self.reqProfile.maxalat = maxGForce

        #Set Step for generation
        index = len(path.poses)-1
        p1, p2 = path.poses[index], path.poses[index-1]
        dist = round(self.getEuclideanDistance(
            p1.pose.position.x, p1.pose.position.y,
            p2.pose.position.x, p2.pose.position.y,),1)
        self.profileStep = math.ceil(1/dist) if (dist<1) else math.ceil(dist)
        self.distPoint = dist
        self.reqProfile.step = dist
        return self.genProfileClient.call_async(self.reqProfile)
    
    def genProfileSrv_cb (self, future):
        """
        Callback  function to the service "speed_profile_gen"
        @params :
            - future : SpeedProfileGen Response, response of the service
        @returns : None
        """
        try:
            result = future.result()
            self.profile = list(result.profile)
            self.srvGenPathFlag = True
        except Exception as e:
            self.get_logger().warn("Speed Profile Generation Service Failed")

    #Speed Calculation Method
    #Method 1 : According to the Speed Profile
    def setSpeedViaProfile(self,point, maxSpeed,minSpeed):
        """
        Set speed according to the speed found in the speed profile generated
        @params :
            - point : int,index of the nearest point of the path
            - maxSpeed : float, maximun Speed allowed
        @returns :
            - speed : float, the new speed of the vehicule
            - self.lastSpeed : in case there is a problem with the speedProfile, it returns the lastSpeed
        """
        # Find index of the next position in the speed profile
        indexProfile = int(point/self.profileStep) +5 
        if (indexProfile > len(self.profile) -1) : indexProfile = len(self.profile)-1
        #Decide which spede to used relative the  next position in speed profile
        if (len(self.profile)>0):
            goalSpeed = self.profile[indexProfile]
            print(f"Index Profile : {indexProfile} - Goal Speed : {goalSpeed}")
            speed = self.lastSpeed
            if (self.lastSpeed<goalSpeed):
                #Acceleration
                speed = min(maxSpeed,speed+(30/100)*maxSpeed)
            if (self.lastSpeed> goalSpeed):
                #Decelleration
                speed = max(minSpeed, speed-(50/100)*minSpeed)
            self.lastSpeed = speed
            return speed
        return self.lastSpeed
    
    #Method 2 : Via Path angle
    def setSpeedViaPathAngle (self, path : Path ,point, maxSpeed,minSpeed):
        """
        Set the speed by calculing the path angle and calating the most suitable speed according to the angle found
        To calculate the path angle, it does the following :
            - Selection of 3 points
            - Find the vectors : Vector 1 (v1) between point 1 (p1) and point 2 (p2) and Vector 2 (v2) between point 2 (p2) and point 3 (p3)
            - Calculate the norms (n1 and n2) of v1 and v2
            - Calculate the angle by doing (v1*v2)/(n1*n2)
        This calculation is repeated many time to obtain the angle on 7 meters
        Then each angle found is add together to do the total angle on this 7 meters
        After that we found the sspeed by using the following expression : -maxSpeed / 90 * totalAngle + maxSpeed
        @params :
            - path : Path, path followed by the vehicule
            - point : int, index of the nearest point on the path
            - maxSpeed : float, maxSpeed allowed
        @returns :
            -speed : float, the new Speed of the vehicule according to path angle
        """
        epsiVectorNorm = 1e-08 #psilon for vectors'norm
        speed = minSpeed
        totalAngle = 0
        #Check if the vehicule is near the end of the trajectory
        if (point < len(path.poses)-math.ceil(7/self.distPoint)):
            windowSize = math.ceil(7/self.distPoint)
        else:
            windowSize = len(path.poses)-point-1
        print(f"windowSize : {windowSize}")
        
        for i in range(point,point+windowSize-2):
            #Select points
            p0, p1, p2 = path.poses[i], path.poses[i+1], path.poses[i+2]
            #print(f"Points : P0 : {i} -- P1 : {i+1} -- P2 : {i+2}")
            #Create vectors
            v1 = [p1.pose.position.x - p0.pose.position.x ,p1.pose.position.y - p0.pose.position.y]
            v2 = [p2.pose.position.x - p1.pose.position.x ,p2.pose.position.y - p1.pose.position.y]
            #print(f"Vector 1 : {v1}  -  Vetor 2 : {v2}")

            #Calculate Vectors Norms
            n1 = math.sqrt(math.pow(v1[0],2) + math.pow(v1[1],2))
            n2 = math.sqrt(math.pow(v2[0],2) + math.pow(v2[1],2))        
            #print (f"Norm : v1 : {n1} -- v2 : {n2}")

            #Check if the norms are inforior to our espilon
            if (n1 < epsiVectorNorm or n2 < epsiVectorNorm):
                self.get_logger().info("Norm vector too short for calculation")
                continue

            #Calculate path angle
            theta = (v1[0]*v2[0] + v1[1]* v2[1])/(n1*n2)
            if (theta<-1) : theta = -1
            if (theta>1) : theta =1
            angle = math.degrees(abs(math.acos(theta)))
            #print(f"theta : {theta} -- Angle : {angle}")
            totalAngle = totalAngle + angle
        print(f"Total Angle : {totalAngle}")
        #Calculate new Speed
        speed = (-maxSpeed/90)*totalAngle + maxSpeed
        speed = max(minSpeed, speed)
        return speed
    
    #Method 3 : Via Cmd Steering
    def setSpeedViaCmdSteering (self, maxSpeed,minSpeed):
        """
        Set speed according to the current angle of the steering wheel
        @params :
            - maxSpeed : float, maximum Speed allowed
        @returns :
            - speed : float, the new veehicule speed
        """
        #Check if we have recieved data from the cmdSteering topic
        if (self.subCmdSteeringFlag):
            print(f"Cmd Steer Angle : {self.cmdSteering.angle}")
            #Calculate new Speed
            speed = (-maxSpeed/540)*abs(self.cmdSteering.angle) + maxSpeed
            speed = max(minSpeed, speed)
            return speed
        else:
            self.get_logger().error("No data recieved from CmdSteering Topic")
            return 0
        
    #Method 4 : Via Dynamic Pure Pursuit
    def setSpeedViaDynamicPurePursuit(self, path : Path ,point, maxSpeed,minSpeed):
        """
        Set speed by calculting th ecurvature of the road
        @params :
            - path : Path, path followed by the car
            - point : int, index of the nearest Point
            - maxSpeed : float, maximum speed allowed
        @return :
            - speed : float, the new speed of the vehicule
        """
        if(len(path.poses)<=point+2):
            return self.lastSpeed
        #Variables for the fonction
        alphaKappa = 0.1
        kV = 0.36
        alphaSpeed = 0.02

        #Calculate local Curvature
        localKappa = self.getLocalCurvature(path,point)
        kappa = alphaKappa*localKappa + (1-alphaKappa)*self.lastKappa
        print(f"Kappa : {kappa}")
        #Calculate Speed
        vRaw = kV / (kappa+0.01)
        vTarget = max(minSpeed, min(maxSpeed, vRaw))
        alphaSpeed = min (0.2, max(0.01, 1-math.exp(-5*abs(kappa - self.lastKappa))))
        vTarget = alphaSpeed*vTarget + (1-alphaSpeed)*self.lastSpeed
        self.lastKappa = kappa
        self.lastSpeed = vTarget
        return vTarget
    
    def getLocalCurvature (self, path : Path, point):
        """
        Calculate the local Curvature of the path at a specific point of the path
        @params :
            -path : Path, path followed by the car
            - point : int, index of the nearest point
        @returns:
            - kappa : float, local curvature
        """
        kappa = 0
        p0, p1, p2 = path.poses[point], path.poses[point+1], path.poses[point+2]
        x0, y0 = p0.pose.position.x, p0.pose.position.y
        x1, y1 = p1.pose.position.x, p1.pose.position.y
        x2, y2 = p2.pose.position.x, p2.pose.position.y
        area = (1/2)*(x0 * (y1-y2) + x1 * (y2-y0) + x2 * (y0-y1))
        d01 = self.getEuclideanDistance(x0,y0,x1,y1)
        d12 = self.getEuclideanDistance(x1,y1,x2,y2)
        d20 = self.getEuclideanDistance(x2,y2,x0,y0)
        kappa = (4*area)/(d01*d12*d20)
        return kappa
    
    #MainLoop
    def mainLoop (self):
        """
        Main loop of the node
        THe node follows the following steps:
            1 ) Set new values for parameters (if new ones are entered)
            2 ) Check if all the data has been recieved
            3 ) Create the spped profile
            4 ) Calculates the speed of the vehicule which the selected methods
            5 ) Publish the message on the /cmd_vel topic
        """
        #Clear Terminal
        os.system('cls' if os.name == 'nt' else 'clear')

        #Get new value for the parameters
        maxSpeed = self.get_parameter("maxSpeed").get_parameter_value().integer_value
        minSpeed = self.get_parameter("minSpeed").get_parameter_value().integer_value
        maxGForce = self.get_parameter("maxGForce").get_parameter_value().double_value
        method = self.get_parameter("methodSelect").get_parameter_value().integer_value
        self.get_logger().info(f"Parameters used :\n\t-> maxSpeed : {maxSpeed}\n\t-> minSpeed : {minSpeed}\n\t-> maxGForce : {maxGForce}\n\t-> Method Select : {method}")
        maxSpeedConvert = maxSpeed/3.6 #Convertion in m/s
        minSpeedConvert = minSpeed/3.6

        #Check Data Reception
        if not(self.subPathFlag and self.subGpsStartFlag and self.subOdomFlag and self.subGpsFlag):
            self.get_logger().info("Waiting For Data")
            print(f"Received :\n\t- Path : {self.subPathFlag}\n\t- Gps Start : {self.subGpsStartFlag}\n\t- Gps : {self.subGpsFlag}\n\t- Odom : {self.subOdomFlag}")
            return
        self.get_logger().info("All Data Received")
        
        #Generate Speed Profile
        if not(self.profile and self.srvGenPathFlag):
            self.reqProfile = SpeedProfileGen.Request()
            srvGenProfile = self.requestGenProfile(self.path, maxSpeedConvert, maxGForce)
            srvGenProfile.add_done_callback(self.genProfileSrv_cb)
        self.get_logger().info("Profile Generated")

        #Set Current position
        xDiff, yDiff = self.setOffset(self.gpsStart, self.gps)
        odomRect = self.odom
        odomRect.pose.pose.position.x = xDiff
        odomRect.pose.pose.position.y = yDiff
        print(f"Current Pos : {odomRect.pose.pose.position.x , odomRect.pose.pose.position.y}")

        #Find Trajectory Point
        nearPoint, nearDist = self.getNeareastPoint(self.path,odomRect.pose.pose.position.x,odomRect.pose.pose.position.y)
        print(f"Near Point : {nearPoint} - Dist : {nearDist}")

        #Set Speed According to the method select
        if (method == 1):
            self.get_logger().info("Method : Profile Generated")
            speed = self.setSpeedViaProfile(nearPoint, maxSpeedConvert,minSpeedConvert)
        elif (method == 2):
            self.get_logger().info("Method : Path Angle")
            speed = self.setSpeedViaPathAngle(self.path, nearPoint, maxSpeedConvert,minSpeedConvert)
        elif (method == 3):
            self.get_logger().info("Method : Steering Angle")
            speed = self.setSpeedViaCmdSteering(maxSpeedConvert,minSpeedConvert)
        elif (method == 4):
            self.get_logger().info("Method : Dynamic Pure Pursuit")
            speed = self.setSpeedViaDynamicPurePursuit(self.path,nearPoint, maxSpeedConvert,minSpeedConvert)
        else:
            self.get_logger().warn("Invalid Method ID entered")
            return
        print(f"Speed after calculation : {speed}")

        #Publish Speed Message
        speed = float(min(maxSpeedConvert, speed))
        print(f"Speed Send to topic : {speed}")
        if not(speed is None):
            msgSpeed = Twist()
            msgSpeed.linear.x = speed
            msgSpeed.linear.y = 0.0
            msgSpeed.linear.z = 0.0
            msgSpeed.angular.x = 0.0
            msgSpeed.angular.y = 0.0
            msgSpeed.angular.z = 0.0
            self.pubCmdVel.publish(msgSpeed)
            self.get_logger().info(f"Message Sent to the /cmd_vel topic (speed : {round(speed,2)} m/s <=> {round(speed*3.6,2)} km/h)")

# --- Main Function --- #
def main():
    rclpy.init()
    analyzer = SpeedAnalyzer()
    rclpy.spin(analyzer)
    analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()