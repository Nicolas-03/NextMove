#####
# @Title : profile_generator
# @Author : MALET Pierre @IG2I - Centrale Lille Insitut, Class 2028
# @Date : V1 | Begin : 30 June 2025 - End : 02 July 2025
# @Project : Autnomous Vehcule Project with Renault Zoé of the @PRETIL from the @CRIStAL laboratory
# @Description :  This ROS Node analyzes the path followed by the car and create a speed profile
#####

# --- Libraries Imports --- #
import rclpy
from rclpy.node import Node
from av_path_speed_analyzer_interface.srv import SpeedProfileGen
from nav_msgs.msg import Path
import math

# --- ProfileGenerator Node --- #
class ProfileGenerator (Node):
    #Variables
    maxSpeed = 0 #m/s
    maxALat = 0 #m/s^2
    step = 1

    #Constructor
    def __init__ (self):
        """
        Create a new instance of ProfileGenerator, a child Class of Node Class
        This class is the server side of a service name speed_profile_gen.
        This service allows the user to create a speed profile according to the trajectory defined
        """
        super().__init__("profile_generator")
        self.generator = self.create_service(SpeedProfileGen,"speed_profile_gen", self.genProfile_cb)
        self.get_logger().info("GenProfile : Init Done\nGenProfile : Waiting for a call")
    
    #General Method
    def getPathXi (self, path : Path, ind):
        """
        Obtain X coord of point belonging to the path
        @params :
            - path : Path, the path followed by the véhicule
            - ind : int, index of the point whose we want the X coord
        @returns :
            - float, X coord of the point
        """
        return path.poses[ind].pose.position.x
    def getPathYi (self, path : Path, ind):
        """
        Obtain X coord of point belonging to the path
        @params :
            - path : Path, the path followed by the véhicule
            - ind : int, index of the point whose we want the Y coord
        @returns :
            - float, Y coord of the point
        """
        return path.poses[ind].pose.position.y
    def getEuclideanDistance (self, x1,y1, x2, y2):
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
        return math.sqrt(
            math.pow(x2-x1,2) + math.pow(y1-y2,2)
        )
    
    #Main Methods
    def genProfile_cb (self, request, response):
        """
        Callback function for the service linked to this node
        This function generated the speedProfile used in the main node.
        To generate the profile, it calculates the curvature and set a max speed relative to the curvature
        @params :
            -request : SpeedProfileGen Request, a request message to the service
            - response : SpeedProfileGen Response, a response message send to the client node
        @returns :
            - response : SpeedProfileGen Response, complete response message
        """
        #Save parameters used for the generation
        self.step = request.step
        self.maxSpeed = request.maxspd
        self.maxALat = request.maxalat
        #Define the step used to generate the profile (<=> distance in meter between 2 points of the profile)
        if (self.step <=1):
            genProfileStep = math.ceil(1/self.step) #Divide 1 meter by the step
        else:
            genProfileStep = math.ceil(self.step)
        print(f"genProfileStep = {genProfileStep}")
        #Create the profile by calculating the curvature named kappa
        response.profile = []
        localKappa = [] #list containning all the cuvature between two points distance by the step used.
        ##Loop through the path to calculate the curvature
        for i in range(0,len(request.path.poses),genProfileStep):
            if (len(request.path.poses)<= i+1):
                continue
            print(f"i = {i}")
            maxLoop = 1 if (1<=self.step) else math.ceil(1/self.step)
            print(f"Max Loop = {maxLoop}")
            ##Calculate curvature with many points to be more precise
            if (i+maxLoop<=len(request.path.poses)):
                for p in range (i,i+maxLoop):
                    p0, p1, p2 = p, p+1, p+2
                    print(f"p0 = {p0} -- p1 = {p1} -- p2 = {p2}")
                    localKappa.append(self.getCurvature(request.path,p0,p1,p2))
                kappa = abs(1/(maxLoop) * sum(localKappa))
                localKappa = []
            else:
                kappa = 0
            print(f"Final kappa : {kappa}")
            ##Set speed in profile according the curvature
            if (kappa <= 0):
                response.profile.append(self.maxSpeed)
            else:
                response.profile.append( float(min (self.maxSpeed, math.sqrt(self.maxALat/kappa))))
            print("\n--------\n")
        #Sent reponse
        self.get_logger().info(f"GenProfile : Step found : {request.step} - Max Speed : {round(request.maxspd,2)}m/s (~ {round(request.maxspd*3.6,2)} km/h) - Max Accell : {request.maxalat}")
        print(len(response.profile))
        return response
    
    def getCurvature (self, path : Path , p0, p1, p2):
        """
        Method used to calculate the curvature between 3 points
        To calculate the curvature, we calculate the area of the triangle formed by 3 points
        Then we multpily this area by 4 and divide by the product of the distance between points 1 and 2 , 2 and 3 , 3 and 1
        @params :
            - path : Path, trajectory foolowed by the vehicule
            - p0 : int , index of the 1st Point
            - p1 : int, index of the 2nd Point
            - p2 : int, index of the 3rd Point
        @returns :
            - k : float, cuvature between the points entered
        """
        #Get the coordinates of each point
        x0, y0 = round(self.getPathXi (path, p0),2) , round(self.getPathYi(path, p0),2)
        x1, y1 = round(self.getPathXi (path, p1),2) , round(self.getPathYi(path, p1),2)
        x2, y2 = round(self.getPathXi (path, p2),2) , round(self.getPathYi(path, p2),2)
        print (f"P0 = {x0,y0} --- P1 = {x1,y1} --- P2 = {x2,y2}")
        #Calculate area
        area =  (1/2) * (x0*(y1 -y2) + x1*(y2 -y0) + x2*(y0 - y1))
        print(f"area : {area}")
        #Calculate Distance between points
        d01 = self.getEuclideanDistance(x0,y0,x1,y1)
        d12 = self.getEuclideanDistance(x1,y1,x2,y2)
        d20 = self.getEuclideanDistance(x2,y2,x0,y0)
        #Calculate local curvature
        k= (4*area)/(d01 * d12 * d20)
        return k

# --- Main --- #
def main():
    rclpy.init()
    generator = ProfileGenerator()
    rclpy.spin(generator)
    rclpy.shutdown()

if __name__ == "__main__":
    main()