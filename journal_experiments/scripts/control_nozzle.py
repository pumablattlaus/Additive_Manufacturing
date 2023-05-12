#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
#import matplotlib.pyplot as plt

class ControlNozzle():
    
    def config(self):
        #initialize keyence profile values
        self.profiles_topic = "/profiles"
        self.reference_distance = -0.0447 #meters

        #initialize volume flow override variables 0.5 - 1.5
        self.Kp_volume_flow = 0.5
        self.volume_flow_min = 0.5  #Servopositionvalue
        self.volume_flow_max = 1.5 #Servopositionvalue

        #initialize nozzle feedrate override variables 0.5 - 1.5
        self.Kp_nozzle_feedrate = 0.5
        self.nozzle_feedrate_min = 0.5 #% of standard feedrate
        self.nozzle_feedrate_max = 1.5 #% of standard feedrate

        #initialize nozzle pose override vairables -1.0 - 1.0
        self.kp_nozzle_pose = 0.0025             #1/200 to reach the motion limits at 400 points of deviation
        self.nozzle_pose_override_min = -1.0    #of standard nozzle pose
        self.nozzle_pose_override_max =  1.0    #of standard nozzle pose
    

    def __init__(self):
        rospy.init_node("control_nozzle_node")  
        self.config()
        self.volume_flow_override = 0.0
        self.nozzle_feedrate_override = 0.0
        self.nozzle_pose_override = 0.0

        #initialize publishers
        self.nozzle_feedrate_override_publisher = rospy.Publisher('/nozzle_feedrate_override', Float32, queue_size=1)
        self.volume_flow_override_publisher = rospy.Publisher('/volume_flow_override', Float32, queue_size=1)
        self.lateral_nozzle_pose_override_publisher = rospy.Publisher('/lateral_nozzle_pose_override', Float32, queue_size=1)
        #wait for publishers to be ready
        rospy.sleep(1)

        #initialize subscribers
        rospy.Subscriber(self.profiles_topic, PointCloud2, self.profiles_callback)


        
    def main(self):
        #wait for messages
        rospy.wait_for_message(self.profiles_topic, PointCloud2)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():    
            
            #get maximum z value
            self.max_z = max(self.z)
            #get index of maximum z value
            self.max_z_index = self.z.index(self.max_z)
            #get lenght of z list
            self.z_len = len(self.z)

            #reduce volume flow in case of maximum z value higher than reference distance
            if self.max_z > self.reference_distance:
                self.volume_flow_override = (self.reference_distance-self.max_z)*self.Kp_volume_flow
                # limit the volume flow override to the range of volume_flow_min and volume_flow_max
                if self.volume_flow_override < self.volume_flow_min:
                    self.volume_flow_override = self.volume_flow_min
                elif self.volume_flow_override > self.volume_flow_max:
                    self.volume_flow_override = self.volume_flow_max
            elif self.max_z < self.reference_distance or self.max_z == -9999:
                    self.volume_flow_override = 1.0
            #publish the volume flow override
            self.volume_flow_override_publisher.publish(self.volume_flow_override)


            #reduce feed rate in case of maximum z value lower than reference distance
            if self.max_z < self.reference_distance:
                self.nozzle_feedrate_override = (self.max_z-self.reference_distance)*self.Kp_nozzle_feedrate
                # limit the nozzle feedrate override to the range of nozzle_feedrate_min and nozzle_feedrate_max
                if self.nozzle_feedrate_override < self.nozzle_feedrate_min:
                    self.nozzle_feedrate_override = self.nozzle_feedrate_min
                elif self.nozzle_feedrate_override > self.nozzle_feedrate_max:
                    self.nozzle_feedrate_override = self.nozzle_feedrate_max
            elif self.max_z > self.reference_distance or self.max_z == -9999:
                self.nozzle_feedrate_override = 1.0
            #publish the nozzle feedrate override
            self.nozzle_feedrate_override_publisher.publish(self.nozzle_feedrate_override)


            #calculate lateral nozzle pose override
            self.nozzle_pose_override = (self.max_z_index-(self.z_len*0.5))*self.kp_nozzle_pose
            #check if nozzle pose override is within the range of nozzle_pose_min and nozzle_pose_max
            if self.nozzle_pose_override < self.nozzle_pose_override_min:
                self.nozzle_pose_override = self.nozzle_pose_override_min
            elif self.nozzle_pose_override > self.nozzle_pose_override_max:
                self.nozzle_pose_override = self.nozzle_pose_override_max
            #set nozzle pose override to 0.0 if maximum z value is -9999 because no profile is detected
            elif self.max_z == -9999:
                self.nozzle_pose_override = 0.0
            #publish the nozzle pose override
            self.lateral_nozzle_pose_override_publisher.publish(self.nozzle_pose_override)


            rate.sleep()


    def profiles_callback(self, msg=PointCloud2()):
        self.profiles_msg = msg
        cloud_points = list(pc2.read_points(self.profiles_msg, skip_nans=True, field_names = ("x", "y", "z")))
        #get first column of points
        self.x = [x[0] for x in cloud_points]
        self.y = [x[1] for x in cloud_points]
        self.z = [x[2] for x in cloud_points]
        #exchange inf in z with -9999
        self.z = [-9999 if math.isinf(x) else x for x in self.z]


if __name__ == "__main__":
    ControlNozzle().main()