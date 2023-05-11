#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt

class ControlNozzle():
    
    def config(self):
        self.profiles_topic = "/profiles"
    
    def __init__(self):
        rospy.init_node("control_nozzle_node")  
        self.config() 

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

        #

        # plot the x and z points
        plt.plot(self.x, self.z)
        plt.show()

        rospy.spin()   
        #print("Control Nozzle")
        #print(self.profiles_msg)
        pass


    def profiles_callback(self, msg=PointCloud2()):
        self.profiles_msg = msg
        cloud_points = list(pc2.read_points(self.profiles_msg, skip_nans=True, field_names = ("x", "y", "z")))
        #get first column of points
        self.x = [x[0] for x in cloud_points]
        self.y = [x[1] for x in cloud_points]
        self.z = [x[2] for x in cloud_points]

        
        







    
if __name__ == "__main__":
    ControlNozzle().main()