#! /usr/bin/env python3

import rospy
import math
import numpy as np
import tf
from tf import transformations
from mirX import mirX
from mirY import mirY

from toolX import toolX
from toolY import toolY
from toolZ import toolZ

from timeStamp import timeStamp

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class parse_path_trafo():

    def __init__(self):
        rospy.init_node("parse_path_trafo_node")
        self.mir_x= mirX()
        self.mir_y= mirY()

        self.wall_x = toolX()
        self.wall_y = toolY()
        self.wall_z = toolZ()
        
        self.time_stamps = timeStamp()

        #print(mir_y)

        self.mir_pub = rospy.Publisher("/mir_path", Path, queue_size= 1)
        self.ur_pub = rospy.Publisher("/ur_path", Path, queue_size= 1)


        self.mir_path = Path()
        self.mir_path_point = PoseStamped()
        self.mir_path.header.frame_id = "map"
        self.mir_path.header.stamp = rospy.Time.now()
        self.ur_path = Path()
        self.ur_path_point = PoseStamped()
        self.ur_path.header.frame_id = "map"
        self.ur_path.header.stamp = rospy.Time.now()

        rospy.Subscriber("/wall_frame_base", PoseWithCovarianceStamped, self.wall_frame_cb)
        rospy.spin()



    def wall_frame_cb(self,PoseWithCovarianceStamped):
        self.x = PoseWithCovarianceStamped.pose.pose.position.x
        self.y = PoseWithCovarianceStamped.pose.pose.position.y
        self.z = PoseWithCovarianceStamped.pose.pose.position.z
        q1 = PoseWithCovarianceStamped.pose.pose.orientation.x
        q2 = PoseWithCovarianceStamped.pose.pose.orientation.y
        q3 = PoseWithCovarianceStamped.pose.pose.orientation.z
        q4 = PoseWithCovarianceStamped.pose.pose.orientation.w
        self.R = transformations.quaternion_matrix([q1,q2,q3,q4])

        self.run()


    def run(self):

        self.mir_path.poses = [PoseStamped() for i in range(len(self.mir_x))] 
        self.ur_path.poses = [PoseStamped() for i in range(len(self.wall_x))] 


        for i in range(0,len(self.mir_x)-1): #len(robot0_xhat)
            self.mir_path_point.pose.position.x = self.x + self.mir_x[i] * self.R[0][0] + self.mir_y[i] * self.R[0][1]
            self.mir_path_point.pose.position.y = self.y + self.mir_x[i] * self.R[1][0] + self.mir_y[i] * self.R[1][1]

            self.mir_path.poses[i].pose.position.x = self.mir_path_point.pose.position.x
            self.mir_path.poses[i].pose.position.y = self.mir_path_point.pose.position.y
            
            # add orientation
            orientation = math.atan2(self.mir_y[i+1]-self.mir_y[i], self.mir_x[i+1]-self.mir_x[i])
            q = tf.transformations.quaternion_from_euler(0, 0, orientation)
            self.mir_path.poses[i].pose.orientation.x = q[0]
            self.mir_path.poses[i].pose.orientation.y = q[1]
            self.mir_path.poses[i].pose.orientation.z = q[2]
            self.mir_path.poses[i].pose.orientation.w = q[3]
            
            self.mir_path.poses[i].header.stamp = rospy.Duration(self.time_stamps[i])

        for i in range(0,len(self.wall_x)-1): #len(robot0_xhat)
            self.ur_path_point.pose.position.x = self.x + self.wall_x[i] * self.R[0][0] + self.wall_y[i] * self.R[0][1]
            self.ur_path_point.pose.position.y = self.y + self.wall_x[i] * self.R[1][0] + self.wall_y[i] * self.R[1][1]
            self.ur_path_point.pose.position.z = self.z + self.wall_z[i] + 0.3

            self.ur_path.poses[i].pose.position.x = self.ur_path_point.pose.position.x
            self.ur_path.poses[i].pose.position.y = self.ur_path_point.pose.position.y
            self.ur_path.poses[i].pose.position.z = self.ur_path_point.pose.position.z
            
            # add orientation
            orientation = math.atan2(self.wall_y[i+1]-self.wall_y[i], self.wall_x[i+1]-self.wall_x[i])
            q = tf.transformations.quaternion_from_euler(0, 0, orientation)
            
            #  rotate around x so that the gripper is pointing down
            q_rot = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
            q_ur=tf.transformations.quaternion_multiply(q_rot, q)
            q_rot = tf.transformations.quaternion_from_euler(0, 0, np.pi/2)
            q_ur=tf.transformations.quaternion_multiply(q_rot, q_ur)
            
            self.ur_path.poses[i].pose.orientation.x = q_ur[0]
            self.ur_path.poses[i].pose.orientation.y = q_ur[1]
            self.ur_path.poses[i].pose.orientation.z = q_ur[2]
            self.ur_path.poses[i].pose.orientation.w = q_ur[3]
            
            self.ur_path.poses[i].header.stamp = rospy.Duration(self.time_stamps[i])

        self.mir_pub.publish(self.mir_path)
        rospy.sleep(2.0)
        self.ur_pub.publish(self.ur_path)
        rospy.sleep(2.0)
        rospy.loginfo("paths published")
        rospy.sleep(1.1)
        print("path published")
        



if __name__=="__main__":
    parse_path_trafo()
