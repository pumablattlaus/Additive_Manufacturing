#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
import math

class MoveURToStartPose():
    
    def config(self):
        pass
    
    
    def __init__(self):
        rospy.init_node("control_ur_node")
        
        ur_start_pose_array = rospy.get_param("~ur_start_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.ur_start_pose = Pose()
        self.ur_start_pose.position.x = ur_start_pose_array[0]
        self.ur_start_pose.position.y = ur_start_pose_array[1]
        self.ur_start_pose.position.z = ur_start_pose_array[2]
        self.ur_start_pose.orientation.x = ur_start_pose_array[3]
        self.ur_start_pose.orientation.y = ur_start_pose_array[4]   
        self.ur_start_pose.orientation.z = ur_start_pose_array[5]
        self.ur_start_pose.orientation.w = ur_start_pose_array[6]
        
        self.ur_command_topic = rospy.get_param("~ur_command_topic", "/mur620/UR10_r/twist_controller/command_safe")
        
        self.ur_twist_publisher = rospy.Publisher(self.ur_command_topic, Twist, queue_size=1)
        
        self.config()
        

    def move_ur_to_start_pose(self):
                
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
             # broadcast start pose
            self.broadcast_target_pose(self.ur_start_pose)
            
            # get ur pose from listener
            lin, ang = self.ur_pose_listener.lookupTransform("/mocap", "/mur620b/UR10_r/tool0", rospy.Time(0))
        
            # compute rotation matrix
            R = tf.transformations.quaternion_matrix(ang)
        
            # invert rotation matrix
            R = tf.transformations.inverse_matrix(R)
        
            # calculate error
            e_x = pose.position.x - lin[0]
            e_y = pose.position.y - lin[1]
            e_z = pose.position.z - lin[2]
            
            # transform error to base frame
            e_x_base = R[0,0] * e_x + R[0,1] * e_y + R[0,2] * e_z
            e_y_base = R[1,0] * e_x + R[1,1] * e_y + R[1,2] * e_z
            e_z_base = R[2,0] * e_x + R[2,1] * e_y + R[2,2] * e_z
            
            # calculate command
            self.ur_command.linear.x = e_x_base * self.Kpx
            self.ur_command.linear.y = e_y_base * self.Kpy
            self.ur_command.linear.z = e_z_base * self.Kpz
                       
            # limit velocity
            ur_command = self.limit_velocity(self.ur_command, self.ur_command_old)
            self.ur_command_old = ur_command
            
            self.ur_command_publisher.publish(ur_command)
        
            # check if target is reached
            if abs(e_x) < self.ur_target_tolerance and abs(e_y) < self.ur_target_tolerance:
                self.ur_command.linear.x = 0
                self.ur_command.linear.y = 0
                self.ur_command.linear.z = 0
                self.ur_command_publisher.publish(self.ur_command)
                rospy.sleep(0.1)
                break
            
            rate.sleep()
    

    
    def broadcast_target_pose(self, target_pose = Pose()):
        self.ur_target_pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, target_pose.position.z), (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, 1), rospy.Time.now(), "/ur_target_pose", "/mocap")
    
    def limit_velocity(self, ur_command, ur_command_old):
        vel_scale = 1.0
        
        # limit velocity
        if abs(ur_command.linear.x) > self.ur_velocity_limit:
            vel_scale = self.ur_velocity_limit / abs(ur_command.linear.x)
        if abs(ur_command.linear.y) > self.ur_velocity_limit and abs(ur_command.linear.y) * vel_scale > self.ur_velocity_limit:
            vel_scale = self.ur_velocity_limit / abs(ur_command.linear.y)
        if abs(ur_command.linear.z) > self.ur_velocity_limit and abs(ur_command.linear.z) * vel_scale > self.ur_velocity_limit:
            vel_scale = self.ur_velocity_limit / abs(ur_command.linear.z)
        
        # apply vel_scale
        ur_command.linear.x = ur_command.linear.x * vel_scale
        ur_command.linear.y = ur_command.linear.y * vel_scale
        ur_command.linear.z = ur_command.linear.z * vel_scale    
        
        vel_scale = 1.0
        # limit acceleration
        if abs(ur_command.linear.x - ur_command_old.linear.x) > self.ur_acceleration_limit:
            vel_scale = self.ur_acceleration_limit / abs(ur_command.linear.x - ur_command_old.linear.x)
        if abs(ur_command.linear.y - ur_command_old.linear.y) > self.ur_acceleration_limit and abs(ur_command.linear.y - ur_command_old.linear.y) * vel_scale > self.ur_acceleration_limit:
            vel_scale = self.ur_acceleration_limit / abs(ur_command.linear.y - ur_command_old.linear.y)
        if abs(ur_command.linear.z - ur_command_old.linear.z) > self.ur_acceleration_limit and abs(ur_command.linear.z - ur_command_old.linear.z) * vel_scale > self.ur_acceleration_limit:
            vel_scale = self.ur_acceleration_limit / abs(ur_command.linear.z - ur_command_old.linear.z)
            
        # apply vel_scale
        ur_command.linear.x = ur_command.linear.x * vel_scale
        ur_command.linear.y = ur_command.linear.y * vel_scale
        ur_command.linear.z = ur_command.linear.z * vel_scale

        return ur_command
    
    
    
    
    
if __name__ == "__main__":
    MoveURToStartPose().main()