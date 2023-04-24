#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
import math

class MoveURToStartPose():
    
    def config(self):
        self.ur_velocity_limit = rospy.get_param("~ur_velocity_limit", 0.1)
        self.ur_acceleration_limit = rospy.get_param("~ur_acceleration_limit", 0.2)
        self.Kpx = rospy.get_param("~Kpx", -0.02)
        self.Kpy = rospy.get_param("~Kpy", -0.02)
        self.Kpz = rospy.get_param("~Kpz", 0.02)
        self.ur_target_tolerance = rospy.get_param("~ur_target_tolerance", 0.01)
        pass
    
    
    def __init__(self):
        rospy.init_node("control_ur_node")
        
        self.ur_command = Twist()
        self.ur_command_old = Twist()
        ur_start_pose_array = rospy.get_param("~ur_start_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.ur_start_pose = Pose()
        self.ur_start_pose.position.x = ur_start_pose_array[0]
        self.ur_start_pose.position.y = ur_start_pose_array[1]
        self.ur_start_pose.position.z = ur_start_pose_array[2]
        self.ur_start_pose.orientation.x = ur_start_pose_array[3]
        self.ur_start_pose.orientation.y = ur_start_pose_array[4]   
        self.ur_start_pose.orientation.z = ur_start_pose_array[5]
        self.ur_start_pose.orientation.w = ur_start_pose_array[6]
        
        self.ur_command_topic = rospy.get_param("~ur_command_topic", "/mur620c/UR10_r/twist_controller/command_safe")
        self.ur_pose_topic = rospy.get_param("~ur_pose_topic", "/mur620c/UR10_r/ur_calibrated_pose")
        self.ur_base_link_frame_id = rospy.get_param("~ur_base_link_frame_id", "mur620c/UR10_r/base_link")
        self.ur_twist_publisher = rospy.Publisher(self.ur_command_topic, Twist, queue_size=1)
        self.ur_target_pose_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber(self.ur_pose_topic, PoseStamped, self.ur_pose_callback)
        
        self.config()
        

    def move_ur_to_start_pose(self):
                
        # wait until ur_pose is published
        rospy.wait_for_message(self.ur_pose_topic, PoseStamped)
                
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
             # broadcast start pose
            self.broadcast_target_pose(self.ur_start_pose)
        
            # calculate error
            e_x = self.ur_start_pose.position.x - self.ur_pose_current.position.x
            e_y = self.ur_start_pose.position.y - self.ur_pose_current.position.y
            e_z = self.ur_start_pose.position.z - self.ur_pose_current.position.z
            
            # calculate command
            self.ur_command.linear.x = e_x * self.Kpx
            self.ur_command.linear.y = e_y * self.Kpy
            self.ur_command.linear.z = e_z * self.Kpz
                       
            # limit velocity
            ur_command = self.limit_velocity(self.ur_command, self.ur_command_old)
            self.ur_command_old = ur_command
            
            self.ur_twist_publisher.publish(ur_command)
        
            # check if target is reached
            if abs(e_x) < self.ur_target_tolerance and abs(e_y) < self.ur_target_tolerance:
                self.ur_command.linear.x = 0
                self.ur_command.linear.y = 0
                self.ur_command.linear.z = 0
                self.ur_twist_publisher.publish(self.ur_command)
                rospy.sleep(0.1)
                break
            
            rate.sleep()
    

    
    def broadcast_target_pose(self, target_pose = Pose()):
        self.ur_target_pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, target_pose.position.z), (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, 1), rospy.Time.now(), "/ur_target_pose", self.ur_base_link_frame_id)
    
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
    
    
    def ur_pose_callback(self, msg):
        self.ur_pose_current = msg.pose
    
    
if __name__ == "__main__":
    MoveURToStartPose().move_ur_to_start_pose()