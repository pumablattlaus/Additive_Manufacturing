#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist

class Control_ur():
    
    def config(self):
        self.Kpx = -0.2
        self.Kpy = 0.2
        self.Kpz = 0.0
        self.ur_target_tolerance = 0.01
        self.ur_acceleration_limit = 0.01
        self.ur_velocity_limit = 0.15
        pass
    
    
    def __init__(self):
        rospy.init_node("control_ur_node")
        self.ur_pose_listener = tf.TransformListener()
        self.ur_target_pose_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("/path_index", Int32, self.path_index_callback)
        self.ur_command_publisher = rospy.Publisher("/UR10_r/twist_controller/command", Twist, queue_size=2)
        self.ur_command = Twist()
        self.ur_command_old = Twist()
        self.path_index = 0
        
        
        self.config()
        
    
    
    def main(self):

        print("waiting for path")
        self.ur_path = rospy.wait_for_message("/ur_path", Path)
        print("got path")
        
        # move ur to start pose
        self.move_ur_to_start_pose(self.ur_path.poses[0].pose)
        
        # start control loop
        self.update()
        
        rospy.sleep(1)
        rospy.spin()
    
    
    def update(self):
        
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # get ur pose from listener
            lin, ang = self.ur_pose_listener.lookupTransform("/mocap", "/mur620b/UR10_r/tool0", rospy.Time(0))

            # get target pose from path
            target_pose = self.ur_path.poses[self.path_index].pose
            
            # broadcast target pose
            self.ur_target_pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, target_pose.position.z), (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, 1), rospy.Time.now(), "/ur_target_pose", "/mocap")
            
            # compute rotation matrix
            R = tf.transformations.quaternion_matrix(ang)
        
            # invert rotation matrix
            R = tf.transformations.inverse_matrix(R)
        
            # calculate error
            e_x = target_pose.position.x - lin[0]
            e_y = target_pose.position.y - lin[1]
            e_z = target_pose.position.z - lin[2]
            
            print("e_x: " + str(e_x))
            print("e_y: " + str(e_y))
            
            # transform error to base frame
            e_x_base = R[0,0] * e_x + R[0,1] * e_y + R[0,2] * e_z
            e_y_base = R[1,0] * e_x + R[1,1] * e_y + R[1,2] * e_z
            e_z_base = R[2,0] * e_x + R[2,1] * e_y + R[2,2] * e_z
            
            # calculate command
            self.ur_command.linear.x = e_x_base * self.Kpx
            self.ur_command.linear.y = e_y_base * self.Kpy
            self.ur_command.linear.z = e_z_base * self.Kpz
            
            # limit velocity
            if abs(self.ur_command.linear.x) > self.ur_velocity_limit:
                self.ur_command.linear.x = self.ur_velocity_limit * abs(self.ur_command.linear.x) / self.ur_command.linear.x
            if abs(self.ur_command.linear.y) > self.ur_velocity_limit:
                self.ur_command.linear.y = self.ur_velocity_limit * abs(self.ur_command.linear.y) / self.ur_command.linear.y
            if abs(self.ur_command.linear.z) > self.ur_velocity_limit:
                self.ur_command.linear.z = self.ur_velocity_limit * abs(self.ur_command.linear.z) / self.ur_command.linear.z
            
            # limit acceleration
            if abs(self.ur_command.linear.x - self.ur_command_old.linear.x) > self.ur_acceleration_limit:
                self.ur_command.linear.x = self.ur_command_old.linear.x + self.ur_acceleration_limit * abs(self.ur_command.linear.x - self.ur_command_old.linear.x) / (self.ur_command.linear.x - self.ur_command_old.linear.x)
            if abs(self.ur_command.linear.y - self.ur_command_old.linear.y) > self.ur_acceleration_limit:
                self.ur_command.linear.y = self.ur_command_old.linear.y + self.ur_acceleration_limit * abs(self.ur_command.linear.y - self.ur_command_old.linear.y) / (self.ur_command.linear.y - self.ur_command_old.linear.y)
            if abs(self.ur_command.linear.z - self.ur_command_old.linear.z) > self.ur_acceleration_limit:
                self.ur_command.linear.z = self.ur_command_old.linear.z + self.ur_acceleration_limit * abs(self.ur_command.linear.z - self.ur_command_old.linear.z) / (self.ur_command.linear.z - self.ur_command_old.linear.z)
            self.ur_command_old = self.ur_command
            
            self.ur_command_publisher.publish(self.ur_command)
            
            rate.sleep()
    
    def move_ur_to_start_pose(self, pose):
        
        # broadcast start pose
        self.ur_target_pose_broadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, 1), rospy.Time.now(), "/ur_target_pose", "/mocap")
        
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
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
            
            print("e_x: " + str(e_x))
            print("e_y: " + str(e_y))
            
            
            # transform error to base frame
            e_x_base = R[0,0] * e_x + R[0,1] * e_y + R[0,2] * e_z
            e_y_base = R[1,0] * e_x + R[1,1] * e_y + R[1,2] * e_z
            e_z_base = R[2,0] * e_x + R[2,1] * e_y + R[2,2] * e_z
            
            # calculate command
            self.ur_command.linear.x = e_x_base * self.Kpx
            self.ur_command.linear.y = e_y_base * self.Kpy
            self.ur_command.linear.z = e_z_base * self.Kpz
            
            # limit velocity
            if abs(self.ur_command.linear.x) > self.ur_velocity_limit:
                self.ur_command.linear.x = self.ur_velocity_limit * abs(self.ur_command.linear.x) / self.ur_command.linear.x
            if abs(self.ur_command.linear.y) > self.ur_velocity_limit:
                self.ur_command.linear.y = self.ur_velocity_limit * abs(self.ur_command.linear.y) / self.ur_command.linear.y
            if abs(self.ur_command.linear.z) > self.ur_velocity_limit:
                self.ur_command.linear.z = self.ur_velocity_limit * abs(self.ur_command.linear.z) / self.ur_command.linear.z
            
            # limit acceleration
            if abs(self.ur_command.linear.x - self.ur_command_old.linear.x) > self.ur_acceleration_limit:
                self.ur_command.linear.x = self.ur_command_old.linear.x + self.ur_acceleration_limit * abs(self.ur_command.linear.x - self.ur_command_old.linear.x) / (self.ur_command.linear.x - self.ur_command_old.linear.x)
            if abs(self.ur_command.linear.y - self.ur_command_old.linear.y) > self.ur_acceleration_limit:
                self.ur_command.linear.y = self.ur_command_old.linear.y + self.ur_acceleration_limit * abs(self.ur_command.linear.y - self.ur_command_old.linear.y) / (self.ur_command.linear.y - self.ur_command_old.linear.y)
            if abs(self.ur_command.linear.z - self.ur_command_old.linear.z) > self.ur_acceleration_limit:
                self.ur_command.linear.z = self.ur_command_old.linear.z + self.ur_acceleration_limit * abs(self.ur_command.linear.z - self.ur_command_old.linear.z) / (self.ur_command.linear.z - self.ur_command_old.linear.z)
            self.ur_command_old = self.ur_command
            
            self.ur_command_publisher.publish(self.ur_command)
        
            
        
            # check if target is reached
            if abs(e_x) < self.ur_target_tolerance and abs(e_y) < self.ur_target_tolerance:
                self.ur_command.linear.x = 0
                self.ur_command.linear.y = 0
                self.ur_command.linear.z = 0
                self.ur_command_publisher.publish(self.ur_command)
                rospy.sleep(0.1)
                break
            
            rate.sleep()
    
    
    def path_index_callback(self, msg):
        print(msg.data)
        self.path_index = msg.data
    
    
    
if __name__ == "__main__":
    Control_ur().main()