#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
import math

class Control_ur():
    
    def config(self):
        self.Kpx = -0.2
        self.Kpy = 0.2
        self.Kpz = 0.0
        self.Kpffx = 0.0  # feed-forward gain
        self.kpffy = 0.0
        self.ur_target_tolerance = 0.02
        self.ur_acceleration_limit = 0.003
        self.ur_jerk_limit = 0.1 
        self.ur_velocity_limit = 0.1
        self.ur_target_velocity = 0.03
        self.path_distance_between_points = 0.01
        pass
    
    
    def __init__(self):
        rospy.init_node("control_ur_node")
        self.ur_pose_listener = tf.TransformListener()
        self.ur_target_pose_broadcaster = tf.TransformBroadcaster()
        #rospy.Subscriber("/path_index", Int32, self.path_index_callback)
        self.ur_command_publisher = rospy.Publisher("/UR10_r/twist_controller/command_safe", Twist, queue_size=2)
        self.path_index_publisher = rospy.Publisher('/path_index', Int32, queue_size=1)
        self.ur_command = Twist()
        self.ur_command_old = Twist()
        self.path_index = 1
        self.path_speed = 0.0
        self.path_distance = 0.0
        self.initial_run = True
        self.path_index_timestamp = rospy.Time.now()
        
        
        self.config()
        
    
    
    def main(self):

        print("waiting for path")
        self.ur_path = rospy.wait_for_message("/ur_path", Path)
        print("got path")
        # compute distance between points
        self.compute_path_lengths()
        
        # move ur to start pose
        self.move_ur_to_start_pose(self.ur_path.poses[0].pose)
        print("UR in start pose")
        
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
            
            # transform error to base frame
            e_x_base = R[0,0] * e_x + R[0,1] * e_y + R[0,2] * e_z
            e_y_base = R[1,0] * e_x + R[1,1] * e_y + R[1,2] * e_z
            e_z_base = R[2,0] * e_x + R[2,1] * e_y + R[2,2] * e_z
            
            # compute direction of target pose
            target_pose_direction = math.atan2(target_pose.position.y - self.ur_path.poses[self.path_index-1].pose.position.y, target_pose.position.x - self.ur_path.poses[self.path_index-1].pose.position.x)
                        
            # calculate command
            self.ur_command.linear.x = e_x_base * self.Kpx + self.Kpffx * self.ur_target_velocity * math.sin(target_pose_direction)
            self.ur_command.linear.y = e_y_base * self.Kpy + self.kpffy * self.ur_target_velocity * math.cos(target_pose_direction)
            self.ur_command.linear.z = e_z_base * self.Kpz
            
            # limit velocity
            ur_command = self.limit_velocity(self.ur_command, self.ur_command_old)
            
            # set timestamp on initial run
            if self.initial_run:
                self.time_old = rospy.Time.now()
                self.initial_run = False
            
            # compute path speed
            path_speed = self.compute_path_speed_and_distance(ur_command)
            
            # increase path index periodically based on the target velocity
            time_per_point = self.path_distance_between_points / self.ur_target_velocity
            if abs(rospy.Time.now() - self.path_index_timestamp) > rospy.Duration(time_per_point):
                self.path_index += 1
                path_index_msg = Int32()
                path_index_msg.data = self.path_index
                self.path_index_publisher.publish(path_index_msg)
                self.path_index_timestamp = rospy.Time.now() 
                continue         
            
            self.broadcast_target_pose(target_pose)
            # check if next path point is reached
            if self.path_distance + path_speed > self.path_lengths[self.path_index]:
                self.path_index += 1
                path_index_msg = Int32()
                path_index_msg.data = self.path_index
                self.path_index_publisher.publish(path_index_msg)
                continue
            
            # publish command
            self.ur_command_old = ur_command
            self.ur_command_publisher.publish(ur_command)
            
            rate.sleep()
    
    def move_ur_to_start_pose(self, pose):
                
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
             # broadcast start pose
            self.broadcast_target_pose(pose)
            
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
    
    def compute_path_speed_and_distance(self,ur_command):
        now = rospy.Time.now()
        dt = (now - self.time_old).to_sec()
        path_speed = math.sqrt(ur_command.linear.x**2 + ur_command.linear.y**2 + ur_command.linear.z**2) 
        self.path_distance = self.path_distance + path_speed * dt
        self.time_old = now
        return path_speed * dt
       
    
    def path_index_callback(self, msg):
        print(msg.data)
        self.path_index = msg.data
    
    
    def compute_path_lengths(self):
        # compute path lengths
        self.path_lengths = [0.0]
        
        for i in range(len(self.ur_path.poses)-1):
            self.path_lengths.append(self.path_lengths[i] + math.sqrt((self.ur_path.poses[i+1].pose.position.x - self.ur_path.poses[i].pose.position.x)**2 + (self.ur_path.poses[i+1].pose.position.y - self.ur_path.poses[i].pose.position.y)**2 + (self.ur_path.poses[i+1].pose.position.z - self.ur_path.poses[i].pose.position.z)**2))
    
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
    Control_ur().main()