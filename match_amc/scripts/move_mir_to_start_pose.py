#!/usr/bin/env python3
import rospy
import tf
from tf import transformations
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist,  Pose
from custom_path import MyTrajectory
import math

from match_lib.match_robots import MirNav2Goal

class move_mir_to_start_pose():


    def __init__(self):
        rospy.init_node("move_mir_to_start_pose_node")
        self.config()

    def run(self):
        # wait for all trajectories to be received
        while not rospy.get_param("/move_mir_to_start_pose") and not rospy.is_shutdown():
            rospy.sleep(0.1)
                
        rospy.loginfo("all trajecories received")


        # Move to initial pose
        # Because first goal of self.target_trajectories is not reached by cartesain formation controller
        mir=MirNav2Goal("/mur216")
        first_pose = Pose()
        first_pose.position.x = self.target_trajectories[0].x[0]
        first_pose.position.y = self.target_trajectories[0].y[0]
        first_pose.orientation.w = 1.0
        mir.sendGoalPos(first_pose)
        while not rospy.is_shutdown():
            res = mir.is_ready()
            if res:
                break
            rospy.sleep(0.5)
        rospy.logdebug("MiR at goal")
           
        # set correct orientation
        rospy.loginfo("setting correct orientation")
        correct_orientation = 0
        while not rospy.is_shutdown() and correct_orientation < self.number_of_robots:
            for i in range(0,self.number_of_robots) :
                act_pose        = self.robot_poses[i]
                phi_actual = transformations.euler_from_quaternion([act_pose.orientation.x,act_pose.orientation.y,act_pose.orientation.z,act_pose.orientation.w])
                phi_target = self.target_trajectories[i].phi[0]
                e_phi = phi_target - phi_actual[2]

                self.robot_command.angular.z = self.K_phi * e_phi
                if abs(self.robot_command.angular.z) > self.limit_w:
                    self.robot_command.angular.z = self.robot_command.angular.z / abs(self.robot_command.angular.z) * self.limit_w
                if abs(e_phi) < self.target_threshhold_angular:
                    self.robot_command.angular.z = 0
                    correct_orientation += 1
                self.cmd_vel_publishers[i].publish(self.robot_command)
            
                rospy.loginfo("turnung",e_phi)


        rospy.set_param("/mir_initialized",True)
        rospy.loginfo("MiR initialized")

    def target_trajectories_cb(self,Path):
        self.target_trajectories = MyTrajectory()
        self.target_trajectories.x = []
        self.target_trajectories.y = []
        self.target_trajectories.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            self.target_trajectories.x.append(Path.poses[i].pose.position.x)
            self.target_trajectories.y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            self.target_trajectories.phi.append(phi)
        
        self.target_trajectories.v = [0.0]
        self.target_trajectories.w = [0.0]
        for i in range(1,path_len-2):
            self.target_trajectories.v.append(math.sqrt((self.target_trajectories.x[i+1]-self.target_trajectories.x[i])**2 + (self.target_trajectories.y[i+1]-self.target_trajectories.y[i])**2 ))
            self.target_trajectories.w.append(self.target_trajectories.phi[i+1]-self.target_trajectories.phi[i])

    def robot_pose_cb(self,msg=Odometry()):
        self.mir_pose = msg.pose.pose

    def config(self):
        self.control_rate = rospy.get_param("~control_rate")
        self.mir_target_trajectories_topic = rospy.get_param("~mir_target_trajectories_topic")
        self.mir_pose_topic = rospy.get_param("~mir_pose_topic")
        self.mir_cmd_vel_topic = rospy.get_param("~mir_cmd_vel_topic")
        self.target_trajectories = []
        self.mir_pose = Pose()
        self.cmd_vel_publishers = []
        self.robot_command = Twist()
        self.pose_broadcaster = tf.TransformBroadcaster()
        self.K_phi = 0.3
        self.K_d = 0.5
        self.limit_w = 0.3
        self.limit_x = 0.1
        self.target_threshhold_angular = 0.05
        self.target_threshhold_linear = 0.15
        self.filter_const = 0.1
        self.filter_const_vel = 1.0
        self.multiplicator = 1.0

        rospy.Subscriber(self.mir_target_trajectories_topic, Path, self.target_target_trajectories_cb)
        rospy.Subscriber(self.mir_pose_topic, Odometry, self.mir_pose_cb)
        self.cmd_vel_publisher = rospy.Publisher(self.mir_cmd_vel_topic, Twist, queue_size=1)


if __name__=="__main__":
    exe = move_mir_to_start_pose()
    rospy.sleep(1.0)
    exe.run()