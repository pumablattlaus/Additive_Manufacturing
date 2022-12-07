#!/usr/bin/env python3
import rospy
import tf
from tf import transformations
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist,  Pose, PoseStamped
from custom_path import MyTrajectory
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
import actionlib

class move_mir_to_start_pose():


    def __init__(self):
        rospy.init_node("move_mir_to_start_pose_node")
        self.config()
        self.mir_start_pose = PoseStamped()

    def run(self):
        # wait for all trajectories to be received
        while not rospy.get_param("/state_machine/move_mir_to_start_pose") and not rospy.is_shutdown():
            rospy.sleep(0.1)
            rospy.loginfo_throttle(3,"waiting for all trajectories to be received")
                

        rospy.sleep(1)
        # Move to initial pose
        # Because first goal of self.target_trajectory is not reached by cartesain formation controller
        self.mir_start_pose.header.frame_id = "map"
        self.mir_start_pose.pose.position.x = self.target_trajectory.x[0]
        self.mir_start_pose.pose.position.y = self.target_trajectory.y[0]
        q = transformations.quaternion_from_euler(0,0,self.target_trajectory.phi[0])
        self.mir_start_pose.pose.orientation.x = q[0]
        self.mir_start_pose.pose.orientation.y = q[1]
        self.mir_start_pose.pose.orientation.z = q[2]
        self.mir_start_pose.pose.orientation.w = q[3]
        while not rospy.is_shutdown():
            self.move_to_goal(self.mir_start_pose)
            rospy.sleep(0.1)
            if self.reached_goal == True:
                break
            else:
                rospy.logerr("could not reach initial pose. Retrying...")  

        # wait for mir to reach goal
        while not rospy.is_shutdown():
            distance_to_goal = math.sqrt((self.mir_pose.position.x - self.target_trajectory.x[0])**2 + (self.mir_pose.position.y - self.target_trajectory.y[0])**2)
            if distance_to_goal < 0.1:
                rospy.loginfo("mir reached initial pose")
                break
            rospy.sleep(0.1)
           
        # set correct orientation
        rospy.loginfo("setting correct orientation")
        while not rospy.is_shutdown():
            act_pose        = self.mir_pose
            phi_actual = transformations.euler_from_quaternion([act_pose.orientation.x,act_pose.orientation.y,act_pose.orientation.z,act_pose.orientation.w])
            phi_target = self.target_trajectory.phi[0]
            e_phi = phi_target - phi_actual[2]

            if abs(e_phi) > math.pi:
                e_phi = e_phi - math.copysign(2*math.pi,e_phi)

            self.robot_command.angular.z = self.K_phi * e_phi
            if abs(self.robot_command.angular.z) > self.limit_w:
                self.robot_command.angular.z = self.robot_command.angular.z / abs(self.robot_command.angular.z) * self.limit_w
            if abs(e_phi) < self.target_threshhold_angular:
                self.robot_command.angular.z = 0
                break
            self.cmd_vel_publisher.publish(self.robot_command)
        
            rospy.loginfo("turning")


        rospy.set_param("/mir_initialized",True)
        rospy.loginfo("MiR initialized")

    def target_trajectory_cb(self,Path):
        rospy.logerr("target_trajectory_cb")
        self.target_trajectory = MyTrajectory()
        self.target_trajectory.x = []
        self.target_trajectory.y = []
        self.target_trajectory.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            self.target_trajectory.x.append(Path.poses[i].pose.position.x)
            self.target_trajectory.y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            self.target_trajectory.phi.append(phi)
        
        self.target_trajectory.v = [0.0]
        self.target_trajectory.w = [0.0]
        for i in range(1,path_len-2):
            self.target_trajectory.v.append(math.sqrt((self.target_trajectory.x[i+1]-self.target_trajectory.x[i])**2 + (self.target_trajectory.y[i+1]-self.target_trajectory.y[i])**2 ))
            self.target_trajectory.w.append(self.target_trajectory.phi[i+1]-self.target_trajectory.phi[i])

    def mir_pose_cb(self,msg=Odometry()):
        self.mir_pose = msg.pose.pose

    def move_to_goal(self,pose=PoseStamped()):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move base server")
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose.header.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose.pose

        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()


    def config(self):
        self.control_rate = rospy.get_param("~control_rate")
        self.mir_target_trajectory_topic = rospy.get_param("~mir_target_trajectory_topic")
        self.mir_pose_topic = rospy.get_param("~mir_pose_topic")
        self.mir_cmd_vel_topic = rospy.get_param("~mir_cmd_vel_topic")
        self.move_base_simple_goal_topic = rospy.get_param("~move_base_simple_goal_topic")
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

        rospy.Subscriber(self.mir_target_trajectory_topic, Path, self.target_trajectory_cb)
        rospy.Subscriber(self.mir_pose_topic, Odometry, self.mir_pose_cb)
        self.cmd_vel_publisher = rospy.Publisher(self.mir_cmd_vel_topic, Twist, queue_size=1)
        self.simple_goal_publisher = rospy.Publisher(self.move_base_simple_goal_topic, PoseStamped, queue_size=1)





if __name__=="__main__":
    exe = move_mir_to_start_pose()
    rospy.sleep(1.0)
    exe.run()