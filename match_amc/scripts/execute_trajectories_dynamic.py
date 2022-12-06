#! /usr/bin/env python3

import rospy
import tf
from tf import transformations
from custom_path import MyTrajectory
from nav_msgs.msg import Path, Odometry
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose
from cartesian_controller import cartesian_controller
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from match_lib.match_robots import MirNav2Goal



class execute_trajectories_node():

    def __init__(self):
        rospy.init_node("lyapunov_controller_node")
        rospy.loginfo("lyapunov controller running")
        rospy.set_param("/mir_initialized",False)
        self.config()

    def run(self):
        rospy.loginfo(f"waiting for trajecories{len(self.target_trajectories)}")
        while len(self.target_trajectories) != self.number_of_robots and not rospy.is_shutdown():
            rospy.sleep(0.1)
                
        rospy.loginfo("all trajecories received")
        rate = rospy.Rate(self.control_rate)
        idx = 0

        # Move to initial pose
        # Because first goal of trajectory is not reached by cartesain formation controller
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
            
                rospy.logdebug(e_phi)

        rospy.set_param("/mir_initialized",True)
        rospy.loginfo("MiR initialized")

        ### wait for UR to continue ###
        ur_request = False
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and ur_request == False:
            ur_request = rospy.get_param("/ur_initialized", False)
            if ur_request == True:
                rospy.loginfo("UR initialized")
            else: 
                #rospy.loginfo("Waiting for UR to be initialized...")
                pass
            rate.sleep()
            
            
        #### Main loop #####  
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown() and idx < len(self.target_trajectories[0].v):
            for i in range(0,self.number_of_robots):
                actual_pose     = self.robot_poses[i]

                target_pose     = Pose()
                target_pose.position.x = self.target_trajectories[i].x[idx]
                target_pose.position.y = self.target_trajectories[i].y[idx]
                q = transformations.quaternion_from_euler(0,0,self.target_trajectories[i].phi[idx])
                target_pose.orientation.x = q[0]
                target_pose.orientation.y = q[1]
                target_pose.orientation.z = q[2]
                target_pose.orientation.w = q[3]

                target_velocity = Twist()
                target_velocity.linear.x = self.target_trajectories[i].v[idx] * self.control_rate
                target_velocity.angular.z = self.target_trajectories[i].w[idx] * self.control_rate

                u_v, u_w = cartesian_controller(actual_pose,target_pose,target_velocity)
                rospy.loginfo_throttle(1, "u_v: %f, u_w: %f", u_v, u_w)
                rospy.loginfo_throttle(1, "target velocity= " + str(target_velocity.linear.x))

                self.robot_command.linear.x = u_v
                self.robot_command.angular.z = u_w
                self.cmd_vel_publishers[i].publish(self.robot_command)

                self.target_pose_broadcaster(target_pose,i)
                # self.actual_pose_broadcaster(actual_pose,i)
                

            idx += 1
            rate.sleep()


    def trajectory_cb(self,Path,robot_index):
        trajectory = MyTrajectory()
        trajectory.x = []
        trajectory.y = []
        trajectory.phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            trajectory.x.append(Path.poses[i].pose.position.x)
            trajectory.y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            trajectory.phi.append(phi)
        
        trajectory.v = [0.0]
        trajectory.w = [0.0]
        for i in range(1,path_len-2):
            trajectory.v.append(math.sqrt((trajectory.x[i+1]-trajectory.x[i])**2 + (trajectory.y[i+1]-trajectory.y[i])**2 ))
            trajectory.w.append(trajectory.phi[i+1]-trajectory.phi[i])

        self.target_trajectories.append(trajectory)
        rospy.loginfo(f"trajectory {str(robot_index)} received")


    def robot_pose_cb(self,msg=PoseWithCovarianceStamped(),robot_index=0):
        self.robot_poses[robot_index] = msg.pose.pose


    def target_pose_broadcaster(self,target_pose,robot_id):
        frame_id = "robot" + str(robot_id) + "/target_pose"
        self.pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, 0),
                     (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w),
                     rospy.Time.now(), frame_id, "map")

    def actual_pose_broadcaster(self,actual_pose,robot_id):
            frame_id = "robot" + str(robot_id) + "/actual_pose"
            self.pose_broadcaster.sendTransform((actual_pose.position.x, actual_pose.position.y, 0),
                     (actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w),
                     rospy.Time.now(), frame_id, "map")
            
    def joint_states_cb(self, data):
        self.ur_base_angle = data.position[2]

    def config(self):
        rospy.Subscriber("joint_states", JointState, self.joint_states_cb)
        self.number_of_robots = rospy.get_param("~number_of_robots")
        self.control_rate = rospy.get_param("~control_rate")
        self.target_trajectories = []
        self.robot_poses = []
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

        for i in range(0,self.number_of_robots):
            param = "~robot" + str(i) + "_trajectory_topic"
            robotX_trajectory_topic = rospy.get_param(param)
            rospy.logdebug(f"robotX_trajectory_topic: {robotX_trajectory_topic}")
            rospy.Subscriber(robotX_trajectory_topic, Path, self.trajectory_cb, i)

            param = "~robot" + str(i) + "_pose_topic"
            robotX_pose_topic = rospy.get_param(param)
            self.robot_poses.append(Pose())
            rospy.Subscriber(robotX_pose_topic, Odometry, self.robot_pose_cb, i)

            param = "~robot" + str(i) + "_cmd_vel_topic"
            topic = rospy.get_param(param)
            self.cmd_vel_publishers.append(rospy.Publisher(topic,Twist,queue_size=5))

       



if __name__=="__main__":
    exe = execute_trajectories_node()
    exe.run()
