#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist
import tf
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from std_msgs.msg import Int32

class Control_ur_helper():

    def __init__(self,base_node):
        self.base_node = base_node
        self.setup_ddynamic_reconfigure()

        self.base_node.ur_command_topic = rospy.get_param("~ur_command_topic", "/mur620c/UR10_r/twist_controller/command_safe")
        self.base_node.ur_pose_topic = rospy.get_param("~ur_pose_topic", "/mur620c/UR10_r/ur_calibrated_pose")
        self.base_node.mir_pose_topic = rospy.get_param("~mir_pose_topic", "/mur620c/mir/robot_pose")
        self.base_node.ur_base_link_frame_id = rospy.get_param("~ur_base_link_frame_id", "mur620c/UR10_r/base_link")
        self.base_node.mir_cmd_vel_topic = rospy.get_param("~mir_cmd_vel_topic", "/mur620c/mir/cmd_vel")
        self.base_node.tf_prefix = rospy.get_param("~tf_prefix", "mur620c/")
        self.base_node.ur_twist_publisher = rospy.Publisher(self.base_node.ur_command_topic, Twist, queue_size=1)
        self.base_node.ur_target_pose_broadcaster = tf.TransformBroadcaster()
        self.base_node.initial_run = True

        self.base_node.path_index_publisher = rospy.Publisher('/path_index', Int32, queue_size=1)
        self.base_node.mir_target_velocity_publisher = rospy.Publisher('/mir_target_velocity', Twist, queue_size=1)
        self.base_node.ur_command = Twist()
        self.base_node.ur_command_old = Twist()
        self.base_node.path_index = 1
        self.base_node.path_speed = 0.0
        self.base_node.path_distance = 0.0
        
        self.base_node.path_index_timestamp = rospy.Time.now()

        rospy.Subscriber(self.base_node.ur_pose_topic, PoseStamped, self.ur_pose_callback)
        rospy.Subscriber(self.base_node.mir_cmd_vel_topic, Twist, self.mir_cmd_vel_callback)
        rospy.Subscriber(self.base_node.mir_pose_topic, Pose, self.mir_pose_callback)

    def ur_pose_callback(self, data = PoseStamped()):
        self.base_node.ur_pose = data.pose
        
    def mir_cmd_vel_callback(self, msg = Twist()):
        self.base_node.mir_cmd_vel = msg
        
    def mir_pose_callback(self, msg = Pose()):
        self.base_node.mir_pose = msg

    def dyn_rec_callback(self,config, level):
        self.base_node.ur_target_velocity = config["ur_target_velocity"]
        self.base_node.ur_velocity_limit = config["ur_velocity_limit"]
        self.base_node.ur_acceleration_limit = config["ur_acceleration_limit"]
        self.base_node.Kpx = config["Kpx"]
        self.base_node.Kpy = config["Kpy"]
        self.base_node.Kpz = config["Kpz"]
        self.base_node.Kp_mir = config["Kp_mir"]
        self.base_node.Kp_ffx = config["Kp_ffx"]
        self.base_node.Kp_ffy = config["Kp_ffy"]
        return config
    
    def setup_ddynamic_reconfigure(self):
        # Create a D(ynamic)DynamicReconfigure
        ddynrec = DDynamicReconfigure("example_dyn_rec")

        # Add variables (name, description, default value, min, max, edit_method)
        ddynrec.add_variable("ur_target_velocity", "float/double variable", 0.05, 0, 0.3)
        ddynrec.add_variable("ur_velocity_limit", "float/double variable", 0.15, 0, 0.3)
        ddynrec.add_variable("ur_acceleration_limit", "float/double variable", 0.6, 0, 2.0)
        ddynrec.add_variable("Kpx", "float/double variable", -0.5, -1.0, 1.0)
        ddynrec.add_variable("Kpy", "float/double variable", -0.5, -1.0, 1.0)
        ddynrec.add_variable("Kpz", "float/double variable", 0.35, -1.0, 1.0)
        ddynrec.add_variable("Kp_mir", "float/double variable", 0.1, -1.0, 1.0)
        ddynrec.add_variable("Kp_ffx", "float/double variable", 0.0, -1.0, 1.0)
        ddynrec.add_variable("Kp_ffy", "float/double variable", 0.0, -1.0, 1.0)
        # ddynrec.add_variable("decimal", "float/double variable", -0.4, -1.0, 1.0)
        # ddynrec.add_variable("integer", "integer variable", 0, -1, 1)
        # ddynrec.add_variable("bool", "bool variable", True)
        # ddynrec.add_variable("string", "string variable", "string dynamic variable")
        # enum_method = ddynrec.enum([ ddynrec.const("Small",      "int", 0, "A small constant"),
        #                 ddynrec.const("Medium",     "int", 1, "A medium constant"),
        #                 ddynrec.const("Large",      "int", 2, "A large constant"),
        #                 ddynrec.const("ExtraLarge", "int", 3, "An extra large constant")],
        #                 "An enum example")
        # ddynrec.add_variable("enumerate", "enumerate variable", 1, 0, 3, edit_method=enum_method)

        # Start the server
        ddynrec.start(self.dyn_rec_callback)
