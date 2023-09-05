#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslaunch
from tf import transformations, TransformListener
import math
from copy import deepcopy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np

# In this state all the necessary variables are received from the parameter server and via topics and stored in userdata
class GetVars(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['variables_received'], input_keys=[],
                             output_keys=['mir_wait_pose', 'mir_path_print', 'ur_path_print', 'mir_pickup_pose', 'mir_dropoff_pose', 'mir_dropoff_idx'])
        
    def execute(self, userdata):
        formatin_plan_topic = rospy.get_param("/formation_plan_topic","/mir_path")
        mir_pickup_topic = rospy.get_param("dropper/pickup_pose_topic","/dropper/mir_pickup")
        mir_wait_topic = rospy.get_param("dropper/wait_pose_topic","/dropper/mir_wait")
        userdata.mir_dropoff_idx = rospy.get_param("dropper/dropoff_pose_idx",0)

        rospy.loginfo('Waiting for mir path')
        state_publisher("idle")

        # Commented FOR TESTING:
        # userdata.mir_path_print = rospy.wait_for_message(formatin_plan_topic, Path)
        # rospy.loginfo('mir path received')
        
        # rospy.loginfo('Waiting for ur path')
        # userdata.ur_path_print = rospy.wait_for_message("/ur_path", Path)
        # rospy.loginfo('ur path received')
        
        # Commented FOR TESTING:
        # userdata.mir_pickup_pose = rospy.wait_for_message(mir_pickup_topic, PoseStamped)
        # userdata.mir_dropoff_pose = userdata.mir_path_print.poses[mir_dropoff_idx]
        userdata.mir_wait_pose = rospy.wait_for_message(mir_wait_topic, PoseStamped)
        
        return 'variables_received'

# In this state the MiR-dropper waits for the printer to finish printing the object part before dropping the steel rod
class MiRWaitForPrinter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dropoff_pose_free'])
        
    def execute(self, userdata):
        state_publisher("waiting_for_printer")
        # wait for the printer to reach the dropoff pose/idx:
        # TODO: is current already idx published by other state machine?
        mir_dropoff_idx = userdata.mir_dropoff_idx

        print("goal_pose: ", pose)

        
        return 'variables_received'

# In this state the UR is moved to the first pose of the UR path
class UR_DropOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['object_dropped'])
        

    def execute(self, userdata):

        return 'object_dropped'


# In this state the formation controller and UR controller are run
class Start_formation_controller(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_pose_reached', 'target_pose_not_reached'])

    def execute(self, userdata):
        state_publisher("moving_to_target_pose")

        path_array = []
        for pose in mir_path_print.poses:
            theta = transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2]
            path_array.append([pose.pose.position.x, pose.pose.position.y , theta])

        # launch the formation_controller node
        process = launch_ros_node("control_mir","journal_experiments","control_mir.py", 
                                  "", "", path_array=path_array, active_robots=active_robots,
                                    robot_names=robot_names, relative_positions_x=relative_positions_x, relative_positions_y=relative_positions_y)

        # convert ur path to a list of poses 
        ur_path_array = []
        timestamps = []
        for pose in ur_path_print.poses:
            ur_path_array.append([pose.pose.position.x, pose.pose.position.y , pose.pose.position.z + 0.072, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            timestamps.append(pose.header.stamp.to_sec())
            if pose.header.stamp.to_sec() == 0.0:
                rospy.logwarn("timestamp = 0.0")
        
        # compute length of ur path and mir path to compare them
        mir_path_length = compute_path_length(mir_path_print)
        ur_path_length = compute_path_length(ur_path_print)
        length_factor = mir_path_length / ur_path_length

        # launch the ur controller node
        process = launch_ros_node("control_ur","journal_experiments","control_ur.py", "", "", ur_path_array=ur_path_array, length_factor=length_factor, mir_path_array = path_array, timestamps=timestamps)

        while process.is_alive() and not rospy.is_shutdown():
                rospy.sleep(0.1)
                pass

        state_publisher("target_pose_reached")
        return 'target_pose_reached'
        



# empty state
class Follow_trajectory(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        

    def execute(self, userdata):
        rospy.set_param("/state_machine_dropper/follow_trajectory",True)
        while not rospy.get_param("/state_machine_dropper/done") and not rospy.is_shutdown():
            rospy.sleep(0.1)
        pass
        return 'done'

def launch_ros_node(node_name, package_name, node_executable, namespace="/", node_args="", **params):
    # get param names from kwargs
    param_names = params.keys()
    # set params on param server
    for param_name in param_names:
        rospy.set_param(namespace + node_name + "/" + param_name, params[param_name])

    package = package_name
    executable = node_executable
    name = node_name
    node = roslaunch.core.Node(package=package, node_type=executable, name=name, namespace=namespace,
                                    machine_name=None, args=node_args, output="screen")
    
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    return process

def compute_path_length(path):
    path_length = 0.0
    for i in range(len(path.poses)-1):
        path_length += math.sqrt((path.poses[i+1].pose.position.x - path.poses[i].pose.position.x)**2 + (path.poses[i+1].pose.position.y - path.poses[i].pose.position.y)**2)
    return path_length

# main
def main():
    rospy.init_node('smach_example_state_machine_dropper')
    init_ros_param_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['sm_finished','sm_failed'])
    sm.userdata.mir_path_print = Path()
    sm.userdata.ur_path_print = Path()
    sm.userdata.mir_wait_pose = PoseStamped()
    sm.userdata.mir_pickup_pose = PoseStamped()
    sm.userdata.mir_dropoff_pose = PoseStamped()
    
    sm.userdata.active_robots = rospy.get_param("state_machine_dropper/active_robots", 1)
    sm.userdata.robot_names = rospy.get_param("state_machine_dropper/robot_names", ["mur620b"])
    sm.userdata.ur_prefixes = rospy.get_param("state_machine_dropper/ur_prefixes", ["UR10_l"])
    sm.userdata.relative_positions_x = rospy.get_param("state_machine_dropper/relative_positions_x", [0])
    sm.userdata.relative_positions_y = rospy.get_param("state_machine_dropper/relative_positions_y", [0])
    sm.userdata.state = ""

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GetVars', GetVars(), 
                               transitions={'variables_received':'MiRMoveToWaitPose'},
                               remapping={'mir_path_print':'mir_path_print', 
                                          'mir_wait_pose':'mir_wait_pose',
                                          'ur_path_print':'ur_path_print',
                                          'mir_pickup_pose':'mir_pickup_pose',
                                          'mir_dropoff_pose':'mir_dropoff_pose'})
        # smach.StateMachine.add('MiRMoveToWaitPose', MiRMoveToWaitPose(), 
        #                        transitions={'at_pose':'Move_UR_to_start_pose'},
        #                        remapping={'mir_wait_pose':'mir_wait_pose',
        #                                   'relative_positions_x':'relative_positions_x',
        #                                   'relative_positions_y':'relative_positions_y',
        #                                   'active_robots':'active_robots',
        #                                   'robot_names':'robot_names'})
        
        def create_move_base_goal(userdata, arg):
            goal = MoveBaseGoal()
            # goal.goal.target_pose.header.frame_id = "map"
            goal.target_pose = userdata.goal_pose
            return goal
        
        
        smach.StateMachine.add('MiRMoveToWaitPose', SimpleActionState('/mur620c/move_base',
                                                                      MoveBaseAction,
                                                                      goal_cb=create_move_base_goal,
                                                                    #   arg=sm.userdata.mir_wait_pose,
                                                                    #   result_slots=['max_effort', 
                                                                    #                 'position']),
                                                                      input_keys=['goal_pose']),
                               transitions={'succeeded':'MiRWaitForPrinter', 'preempted':'sm_failed', 'aborted':'sm_failed'},
                               remapping={'goal_pose':'mir_wait_pose'})
        
        smach.StateMachine.add('MiRWaitForPrinter', MiRWaitForPrinter(),
                               transitions={'dropoff_pose_free':'UR_DropOff'},
                               remapping={'mir_dropoff_idx':'mir_dropoff_idx'})
        
        smach.StateMachine.add('MiRMoveToDropPose', SimpleActionState('/mur620c/move_base',
                                                                      MoveBaseAction,
                                                                      goal_cb=create_move_base_goal,
                                                                    #   arg=sm.userdata.mir_wait_pose,
                                                                    #   result_slots=['max_effort', 
                                                                    #                 'position']),
                                                                      input_keys=['goal_pose']),
                               transitions={'succeeded':'sm_finished', 'preempted':'sm_failed', 'aborted':'sm_failed'},
                               remapping={'goal_pose':'mir_dropoff_pose'})
        
    # Execute SMACH plan

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

def state_publisher(state):
    state_pub = rospy.Publisher('/state_machine_dropper/state', String, queue_size=10)
    state_string = String()
    state_string.data = state
    state_pub.publish(state_string)


def init_ros_param_server():
    rospy.set_param("/state_machine_dropper/move_mir_to_start_pose",False)
    rospy.set_param("/state_machine_dropper/move_ur_to_start_pose",False)
    rospy.set_param("/state_machine_dropper/paths_received",False)
    rospy.set_param("/state_machine_dropper/trajectories_received",False)
    rospy.set_param("/state_machine_dropper/ur_trajectory_received",False)
    rospy.set_param("/state_machine_dropper/mir_trajectory_received",False)
    rospy.set_param("/state_machine_dropper/mir_initialized",False)
    rospy.set_param("/state_machine_dropper/ur_initialized",False)
    rospy.set_param("/state_machine_dropper/follow_trajectory",False)
    rospy.set_param("/state_machine_dropper/done",False)




if __name__ == '__main__':
    main()