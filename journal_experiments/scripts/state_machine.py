#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from nav_msgs.msg import Path
import roslaunch
from tf import transformations
import math
from copy import deepcopy
from std_msgs.msg import String

# define global variables
path = Path();
active_robots = rospy.get_param("~active_robots", 1)
robot_names = rospy.get_param("~robot_names", ["mur620b"])
relative_positions_x = rospy.get_param("~relative_positions_x", [0])
relative_positions_y = rospy.get_param("~relative_positions_y", [0])
state = ""

# define state Parse_path
class Move_to_start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['formation_path_received'])
        
    def execute(self, userdata):
        formatin_plan_topic = rospy.get_param("/formation_plan_topic","/mir_path")
        rospy.loginfo('Waiting for path')
        state_publisher("idle")

        global path
        path = rospy.wait_for_message(formatin_plan_topic, Path)
        rospy.loginfo('formation path received')
        state_publisher("moving_to_start_pose")
        start_pose = path.poses[0].pose

        # teleport the robot away from the formation to avoid collision
        #teleport_robots_away(robot_names)

        for i in range(0,active_robots):
            # compute the target pose 
            target_pose = deepcopy(start_pose)
            theta = transformations.euler_from_quaternion([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])[2]
            #theta = math.atan2(path.poses[1].pose.position.y - path.poses[0].pose.position.y, path.poses[1].pose.position.x - path.poses[0].pose.position.x)
            target_pose.position.x += relative_positions_x[i] * math.cos(theta) - relative_positions_y[i] * math.sin(theta)
            target_pose.position.y += relative_positions_x[i] * math.sin(theta) + relative_positions_y[i] * math.cos(theta)
            target_pose_ = [target_pose.position.x, target_pose.position.y, theta]

            # teleport the robot to the start pose
            # process = launch_ros_node("teleport_to_start_pose","formation_controller","teleport_to_start_pose.py", "", "", target_pose=target_pose_, robot_name=robot_names[i])

            # # launch the move_to_start_pose node                
            process = launch_ros_node("move_to_start_pose","journal_experiments","move_to_start_pose.py", robot_names[i] + "/", "", target_pose=target_pose_)

            # wait for the node to finish
            while process.is_alive() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.loginfo(robot_names[i] + " in start pose")

        return 'formation_path_received'

def teleport_robots_away(robot_names):
    for i in range(0,len(robot_names)):
        storage_pose = [1000 + 2*i, 0, 0]
        process = launch_ros_node("teleport_to_start_pose","journal_experiments","teleport_to_start_pose.py", "", "", target_pose=storage_pose, robot_name=robot_names[i])

        # wait for the node to finish
        while process.is_alive() and not rospy.is_shutdown():
            rospy.sleep(0.1)

# define state Compute_trajectory
class Start_formation_controller(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_pose_reached', 'target_pose_not_reached'])

    def execute(self, userdata):
        state_publisher("moving_to_target_pose")

        path_array = []
        for pose in path.poses:
            theta = transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2]
            path_array.append([pose.pose.position.x, pose.pose.position.y , theta])

        # launch the formation_controller node
        process = launch_ros_node("control_mir","journal_experiments","control_mir.py", 
                                  "", "", path_array=path_array, active_robots=active_robots,
                                    robot_names=robot_names, relative_positions_x=relative_positions_x, relative_positions_y=relative_positions_y)

        # launch the ur controller node
        process = launch_ros_node("control_ur","journal_experiments","control_ur.py")

        while process.is_alive() and not rospy.is_shutdown():
                rospy.sleep(0.1)
                pass

        state_publisher("target_pose_reached")
        return 'target_pose_reached'
        

class Parse_path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['path_parsed'])

    def execute(self, userdata):
        rospy.loginfo('Parsing path')
        
        process = launch_ros_node("create_path","journal_experiments","create_path.py")
        
        return 'path_parsed'


class Move_UR_to_start_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ur_initialized'])
        

    def execute(self, userdata):
        rospy.set_param("/state_machine/move_ur_to_start_pose",True)
        rospy.loginfo('Executing state Move_UR_to_start_pose')
        while not rospy.get_param("/state_machine/ur_initialized") and not rospy.is_shutdown():
            rospy.sleep(0.1)
            pass
        return 'ur_initialized'

class Follow_trajectory(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        

    def execute(self, userdata):
        rospy.set_param("/state_machine/follow_trajectory",True)
        while not rospy.get_param("/state_machine/done") and not rospy.is_shutdown():
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


# main
def main():
    rospy.init_node('smach_example_state_machine')
    init_ros_param_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Parse_path', Parse_path(), 
                               transitions={'path_parsed':'Move_to_start'})
        smach.StateMachine.add('Move_to_start', Move_to_start(), 
                               transitions={'formation_path_received':'Start_formation_controller'})
        smach.StateMachine.add('Start_formation_controller', Start_formation_controller(), 
                               transitions={'target_pose_reached':'Move_to_start','target_pose_not_reached':'Parse_path'})
        smach.StateMachine.add('Move_UR_to_start_pose', Move_UR_to_start_pose(),
                                 transitions={'ur_initialized':'Follow_trajectory'})
        smach.StateMachine.add('Follow_trajectory', Follow_trajectory(),
                                transitions={'done':'outcome5'})
    # Execute SMACH plan

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

def state_publisher(state):
    state_pub = rospy.Publisher('/state_machine/state', String, queue_size=10)
    state_string = String()
    state_string.data = state
    state_pub.publish(state_string)


def init_ros_param_server():
    rospy.set_param("/state_machine/move_mir_to_start_pose",False)
    rospy.set_param("/state_machine/move_ur_to_start_pose",False)
    rospy.set_param("/state_machine/paths_received",False)
    rospy.set_param("/state_machine/trajectories_received",False)
    rospy.set_param("/state_machine/ur_trajectory_received",False)
    rospy.set_param("/state_machine/mir_trajectory_received",False)
    rospy.set_param("/state_machine/mir_initialized",False)
    rospy.set_param("/state_machine/ur_initialized",False)
    rospy.set_param("/state_machine/follow_trajectory",False)
    rospy.set_param("/state_machine/done",False)




if __name__ == '__main__':
    main()