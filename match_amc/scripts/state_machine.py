#!/usr/bin/env python3

import rospy
import smach
from nav_msgs.msg import Path

# define state Parse_path
class Parse_path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['paths_received','trajectories_received'])
        rospy.set_param("/state_machine/move_mir_to_start_pose",False)

    def execute(self, userdata):
        rospy.loginfo('Executing state Parse_path')
        rospy.wait_for_message('mir_path', Path)
        rospy.loginfo('mir_path received')
        rospy.wait_for_message('ur_path', Path)
        rospy.loginfo('ur_path received')

        return 'paths_received'


# define state Compute_trajectory
class Compute_trajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trajectories_received'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Compute_trajectory')
        # rospy.wait_for_message('mir_trajectory', Path)
        # rospy.loginfo('mir_trajectory received')
        rospy.wait_for_message('ur_trajectory', Path)
        rospy.loginfo('ur_trajectory received')
        return 'trajectories_received'
        

class Move_MiR_to_start_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])
        rospy.set_param("/state_machine/move_mir_to_start_pose",True)

    def execute(self, userdata):
        rospy.loginfo('Executing state Move_MiR_to_start_pose')
        return 'outcome3'




# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Parse_path', Parse_path(), 
                               transitions={'paths_received':'Compute_trajectory', 
                                            'trajectories_received':'outcome4'})
        smach.StateMachine.add('Compute_trajectory', Compute_trajectory(), 
                               transitions={'trajectories_received':'Move_MiR_to_start_pose'})
        smach.StateMachine.add('Move_MiR_to_start_pose', Move_MiR_to_start_pose(), 
                               transitions={'outcome3':'Parse_path'})

    # Execute SMACH plan
    outcome = sm.execute()











if __name__ == '__main__':
    main()