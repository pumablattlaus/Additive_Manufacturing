#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('dropper_publish_testdata')
    pub_wait = rospy.Publisher('/dropper/mir_wait', PoseStamped, queue_size=1, latch=True)
    pub_pickup = rospy.Publisher('/dropper/mir_pickup', PoseStamped, queue_size=1, latch=True)
    
    rospy.sleep(1.0)
    
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 0.0
    pose.pose.position.y = -3.0
    pose.pose.orientation.w = 1.0
    
    pub_pickup.publish(pose)
    
    pose.pose.position.x = 2.0
    pose.pose.position.y = -3.0
    pub_wait.publish(pose)
    
    rospy.sleep(1.0)
    
if __name__ == '__main__':
    main()