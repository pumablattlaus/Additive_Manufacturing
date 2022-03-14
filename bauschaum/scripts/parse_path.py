#! /usr/bin/env python3

from pathlib import Path
import rospy
from MIR_X import mir_x
from MIR_Y import mir_y

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

mir_x= mir_x()
mir_y= mir_y()

print(mir_y)

rospy.init_node("parse_path_node")
pub = rospy.Publisher("mir_path", Path, queue_size= 1)



mir_path = Path()
mir_path_point = PoseStamped()
mir_path.header.frame_id = "map"
mir_path.header.stamp = rospy.Time.now()



mir_path.poses = [PoseStamped() for i in range(len(mir_x))] 


for i in range(0,len(mir_x)): #len(robot0_xhat)
    mir_path_point.pose.position.x = mir_x[i]
    mir_path_point.pose.position.y = mir_y[i]


    mir_path.poses[i].pose.position.x = mir_path_point.pose.position.x
    mir_path.poses[i].pose.position.y = mir_path_point.pose.position.y


rospy.sleep(1)
pub.publish(mir_path)
rospy.sleep(1)

rospy.spin()
