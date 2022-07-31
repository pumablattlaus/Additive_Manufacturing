#! /usr/bin/env python3

from pathlib import Path
import rospy
from MIR_X import mir_x
from MIR_Y import mir_y

from Wall_X import wall_x
from Wall_Y import wall_y
from Wall_Z import wall_z

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

mir_x= mir_x()
mir_y= mir_y()

wall_x = wall_x()
wall_y = wall_y()
wall_z = wall_z()

#print(mir_y)

rospy.init_node("parse_path_node")
mir_pub = rospy.Publisher("mir_path", Path, queue_size= 1)
ur_pub = rospy.Publisher("ur_path", Path, queue_size= 1)


mir_path = Path()
mir_path_point = PoseStamped()
mir_path.header.frame_id = "map"
mir_path.header.stamp = rospy.Time.now()
ur_path = Path()
ur_path_point = PoseStamped()
ur_path.header.frame_id = "map"
ur_path.header.stamp = rospy.Time.now()


mir_path.poses = [PoseStamped() for i in range(len(mir_x))] 
ur_path.poses = [PoseStamped() for i in range(len(wall_x))] 


for i in range(0,len(mir_x)): #len(robot0_xhat)
    mir_path_point.pose.position.x = mir_x[i]
    mir_path_point.pose.position.y = mir_y[i]


    mir_path.poses[i].pose.position.x = mir_path_point.pose.position.x
    mir_path.poses[i].pose.position.y = mir_path_point.pose.position.y

for i in range(0,len(wall_x)): #len(robot0_xhat)
    ur_path_point.pose.position.x = wall_x[i]
    ur_path_point.pose.position.y = wall_y[i]
    ur_path_point.pose.position.z = wall_z[i]

    ur_path.poses[i].pose.position.x = ur_path_point.pose.position.x
    ur_path.poses[i].pose.position.y = ur_path_point.pose.position.y
    ur_path.poses[i].pose.position.z = ur_path_point.pose.position.z

print(ur_path)

rospy.sleep(1)
mir_pub.publish(mir_path)
rospy.sleep(2)
ur_pub.publish(ur_path)
rospy.sleep(1)

rospy.spin()
