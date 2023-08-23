#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-
# Publish the path from the topic /ur_path and /mir_path of the UR and MIR robot with less points
# to the topic /ur_path_pub and /mir_path_pub

import rospy
from nav_msgs.msg import Path

import math
import numpy as np

def ur_path_cb(path):
    ur_path_pub.publish(path_less_points(path))
    
def mir_path_cb(path):
    mir_path_pub.publish(path_less_points(path))
    
def path_less_points(path: Path, min_distance: float = 0.1):
    path_less_points = Path()
    path_less_points.header = path.header
    path_less_points.poses = [path.poses[0]]    # same start point
    for idx in range(1,len(path.poses)-1):
        if math.sqrt((path.poses[idx].pose.position.x - path_less_points.poses[-1].pose.position.x)**2
                     + (path.poses[idx].pose.position.y - path_less_points.poses[-1].pose.position.y)**2
                     + (path.poses[idx].pose.position.z - path_less_points.poses[-1].pose.position.z)**2) >= min_distance:
            path_less_points.poses.append(path.poses[idx])
    path_less_points.poses.append(path.poses[-1]) # same end point
    return path_less_points

if __name__ == "__main__":
    rospy.init_node("pub_paths_to_view")
    ur_path_pub = rospy.Publisher("/ur_path_less", Path, queue_size=1)
    mir_path_pub = rospy.Publisher("/mir_path_less", Path, queue_size=1)
    ur_path_sub = rospy.Subscriber("/ur_path", Path, ur_path_cb)
    mir_path_sub = rospy.Subscriber("/mir_path", Path, mir_path_cb)
    rospy.spin()