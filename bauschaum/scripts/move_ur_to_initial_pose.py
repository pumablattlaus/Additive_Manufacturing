#! /usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist



class move_ur_to_initial_pose():
    
    def __init__(self):
        rospy.init_node("move_ur_to_initial_pose_node")
        rospy.Subscriber("/ur_path", Path, self.path_callback)
        self.ur_twist_publisher = rospy.Publisher("/ur/unsmooth_twist",Twist,queue_size=1)
        self.ur_path = Path()
        self.ur_twist = Twist()
        rospy.spin()
    
    
    
    def run(self):
        
        mir_in_initial_position = rospy.get_param("/mir_in_initial_position")
        while mir_in_initial_position == False and not rospy.is_shutdown():
            mir_in_initial_position = rospy.get_param("/mir_in_initial_position",False)
            rospy.sleep(1.0)
            print("waiting for mir in initial position")
        
        listener = tf.TransformListener()
        listener.waitForTransform("map", "tool0", rospy.Time(), rospy.Duration(4.0))
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('map', 'tool0', rospy.Time(0))
                #print(trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            e_x = self.ur_path.poses[0].pose.position.x - trans[0]
            e_y = self.ur_path.poses[0].pose.position.y - trans[1]
            e_z = self.ur_path.poses[0].pose.position.z - trans[2]
            
            print(e_x,e_y,e_z)
            u_x = e_x * 0.01
            
            self.ur_twist.linear.x = u_x
            
            self.ur_twist_publisher.publish(self.ur_twist)
            
    def path_callback(self,Path):
        self.ur_path = Path
        self.run()
        

if __name__=="__main__":
    exe = move_ur_to_initial_pose()
    