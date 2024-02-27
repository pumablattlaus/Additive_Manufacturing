#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf import transformations
from tf.broadcaster import TransformBroadcaster
import math


class ComputeNextTarget():

    def config(self):
        self.control_rate = 100
        self.ur_target_velocity = 0.1
        self.mir_target_velocity = 0.1
        self.smoothing_factor = 0.0

    def __init__(self):
        rospy.init_node("compute_next_target_node")
        self.config()
        self.mir_path_sub = rospy.Subscriber("/mir_path", Path, self.mir_path_callback)
        self.ur_path_sub = rospy.Subscriber("/ur_path", Path, self.ur_path_callback)
        
        self.mir_target_path = Path()
        self.ur_target_path = Path()
        self.current_mir_pose = PoseStamped()
        self.current_ur_pose = PoseStamped()
        self.interpolated_ur_target = PoseStamped()
        self.interpolated_mir_target = PoseStamped()

        self.mir_path_index = 0
        self.ur_path_index = 0

        self.ur_distance_travelled = 0.0
        self.mir_distance_travelled = 0.0
        self.progress_old = 0.0
        self.ur_mir_distance_old = math.inf


        self.tf_broadcaster = TransformBroadcaster()

        self.run()
        rospy.spin()


    def run(self):
        # wait for mir and ur path
        rospy.loginfo("Waiting for mir and ur path")
        rospy.wait_for_message("/mir_path", Path)
        rospy.wait_for_message("/ur_path", Path)
        rospy.loginfo("Received mir and ur path")

        # check length of mir and ur path
        self.check_path_length()

        # set current mir and ur pose to first pose in path
        self.current_mir_pose = self.mir_target_path.poses[0]
        self.current_ur_pose = self.ur_target_path.poses[0]

        # compute distance travelled by mir and ur
        self.ur_path_distances = self.compute_distance_travelled(self.ur_target_path)
        self.mir_path_distances = self.compute_distance_travelled(self.mir_target_path)

        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self.compute_next_target()
            rate.sleep()


    def compute_next_target(self):
        # get mir and ur target based on path index
        self.get_target_from_path()

        # check if ur target is reached in this iteration and update ur path index
        self.check_target_reached()

        # interpolate between current and next target
        self.interpolate_target()

        # compute mir velocity to reach next target
        self.compute_mir_velocity()

        # compute distance between mir and ur
        self.compute_distance_between_mir_and_ur()

        # broadcast next target
        #self.broadcast_next_target()
        
        # broadcast interpolated target
        self.broadcast_interpolated_target()

        # update distance travelled by ur and mir
        self.update_distance_travelled()

    def compute_distance_between_mir_and_ur(self):
        distance = math.sqrt((self.interpolated_ur_target.pose.position.x - self.interpolated_mir_target.pose.position.x)**2 + (self.interpolated_ur_target.pose.position.y - self.interpolated_mir_target.pose.position.y)**2)
        print("distance between mir and ur: " + str(distance))

        # check if mir is moving 
        if self.mir_target_velocity == 0.0:
            # keep the current position of mir as long as the distance between mir and ur is decreasing
            if distance < self.ur_mir_distance_old:
                pass
            else:
                rospy.loginfo("Distance between mir and ur is increasing")
                self.mir_target_velocity = self.ur_target_velocity

            self.ur_mir_distance_old = distance

            rospy.loginfo("Distance between mir and ur is less than threshold")


    def interpolate_target(self):
        last_ur_target = self.ur_target_path.poses[self.ur_path_index-1] if self.ur_path_index > 0 else self.ur_target_path.poses[self.ur_path_index]
        last_mir_target = self.mir_target_path.poses[self.mir_path_index-1] if self.mir_path_index > 0 else self.mir_target_path.poses[self.mir_path_index]

        self.progress = 1 - (self.ur_path_distances[self.ur_path_index] - (self.ur_distance_travelled +self.ur_target_velocity * (1+self.smoothing_factor) / self.control_rate)) / (self.ur_path_distances[self.ur_path_index] - self.ur_path_distances[self.ur_path_index-1])
        
        self.interpolated_ur_target.pose.position.x = last_ur_target.pose.position.x + (self.ur_target.pose.position.x - last_ur_target.pose.position.x) * self.progress
        self.interpolated_ur_target.pose.position.y = last_ur_target.pose.position.y + (self.ur_target.pose.position.y - last_ur_target.pose.position.y) * self.progress
        self.interpolated_ur_target.pose.position.z = last_ur_target.pose.position.z + (self.ur_target.pose.position.z - last_ur_target.pose.position.z) * self.progress
        self.interpolated_ur_target.pose.orientation = self.ur_target.pose.orientation

        self.interpolated_mir_target.pose.position.x = last_mir_target.pose.position.x + (self.mir_target.pose.position.x - last_mir_target.pose.position.x) * self.progress
        self.interpolated_mir_target.pose.position.y = last_mir_target.pose.position.y + (self.mir_target.pose.position.y - last_mir_target.pose.position.y) * self.progress
        
        # interpolate orientation
        last_orientation = transformations.euler_from_quaternion([last_ur_target.pose.orientation.x, last_ur_target.pose.orientation.y, last_ur_target.pose.orientation.z, last_ur_target.pose.orientation.w])[2]
        target_orientation = transformations.euler_from_quaternion([self.ur_target.pose.orientation.x, self.ur_target.pose.orientation.y, self.ur_target.pose.orientation.z, self.ur_target.pose.orientation.w])[2]
        q = transformations.quaternion_from_euler(0, 0, last_orientation + (target_orientation - last_orientation) * self.progress)
        self.interpolated_mir_target.pose.orientation.x = q[0]
        self.interpolated_mir_target.pose.orientation.y = q[1]
        self.interpolated_mir_target.pose.orientation.z = q[2]
        self.interpolated_mir_target.pose.orientation.w = q[3]

    def broadcast_interpolated_target(self):
        self.tf_broadcaster.sendTransform((self.interpolated_ur_target.pose.position.x, self.interpolated_ur_target.pose.position.y, self.interpolated_ur_target.pose.position.z),
                                        (self.interpolated_ur_target.pose.orientation.x, self.interpolated_ur_target.pose.orientation.y, self.interpolated_ur_target.pose.orientation.z, self.interpolated_ur_target.pose.orientation.w),
                                        rospy.Time.now(),
                                        "interpolated_ur_target",
                                        "map")
        self.tf_broadcaster.sendTransform((self.interpolated_mir_target.pose.position.x, self.interpolated_mir_target.pose.position.y, self.interpolated_mir_target.pose.position.z),
                                        (self.interpolated_mir_target.pose.orientation.x, self.interpolated_mir_target.pose.orientation.y, self.interpolated_mir_target.pose.orientation.z, self.interpolated_mir_target.pose.orientation.w),
                                        rospy.Time.now(),
                                        "interpolated_mir_target",
                                        "map")


    def update_distance_travelled(self):
        self.mir_distance_travelled += self.mir_target_velocity / self.control_rate
        self.ur_distance_travelled += self.ur_target_velocity / self.control_rate


    def compute_distance_travelled(self, path):
        distance = []
        for i in range(len(path.poses)-1):
            distance.append(math.sqrt((path.poses[i].pose.position.x - path.poses[i+1].pose.position.x)**2 + (path.poses[i].pose.position.y - path.poses[i+1].pose.position.y)**2) + distance[i-1] if i > 0 else 0.0)
        return distance


    def check_target_reached(self):
        if self.ur_path_distances[self.ur_path_index] <= self.ur_distance_travelled + self.ur_target_velocity * (1+self.smoothing_factor) / self.control_rate:
            self.ur_path_index += 1
            self.get_target_from_path()
        
        if self.mir_path_distances[self.mir_path_index] <= self.mir_distance_travelled + self.mir_target_velocity * (1+self.smoothing_factor) / self.control_rate:       
            self.mir_path_index += 1
            self.get_target_from_path()

        # print("mir path index: " + str(self.mir_path_index))
        # print("mir distance travelled: " + str(self.mir_distance_travelled))
            

    def get_target_from_path(self):
        self.mir_target = self.mir_target_path.poses[self.mir_path_index]
        self.ur_target = self.ur_target_path.poses[self.ur_path_index]

    def compute_mir_velocity(self):
        # compute change in progress
        delta_progress = self.progress - self.progress_old
        self.progress_old = self.progress

        # detect jump in delta progress when path index is updated
        if delta_progress < 0:
            delta_progress = 1 + delta_progress

        # distance to travel in this iteration
        distance_to_travel = self.mir_path_distances[self.mir_path_index] - self.mir_path_distances[self.mir_path_index-1] 

        # compute mir velocity
        self.mir_target_velocity = distance_to_travel * delta_progress * self.control_rate

        #print("mir target velocity: " + str(self.mir_target_velocity))

    
    def check_path_length(self):
        if len(self.mir_target_path.poses) != len(self.ur_target_path.poses):
            rospy.logerr("Length of mir and ur path do not match")
            return
        else:
            rospy.loginfo("Length of mir and ur path match and is: " + str(len(self.mir_target_path.poses)))

    def broadcast_next_target(self):
        self.tf_broadcaster.sendTransform((self.ur_target.pose.position.x, self.ur_target.pose.position.y, self.ur_target.pose.position.z),
                                        (self.ur_target.pose.orientation.x, self.ur_target.pose.orientation.y, self.ur_target.pose.orientation.z, self.ur_target.pose.orientation.w),
                                        rospy.Time.now(),
                                        "next_ur_target",
                                        "map")
        self.tf_broadcaster.sendTransform((self.mir_target.pose.position.x, self.mir_target.pose.position.y, self.mir_target.pose.position.z),
                                        (self.mir_target.pose.orientation.x, self.mir_target.pose.orientation.y, self.mir_target.pose.orientation.z, self.mir_target.pose.orientation.w),
                                        rospy.Time.now(),
                                        "next_mir_target",
                                        "map")

    def mir_path_callback(self, path):
        self.mir_target_path = path

    def ur_path_callback(self, path):
        self.ur_target_path = path

if __name__ == '__main__':
    ComputeNextTarget()