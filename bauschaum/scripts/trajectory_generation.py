#! /usr/bin/env python3

from math import atan2, sqrt, cos, sin
from mypath import Mypath
import rospy
from nav_msgs.msg import Path
from scipy.signal import savgol_filter
from matplotlib import pyplot as plt


class trajectory_generation():


    def __init__(self):
        rospy.init_node("trajectory_generation_node")
        rospy.Subscriber("mir_path", Path, self.mir_path_cb)
        self.config()
        
        self.target_reached = False
        self.mir_path = Mypath()
        self.mir_path.x = []
        self.mir_path.y = []
        self.mir_target_path = Mypath()
        self.mir_target_path.x = []
        self.mir_target_path.y = []
        self.mir_path.phi = []
        self.mir_path.v = [0.0]
        self.mir_path.w = [0.0]

        rospy.spin()

    def run(self):
        pass



        path_len = len(self.raw_mir_path.poses)
        for i in range(0,path_len):
            self.mir_path.x.append(self.raw_mir_path.poses[i].pose.position.x)
            self.mir_path.y.append(self.raw_mir_path.poses[i].pose.position.y)

        
        for i in range(0,path_len-1):
            phi = atan2(self.mir_path.y[i+1]-self.mir_path.y[i],self.mir_path.x[i+1]-self.mir_path.x[i])
            self.mir_path.phi.append(phi)

        for i in range(0,path_len-2):
            self.mir_path.v.append(sqrt((self.mir_path.x[i+1]-self.mir_path.x[i])**2 + (self.mir_path.y[i+1]-self.mir_path.y[i])**2 ))
            self.mir_path.w.append(self.mir_path.phi[i+1]-self.mir_path.phi[i])
        
        #print(self.mir_path.v)
        self.compute_trajectory()



    def compute_trajectory(self):
        
        index = 1
        current_velocity_lin = 0.0
        mir_current_velocity_lin = 0.0
        dist = 0.0
        mir_target_pose = Mypath()
        mir_target_pose.x = self.mir_path.x[0]
        mir_target_pose.y = self.mir_path.y[0]
        mir_path_distance = self.mir_path.v[0]
        mir_current_angle = self.mir_path.phi[0]

        while not rospy.is_shutdown() and self.target_reached == False:
            print("loop")
            # calculate remaining distance to the next control point
            #mir_dist_to_cp = sqrt((self.mir_path.y[index]-mir_target_pose.y)**2 + (self.mir_path.x[index]-mir_target_pose.x)**2)


            mir_acc_lin = self.target_vel_lin / self.control_rate - current_velocity_lin

            # limit acceleration
            if abs(mir_acc_lin) > self.acc_limit_lin:
                mir_acc_lin *= (abs(mir_acc_lin) / mir_acc_lin) * self.acc_limit_lin

            mir_current_velocity_lin += mir_acc_lin/self.control_rate


            if dist + mir_current_velocity_lin >= mir_path_distance:
                while dist + mir_current_velocity_lin >= mir_path_distance and not rospy.is_shutdown():
                    index += 1
                    if index > len(self.mir_path.v)-2:
                        self.target_reached = True
                        print("target reached")
                        break
                    else:
                        mir_path_distance += self.mir_path.v[index]


            robot0_dist_to_cp = sqrt((self.mir_path.y[index]-mir_target_pose.y)**2 + (self.mir_path.x[index]-mir_target_pose.x)**2)
            if robot0_dist_to_cp<mir_current_velocity_lin:
                index += 1
                #print("slow")
            if index >= len(self.mir_path.y):
                break

            mir_target_angle = atan2(self.mir_path.y[index]-mir_target_pose.y, self.mir_path.x[index]-mir_target_pose.x)

            mir_w_target=mir_target_angle-mir_current_angle

            if abs(mir_w_target) > self.w_limit:
                mir_w_target *= (abs(mir_w_target) / mir_w_target) * abs(self.w_limit)
            
            mir_current_angle += mir_w_target

            mir_target_pose.x = mir_target_pose.x + cos(mir_target_angle) * mir_current_velocity_lin
            mir_target_pose.y = mir_target_pose.y + sin(mir_target_angle) * mir_current_velocity_lin
            print(mir_target_pose.x,mir_target_pose.y,mir_current_velocity_lin,mir_target_angle)

            dist += mir_current_velocity_lin / self.control_rate
            self.mir_target_path.x.append(mir_target_pose.x)
            self.mir_target_path.y.append(mir_target_pose.y)


        print(self.mir_target_path.x)
        
        plt.figure()
        f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
        ax2.plot(self.mir_path.x,self.mir_path.y)
        ax1.plot(self.mir_target_path.x,self.mir_target_path.y)
        plt.show()

        
    def config(self):
        self.target_vel_lin = 0.1
        self.control_rate = 100
        self.acc_limit_lin = 0.5
        self.w_limit = 1.0



    def mir_path_cb(self,Path):
        self.raw_mir_path = Path
        self.run()
        


if __name__=="__main__":
    trajectory_generation()
    