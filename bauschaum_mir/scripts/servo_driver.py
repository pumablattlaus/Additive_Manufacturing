#! /usr/bin/env python3

import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_msgs.msg import Int16
import subprocess
from ur_dashboard_msgs.srv import GetSafetyMode

class servo_driver():

    def __init__(self):
        # give serial read/write permission 
        bashCommand = "sudo chmod 666 /dev/ttyUSB2"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        print(output)
        ###
        print("Starting servo driver")
        self.last_command = Int16()
        rospy.init_node("servo_driver_node")
        
        service_topic = "/dynamixel_workbench/dynamixel_command"
        rospy.wait_for_service(service_topic)
        print("Servo service found")
        rospy.sleep(0.1)
        self.servo_command_service = rospy.ServiceProxy(service_topic, DynamixelCommand)
        self.ur_safety_mode_service = rospy.ServiceProxy("/mur620c/UR10_r/ur_hardware_interface/dashboard/get_safety_mode", GetSafetyMode)
        
        rospy.Subscriber("/servo_target_position", Int16, self.servo_cb)
        
        #self.safety_loop()
        rospy.spin()
             
    def servo_cb(self,position_command):
        print(position_command)
        self.last_command = position_command
        try:
            result = self.servo_command_service('"', 1,'Goal_Position',position_command.data) 
            print(result)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            
    def safety_loop(self):
        while not rospy.is_shutdown():
            status = self.ur_safety_mode_service()

            if status.safety_mode.mode != 1 and self.last_command.data != 0:
                position_command = Int16()
                position_command.data = 0
                self.servo_cb(position_command)
            rospy.sleep(0.1)
        
        
if __name__=="__main__":
    exe = servo_driver()