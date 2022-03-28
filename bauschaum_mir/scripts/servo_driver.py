#! /usr/bin/env python

import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_msgs.msg import Int16
import subprocess

class servo_driver():

    def __init__(self):
        # give serial read/write permission 
        bashCommand = "sudo chmod 666 /dev/ttyUSB0"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        print(output)
        ###
        
        rospy.init_node("servo_driver_node")
        service_topic = "/dynamixel_workbench/dynamixel_command"
        rospy.wait_for_service(service_topic)
        print("Servo service found")
        rospy.sleep(0.1)
        self.servo_command_service = rospy.ServiceProxy(service_topic, DynamixelCommand)
        rospy.Subscriber("/servo_target_position", Int16, self.servo_cb)
        rospy.spin()
             
    def servo_cb(self,position_command):
        print(position_command)
        try:
            result = self.servo_command_service('"', 1,'Goal_Position',position_command.data) 
            print(result)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        
        
if __name__=="__main__":
    exe = servo_driver()