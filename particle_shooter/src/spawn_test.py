#!/usr/bin/env python3
from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates



class SpawnModel_test():


    def __init__(self):
        rospy.init_node('insert_object',log_level=rospy.INFO)
        self.spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        import tf
        self.tcp_pose = Pose()
        self.model_states = ModelStates()
        self.timestamp_old = rospy.Time.now()
        self.i = 0
        self.to_be_deleted = []
        self.listener = tf.TransformListener()
        rospy.Subscriber("/mur216/ground_truth",Odometry,self.ground_truth_callback)
        rospy.Subscriber("/gazebo/model_states",ModelStates,self.model_states_callback)
        rospy.sleep(1)

        
        self.run()

    def run(self):
        
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            (trans,rot) = self.listener.lookupTransform('mur216/base_link', "/mur216/UR16/tool0",  rospy.Time(0))


            R = transformations.quaternion_matrix([self.mir_pose.orientation.x,self.mir_pose.orientation.y,self.mir_pose.orientation.z,self.mir_pose.orientation.w])


            self.tcp_pose.position.x = self.mir_pose.position.x + R[0][0] * trans[0] + R[0][1] * trans[1]
            self.tcp_pose.position.y = self.mir_pose.position.y + R[1][0] * trans[0] + R[1][1] * trans[1]
            self.tcp_pose.position.z = trans[2]

            model_name = 'test'+str(self.i)

            self.spawn_model_client(
            model_name=model_name,
                model_xml=open('/usr/share/gazebo-11/models/particle_sphere_fluid/model.sdf', 'r').read(),
                robot_namespace='/foo',
                initial_pose=self.tcp_pose,
                reference_frame='world'
            )
            self.i += 1

            if self.i > 20:
                model_name = 'test'+str(self.i-20)
                #delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                #delete_model(model_name)
                self.to_be_deleted.append(model_name)



    

            rate.sleep()

    def model_states_callback(self,data = ModelStates()):
        now = rospy.Time.now()

        if now - self.timestamp_old > rospy.Duration(1):
            #print(self.to_be_deleted)
            model_states = data

            for i in range(0,len(self.to_be_deleted)):
                if self.to_be_deleted[i] in model_states.name:
                    model_name = self.to_be_deleted[i]
                    self.to_be_deleted.pop(i)
                    pose = data.pose.pop(model_states.name.index(model_name))
                    print(pose)
                    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                    delete_model(model_name)

                    self.spawn_model_client(
                    model_name=model_name,
                    model_xml=open('/usr/share/gazebo-11/models/particle_sphere_solid/model.sdf', 'r').read(),
                    robot_namespace='/foo',
                    initial_pose=pose,
                    reference_frame='world'
                    )
            self.timestamp_old = now



    def ground_truth_callback(self,data = Odometry()):
        self.mir_pose = data.pose.pose



if __name__=="__main__":
    SpawnModel_test()
    