#!/usr/bin/python 

import rospy
import numpy as np 
from sensor_msgs.msg import JointState
from std_msgs.msg import Char
from std_msgs.msg import Float64
from hand_simulator.srv import MoveServos
from std_srvs.srv import Empty, EmptyResponse
import tty, termios, sys
import math

class keyboard_control():
    print("Enter Keybord_class")
    def __init__(self):
        self.instration()
        rospy.init_node('MoveCamera', anonymous=True)
        rospy.loginfo("KeyBoard Initialising...")
        joint_states_topic_name = "/lab_config/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.joint_callback)
        self.pub = rospy.Publisher('/lab_config/camera_position_controller/command', Float64, queue_size=1)
        joints_data = None
        while joints_data is None:
            try:
                joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass

        self.joint_dictionary = dict(zip(joints_data.name, joints_data.position))

        while not rospy.is_shutdown():
            ch = self.getchar()
            # print ch
            if ch == '1': # clock
                # print("check corent values",self.joint_dictionary)
                for i in self.joint_dictionary:
                    # print(i,self.joint_dictionary[i])
                    # print(self.joint_dictionary[i])
                    new_camera_pose = self.joint_dictionary[i] + 0.2
                    # print(new_camera_pose)
                    self.movement_loop(new_camera_pose)
                    
            if ch == '2': # Unclock
                for i in self.joint_dictionary:
                    # print(i,self.joint_dictionary[i])
                    # print(self.joint_dictionary[i])
                    new_camera_pose = self.joint_dictionary[i] - 0.2
                    # print(new_camera_pose)
                    self.movement_loop(new_camera_pose)

            if ch == '0': # Unclock
                    break
        

    def instration(self):
        print("To Move the camera Press")
        print("1 to move clockwise rotation")
        print("2 to move  unclockwise rotation")
        return 0
    def joint_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.joint_dictionary = dict(zip(msg.name, msg.position))
    
    def assertAlmostEqualAngles(self, x, y,):
        c2 = (math.sin(x) - math.sin(y)) ** 2 + (math.cos(x) - math.cos(y)) ** 2
        angle_diff = math.acos((2.0 - c2) / 2.0)
        return angle_diff

    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * math.pi
        # print("complete_rev\n", complete_rev)
        mod_angle = int(angle / complete_rev)
        # print("mod_angle\n", mod_angle)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * math.pi

        return clean_angle

    def check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param joint_name:
        :param value:
        :param error: In radians
        :return:
        """
        joint_reading = self.joint_dictionary.get(joint_name)
        # print("joint_reading\n", joint_reading)
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        #print("clean_joint_reading\n", clean_joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)
        #print("clean_value\n", clean_value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error
        print(similar, "similar")

        return similar
    def move_all_joints(self,camera_next_pose):
        self.pub.publish(camera_next_pose)
    def joint_value_convert(self, new_joint_pose):
        check_rate = 5.0
        camera_next_pose = new_joint_pose

        check_joint_value = False
        # print(type(check_joint_value), "type check_joint_value")
        rate = rospy.Rate(check_rate)
        while not check_joint_value:
            # print(type(check_joint_value), "INSAYD type check_joint_value")
            self.move_all_joints(camera_next_pose)
            # print("here")
            check_joint_value = self.check_continuous_joint_value(joint_name=
                                                                                         "rail_to_camera",
                                                                                         value=
                                                                                         camera_next_pose)
            # print (check_joint_value, "check_joint_value")
            rate.sleep()

    def movement_loop(self, new_joint_pose):
        rospy.loginfo("Start Moving palm_tree... Big Excitement")
        self.joint_value_convert(new_joint_pose)
        
    def getchar(self):
        #Returns a single character from standard input
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

if __name__ == '__main__':

    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass

    
