#!/usr/bin/env python3

import sys, time
import rospy
import kortex_driver.msg as msg
# import pygame

import kortex_driver.srv as server
from kortex_driver.srv import *
from kortex_driver.msg import *
from std_msgs.msg import String

class kinovo_keyboard():
    def __init__(self):
            
        try:
            # rospy.init_node('example_cartesian_poses_with_notifications_python')
            rospy.init_node('keyboard_controller_node', anonymous=True)

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            print('im here 1')
            # # Init the services
            # clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            # rospy.wait_for_service(clear_faults_full_name)
            # self.clear_faults = rospy.x(clear_faults_full_name, Base_ClearFaults)

            # print('im here 2')

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            # print('im here 3')
            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            # print('im here 4')
            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)


        ## velocity publisher 
            self.pub_arm = rospy.Publisher(
                name = '/my_gen3/in/cartesian_velocity', 
                data_class= msg.TwistCommand, 
                queue_size= 10
            )

            self.twist = msg.TwistCommand()
            self.twist.duration = 0
            self.twist.reference_frame = 0

        
        ## clearing faults
            self.clear_faults_pub = rospy.Publisher(
                name = '/my_gen3/in/clear_faults', 
                data_class= msg.Empty, 
                queue_size= 10
            )

        ## Stopper
            self.stop_topic = rospy.Publisher(
                name = '/my_gen3/in/stop', 
                data_class= msg.Empty, 
                queue_size= 10
            )
        
        ## subscibing to keyboard_commands node 
            rospy.Subscriber("keyboard_commands", String, self.keyboard_callback)

            print('subscribed to keyboard')



        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
        
        self.vel = 0.5
        self.a_vel = 0.008

    def clear_faults_2(self):
        try:
            self.clear_faults()
            
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                print(' i m sleeping')
                print(self.last_action_notif_type)
                time.sleep(0.01)

    def homer(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                print('try home')
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                print('im in else of home')
                return self.wait_for_action_end_or_abort()
            

    def keyboard_callback(self,data):
        key = data.data
        
        vel = self.vel
        a_vel = self.a_vel
        # print(key)
        if key == '':
            
            self.twist_c_list = [0, 0, 0, 0, 0, 0]
            self.movement(self.twist_c_list)

        if key == 'u':
            self.twist_c_list = [0, 0, 0, 0, 0, a_vel]
            self.movement(self.twist_c_list)
        
        if key == 'h':

            self.twist_c_list = [0, 0, 0, 0, 0, -a_vel]
            self.movement(self.twist_c_list)
        
        if key == 'i':
            self.twist_c_list = [0, 0, 0, 0, a_vel, 0]
            self.movement(self.twist_c_list)
        
        if key == 'k':
            self.twist_c_list = [0, 0, 0, 0, -a_vel, 0]
            self.movement(self.twist_c_list)

        if key == 'j':
            self.twist_c_list = [0, 0, 0, a_vel, 0, 0]
            self.movement(self.twist_c_list)
        
        if key == "l":
            self.twist_c_list = [0, 0, 0, -a_vel, 0, 0]
            self.movement(self.twist_c_list)
        ##-----------------------------------------------------------##
        
        if key == 'r':
            self.twist_c_list = [0, 0, vel, 0, 0, 0]
            self.movement(self.twist_c_list)
        
        if key == "f":
            self.twist_c_list = [0, 0, -vel, 0, 0, 0]
            self.movement(self.twist_c_list)

        if key == 'w':
            print('i m here')
            self.twist_c_list = [0, vel, 0, 0, 0, 0]
            self.movement(self.twist_c_list)
        
        if key == "s":
            self.twist_c_list = [0, -vel, 0, 0, 0, 0]
            self.movement(self.twist_c_list)
        
        if key == 'a':
            self.twist_c_list = [vel, 0, 0, 0, 0, 0]
            self.movement(self.twist_c_list)
        
        if key == "d":
            self.twist_c_list = [-vel, 0, 0, 0, 0, 0]
            self.movement(self.twist_c_list)

        if key == "c":
            print("clearing faults")
            self.clear_faults_pub.publish(msg.Empty())

        if key == "0":
            print("sending to home")
            self.homer()

        if key == "m":
            self.twist.reference_frame += 1
            self.twist.reference_frame  = self.twist.reference_frame % 3
            print(f'Changed frame to {self.twist.reference_frame}!')

        if key == '[A': ## up arrow for increasing the translation velocity 
            self.a_vel = round(self.a_vel + 0.001,5)
            print(self.a_vel)
        
        if key == '[B': ## down arrow for decreasing the translation velocity 
            self.a_vel = round(self.a_vel - 0.001,5)
            print(self.a_vel)

    def movement(self,twist_list:list):  ## [angular,linear]

        self.twist.twist.angular_x = twist_list[0]
        self.twist.twist.angular_y = twist_list[1]
        self.twist.twist.angular_z = twist_list[2]

        self.twist.twist.linear_x  = twist_list[3]
        self.twist.twist.linear_y  = twist_list[4]
        self.twist.twist.linear_z  = twist_list[5]

        self.pub_arm.publish(self.twist)
        

## frame 2 and 0 is moving along the camera 
if __name__ == "__main__":
    vel = kinovo_keyboard()
    while not rospy.is_shutdown():
	    rospy.spin()