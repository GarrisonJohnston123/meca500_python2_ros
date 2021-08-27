#!/usr/bin/env python

"""
This script shows a demo of using the robot with an xbox controller. 
You can use the direction pad to jog the ee in position while maintaining orientation.
The A button sends the robot home. 
Face the ee for correct mapping between controller and robot

Author: Garrison Johnston 
"""

import rospy
import pygame
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from numpy import pi

class JoystickDemo:
	"""
	Contstructor for the class
	""" 
	def __init__(self):
		# Setup communication with the xbox controller
		pygame.init()
		self.controller = pygame.joystick.Joystick(0)
		self.controller.init()
		self.hat_values = (0,0) 
		print "Detected ",self.controller.get_name()

		# Setup publisher object for desired robot joint velocities
		self.joint_state_pub = rospy.Publisher("/mecademic_joint_command", JointState, queue_size = 1)
		self.pose_pub = rospy.Publisher("/mecademic_pose_command", Pose, queue_size = 1)

		# Setup rospy rate object for controlling loop rate
		self.rate = rospy.Rate(10)

		# The number of joints in the mecademic
		self.joint_num = 6

		# Setup message to publish joint states 
		self.joint_msg = JointState()
		self.joint_msg.position = [0]*self.joint_num
		
		# The position of the ee when all joints = 0 deg
		self.home_position = [190, 0, 308]

		# Variable to store the desired ee position
		self.ee_position = list(self.home_position) # list function will copy list 

		# Ros message for the desired pose
		self.pose_msg = Pose()

		# The quaternion orientation of the ee when all joints = 0 deg. Will be constantly maintained.
		self.pose_msg.orientation.x = 0
		self.pose_msg.orientation.y = 90 #deg
		self.pose_msg.orientation.z = 0
		self.pose_msg.orientation.w = 0

		# The amount the ee will move for a single click of the direction pad.
		self.mm_per_click = 1

	"""
	Reads the state of the direction pad and updates the corresponding member variables
	Also, if the A button is pressed the robot is sent home
	"""
	def update_controller(self):
		pygame.event.get()		

		self.hat_values =  self.controller.get_hat(0)

		if self.controller.get_button(0) == 1:
			self.send_home()
			self.ee_position = list(self.home_position) # list function will copy list 
	
	"""
	Takes the reading from the direction pad, jogs the position of the ee, and publishes to the rostopic 
	"""
	def command_ee_pose(self):
		self.ee_position[1] += self.hat_values[0]*self.mm_per_click 
		self.ee_position[2] += self.hat_values[1]*self.mm_per_click 
		
		self.pose_msg.position.x = self.ee_position[0]
		self.pose_msg.position.y = self.ee_position[1]
		self.pose_msg.position.z = self.ee_position[2]
		self.pose_pub.publish(self.pose_msg)

	"""
	This sends the robot to the home configuration. Gets called in update_controller if A button is pressed
	"""
	def send_home(self):
		print 'Sending Robot Home...'
		self.joint_msg.position = [0]*self.joint_num
		self.joint_state_pub.publish(self.joint_msg)
		rospy.sleep(3)
		print "Homed"

	"""
	Main loop. updates controller, commands the ee, and pauses for the desired loop rate
	"""
	def main(self):
		while not rospy.is_shutdown():
			self.update_controller()
			self.command_ee_pose()
			self.rate.sleep()

if __name__ == '__main__':
	# Initialize node
	rospy.init_node('joystick_demo', anonymous = True)

	joystick = JoystickDemo()
	joystick.main()
