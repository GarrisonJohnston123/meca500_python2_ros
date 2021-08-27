#! /usr/bin/env python
"""
The Mecademic driver uses degrees for their joint positions and rviz requires radians. 
This scipt simiply takes the feedback from the mecademic, converts to radians, and publishes to a topic for rviz

Author: Garrison Johnston
""" 
import rospy
from sensor_msgs.msg import JointState
from numpy import pi

class MecademicRVIZBridge:
	def __init__(self):
		# Setup publishers and subscribers 
		self.joint_state_sub = rospy.Subscriber("/mecademic_joint_fb", JointState, self.joint_state_callback)
		self.joint_vizualizer_pub = rospy.Publisher("/joint_state_rviz", JointState, queue_size = 1)

		# Mecademic number of joints 
		self.joint_num = 6
		
		# Setup message to publish 
		self.rviz_msg = JointState()
		self.rviz_msg.position = [0]*self.joint_num 
		self.rviz_msg.name = ['meca_axis_1_joint', 'meca_axis_2_joint', 'meca_axis_3_joint', 'meca_axis_4_joint', 'meca_axis_5_joint', 'meca_axis_6_joint']

	def joint_state_callback(self,msg):
		for i in range(0,self.joint_num-1):
			self.rviz_msg.position[i]  = msg.position[i]*pi/180
		self.joint_vizualizer_pub.publish(self.rviz_msg)

if __name__ == '__main__':
	
	# Initialize node
	rospy.init_node('mecademic_rviz_bridge', anonymous = True)

	bridge = MecademicRVIZBridge()

	rospy.spin()

