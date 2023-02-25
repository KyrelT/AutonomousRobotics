#!/usr/bin/env python2

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point 

class navigation():
	def __init__(self):
		# Initialise the boolean variables
		self.goalStarted = False
		self.same_navigation = False

		#define a client for to send goal requests to the move_base server through a SimpleActionClient
		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		#wait for the action server to come up
		while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")

		self.goal = MoveBaseGoal()

		#set up the frame parameters
		self.goal.target_pose.header.frame_id = "map"
		self.goal.target_pose.header.stamp = rospy.Time.now()


	# Navigate to the X and Y coordinate on the map
	def moveToCoordinates(self, xCoord, yCoord):

		# moving towards the goal*/
		self.goal.target_pose.pose.position =  Point(xCoord,yCoord,0)
		self.goal.target_pose.pose.orientation.x = 0.0
		self.goal.target_pose.pose.orientation.y = 0.0
		self.goal.target_pose.pose.orientation.z = 0.0
		self.goal.target_pose.pose.orientation.w = 1.0

		rospy.loginfo("Sending goal location ...")
		self.ac.send_goal(self.goal)




if __name__ == '__main__':
    try:	
	rospy.loginfo("You have reached the destination")
        navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("navigation node terminated.")


