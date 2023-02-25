#!/usr/bin/env python2

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point 

class navigation():

	def __init__(self):
		# declare the coordinates of interest 

		self.xRoom1 = -6.10
		self.yRoom1 = 3.10
		
		self.xRoom2 = -6.10 #same
		self.yRoom2 = -1.10 # - 4.2
		
		self.xRoom3 = -1.10 # + 5
		self.yRoom3 = 1.10 # + 2.2
		
		self.xRoom4 = 5.10 # + 6.2
		self.yRoom4 = 1.10 # same
		
		self.xRoom5 = 6.10 # + 1
		self.yRoom5 = -1.60 # - 2.7
		
		self.xRoom6 = 1.10 # - 5
		self.yRoom6 = 3.60 # + 5.2

		self.goalReached = False

		# initiliaze
        	rospy.init_node('navigation', anonymous=False)
		
		rospy.loginfo("Move to first room")
		self.moveToCoordinates(self.xRoom1, self.yRoom1)

		rospy.loginfo("Move to second room")
		self.moveToCoordinates(self.xRoom2, self.yRoom2)

		rospy.loginfo("Move to third room")
		self.moveToCoordinates(self.xRoom3, self.yRoom3)

		rospy.loginfo("Move to fourth room")
		self.moveToCoordinates(self.xRoom4, self.yRoom4)

		rospy.loginfo("Move to fifth room")
		self.moveToCoordinates(self.xRoom5, self.yRoom5)

		rospy.loginfo("Move to sixth room")
		self.moveToCoordinates(self.xRoom6, self.yRoom6)
		
		rospy.loginfo("All room found")		
		

	def moveToCoordinates(self, xCoord, yCoord):

		#define a client for to send goal requests to the move_base server through a SimpleActionClient
		ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		#wait for the action server to come up
		while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")

		goal = MoveBaseGoal()

		#set up the frame parameters
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()

		# moving towards the goal*/

		goal.target_pose.pose.position =  Point(xCoord,yCoord,0)
		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = 0.0
		goal.target_pose.pose.orientation.w = 1.0

		rospy.loginfo("Sending goal location ...")
		ac.send_goal(goal)

		ac.wait_for_result(rospy.Duration(60))

		if(ac.get_state() ==  GoalStatus.SUCCEEDED):
			rospy.loginfo("You have reached the destination")	
			return True
	
		else:
			rospy.loginfo("The robot failed to reach the destination")
			return False


if __name__ == '__main__':
    try:	
	rospy.loginfo("You have reached the destination")
        navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("navigation node terminated.")

