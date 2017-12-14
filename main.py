#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

## Colin Vincent
## Mobile Robotics 
## Final Project
## 12/11/2017

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
import sys
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform
from tf2_msgs.msg import TFMessage
from tf import TransformListener

NUM_NODES = 6
UPDATE_TAGS = False

tag_found = [False] * NUM_NODES
tag_info = [Transform()] * NUM_NODES

def tf_callback(data):

	if not(all(tag_found)) | UPDATE_TAGS:

		val = data.transforms

		for i in val:

			cf_id = i.child_frame_id

			if 'tag_' in cf_id:

				num = int(cf_id[4:])

				#print "Found Tag # " + str(num)

				if tag_found[num] ==  False | UPDATE_TAGS:

					tag_found[num] = True
					tag_info[num] = i.transform

					print tag_info[num]
	
class GoToPose():

	def __init__(self):

		self.goal_sent = False

		# What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)
		
		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		# Allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))

	def goto(self, pos, quat):

		# Send a goal
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
									 Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

		# Start moving
		self.move_base.send_goal(goal)

		# Allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

		state = self.move_base.get_state()
		result = False

		if success and state == GoalStatus.SUCCEEDED:
			# We made it!
			result = True
		else:
			self.move_base.cancel_goal()

		self.goal_sent = False
		return result

	def shutdown(self):

		if self.goal_sent:
			self.move_base.cancel_goal()
		rospy.loginfo("Stop")
		rospy.sleep(1)

if __name__ == '__main__':

	RUN = True

	try: 

		print "Initialized Node..."

		rospy.init_node('nav_test', anonymous=False)
		rospy.Subscriber('tf',TFMessage,tf_callback)

		print "Subscribed to TF..."

		print "Creating GoToPose..."

		navigator = GoToPose()

	except rospy.ROSInterruptException:
		rospy.loginfo("Crt-C caught. Quitting")
		RUN = False

	while RUN:
		try:

			tag_num = 1

			print "Pulling Tag Data..."

			#if tag_found[tag_num]:
			if True:

				print "Found Tag " + str(tag_num)

				tag_transform = tag_info[tag_num]
				tagx = tag_transform.translation.x
				tagy = tag_transform.translation.y
				#tagx = .15
				#tagy = .2

				position = {'x': tagx, 'y' : tagy}
				print position

				quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

				rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
				success = navigator.goto(position, quaternion)

				if success:
					rospy.loginfo("Hooray, reached the desired pose")
				else:
					rospy.loginfo("The base failed to reach the desired pose")

			else:
				print "No tag " + str(tag_num)

			# Sleep to give the last log messages time to be sent
			rospy.sleep(1)

		except rospy.ROSInterruptException:
			rospy.loginfo("Ctrl-C caught. Quitting")
			RUN = False

