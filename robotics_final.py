#!/usr/bin/env python

# Chris Puglia, Team 1
# Mobile Robotics Final Project

# TurtleBot must have minimal.launch & gmapping.launch
# running prior to starting this script

import rospy
import sys
import actionlib
import random

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform
from tf2_msgs.msg import TFMessage
from tf import TransformListener
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import Sound

from math import sin,cos,degrees
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan

# global variables
bump = -1
action_duration = .5
movement_speed = 0.3
turn_speed = 0.6
min_turn_duration = 3.5
max_turn_duration = 5
NUM_NODES=6

#default for blank node
node=[{'navigatedTo':False,'located':False,'pos':None,'quat':None},
	{'navigatedTo':False,'located':False,'pos':None,'quat':None},
	{'navigatedTo':False,'located':False,'pos':None,'quat':None},
	{'navigatedTo':False,'located':False,'pos':None,'quat':None},
	{'navigatedTo':False,'located':False,'pos':None,'quat':None},
	{'navigatedTo':False,'located':False,'pos':None,'quat':None}]

def random_duration():

	# Calculates a random amount of time for the Turtlebot to turn for
	duration = min_turn_duration + random.random() * (max_turn_duration - min_turn_duration)
	str = "Random duration: %s"%duration
	rospy.loginfo(str)

	return duration

class GoToPose():

	def __init__(self):

		self.offset = .4
		self.tf = TransformListener()
		self.wander_msg = Twist()
		self.wander=False
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0
		self.ang = {0:0,001:-1.2,10:-1.2,11:-1.2,100:1.5,101:1.0,110:1.0,111:1.2}
		self.fwd = {0:.25,1:0,10:0,11:0,100:0.1,101:0,110:0,111:0}
		self.dbgmsg = {0:'Move forward',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}
		self.current_pos=None
		self.current_quat=None
		self.goal_sent = False
		self.node_matrix=node
		self.velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.sound_publisher = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)

		# What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)
		
		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		# Allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))   

	#Beep turtlebot
	def makeNoise(self):

		msg = Sound()
		msg.value = Sound.ON
		self.sound_publisher.publish(msg)

	#All of the laser roaming stuff
	def movement(self, sect1, sect2, sect3):

		'''Uses the information known about the obstacles to move robot.

		Parameters are class variables and are used to assign a value to
		variable sect and then  set the appropriate angular and linear 
		velocities, and log messages.
		These are published and the sect variables are reset.'''

		sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
		rospy.loginfo("Sect = " + str(sect)) 
		
		self.wander_msg.angular.z = self.ang[sect]
		self.wander_msg.linear.x = self.fwd[sect]
		rospy.loginfo(self.dbgmsg[sect])
		self.velocity_publisher.publish(self.wander_msg)

		self.reset_sect()
	
	def reset_sect(self):

		'''Resets the below variables before each new scan message is read'''
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0

	def laserCallback(self,scanmsg):

		 ##Passes laser scan message to for_callback function of sub_obj.
		if self.wander:
			self.for_callback(scanmsg)

	def for_callback(self,laserscan):

		'''Passes laserscan onto function sort which gives the sect 
		variables the proper values.  Then the movement function is run 
		with the class sect variables as parameters.

		Parameter laserscan is received from callback function.'''
		self.sort(laserscan)
		self.movement(self.sect_1, self.sect_2, self.sect_3)


	def sort(self, laserscan):
		'''Goes through 'ranges' array in laserscan message and determines 
		where obstacles are located. The class variables sect_1, sect_2, 
		and sect_3 are updated as either '0' (no obstacles within 0.7 m)
		or '1' (obstacles within 0.7 m)

		Parameter laserscan is a laserscan message.'''

		entries = len(laserscan.ranges)
		for entry in range(0,entries):
			if 0.4 < laserscan.ranges[entry] < 0.75:
				self.sect_1 = 1 if (0 < entry < entries/3) else 0 
				self.sect_2 = 1 if (entries/3 < entry < entries/2) else 0
				self.sect_3 = 1 if (entries/2 < entry < entries) else 0
		rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3))

	def reset_sect(self):
		'''Resets the below variables before each new scan message is read'''
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0

	#The bumper wandering stuff
	def wander():
		
		global bump

		twist = Twist()
		if bump==0:
			str = "right bumper, turning left %s"%rospy.get_time()
			rospy.loginfo(str)
			self.turn(random_duration(), turn_speed)
		elif bump==1:
			str = "left bumper, turning right %s"%rospy.get_time()
			rospy.loginfo(str)
			self.turn(random_duration(), -turn_speed)
		elif bump==2:
			str = "both bumpers, turning left %s"%rospy.get_time()
			rospy.loginfo(str)
			self.turn(random_duration(), turn_speed)
		else:
			str = "moving straight ahead %s"%rospy.get_time()
			#rospy.loginfo(str)
			twist.linear.x = movement_speed; twist.linear.y = 0; twist.linear.z = 0
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		bump = -1
		self.velocity_publisher.publish(twist)
		rospy.sleep(action_duration)

	def turn(duration, weight):

		twist = Twist()

		# First, back up slightly from the wall
		twist.linear.x = -movement_speed; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		self.velocity_publisher.publish(twist)
		rospy.sleep(action_duration)

		# Now, keep turning until the end of the specified duration
		currentTime = rospy.get_time();
		stopTime = rospy.get_time() + duration;
		while (rospy.get_time() < stopTime):
			 str = "turning %s"%rospy.get_time()
			 #rospy.loginfo(str)
			 twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
			 twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = weight
			 self.velocity_publisher.publish(twist)
			 rospy.sleep(action_duration)

	def twist(self,deg=360):
		vel_msg = Twist()
		PI = 3.1415926535897
		# Receiveing the user's input
		rospy.loginfo("Rotating In Place")
		speed = 25
		angle = deg
		clockwise = True #True or ru

		#Converting from angles to radians
		angular_speed = speed*2*PI/360
		relative_angle = angle*2*PI/360

		#We wont use linear components
		vel_msg.linear.x=0
		vel_msg.linear.y=0
		vel_msg.linear.z=0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0

		# Checking if our movement is CW or CCW
		if clockwise:
		   vel_msg.angular.z = -abs(angular_speed)
		else:
		  vel_msg.angular.z = abs(angular_speed)
		# Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while(current_angle < 2.5*relative_angle):
		   self.velocity_publisher.publish(vel_msg)
		   t1 = rospy.Time.now().to_sec()
		   current_angle = angular_speed*(t1-t0)


		#Forcing our robot to stop
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		rospy.loginfo("Done Rotating")

	#Callback for tf subscriber -- updates tag information
	def callback(self,data):

		val = data.transforms

		for i in val:

			cf_id = i.child_frame_id

			if 'tag_' in cf_id:

				num = int(cf_id[4:])

				#print "Found Tag # " + str(num)

				if self.node_matrix[num]['navigatedTo'] ==  False:

					self.node_matrix[num]['located'] = True
					self.node_matrix[num]['pos'],self.node_matrix[num]['quat'] = self.tf.lookupTransform("/map", cf_id, self.tf.getLatestCommonTime("/map", cf_id))

					rospy.loginfo("Found Tag" + str(num))

					yaw = euler_from_quaternion(self.node_matrix[num]['quat'])[2]

					self.node_matrix[num]['pos'][0] = self.node_matrix[num]['pos'][0] + sin(yaw)*self.offset
					self.node_matrix[num]['pos'][1] = self.node_matrix[num]['pos'][1] - cos(yaw)*self.offset

			if 'base_footprint' in cf_id:
				self.current_pos,self.current_quat=self.tf.lookupTransform("/map", "/base_link", self.tf.getLatestCommonTime("/map", "/base_link"))

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

	# listen (adapted from line_follower
	def processSensing(BumperEvent):
		 global bump
		 bump = BumperEvent.bumper
		 print bump
		 #newInfo = True

	def shutdown(self):
		if self.goal_sent:
			self.move_base.cancel_goal()
		rospy.loginfo("Stop")
		rospy.sleep(1)

if __name__ == '__main__':

	try:
		rospy.init_node('nav_test', anonymous=False)
		navigator = GoToPose()
		rospy.Subscriber('tf',TFMessage,navigator.callback)
		rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, navigator.processSensing)
		rospy.Subscriber('/scan', LaserScan, navigator.laserCallback)
		current_goal=0

		while current_goal<=NUM_NODES:

			while not(navigator.node_matrix[current_goal]['located']):

				#rospy.loginfo("Searching for: %s",current_goal)
				#navigator.wander()
				navigator.wander=True
				rospy.loginfo("Looking for " + str(current_goal))
				rospy.sleep(1)
				
			navigator.wander=False

			if navigator.node_matrix[current_goal]['located']:

				rospy.loginfo(str(navigator.node_matrix))
				print "Going to Tag " + str(current_goal)
				print "Tag Location: "

				# Customize the following values so they are appropriate for your location
				tagx = navigator.node_matrix[current_goal]['pos'][0]
				tagy = navigator.node_matrix[current_goal]['pos'][1]

				position = {'x': tagx, 'y' : tagy}

				print position

				#quaternion = navigator.node_matrix[current_goal]['quat']
				quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

				rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
				success = navigator.goto(position, quaternion)
				threshold=.2

				if success:
					rospy.loginfo("Hooray, reached %s",current_goal)
					navigator.makeNoise()
					##navigator.twist()
				else:
					rospy.loginfo("The base failed to reach the desired pose")
					#if abs(navigator.current_pos[0]-tagx)<threshold and abs(navigator.current_pos[0]-tagx)<threshold :
					navigator.makeNoise()

				current_goal = current_goal + 1

		rospy.loginfo("Program complete")

		# Sleep to give the last log messages time to be sent
		rospy.sleep(1)

	except rospy.ROSInterruptException:
		rospy.loginfo("Ctrl-C caught. Quitting")

