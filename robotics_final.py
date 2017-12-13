#!/usr/bin/env python

# Chris Puglia, Team 1
# Mobile Robotics Final Project

# TurtleBot must have minimal.launch & 3d.launch & gmapping.launch
# running prior to starting this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import sys
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform
from tf2_msgs.msg import TFMessage
from tf import TransformListener
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent


# global variables
bump = -1

action_duration = .5
movement_speed = 0.3
turn_speed = 0.6
min_turn_duration = 3.5
max_turn_duration = 5

#default for blank node
node=[{'navigatedTo':False,'located':False,'pos':None,'quat':None},
    {'navigatedTo':False,'located':False,'pos':None,'quat':None},
    {'navigatedTo':False,'located':False,'pos':None,'quat':None},
    {'navigatedTo':False,'located':False,'pos':None,'quat':None},
    {'navigatedTo':False,'located':False,'pos':None,'quat':None},
    {'navigatedTo':False,'located':False,'pos':None,'quat':None}]

NUM_NODES=6

class GoToPose():
    def __init__(self):

        self.tf = TransformListener()

        self.goal_sent = False
        self.node_matrix=node
        self.velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))   
    def random_duration():
        # Calculates a random amount of time for the Turtlebot to turn for
        duration = min_turn_duration + random.random() * (max_turn_duration - min_turn_duration)
        str = "Random duration: %s"%duration
        rospy.loginfo(str)
        return duration
    def wander():
        pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
        #listen
        global bump


        twist = Twist()
        while not rospy.is_shutdown():
            if bump==0:
                str = "right bumper, turning left %s"%rospy.get_time()
                rospy.loginfo(str)
                turn(pub, random_duration(), turn_speed)
            elif bump==1:
                str = "left bumper, turning right %s"%rospy.get_time()
                rospy.loginfo(str)
                turn(pub, random_duration(), -turn_speed)
            elif bump==2:
                str = "both bumpers, turning left %s"%rospy.get_time()
                rospy.loginfo(str)
                turn(pub, random_duration(), turn_speed)
            else:
                str = "moving straight ahead %s"%rospy.get_time()
                #rospy.loginfo(str)
                twist.linear.x = movement_speed; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
         bump = -1
         pub.publish(twist)
         rospy.sleep(action_duration)
    def turn(pub, duration, weight):
        twist = Twist()

        # First, back up slightly from the wall
        twist.linear.x = -movement_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(action_duration)

        # Now, keep turning until the end of the specified duration
        currentTime = rospy.get_time();
        stopTime = rospy.get_time() + duration;
        while (rospy.get_time() < stopTime):
             str = "turning %s"%rospy.get_time()
             #rospy.loginfo(str)
             twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
             twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = weight
             pub.publish(twist)
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

                    #rospy.loginfo("Found Tag" + str(num))


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
     
        current_goal=0

        while current_goal<=NUM_NODES:

            while not(navigator.node_matrix[current_goal]['located']):
                #rospy.loginfo("Searching for: %s",current_goal)
                
                # TODO: implement ROAM
                # ROAM
                #navigator.twist(180)
                #rospy.loginfo(str(navigator.node_matrix))
                rospy.loginfo("Looking for " + str(current_goal))
                rospy.sleep(5)

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

                if success:
                    rospy.loginfo("Hooray, reached %s",current_goal)
                    navigator.twist()
                else:
                    rospy.loginfo("The base failed to reach the desired pose")

                current_goal = current_goal + 1

        rospy.loginfo("Program complete")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

