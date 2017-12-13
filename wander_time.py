#!/usr/bin/env pythonpy

#import roslib; roslib.load_manifest('enclosure_escape')
import rospy
import random
from geometry_msgs.msg import Twist
#from turtlebot_node.msg import TurtlebotSensorState
from kobuki_msgs.msg import BumperEvent


# global variables
bump = -1

action_duration = .5
movement_speed = 0.3
turn_speed = 0.6
min_turn_duration = 3.5
max_turn_duration = 5

# listen (adapted from line_follower
def processSensing(BumperEvent):
     global bump
     bump = BumperEvent.bumper
     print bump
     #newInfo = True


def wander():
     pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
     rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, processSensing)
     rospy.init_node('wander')
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

def random_duration():
    # Calculates a random amount of time for the Turtlebot to turn for
    duration = min_turn_duration + random.random() * (max_turn_duration - min_turn_duration)
    str = "Random duration: %s"%duration
    rospy.loginfo(str)
    return duration

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


if __name__ == '__main__':
     try:
         wander()
     except rospy.ROSInterruptException: pass
