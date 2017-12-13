## Colin Vincent
## Mobile Robotics 
## Final Project
## 12/11/2017

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform
from tf2_msgs.msg import TFMessage
from tf import TransformListener

class TF_Listener():

	def __init__(self,NUM_NODES=6,UPDATE_TAGS=False):

		self.NUM_NODES = 6
		self.UPDATE_TAGS = False

		self.tag_found = [False] * self.NUM_NODES
		self.tag_info = [Transform()] * self.NUM_NODES

	def callback(self,data):

		if not(all(self.tag_found)) | self.UPDATE_TAGS:

			val = data.transforms

			for i in val:

				cf_id = i.child_frame_id

				if 'tag_' in cf_id:

					num = int(cf_id[4:])

					#print "Found Tag # " + str(num)

					if self.tag_found[num] ==  False | self.UPDATE_TAGS:

						self.tag_found[num] = True
						self.tag_info[num] = i.transform

						#print tag_info[num]

	def listener(self):

		print 'running listener...'

		rospy.init_node('custom_listener',anonymous=True)
		rospy.Subscriber('tf',TFMessage,self.callback)

		print "Subscribed"

		#rospy.spin()


