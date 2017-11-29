#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import actionlib
import numpy as np
import threading
import rospy

from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty, String
from tf import TransformListener
from acoustic_magic_msgs.msg import SoundSourceAngle

AUDIO_TOPIC = "/cogrob/microphone_direction"
AUDIO_TRACKED_TOPIC = "/cogrob/point_head"
MARKER_TOPIC = "/cogrob/microphone_direction_marker"
ACTION_CLIENT = "head_controller/point_head"

# temporal median filter length
FILTER_LENGTH = 20
# donwsample buffer length
BUFFER_LENGTH = 30


def PolToCart(rho, phi):
	"""Convert Pol to Cart, returns a tuple of (x, y)"""
	x = rho * np.cos(phi)
	y = rho * np.sin(phi)
	rospy.loginfo("x: %f, y: %f", x, y)
	return(x, y)


def TemporalMedian(new_value, history, length):
	"""A temporal median filter"""
	if history.size == 0:
		history = np.hstack((new_value, history))
		m = np.median(history)
		return (m, history)
	elif history.size < length:
		history = np.hstack((new_value, history))
		m = np.median(history)
		return (m, history)
	else:
		history = np.roll(history, 1)
		history[0] = new_value
		m = np.median(history)
		return (m, history)


def DownSample(new_value, buf):
	"""Down sample the filtered input stream, take the majority in the buffer"""
	buf = np.roll(buf, 1)
	buf[0] = new_value
	rospy.loginfo(buf)
	return buf

def GetMode(buf):
	"""Get the mode of an array of numbers"""
	u, indices = np.unique(buf, return_inverse=True)
	return u[np.argmax(np.bincount(indices))]

class Follower():

	def __init__(self, *args):
		self.tf = TransformListener()
		self.client = actionlib.SimpleActionClient(
			ACTION_CLIENT, PointHeadAction)
		self.client.wait_for_server()
		self.person_tracked_pub = rospy.Publisher(
			AUDIO_TRACKED_TOPIC, PointStamped, queue_size=10)
		self.marker = rospy.Publisher(
			MARKER_TOPIC, Marker, queue_size=10)

		self.angle = np.zeros(1)
		self.buf = np.zeros(BUFFER_LENGTH)
		self.counter = 0

		self.ignore_inputs = False
		self.initializtion = True
		self.lock = threading.Lock()
		# self.last_trigger_time = rospy.Time.now()
		rospy.Subscriber(AUDIO_TOPIC, SoundSourceAngle, self.AudioCallback)
		rospy.spin()

	def AudioCallback(self, msg):
		self.lock.acquire()
		if self.initializtion:
			self.initializtion = False
		elif msg.is_valid and not self.ignore_inputs:
			point = PointStamped()
			marker = Marker()
			start_point = Point()
			end_point = Point()
			point.header.stamp = rospy.Time.now()
			point.header.frame_id = 'head_pan_link'
			marker.header.stamp = rospy.Time.now()
			marker.header.frame_id = 'head_pan_link'
			direction = 0

			#rospy.loginfo("Raw angles: %d", msg.angle)
			#rospy.loginfo(type(msg.angle))

			(direction, self.angle) = TemporalMedian(
				msg.angle, self.angle, FILTER_LENGTH)
			rospy.loginfo(self.angle)

			self.buf = DownSample(direction, self.buf)
			
			if self.counter == BUFFER_LENGTH:
				counter = 0
				self.buf = self.buf.astype (int)
				direction = GetMode(self.buf)
				rospy.loginfo("Filtered angles: %d", direction)

				(x, y) = PolToCart(3.0, (direction / 180.0) * np.pi)

				point.point.x = x
				point.point.y = y
				point.point.z = 0.0

				start_point.x = 0.0
				start_point.y = 0.0
				start_point.z = 0.0
				end_point.x = x
				end_point.y = y
				end_point.z = 0.0

				# define the marker parameters
				# marker is used to illustrate the source direction in RVIZ
				marker.type = marker.ARROW
				marker.action = marker.ADD
				marker.scale.x = 0.1 #shaft width
				marker.scale.y = 0.2 #arrow head width
				marker.scale.z = 1.0
				marker.color.a = 1.0
				marker.pose.orientation.w = 1.0
				marker.points.append(start_point)
				marker.points.append(end_point)
				self.person_tracked_pub.publish(point)
				# Publish so we can see the marker
				self.marker.publish(marker)
				self.SetPointHead(point)
			else:
				self.counter = self.counter + 1
				rospy.loginfo("Counter: %d", self.counter)
		self.lock.release()

	def SetPointHead(self, point):
		goal = PointHeadGoal()
		corrected_point = self.tf.transformPoint("head_pan_link", point)
		rospy.loginfo(corrected_point)
		goal.target = corrected_point
		goal.min_duration = rospy.Duration(.5)
		goal.max_velocity = 1

		# send the goal to the server
		self.ignore_inputs = True
		self.client.send_goal(goal)	
		# block and wait for the turn to be finished
		# doing this because there are a lot of noises when turning the head
		# ignore the input during that period by blocking here
		result = self.client.wait_for_result(rospy.Duration(0.0))
		rospy.loginfo("Result:")
		rospy.loginfo(result)
		rospy.sleep(2)
		self.ignore_inputs = False
		
		# re-initialized history array, buffer array and counter
		self.angle = np.array([])
		self.buf = np.zeros(BUFFER_LENGTH)
		self.counter = 0

		# self.last_trigger_time = rospy.Time.now()

if __name__ == '__main__':
	rospy.init_node('acoustic_magic_looker')
	rospy.loginfo("Program started")
	try:
		node_instance = Follower()
	except rospy.ROSInterruptException:
		pass
