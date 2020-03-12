#!/usr/bin/env python
import rospy
import math
import time
import os
import pickle
from std_msgs.msg import Int16,UInt8
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import tf



class Steering():

	def getCurrentPose(self, data,  event = None):
		#position|
		self.currentPose = data.pose.pose
		#orientation
		q = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
		[_,_,self.theta] = tf.transformations.euler_from_quaternion(q)
		if self.observations > 0 and self.start:
			#for positives
			if self.expected_theta - self.theta_error <= self.theta <= self.expected_theta + self.theta_error:
				self.x1=self.currentPose.position.x
				self.y1=self.currentPose.position.y
				print "1"
				print self.x1,self.y1
			#for negatives
			if -self.expected_theta - self.theta_error <= self.theta <= -self.expected_theta + self.theta_error:
				self.x2=self.currentPose.position.x
				self.y2=self.currentPose.position.y
				print "2"
				print self.x2,self.y2
				self.get_radius()

	def get_radius(self):

		diameter = math.sqrt((self.x2-self.x1)**2 + (self.y2-self.y1)**2)
		self.data.append(diameter)
		self.observations-=1
		if self.observations <=0:
			print "Task done"
			with open("steering_radius.txt", "wb") as fp:
				pickle.dump(self.data, fp)


	def prepare_configurations(self, theta=math.pi/2, observations = 30, event = None):

		#preparing steering configuration
		print ("Loading steer configuration"),
		rospy.sleep(0.5)
		self.steering_publisher.publish(90+40)
		rospy.sleep(3.0)
		print "...Done"
		print ("Loading speed configuration"),
		self.speed_publisher.publish(400)
		rospy.sleep(0.5)
		print "...Done"
		self.start = True




	def __init__(self):
		self.speed = 0
		self.currentPose = PoseWithCovarianceStamped()
		self.theta = 0
		self.expected_theta = math.pi/2
		self.theta_error = 0.05


		self.x1=0
		self.y1=0
		self.x2=0
		self.y2=0
		#number of observations
		self.observations = 30
		#array with diameters
		self.data = []
		#flag
		self.start=False

		self.speed_publisher = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
		#steering publisher +- 40 degrees
		self.steering_publisher = rospy.Publisher("/steering", UInt8, queue_size=1)
		#pose estimation with fake_localization
		self.pose_subscriber = rospy.Subscriber("/seat_car/amcl_pose", PoseWithCovarianceStamped, self.getCurrentPose)


if __name__ == '__main__':
    try:
    	rospy.init_node('speed_util', anonymous=True)
        s = Steering()
        s.prepare_configurations()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass