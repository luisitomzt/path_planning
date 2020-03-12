#!/usr/bin/env python
import rospy
import math
import time
import os
import pickle
from std_msgs.msg import Int16,UInt8
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np



class Speed():

	def getCurrentPose(self, data,  event = None):
		#position|
		self.currentPose = data.pose.pose

	def publishSpeed(self, step = 50, observations = 30, event = None):

		r = rospy.Rate(1.0)
		data = []

		while self.speed < 1000:

			self.speed+=step
			print self.speed
			rospy.sleep(0.5)
			self.speed_publisher.publish(self.speed)

			#sleep for 1 second then start data collection
			rospy.sleep(1.0)
			velocities = []
			#sample 30 observations
			for i in range(observations+2):

				x1 = self.currentPose.position.x
				y1 = self.currentPose.position.y
				t1 = rospy.Time.from_sec(time.time())
				r.sleep()
				t2 = rospy.Time.from_sec(time.time())
				x2 = self.currentPose.position.x
				y2 = self.currentPose.position.y
				ms = math.sqrt((x2-x1)**2 + (y2-y1)**2)/(t2-t1).to_sec()
				if i !=0:
					velocities.append(ms)

				print i," ",ms, "meter per second"

			print "mean ",np.mean(velocities)
			print "std ",np.std(velocities)
			data.append(velocities)

			print data
			print velocities
		with open("velocities.txt", "wb") as fp:
			pickle.dump(data, fp)


	def __init__(self):
		self.speed = 0
		self.currentPose = PoseWithCovarianceStamped()
		self.speed_publisher = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
		#pose estimation with fake_localization
		self.pose_subscriber = rospy.Subscriber("/seat_car/amcl_pose", PoseWithCovarianceStamped, self.getCurrentPose)


if __name__ == '__main__':
    try:
    	rospy.init_node('speed_util', anonymous=True)
        s = Speed()
        s.publishSpeed()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass