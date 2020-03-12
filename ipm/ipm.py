#!/usr/bin/env python
import rospy
import os
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


import numpy as np
import time

#image width (m) and height (n) parameters in px
m=480.0
n=640.0
m2=300
n2=200


def extract_white_values(src):
	#extract white values in image
	# Convert BGR to HSV
	hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV )
	# define range of white color in HSV
	lower_white = np.array([0,0,50])
	upper_white= np.array([255,0,255])

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower_white, upper_white)


	# Bitwise-AND mask and original image
	#res = cv2.bitwise_and(src,src, mask= mask)

	return mask

class IPM():


		

	def __init__(self):

		#image publisher
		self.ipm_pub = rospy.Publisher("/camera/rgb/ipm",Image, queue_size = 1)
		#image publisher
		self.ipm_mask_pub = rospy.Publisher("/camera/rgb/ipm_mask",Image, queue_size = 1)

		self.bridge = CvBridge()
		#image acquisition from camera topic
		self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

		# Four corners of the book in source image
		self.pts_src = np.array([[0,0],[n,0],[0,m],[n,m]])
		# Four corners of the book in destination image.
		self.pts_dst = np.array([[127,336],[72,336],[72,263],[127,263]])
		# Calculate Homography
		self.h, self.status = cv2.findHomography(self.pts_src, self.pts_dst)

		

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		#extract white values from cv_image
		mask = extract_white_values(cv_image)
		

		# Warp source image to destination based on homography
		ipm_image = cv2.warpPerspective(cv_image, self.h, (n2,m2))
		# Warp masked source image to destination based on homography
		ipm_mask_image = cv2.warpPerspective(mask, self.h, (n2,m2))

		#(rows,cols,channels) = cv_image.shape
		#if cols > 60 and rows > 60 :
		#	cv2.circle(cv_image, (50,50), 10, 255)

		#cv2.imshow("Image window", cv_image)
		#cv2.waitKey(3)

		try:
			self.ipm_pub.publish(self.bridge.cv2_to_imgmsg(ipm_image, "bgr8"))
			self.ipm_mask_pub.publish(self.bridge.cv2_to_imgmsg(ipm_mask_image, "mono8"))
		except CvBridgeError as e:
			print(e)
		
		


if __name__ == '__main__':
    try:
    	rospy.init_node('image_ipm', anonymous=True)
    	print ("Loading camera parameters...Ok")
    	print ("Establishing control points for IPM...Ok")
        ipm = IPM()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass