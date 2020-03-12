#!/usr/bin/env python
import rospy
import os
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import bezierpath as bp


import numpy as np
import time

#image width (m) and height (n) parameters in px
m=480.0
n=640.0
m2=300
n2=200

def control_points2img(control_points, img, color=(255,255,255)):
	for cp in control_points:
		for p in cp:
			cv2.circle(img,(np.uint8(p[0]),np.uint8(p[1])),2,color,-1)

	return img

class lane_finder():


		

	def __init__(self):
		#image publisher
		self.image_pub = rospy.Publisher("/camera/rgb/ipm_lanes",Image, queue_size = 1)
		self.bridge = CvBridge()
		#image acquisition from camera topic
		self.camera_subscriber = rospy.Subscriber("/camera/rgb/ipm_mask", Image, self.callback)
		self.kernel = np.ones((3,3), np.uint8) 

		

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)


		#apply 2 smooths to lessen the black ines on the image
		blur = cv2.blur(cv_image,(3,3))
		blur = cv2.blur(blur,(3,3))
		#dilate white lines to make them thicker
		dilate = cv2.dilate(blur,self.kernel,iterations = 1)

		#binarize image
		_,binarized = cv2.threshold(dilate,35,255,cv2.THRESH_BINARY)
		lines = cv2.HoughLinesP(binarized,cv2.HOUGH_PROBABILISTIC, (np.pi/180), 30, minLineLength = 10,maxLineGap = 3)
		# transform ipm grayscale img to bgr space 
		# to allow the addition of colored lines		
		ipm_lines = cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR )
		for x in range(0, len(lines)):
		    for x1,y1,x2,y2 in lines[x]:
		        cv2.line(ipm_lines,(x1,y1),(x2,y2),(255,0,0),1, cv2.LINE_AA)
		
		ipm_lines_grayscale = cv2.cvtColor(ipm_lines, cv2.COLOR_BGR2GRAY )

		
		#connected components algorithm for labeling common regions
		ret, labels = cv2.connectedComponents(ipm_lines_grayscale, connectivity = 8)

		#for visual representation only
		labels_normalized = labels*(255/labels.max())
		img_label = np.zeros([m2,n2,3],dtype=np.uint8)

		img_label[:,:,0] = labels_normalized
		

		center_curve = np.array([]).reshape((0,2))
		for i in range(1,ret+1):
			aux =  np.nonzero(labels == i)

			length = len(aux[0])
			if length != 0:
				sum_y = np.sum(aux[0])/ length
				sum_x = np.sum(aux[1])/length
				if length > 100 and length < 500:
					cv2.circle(ipm_lines,(sum_x,sum_y),5,(0,0,255),-1)
					center_curve = np.vstack((center_curve,[sum_x,sum_y]))

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(ipm_lines, "bgr8"))
		except CvBridgeError as e:
			print(e)


		#generate bezier curve for center of the lanes (white dashes)
		_, center_control_points,center_bezier_line =  bp.generate_BezierCurve(center_curve)

		#compute tangent vectors of the bezier curve's control points
		normalized_center_tangent_vectors = bp.normalized_tangent_vectors(center_control_points)

		#control points perpendiculars to center lane (normal vectors)
		left_lane_pts =[]
		right_lane_pts = []
		

		#compute normal vector of the center curve
		for p in normalized_center_tangent_vectors:
			#left lane new (x,y)
			lx = math.cos(math.pi/2)*p[0] - math.sin(math.pi/2)*p[1]
			ly = math.sin(math.pi/2)*p[0] + math.cos(math.pi/2)*p[1]

			#right lane new (x,y)
			rx = math.cos(-math.pi/2)*p[0] - math.sin(-math.pi/2)*p[1]
			ry = math.sin(-math.pi/2)*p[0] + math.cos(-math.pi/2)*p[1]
			left_lane_pts.append([lx,ly])
			right_lane_pts.append([rx,ry])

		#distance from center dotted line to left and right lane 
		lane_width = 43
		half_lane_width = lane_width/2
		danger_width = 2
		precaution_width = 4 + danger_width
		safe_width = 6 + precaution_width
		
		left_curve_ideal = []
		right_curve_ideal = []

		left_curve = []
		left_curve_safe_l = []
		left_curve_safe_r = []
		left_curve_precaution_l =[]
		left_curve_precaution_r =[]
		left_curve_danger_l = []
		left_curve_danger_r = []

		right_curve = []
		right_curve_safe_l = []
		right_curve_safe_r = []
		right_curve_precaution_l =[]
		right_curve_precaution_r =[]
		right_curve_danger_l = []
		right_curve_danger_r = []

		#add normal vector to center curve
		for i in range(len(left_lane_pts)):
			#left and right lane lines
			left_curve.append( ( center_curve[i][0]+(left_lane_pts[i][0]*lane_width), center_curve[i][1]+(left_lane_pts[i][1]*lane_width) ) )
			right_curve.append( ( center_curve[i][0]+(right_lane_pts[i][0]*lane_width), center_curve[i][1]+(right_lane_pts[i][1]*lane_width) ) )

			#idealzone lane lines
			left_curve_ideal.append( ( center_curve[i][0]+(left_lane_pts[i][0]*half_lane_width), center_curve[i][1]+(left_lane_pts[i][1]*half_lane_width) ) )
			right_curve_ideal.append( ( center_curve[i][0]+(right_lane_pts[i][0]*half_lane_width), center_curve[i][1]+(right_lane_pts[i][1]*half_lane_width) ) )

			#left safe zones
			left_curve_safe_l.append( ( center_curve[i][0]+(left_lane_pts[i][0]*(lane_width-safe_width) ), center_curve[i][1]+(left_lane_pts[i][1]*(lane_width-safe_width) ) ) )
			left_curve_safe_r.append( ( center_curve[i][0]+(left_lane_pts[i][0]* safe_width ), center_curve[i][1]+(left_lane_pts[i][1]* safe_width  ) ) )
			#right safe zones
			right_curve_safe_l.append( ( center_curve[i][0]+(right_lane_pts[i][0]*safe_width), center_curve[i][1]+(right_lane_pts[i][1]*safe_width)  ) ) 
			right_curve_safe_r.append( ( center_curve[i][0]+(right_lane_pts[i][0]*(lane_width-safe_width) ), center_curve[i][1]+(right_lane_pts[i][1]*(lane_width-safe_width) ) ) )

			#left precaution zones
			left_curve_precaution_l.append( ( center_curve[i][0]+(left_lane_pts[i][0]*(lane_width-precaution_width) ), center_curve[i][1]+(left_lane_pts[i][1]*(lane_width-precaution_width) ) ) )
			left_curve_precaution_r.append( ( center_curve[i][0]+(left_lane_pts[i][0]* precaution_width ), center_curve[i][1]+(left_lane_pts[i][1]* precaution_width  ) ) )
			#right precaution zones
			right_curve_precaution_l.append( ( center_curve[i][0]+(right_lane_pts[i][0]*precaution_width), center_curve[i][1]+(right_lane_pts[i][1]*precaution_width)  ) ) 
			right_curve_precaution_r.append( ( center_curve[i][0]+(right_lane_pts[i][0]*(lane_width-precaution_width) ), center_curve[i][1]+(right_lane_pts[i][1]*(lane_width-precaution_width) ) ) )

			#left danger zones
			left_curve_danger_l.append( ( center_curve[i][0]+(left_lane_pts[i][0]*(lane_width-danger_width) ), center_curve[i][1]+(left_lane_pts[i][1]*(lane_width-danger_width) ) ) )
			left_curve_danger_r.append( ( center_curve[i][0]+(left_lane_pts[i][0]* danger_width ), center_curve[i][1]+(left_lane_pts[i][1]* danger_width  ) ) )
			#right danger zones
			right_curve_danger_l.append( ( center_curve[i][0]+(right_lane_pts[i][0]*danger_width), center_curve[i][1]+(right_lane_pts[i][1]*danger_width)  ) ) 
			right_curve_danger_r.append( ( center_curve[i][0]+(right_lane_pts[i][0]*(lane_width-danger_width) ), center_curve[i][1]+(right_lane_pts[i][1]*(lane_width-danger_width) ) ) )
			

		_, left_control_points,left_bezier_line =  bp.generate_BezierCurve(left_curve)
		_, right_control_points,right_bezier_line =  bp.generate_BezierCurve(right_curve)

		#ideal zone lines
		_, left_control_points_ideal,left_bezier_line_ideal =  bp.generate_BezierCurve(left_curve_ideal)
		_, left_control_points_ideal,right_bezier_line_ideal =  bp.generate_BezierCurve(right_curve_ideal)

		#safe lines
		_, left_control_points_safe_l,left_bezier_line_safe_l =  bp.generate_BezierCurve(left_curve_safe_l)
		_, left_control_points_safe_r,left_bezier_line_safe_r =  bp.generate_BezierCurve(left_curve_safe_r)

		_, right_control_points_safe_l,right_bezier_line_safe_l =  bp.generate_BezierCurve(right_curve_safe_l)
		_, right_control_points_safe_r,right_bezier_line_safe_r =  bp.generate_BezierCurve(right_curve_safe_r)

		#precaution lines
		_, left_control_points_precaution_l,left_bezier_line_precaution_l =  bp.generate_BezierCurve(left_curve_precaution_l)
		_, left_control_points_precaution_r,left_bezier_line_precaution_r =  bp.generate_BezierCurve(left_curve_precaution_r)

		_, right_control_points_precaution_l,right_bezier_line_precaution_l =  bp.generate_BezierCurve(right_curve_precaution_l)
		_, right_control_points_precaution_r,right_bezier_line_precaution_r =  bp.generate_BezierCurve(right_curve_precaution_r)

		#danger lines
		_, left_control_points_danger_l,left_bezier_line_danger_l =  bp.generate_BezierCurve(left_curve_danger_l)
		_, left_control_points_danger_r,left_bezier_line_danger_r =  bp.generate_BezierCurve(left_curve_danger_r)

		_, right_control_points_danger_l,right_bezier_line_danger_l =  bp.generate_BezierCurve(right_curve_danger_l)
		_, right_control_points_danger_r,right_bezier_line_danger_r =  bp.generate_BezierCurve(right_curve_danger_r)


		

		#draw control points into the image (not really neccesary, just for debugging!)
		bezier_image = control_points2img(center_control_points, ipm_lines)
		bezier_image = control_points2img(left_control_points, bezier_image, (255,255,0))
		bezier_image = control_points2img(right_control_points, bezier_image, (0,255,255))

		#cv2.imshow("asdasd",bezier_image)

		#all of the code below if for visualization  in RGB images
		#main lanes's lines
		if (0):
			cv2.polylines(bezier_image, np.int32([center_bezier_line]), isClosed=False, color=(0,0,255),  thickness=1, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([left_bezier_line]), isClosed=False, color=(255,255,0),  thickness=1, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([right_bezier_line]), isClosed=False, color=(0,255,255),  thickness=1, lineType=cv2.LINE_8)

			
			#safe zone lines (green color)
			cv2.polylines(bezier_image, np.int32([left_bezier_line_safe_l]), isClosed=False, color=(0,255,0),  thickness=6, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([left_bezier_line_safe_r]), isClosed=False, color=(0,255,0),  thickness=6, lineType=cv2.LINE_8)

			cv2.polylines(bezier_image, np.int32([right_bezier_line_safe_l]), isClosed=False, color=(0,255,0),  thickness=6, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([right_bezier_line_safe_r]), isClosed=False, color=(0,255,0),  thickness=6, lineType=cv2.LINE_8)

			#precaution zone lines (yellow color)
			cv2.polylines(bezier_image, np.int32([left_bezier_line_precaution_l]), isClosed=False, color=(0,255,255),  thickness=4, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([left_bezier_line_precaution_r]), isClosed=False, color=(0,255,255),  thickness=4, lineType=cv2.LINE_8)

			cv2.polylines(bezier_image, np.int32([right_bezier_line_precaution_l]), isClosed=False, color=(0,255,255),  thickness=4, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([right_bezier_line_precaution_r]), isClosed=False, color=(0,255,255),  thickness=4, lineType=cv2.LINE_8)

			#danger zone lines (red color)
			cv2.polylines(bezier_image, np.int32([left_bezier_line_danger_l]), isClosed=False, color=(0,0,255),  thickness=2, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([left_bezier_line_danger_r]), isClosed=False, color=(0,0,255),  thickness=2, lineType=cv2.LINE_8)

			cv2.polylines(bezier_image, np.int32([right_bezier_line_danger_l]), isClosed=False, color=(0,0,255),  thickness=2, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([right_bezier_line_danger_r]), isClosed=False, color=(0,0,255),  thickness=2, lineType=cv2.LINE_8)

			#ideal zone lines (green color)
			cv2.polylines(bezier_image, np.int32([left_bezier_line_ideal]), isClosed=False, color=(255,255,255),  thickness=10, lineType=cv2.LINE_8)
			cv2.polylines(bezier_image, np.int32([right_bezier_line_ideal]), isClosed=False, color=(255,255,255),  thickness=10, lineType=cv2.LINE_8)
			
			#cv2.imshow("a",bezier_image)

		#apply the same code as before, but for grayscale image
		#initializa black image
		left_lane_potential_img = np.zeros([m2,n2,3],dtype=np.uint8)
		right_lane_potential_img = np.zeros([m2,n2,3],dtype=np.uint8)

		#safe zone lines (green color)
		cv2.polylines(left_lane_potential_img, np.int32([left_bezier_line_safe_l]), isClosed=False, color=(4,4,4),  thickness=6, lineType=cv2.LINE_8)
		cv2.polylines(left_lane_potential_img, np.int32([left_bezier_line_safe_r]), isClosed=False, color=(4,4,4),  thickness=6, lineType=cv2.LINE_8)

		cv2.polylines(right_lane_potential_img, np.int32([right_bezier_line_safe_l]), isClosed=False, color=(4,4,4),  thickness=6, lineType=cv2.LINE_8)
		cv2.polylines(right_lane_potential_img, np.int32([right_bezier_line_safe_r]), isClosed=False, color=(4,4,4),  thickness=6, lineType=cv2.LINE_8)

		#precaution zone lines (yellow color)
		cv2.polylines(left_lane_potential_img, np.int32([left_bezier_line_precaution_l]), isClosed=False, color=(16,16,16),  thickness=4, lineType=cv2.LINE_8)
		cv2.polylines(left_lane_potential_img, np.int32([left_bezier_line_precaution_r]), isClosed=False, color=(16,16,16),  thickness=4, lineType=cv2.LINE_8)

		cv2.polylines(right_lane_potential_img, np.int32([right_bezier_line_precaution_l]), isClosed=False, color=(16,16,16),  thickness=4, lineType=cv2.LINE_8)
		cv2.polylines(right_lane_potential_img, np.int32([right_bezier_line_precaution_r]), isClosed=False, color=(16,16,16),  thickness=4, lineType=cv2.LINE_8)

		#danger zone lines (red color)
		cv2.polylines(left_lane_potential_img, np.int32([left_bezier_line_danger_l]), isClosed=False, color=(64,64,64),  thickness=2, lineType=cv2.LINE_8)
		cv2.polylines(left_lane_potential_img, np.int32([left_bezier_line_danger_r]), isClosed=False, color=(64,64,64),  thickness=2, lineType=cv2.LINE_8)

		cv2.polylines(right_lane_potential_img, np.int32([right_bezier_line_danger_l]), isClosed=False, color=(64,64,64),  thickness=2, lineType=cv2.LINE_8)
		cv2.polylines(right_lane_potential_img, np.int32([right_bezier_line_danger_r]), isClosed=False, color=(64,64,64,),  thickness=2, lineType=cv2.LINE_8)

		#ideal zone lines (white color)
		cv2.polylines(left_lane_potential_img, np.int32([left_bezier_line_ideal]), isClosed=False, color=(2,2,2),  thickness=10, lineType=cv2.LINE_8)
		cv2.polylines(right_lane_potential_img, np.int32([right_bezier_line_ideal]), isClosed=False, color=(2,2,2),  thickness=10, lineType=cv2.LINE_8)

		left_lane_potential_img = cv2.GaussianBlur(left_lane_potential_img,(3,3),0)
		left_lane_potential_img = cv2.cvtColor(left_lane_potential_img, cv2.COLOR_BGR2GRAY )

		right_lane_potential_img = cv2.GaussianBlur(right_lane_potential_img,(3,3),0)
		right_lane_potential_img = cv2.cvtColor(right_lane_potential_img, cv2.COLOR_BGR2GRAY )

		
		
		


if __name__ == '__main__':
    try:
    	rospy.init_node('lane_finder', anonymous=True)
        lf = lane_finder()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass