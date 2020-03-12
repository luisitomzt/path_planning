import numpy as np
import math
from numpy import linalg as LA
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import pickle

#quintic bezier curve basis matrix
Mb5 = np.array([
	[-1,5,-10,10,-5,1],
    [5,-20,30,-20,5,0],
    [-10,30,-30,10,0,0],
    [10,-20,10,0,0,0],
    [-5,5,0,0,0,0],
    [1,0,0,0,0,0]
    ])
dMb5 = np.array([
	[-5,25,-50,50,-25,5],
    [20,-80,120,-80,20,0],
    [-30,90,-90,30,0,0],
    [20,-40,20,0,0,0],
    [-5,5,0,0,0,0],
    [0,0,0,0,0,0]
    ])
ddMb5 = np.array([
	[-20,100,-200,200,-100,20],
    [60,-240,360,-240,60,0],
    [-60,180,-180,60,0,0],
    [20,-40,20,0,0,0],
    [0,0,0,0,0,0],
    [0,0,0,0,0,0]
    ])

#cubic bezier curve basis matrix
Mb3= np.array([
	[-1,3,-3,1],
    [3,-6,3,0],
    [-3,3,0,0],
    [1,0,0,0]
    ])
dMb3= np.array([
	[-3,9,-9,3],
    [6,-12,6,0],
    [-3,3,0,0],
    [0,0,0,0]
    ])
ddMb3= np.array([
	[-6,18,-18,6],
    [6,-12,6,0],
    [0,0,0,0],
    [0,0,0,0]
    ])


waypoints = np.array([
		[0,0], 
		[1,0],
		[1,1],
		[0,1]
	])


def normalized_tangent_vectors(points):
	n = len(points)+1
	#print n
	#dT (first derivative)
	#t=0 to get the tangent vector of the first control point
	t=0
	dT=np.array([t**4, t**3, t**2, t , 1, 0])
	
	tangents = []

	for i in range(n-1):
		#compute tangent
		tangent =  np.dot(dT,dMb5).dot(points[i])
		#print tangent
		tangent_magnitude = math.sqrt(tangent[0]**2+tangent[1]**2)
		#print tangent_magnitude
		tangent = tangent/tangent_magnitude
		#print tangent
		tangents.append(tangent)
		if i == n-2:
			#t=1 for the last control point
			t=1
			tangent =  np.dot(dT,dMb5).dot(points[i])
			#print tangent
			tangent_magnitude = math.sqrt(tangent[0]**2+tangent[1]**2)
			#print tangent_magnitude
			tangent = tangent/tangent_magnitude
			#print tangent
			tangents.append(tangent)

	return tangents




def bisector_perpendicular(v,theta):
    
    a = math.cos(theta) * v[0] + -math.sin(theta) * v[1]
    b = math.sin(theta) * v[0] + math.cos(theta) * v[1]
    
    return [a,b]


def generate_BezierCurve(waypoints):

	#print waypoints
	scl = 5.0
	#compute inner waypoints first and second derivative
	#		segment
	# p0, p1, p2, p3, p4, p5
	segments = len(waypoints) - 1
	n = segments -1

	q = np.zeros((segments,6,2))
	q3 = np.zeros((segments,4,2))

	#fill p0 and p5 on all segments
	for i in range(segments):
		q[i][0] = waypoints[i]
		q[i][5] = waypoints[i+1]
		q3[i][0] = waypoints[i]
		q3[i][3] = waypoints[i+1]



	#if not loop:
	#compute first and last waypoints tangents and second derivatives values
	#first waypoints tangent
	heading = q[0][5] - q[0][0]
	heading = math.atan2(heading[1],heading[0])
	magnitude = LA.norm(q[0][0] - q[0][5])/scl
	q[0][1] = q[0][0] + [math.cos(heading)*magnitude,math.sin(heading)*magnitude]
	q3[0][1] = q[0][1]
	#last waypoint tangent
	heading = q[n][0] - q[n][5]
	heading = math.atan2(heading[1],heading[0])
	magnitude = LA.norm(q[n][0] - q[n][5])/scl
	q[n][4] = q[n][5] + [math.cos(heading)*magnitude,math.sin(heading)*magnitude]
	q3[n][2] = q[n][4]


	#fill inner waypoints first and second derivative

	#first derivative
	for i in range(1,segments):
		#find angle bisector of the inner waypoint
		BA = q[i-1][0] - q[i][0]
		BC = q[i][5] - q[i][0]
		#bisector ||u|| v + ||v|| u / ||u||.||v||
		bisector = (LA.norm(BC)*BA + LA.norm(BA)*BC)/(LA.norm(BA)*LA.norm(BC));
		#bisector from inner waypoint
		bisector_vector = q[i][0] + bisector

		#perpendicular angle of the bisector
		bp1 = bisector_perpendicular(bisector/LA.norm(bisector), math.pi/2)
		bp2 = bisector_perpendicular(bisector/LA.norm(bisector),-math.pi/2)
		

		minimum = min(LA.norm(BA), LA.norm(BC))

		if LA.norm(BC - bp1) < LA.norm(BC - bp2):
			p_vector =  bp1

		else:
			p_vector =  bp2

		#p1 of segment BC
		p_vector1 = np.dot(p_vector,minimum/scl)
		p_vector1 = q[i][0] + p_vector1
		q[i][1] = p_vector1
		q3[i][1] = p_vector1

		#p4 of segment AB
		p_vector2 = np.dot(p_vector,minimum/scl)
		p_vector2 = q[i][0] + bisector_perpendicular(p_vector2, math.pi)
		q[i-1][4] = p_vector2
		q3[i-1][2] = p_vector2

	#if not loop:
	#first segment second derivative
	
	#3(p1-p0) of segment AB
	B = q3[0][0]
	C = q3[0][3]
	tB =  3*(q3[0][1]-q3[0][0])
	
	tC =  3*(q3[0][3]-q3[0][2])
	
	a = (-6*B-4*tB-2*tC+6*C)

	q[0][2] = 2*q[0][1] + (a/20) - q[0][0]

	#last segment second derivative

	#3(p1-p0) of segment BC
	A = q3[n][0]
	B = q3[n][3]
	tA =  3*(q3[n][1]-q3[n][0])
	tB =  3*(q3[n][3]-q3[n][2])

	a= (6*A+2*tA+4*tB-6*B)
	q[n][3] = 2*q[n][4] + (a/20) - q[n][5]



	#second derivative
	for i in range(1,segments):
		#segments P1,P2,P3,P4 coordinates
		#values of tA , tB and tC
		#3(p1-p0) of segment AB
		tA =  3*(q3[i-1][1]-q3[i-1][0])
		#3(p3-p2) of segment AB
		tB =  3*(q3[i][1]-q3[i][0])

		#print 3*(q3[i-1][3]-q3[i-1][2]), 3*(q3[i][1]-q3[i][0])
		#3(p1-p0) of segment BC
		tC =  3*(q3[i][3]-q3[i][2])

		#A,B,C vertex values
		A=q3[i-1][0]
		B=q3[i][0]
		C=q3[i][3]

		#weighted average of second derivative of the cubic bezier curves
		#magnitude of segments ab, bc
		dab = float(LA.norm( q3[i][0] - q[i-1][0]))
		dbc = float(LA.norm( q3[i][0] - q[i][3]))
		#scale values
		alpha = dbc / (dab+dbc)
		beta = dab / (dab+dbc)
		alpha = 0.5
		beta = 0.5
		#weighted values of second derivatives at the inner point B
		d2ab1 = (6*A+2*tA+4*tB-6*B)*alpha
		d2bc0 = (-6*B-4*tB-2*tC+6*C)*beta
		#weighted average result
		a = d2ab1+d2bc0

		#P3 of previuos segment
		q[i-1][3] = 2*q[i-1][4] - q[i-1][5] + (a/20)
		#P2 of current segment
		q[i][2] = 2*q[i][1] + (a/20) - q[i][0]

	#x= []
	#y=[]
	trajectory = open("trajectory.txt","w")
	path = Path()
	path.header.frame_id = "map"
	no_ros_path = []
	for segment in range(segments):
		for t in np.arange(0,1.01,0.01):

			T=np.array([t**5, t**4, t**3, t**2 , t, 1.0])

			res =  np.dot(T,Mb5).dot(q[segment])



			p = PoseStamped()
			p.header.frame_id = "map"
			p.pose.position.x = res[0]
			p.pose.position.y = res[1]

			print  >>trajectory, res[0], ',', res[1]

			path.poses.append(p)
			no_ros_path.append(res)

			#x.append(res[0])
			#y.append(res[1])

			#print res

	with open("curve.txt", "wb") as fp:
		pickle.dump(q, fp)


	return path,q,no_ros_path
	#plt.show()





