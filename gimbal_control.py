#!/usr/bin/env python

import rospy
from missionplan.msg import CentroidDist
from mavros_msgs.msg import *
from storm32_gimbal.msg import GimbalOrientation
import math
import tf

#Message type to publish -->insert<---
#from simple_pid import PID

cdist = CentroidDist()
gim_pos = GimbalOrientation()
prev_error = 0
error = 0
i = 0

def PID(kp, kd, ki, setpoint, t, axis):
	global cdist
	global error
	global prev_error
	global i
	rospy.Subscriber("CentroidDistance", CentroidDist, centroid_dist_mapper_cb)
	prev_error = error
	p = 0
	i = 0
	d = 0
	dt = rospy.get_time() - t
	# print(dt)
	# if(dt>0.001):
	# 	print('Sampling')
	if axis == 'yaw':
		upper_limit = 80
		lower_limit = -80
		cur = cdist.dx
	if axis == 'pitch':
		cur = cdist.dy
		upper_limit = 60
		lower_limit = -50

	#pitch -15.4 to 55
	#yaw -80 to 80

	error = setpoint - cur
	# print('Error: ',error)
	

	# p = kp*error 

	if abs(error)<30:
		p = kp*error *0.8
	else:
		p = kp*error 


	# if(error > -35 and error < 35):
	# 	i = 20*error*dt
	# else:
	# 	i = ki*error*dt

	i = ki*error*dt

	# if error<20:
	# 	i = ki*error*dt
	# else:
	# 	i = 0


	d= kd*(error - prev_error)/(dt)

	# if abs(error)<40:
	# 	d = 0
	# else:
	# 	d= kd*(error - prev_error)/(dt)


	# d = kd*(error - prev_error)/(dt)


	if axis == 'yaw':
		print("dx: ", cur)
	if axis == 'pitch':
		# if i%10==0:
		print("dy: ", cur)

	# if q>1 and error<15:
	# 	d = 0

	# if i%10==0:
	print("P = ", p)
	print("I = ", i)
	print("D = ", d)

	pid = p+i+d

	# if axis == 'yaw':
	# 	pid = -pid


	# if error>10 and q>0:
	# if pid>upper_limit:
	# 	print("\nUpper limit hit\n")
	# 	return upper_limit
	# elif pid<lower_limit:
	# 	print("\nLower limit hit\n")
	# 	return lower_limit
	# else:
	# 	return pid


	return pid

	# if abs(error)<10:
	# 	return 0

	# else:
	# 	return pid

	# else:
	# 	print("PID OFF")
	# 	return 0
		# if error<15:
		# 	return pid
		# else:
		# 	print("PID OFF")
		# 	return 0

def centroid_dist_mapper_cb(cdist_msg):
	global cdist
	cdist = cdist_msg

def decay_fn(a,k,t):
	return a*math.exp(k*t)


def gimbal_output_publisher():
	global cdist
	global i
	pub = rospy.Publisher("/unnamed/control/target_orientation", GimbalOrientation, queue_size = 10)
	# pub = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size = 10)
	rospy.init_node("Gimbal_Output", anonymous = True)
	rate = rospy.Rate(30)
	# rate = rospy.Rate(1000)

	while not rospy.is_shutdown():
		t = rospy.get_time()
		# print(t)
		#PDI 0.000001
		# yaw = PID(0.0125, 0.000004, 30, 0, t, 'yaw')
		yaw = 0
		t = rospy.get_time()
		# print(t)
		# pitch = PID(0.0125, 0.004, 30, 0, t, 'pitch')
		pitch = PID(0.00125, 0.0004, 300, 0, t, 'pitch')
		# pitch = PID(0.008, 0.00004, 0, 0, t, 'pitch')
		# pitch = 0
		# if i%10==0:
		print('yaw: ', yaw)
		print('pitch: ',pitch)

		print("\n")

		# i=i+1


		#pitch -15.4 to 55
		#yaw -80 to 80
		quaternion = tf.transformations.quaternion_from_euler(0,pitch, yaw)

		gim_pos.orientation.x = quaternion[0]
		gim_pos.orientation.y = quaternion[1]
		gim_pos.orientation.z = quaternion[2]
		gim_pos.orientation.w = quaternion[3]
		gim_pos.unlimited = False

		pub.publish(gim_pos)
		rate.sleep()

	print("\n\nExiting...")

def reset_gimbal():
	print("\n\nRESETTING...\n")
	pub = rospy.Publisher("/unnamed/control/target_orientation", GimbalOrientation, queue_size = 10)

	gim_pos.orientation.x = 0
	gim_pos.orientation.y = 0
	gim_pos.orientation.z = 0
	gim_pos.orientation.w = 0
	gim_pos.unlimited = False

	print(gim_pos)


	pub.publish(gim_pos)

	print("\nDONE\n")




if __name__ == '__main__':
	gimbal_output_publisher()

	# reset_gimbal()





