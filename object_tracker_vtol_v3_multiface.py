#!/usr/bin/env python

# USAGE
# python object_tracker.py --prototxt deploy.prototxt --model res10_300x300_ssd_iter_140000.caffemodel

# import the necessary packages
from pyimagesearch.centroidtracker import CentroidTracker
from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import time
import cv2

import rospy
from missionplan.msg import CentroidDist

# construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-p", "--prototxt", required=True,
# 	help="path to Caffe 'deploy' prototxt file")
# ap.add_argument("-m", "--model", required=True,
# 	help="path to Caffe pre-trained model")
# ap.add_argument("-c", "--confidence", type=float, default=0.5,
# 	help="minimum probability to filter weak detections")
# args = vars(ap.parse_args())

def tracker():
	rospy.init_node("Centroid_Dist_Publisher", anonymous = True)
	pub = rospy.Publisher("CentroidDistance", CentroidDist, queue_size = 10)
	rate = rospy.Rate(30)

	centre_x = 0
	centre_y = 0

	# initialize our centroid tracker and frame dimensions
	ct = CentroidTracker()
	(H, W) = (None, None)

	confidence = 0.7

	# load our serialized model from disk
	print("[INFO] loading model...")
	net = cv2.dnn.readNetFromCaffe("/home/yash/AeroMit/VTOL_Sim/ROS_VTOL/src/missionplan/src/deploy.prototxt", "/home/yash/AeroMit/VTOL_Sim/ROS_VTOL/src/missionplan/src/res10_300x300_ssd_iter_140000.caffemodel")


	# initialize the video stream and allow the camera sensor to warmup
	print("[INFO] starting video stream...")
	# resource_name = "/dev/video" + resource
	# resource = int(resource)
	# vs = cv2.VideoCapture('rtsp://192.168.1.254/sjcam.mov')
	# vs = VideoStream(src='rtsp://192.168.1.254/sjcam.mov').start()
	vs = VideoStream(src=0).start()
	time.sleep(2.0)

	while not rospy.is_shutdown():
	# loop over the frames from the video stream
		while True:
			# read the next frame from the video stream and resize it
			# ret, frame = vs.read()
			frame = vs.read()
			frame = imutils.resize(frame, width=400)
			# frame = cv2.resize(frame, (225,400))
			# print(frame.shape)

			# if the frame dimensions are None, grab them
			if W is None or H is None:
				(H, W) = frame.shape[:2]

			# construct a blob from the frame, pass it through the network,
			# obtain our output predictions, and initialize the list of
			# bounding box rectangles
			blob = cv2.dnn.blobFromImage(frame, 1.0, (W, H),
				(104.0, 177.0, 123.0))
			# fps = video.get(cv2.CAP_PROP_FPS)

			net.setInput(blob)
			detections = net.forward()
			rects = []

			# loop over the detections
			for i in range(0, detections.shape[2]):
			# if len(detections[2]) == 1:
				# filter out weak detections by ensuring the predicted
				# probability is greater than a minimum threshold
				if detections[0, 0, i, 2] > confidence:
					# compute the (x, y)-coordinates of the bounding box for
					# the object, then update the bounding box rectangles list
					box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
					rects.append(box.astype("int"))

					# draw a bounding box surrounding the object so we can
					# visualize it
					(startX, startY, endX, endY) = box.astype("int")
					cv2.rectangle(frame, (startX, startY), (endX, endY),
						(0, 255, 0), 2)
					midX = (startX + endX)/2
					midY = (startY + endY)/2


			# update our centroid tracker using the computed set of bounding
			# box rectangles
			objects = ct.update(rects)

			c = 0


			# loop over the tracked objects
			for (objectID, centroid) in objects.items():

				CDist = CentroidDist()
				# draw both the ID of the object and the centroid of the
				# object on the output frame
				text = "ID {}".format(objectID)

				cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
				
				# print(centroid[0], centroid[1])
				# print("\n\n")
				centre_x += (200 - centroid[0])
				centre_y += (112 - centroid[1])

				c=c+1

				
				# print(200 - centroid[0], 150 - centroid[1])
				# # print("\n")
				
        		# print "Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps)


				# if(centroid[0]>220):
				# 	com ="Move Left"
				# elif(centroid[0]<180):
				# 	com ="Move Right"
				# else:
				# 	if centroid[1] > 220:
				# 		com = "Move Down"
				# 	elif centroid[1] < 180:
				# 		com = "Move Up"
				# 	else:
				# 		com = "Centered!"

				object_count = "Count: {}".format(len(objects.items()))
				
				cv2.putText(frame, object_count, (300,290),
					cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

			# show the output frame
			# cv2.putText(frame, fps, (140,20),
			# 		cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

			if c!=0:
				CDist.dx = centre_x/c
				CDist.dy = centre_y/c

			else:
				CDist.dx = 0
				CDist.dy = 0
				
			pub.publish(CDist)

			centre_x = 0
			centre_y = 0

			c = 0

			cv2.imshow("Frame", frame)
			key = cv2.waitKey(1) & 0xFF


			# if the ` ` key was pressed, break from the loop
			if key == ord(" "):
				break

			rate.sleep()


	# do a bit of cleanup
	#cv2.destroyAllWindows()
	vs.stop()


if __name__ == '__main__':
	tracker()
	
	# rospy.init_node("Centroid_Dist_Publisher", anonymous = True)
