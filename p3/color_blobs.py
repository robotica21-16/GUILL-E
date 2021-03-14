# -*- coding: utf-8 -*-
#!/usr/bin/python

# Standard imports
import cv2
import numpy as np
from picamera.array import PiRGBArray

# Defaulted to red blobs
def search_blobs(cam, colorMin = (10, 10, 100), colorMax = (50, 50, 255)):
	
	#rawCapture = PiRGBArray(cam, size=(320, 240))
	# Read image from camera
	#img_BGR = cam.capture(rawCapture, format="bgr", use_video_port=True)
	img_BGR = cv2.imread("photos/2021-3-14-10_14.png")
	
	# Setup default values for SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()

	# These are just examples, tune your own if needed
	# Change thresholds
	params.minThreshold = 10
	params.maxThreshold = 200

	# Filter by Area
	params.filterByArea = True
	params.minArea = 200
	params.maxArea = 10000

	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.1

	# Filter by Color
	params.filterByColor = False
	# not directly color, but intensity on the channel input
	#params.blobColor = 0
	params.filterByConvexity = False
	params.filterByInertia = False


	# Create a detector with the parameters
	ver = (cv2.__version__).split('.')
	if int(ver[0]) < 3 :
		detector = cv2.SimpleBlobDetector(params)
	else :
		detector = cv2.SimpleBlobDetector_create(params)

	# keypoints on original image (will look for blobs in grayscale)
	keypoints = detector.detect(img_BGR)
	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
	# the size of the circle corresponds to the size of blob
	# im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	# Show blobs
	# cv2.imshow("Keypoints on Gray Scale", im_with_keypoints)
	# cv2.waitKey(0)

	# filter certain COLOR channels

	# Pixels with 100 <= R <= 255, 15 <= B <= 56, 17 <= G <= 50 will be considered red.
	# similar for BLUE

	mask=cv2.inRange(img_BGR, colorMin, colorMax)


	# apply the mask
	color = cv2.bitwise_and(img_BGR, img_BGR, mask = mask)
	# show resulting filtered image next to the original one
	# cv2.imshow("Red regions", np.hstack([img_BGR, color]))


	# detector finds "dark" blobs by default, so invert image for results with same detector
	keypoints = detector.detect(255-mask)

	# documentation of SimpleBlobDetector is not clear on what kp.size is exactly,
	# but it looks like the diameter of the blob.
	# for kp in keypoints:
		# print(kp.pt[0], kp.pt[1], kp.size)

	

	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
	# the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints, np.array([]),
	 	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	# im_with_keypoints2 = cv2.drawKeypoints(img_BGR, keypoints_blue, np.array([]),
	# 	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	
	cv2.imshow("Keypoints on RED", im_with_keypoints)
	cv2.waitKey(0)

	return max(keypoints, key=lambda kp: kp.size)
