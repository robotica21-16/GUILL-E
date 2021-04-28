# -*- coding: utf-8 -*-
#!/usr/bin/python

# Standard imports
import cv2
import numpy as np
from picamera.array import PiRGBArray

red_hsv_range1 = [(0, 220, 60), (1, 255, 200)]
red_hsv_range2 = [(170, 220, 60), (180, 255, 200)]
	
def init_detector():
	
	# Setup default values for SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()

	# These are just examples, tune your own if needed
	# Change thresholds
	params.minThreshold = 10
	params.maxThreshold = 255

	# Filter by Area
	params.filterByArea = True
	params.minArea = 200
	params.maxArea = 50000

	# Filter by Circularity
	params.filterByCircularity = False
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
	return detector
	

# Defaulted to red blobs
def search_blobs(cam, imagefile, 
	hsv1=red_hsv_range1[0], hsv2=red_hsv_range1[1], 
	hsv3= red_hsv_range2[0], hsv4=red_hsv_range2[1], show=False):
	
	
	detector = init_detector()
	img= cv2.imread(imagefile)
	verbose=True
	result= search_blobs_detector(cam, img, detector, hsv1,hsv2,hsv3,hsv4,verbose, show=show)
	cv2.waitKey(0)
	return result
	
def search_blobs_detector(cam, img_BGR, detector,
	hsv1=red_hsv_range1[0], hsv2=red_hsv_range1[1], 
	hsv3= red_hsv_range2[0], hsv4=red_hsv_range2[1],  verbose=False, show=False):
	
	nPixelsBorder = 10 # numero de pixeles a añadir al borde (para facilitar deteccion)
	img_BGR = cv2.copyMakeBorder(img_BGR, nPixelsBorder, nPixelsBorder, nPixelsBorder, nPixelsBorder, cv2.BORDER_CONSTANT)
	# Show blobs
	# cv2.imshow("Keypoints on Gray Scale", im_with_keypoints)
	# cv2.waitKey(0)

	# filter certain COLOR channels

	# Pixels with 100 <= R <= 255, 15 <= B <= 56, 17 <= G <= 50 will be considered red.
	# similar for BLUE
	
	img_HSV = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)
	
	mask1 = cv2.inRange(img_HSV, hsv1, hsv2)
	mask2 = cv2.inRange(img_HSV, hsv3, hsv4)
    
	mask = cv2.bitwise_or(mask1, mask2)
	
	#mask=cv2.inRange(img_BGR, colorMin, colorMax)
	color = cv2.bitwise_and(img_BGR, img_BGR, mask = mask)
	if show:
		cv2.imshow("Mascara aplicada", color)
	#cv2.waitKey(0)


	# apply the mask
	#color = cv2.bitwise_and(img_BGR, img_BGR, mask = mask)
	# show resulting filtered image next to the original one
	
	#cv2.imshow("Red regions", np.hstack([img_BGR, color]))

	
	# detector finds "dark" blobs by default, so invert image for results with same detector
	keypoints = detector.detect(255-mask)
	#keypoints = detector.detect(mask)
	if show:
		cv2.imshow("Mask", 255-mask)
	#cv2.waitKey(0)

	# documentation of SimpleBlobDetector is not clear on what kp.size is exactly,
	# but it looks like the diameter of the blob.
	if verbose:
		for kp in keypoints:
			print(kp.pt[0], kp.pt[1], kp.size)

	

	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
	# the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints, np.array([]),
	 	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	
	if show:
		cv2.imshow("Keypoints", im_with_keypoints)
	
	return max(keypoints, key=lambda kp: kp.size) if len(keypoints)>0 else None
