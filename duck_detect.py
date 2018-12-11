import os
import sys
import cv2
import glob

# HSV color thresholds for YELLOW
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Webcam parameters (your desired resolution)
THRESHOLD_LOW = (15, 210, 20);
THRESHOLD_HIGH = (35, 255, 255);

# Minimum required radius of enclosing circle of contour
MIN_RADIUS = 15

def duck_center(fname, display=False):

	# Initialize camera and get actual resolution
	# cam = cv2.VideoCapture(0)
	# cam.set(cv2.CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
	# cam.set(cv2.CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
	# camWidth = cam.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
	# camHeight = cam.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
	# print "Camera initialized: (" + str(camWidth) + ", " + str(camHeight) + ")"


    # Get image from camera
    # ret_val, img = cam.read()
	img = cv2.imread(fname)
	if img is None:
		return -1

    # Blur image to remove noise
	img_filter = cv2.medianBlur(img.copy(), 5)
	# img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

    # Convert image from BGR to HSV
	img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)

    # Set pixels to white if in color range, others to black (binary bitmap)
	img_binary = cv2.inRange(img_filter.copy(), THRESHOLD_LOW, THRESHOLD_HIGH)

    # Dilate image to make white blobs larger
	img_binary = cv2.dilate(img_binary, None, iterations = 1)

    # Find center of object using contours instead of blob detection. From:
    # http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
	img_contours = img_binary.copy()
	contours = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, \
		cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Find the largest contour and use it to compute the min enclosing circle
	center = None
	radius = 0
	if len(contours) > 0:
                sorted_contours = sorted(contours,key=cv2.contourArea, reverse=True)
		# c = max(contours, key=cv2.contourArea)
                for c in sorted_contours:
                    center = None
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                            if center[1] <= 2*CAMERA_HEIGHT / 3.0:
                                continue
                            else:
                                if radius < MIN_RADIUS:
                                    center = None
                                break

	if not display:
		if not center:
			return -1
		return center

    # Print out the location and size (radius) of the largest detected contour
	if center != None:
		print(str(center) + " " + str(radius))

    # Draw a green circle around the largest enclosed contour
	if center != None:
		cv2.circle(img, center, int(round(radius)), (0, 255, 0))

    # Show image windows
	cv2.imshow('webcam', img)
	# cv2.imshow('binary', img_binary)
	# cv2.imshow('contours', img_contours)
	# cv2.imshow('gaussian', cv2.GaussianBlur(img.copy(), (3, 3), 0))
	# cv2.imshow('median', img_filter)
	cv2.waitKey(1) 

def get_latest(folder):
	os.chdir(folder)
	sorted_files = listing = glob.glob('*.jpg')
	sorted_files.sort(key=os.path.getmtime)
	latest = sorted_files[-1]
	os.chdir('..')
	return latest

def duck_direction(folder):

	fname = get_latest(folder)
	center = duck_center(folder + '/' + fname)

	# print(center, folder + '/' + fname)
	if center == -1 or center[1] <= (2*CAMERA_HEIGHT / 3.0):
		return 0, 0, 0

	# centered
	if center[0] >= (CAMERA_WIDTH / 2.0 - 0.1 * CAMERA_WIDTH) and center[0] <= (CAMERA_WIDTH / 2.0 + 0.1 * CAMERA_WIDTH):
		print('centered', center, folder + '/' + fname)
		return 0, 1, 0

	# left
	if center[0] <= (CAMERA_WIDTH / 2.0 - 0.1 * CAMERA_WIDTH):	
		print('left', center, folder + '/' + fname)
		return 1, 0, 0

	# right
	if center[0] >= (CAMERA_WIDTH / 2.0 + 0.1 * CAMERA_WIDTH):	
		print('right', center, folder + '/' + fname)
		return 0, 0, 1




