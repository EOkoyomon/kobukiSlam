from __future__ import print_function
import cv2 as cv
import argparse
def detectAndDisplay(frame):
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)
    #-- Detect ducks
    ducks = duck_cascade.detectMultiScale(frame_gray)
    for (x,y,w,h) in ducks:
        center = (x + w//2, y + h//2)
        frame = cv.ellipse(frame, center, (w//2, h//2), 0, 0, 360, (255, 0, 255), 4)
    cv.imshow('Capture - duck detection', frame)
parser = argparse.ArgumentParser(description='Code for Cascade Classifier tutorial.')
parser.add_argument('--duck_cascade', help='Path to duck cascade.')
parser.add_argument('--camera', help='Camera devide number.', type=int, default=0)
args = parser.parse_args()
duck_cascade_name = args.duck_cascade
duck_cascade = cv.CascadeClassifier()
#-- 1. Load the cascades
if not duck_cascade.load(duck_cascade_name):
    print('--(!)Error loading duck cascade')
    exit(0)
camera_device = args.camera
#-- 2. Read the video stream
cap = cv.VideoCapture(camera_device)
if not cap.isOpened:
    print('--(!)Error opening video capture')
    exit(0)
while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break
    detectAndDisplay(frame)
    if cv.waitKey(10) == 27:
        break