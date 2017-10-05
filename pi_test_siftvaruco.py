import cv2
import cv2.aruco as aruco
import numpy as np
#import matplotlib as plt
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import sys

# usage:
# python test_siftvaruco.py [# of pictures] [width resolution] [height resolution] [file name]

if (len(sys.argv) != 5):
    print ("usage: python test_siftvaruco.py [# of pictures] [width resolution] [height resolution] [file name]")

# set up camera
camera = PiCamera()
width = int(sys.argv[2])
height = int(sys.argv[3])
camera.resolution = (width, height)
camera.framerate = 30
camera.shutter_speed = 0
camera.ISO = 0
camera.meter_mode = 'matrix'
rawCapt = PiRGBArray(camera, size = (width, height))
numPics = int(sys.argv[1])

# set up aruco
aruco_dic = aruco.Dictionary_get(aruco.DICT_5X5_50)

# set up sift
grayImg = cv2.imread('/home/pi/speedtest/marker5.jpg', 0) # TODO PUT THIS IN
sift = cv2.xfeatures2d.SIFT_create()
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv2.FlannBasedMatcher(index_params, search_params)
imgKP, imgDes = sift.detectAndCompute(grayImg, None)


# set up file
f = open(str(sys.argv[4]) + ".txt", "w")
num_aruco_found = 0
num_sift_found = 0
for i in range(1, numPics):
    rawCapt.truncate(0)
    camera.capture(rawCapt, format = "bgr", use_video_port = True)
    pic = rawCapt.array
    find_target(pic, "sift", time, found)
    f.write(str(time) + " ")
    if found:
        num_sift_found = num_sift_found + 1


    find_target(pic, "aruco", time, found)
    f.write(str(time) + "\n")
    if found:
        num_aruco_found = num_aruco_found + 1

f.write("sift: " + num_sift_found + " " + "aruco: " + num_aruco_found)
camera.close()

def find_target(pic, method, time, found):

    nomatch = False
    time_start = time.time()

    if (method == "sift"):

        grayPic = cv2.cvtColor(vid, cv2.COLOR_BGR2GRAY)
        picKP, picDes = sift.detectAndCompute(grayPic, None)

        matches = []

        if (imgDes != None) and (picDes != None):
            matches = flann.knnMatch(imgDes, picDes, k=2)
            if (matches != None):
                goodMatches = []
                for m, n in matches:
                    if m.distances < 0.7*n.distance:
                        goodMatches.append(m)
                if (len(goodMatches) > 10):
                    src_pts = np.float32([ imgKP[m.queryIdx].pt for m in goodMatches ]).reshape(-1,1,2) #query images' features
		    dst_pts = np.float32([ picKP[m.trainIdx].pt for m in goodMatches ]).reshape(-1,1,2) #train images' features
		    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0) 	#finds the perspective transformation of object
		    h,w = grayImg.shape
		    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
		    if M is not None:
			dst = cv2.perspectiveTransform(pts, M)
                        nomatch = True

    elif (method == "aruco"):

        corners, ids, rejects = aruco.detectMarkers(pic, aruco_dic)
        if (len(corners) != 0):
            nomatch = True

    else:
        print("Invalid method")

    time = time.time() - time_start
    found = nomatch
