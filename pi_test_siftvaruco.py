#!/usr/bin/env python

import cv2
import cv2.aruco as aruco
import numpy as np
#import matplotlib as plt
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
#import math
import sys
import csv

# usage:
# python test_siftvaruco.py [# of pictures] [resolution] [file name]

def find_target(pic, method):
    found = False
    
    time_start = time.time()
    if (method == 'sift'):
        list_time = time_start()
        grayPic = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
        picKP, picDes = sift.detectAndCompute(grayPic, None)
        list_time.append(time.time())

        matches = []
        if (imgDes != None) and (picDes != None):
            matches = flann.knnMatch(imgDes, picDes, k=2)
            list_time.append(time.time())
            if (matches != None):
                goodMatches = []
                for m, n in matches:
                    if m.distances < 0.7*n.distance:
                        goodMatches.append(m)
                    if (len(goodMatches) > 10):
                        src_pts = np.float32([ imgKP[m.queryIdx].pt for m in goodMatches ]).reshape(-1,1,2) #query images' features
                        dst_pts = np.float32([ picKP[m.trainIdx].pt for m in goodMatches ]).reshape(-1,1,2) #train images' features
                        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0) 	#finds the perspective transformation of object
                        list_time.append(time.time())                        
                        h,w = grayImg.shape
                        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                if (M):
                    dst = cv2.perspectiveTransform(pts, M)
                    list_time.append(time.time())
                    found = True
                
        time_elapsed = time.time() - time_start
        for ind in xrange(len(list_time)-1):
            print '   SIFT process: %0.2f' % (list_time[ind+1] - list_time[ind])

    elif (method == 'aruco'):
        corners, ids, rejects = aruco.detectMarkers(pic, aruco_dic)
        if (len(corners) != 0):
            found = True
        time_elapsed = time.time() - time_start    

    else:
        print("Invalid method")

    return (time_elapsed, found)


if __name__ == '__main__':
    time_start = time.time()
    if (len(sys.argv) != 4):
        print ('usage: python test_siftvaruco.py [# of pictures] [resolution, 0:small, 1:large] [file name]')
    
    num_pics = int(sys.argv[1])    
    
    # set up camera
    print '%0.2f: Initializing Camera' % (time.time() - time_start)
    camera = PiCamera()
    if (int(sys.argv[2] == 0)):
        width = 640     # Mode 7, 640x480, aspect ratio 4:3, 40 to 90 fps, full FoV
        height = 480
    else:
        width = 1640    # Mode 4, 1640x1232, aspect ratio 4:3, 1/10 to 40 fps, full FoV
        height = 1232
    filename = sys.argv[3]
    camera.resolution = (width, height)
    camera.framerate = 30
    camera.shutter_speed = 0    # Automatic selection
    camera.ISO = 0              # Automatic selection
    camera.meter_mode = 'matrix'
    rawCapt = PiRGBArray(camera, size = (width, height))
    time.sleep(2)
    print '%0.2f: Initializing Camera Finished' % (time.time() - time_start)
    
    # set up aruco
    print '%0.2f: Initializing ArUco' % (time.time() - time_start)
    aruco_dic = aruco.Dictionary_get(aruco.DICT_5X5_50)
    print '%0.2f: Initializing ArUco Finished' % (time.time() - time_start)
    
    # set up sift
    print '%0.2f: Initializing SIFT' % (time.time() - time_start)
    grayImg = cv2.imread('/home/pi/speedtest/marker5.jpg', 0) # TODO PUT THIS IN
    sift = cv2.xfeatures2d.SIFT_create()
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    imgKP, imgDes = sift.detectAndCompute(grayImg, None)
    print '%0.2f: Initializing SIFT Finished' % (time.time() - time_start)
    
    # Run trials
    list_results = [[0]*6]*(num_pics + 1)
    list_results[0] = ['Trial', 
                       't (take pic)',
                       't (save pic)',
                       't (SIFT)',
                       'Found? (SIFT)',
                       't (ArUco)',
                       'Found? (ArUco)',
                       ]
    for i in range(num_pics):
        print '%0.2f: Trial %03d' % ((time.time() - time_start), i)
        list_results[i+1][0] = i
        
        time_meas = time.time()
        rawCapt.truncate(0)
        camera.capture(rawCapt, format = 'bgr', use_video_port = True)
        pic = rawCapt.array
        list_results[i+1][1] = time.time() - time_meas
        
        time_meas = time.time()
        path = 'pic_%s_Trial%03d.jpg' % (filename, i)
        cv2.imwrite(path, pic)
        list_results[i+1][2] = time.time() - time_meas
        
        list_results[i+1][3], list_results[i+1][4] = find_target(pic, 'sift')
        
        list_results[i+1][5], list_results[i+1][6] = find_target(pic, 'aruco')
    
    camera.close()
    print '%0.2f: Writing results to %s.csv' % ((time.time() - time_start), filename)
    with open(filename + '.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerows(list_results)
