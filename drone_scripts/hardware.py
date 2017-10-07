"""
Provide 'drivers' for the various hardware peripherals the drone can have.

This currently only has the AirSensor object, the RF sensor object should
probably be merged into this file, or this file should be renamed.
"""
import threading
#import json
#import random
import math
#import serial
import time
#import drone_control
#import dronekit
#from subprocess import Popen, PIPE, call
#import sys
from pubsub import pub

import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera
import os
import sys
from time import localtime, strftime

class LandingCamera(threading.Thread):

    def __init__(self, simulated=False, target=None):
        super(LandingCamera, self).__init__()
        #self.daemon = True
        self._simulated = simulated
        if (simulated == False):
            self._width = 640
            self._height = 480
            self._xfov = 62.2 * math.pi/180
            self._yfov = 48.8 * math.pi/180
            self._camera = PiCamera()
            self._camera.resolution = (self._width, self._height)
            self._camera.framerate = 60
            self._camera.shutter_speed = 0
            self._camera.ISO = 0
            self._camera.meter_mode = 'matrix'
            self._rawCapt = PiRGBArray(self._camera, size = (self._width, self._height))
            self._aruco_dic = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self._stop_event = threading.Event()
        self._stop_event.clear() # Unnecessary
        self._notpause_event = threading.Event()
        self._notpause_event.set() # Allow thread to run
        self.start()


    def run(self):
        cameratest = False
        CPUusagetest = False
        
        save_delay = 0.5
        time_start = time.time()
        if (self._simulated == False):
            while not(self.stopped()):
                self._notpause_event.wait(2)

                self._take_pic()
                results = self._find_target(self._rawCapt.array)
                
                if (results['found'] == True):
                    print ("Hardware.py: Target found")
                    if ((time.time() - time_start) >= save_delay):
                        path = str(sys.path[0]) + '/Pics/' + strftime("%Y_%m_%d__%I_%M_%S", localtime()) + '_Found.jpg'
                        cv2.imwrite(path, self._rawCapt.array)
                        time_start = time.time()   
                else:
                    print ("Hardware.py: Target not found")
                    time_start = time.time() - save_delay # Save next successful photo
                    path = str(sys.path[0]) + '/Pics/' + strftime("%Y_%m_%d__%I_%M_%S", localtime()) + '_Fail.jpg'
                    cv2.imwrite(path, self._rawCapt.array)          

                if (cameratest == True):
                    # take picture but send fake messages to see if camera causes interference
                    results = {'xoffset': 0.3, 'yoffset': 0, 'distance': 1, 'found':True}
                    self._callback(results)
                else:
                    self._callback(results)

                time.sleep(0.01)
            self._camera.close()

        else:
            distance_sim = 4

            while not(self.stopped()):
                self._notpause_event.wait(2)
                if (CPUusagetest == True):
                    # process picture on file to test CPU usage
                    pic = cv2.imread('/home/pi/drone_scripts/seelab_drones_landing/launch/Found_2017_09_18__11_05_12.jpg', 0)
                    dc = self._find_target(pic)
                    
                if (distance_sim == 0):
                    results = {'xoffset': 0.3, 'yoffset': 0, 'distance': 0, 'found':True}
                else:
                    results = {'xoffset': 0.3, 'yoffset': 0, 'distance': distance_sim, 'found':True}
                    distance_sim = distance_sim - 0.1

                self._callback(results)
                time.sleep(0.25)


    def get_proc_id(self, b_only_pid=False):
        ''' Gets the class name and process id of the thread '''
        if (b_only_pid == True):
            return os.getpid()
        else:
            return {'name':self.__class__.__name__, 'pid':os.getpid()}


    def set_resolution(self, width, height):
        '''Change the resolution of the camera. Standard settings can be found
           at https://picamera.readthedocs.io/en/release-1.13/fov.html .
           
           #	Resolution	Aspect   Framerates	        Video	FoV	    Binning
           1	1920x1080	16:9	 1/10 <= fps <= 30	x	 	Partial	None
           4	1648x1232	4:3	     1/10 <= fps <= 40	x	 	Full	2x2
           5	1640x922	16:9	 1/10 <= fps <= 40	x	 	Full	2x2
           6	1280x720	16:9	 40 < fps <= 90	    x	 	Partial	2x2
           7	640x480	    4:3	     40 < fps <= 90	    x	 	Partial	2x2
        '''
        self._width = width
        self._height = height
        self._xfov = 62.2 * math.pi/180
        self._yfov = 48.8 * math.pi/180
        self._camera.resolution = (self._width, self._height)


    def stop(self):
        self._stop_event.set()


    def pause(self):
        self._notpause_event.clear()


    def resume(self):
        self._notpause_event.set()


    def stopped(self):
        return self._stop_event.is_set()


    def distance(self, x1, y1, x2, y2):
	    return math.hypot(x2-x1, y2-y1)


    def _callback(self, land_data):
        pub.sendMessage("sensor-messages.landingcam-data", arg1=land_data)


    def _take_pic(self):
        self._rawCapt.truncate(0)
    	self._camera.capture(self._rawCapt, format = "bgr", use_video_port = True)


    def _find_target(self, pic):
        corners, ids, rejects = aruco.detectMarkers(pic, self._aruco_dic)
        self._rawCapt.array = aruco.drawDetectedMarkers(pic, corners, ids)

        numMarkers = len(corners)

        # TODO Fix to catch error cases
        if numMarkers == 1:
            if (ids[0][0] == 5):
                print ("Hardware.py: Found ID 5: use big target")
                data = self._calculate_xyz(corners[0][0], 0.20)

            elif (ids[0][0] == 8):
                print ("Hardware.py: Found ID 8: use small target")
                data = self._calculate_xyz(corners[0][0], 0.07)

            else:
                print ("Hardware.py: Found ID %d: revert to small target" % ids[0][0])
                data = self._calculate_xyz(corners[0][0], 0.07)

        # TODO Will the smaller marker always come second in the list?
        elif numMarkers == 2:
            print ("Hardware.py: Found both: use small target")
            data = self._calculate_xyz(corners[1][0], 0.07)

        else:
            print("Hardware.py: Corners not found")
            data = {'xoffset': -1, 'yoffset': -1, 'distance': -1, 'found':False}

        return data


    def _calculate_xyz(self, corners, size):
        sumx = 0
        sumy = 0

        for pos in corners:
            sumx = sumx + pos[0]
            sumy = sumy + pos[1]

        avgx = sumx/4
        avgy = sumy/4

	    # Adjusting for camera rotation
	    # From testing on 6 Oct 2017
        # Red arms are top photo (-Y)
		#   +Y in cmd msg
    	# White arms are bottom photo (+Y)
		#   -Y in cmd msg
    	# Battery connector is left photo (-X)
		#   +X in cmd msg
    	# Battery bumper is right photo (-X)
		#   -X in cmd msg

        # X from photo is width, starting from left-right
        # Y from photo is height, starting from top-down

        # So +X in cmd msg is -X of photo
        # and +Y in cmd msg is -Y of photo

        x = -(avgx - self._width/2)*self._xfov/self._width

        y = -(avgy - self._height/2)*self._yfov/self._height

        # TODO Update with more accurate way to calculate distance
        side1 = self.distance(corners[0][0], corners[0][1], corners[1][0], corners[1][1])
        side2 = self.distance(corners[0][0], corners[0][1], corners[2][0], corners[2][1])
        area = side1 * side2

        if size == .07:
            z = (4201.1 * (area ** (-0.496)))/100
        else:
            z = (12632 * (area ** -(0.502)))/100

        data = {'xoffset': x, 'yoffset': y, 'distance': z, 'found': True}
        return data


class SavePhoto(threading.Thread):
    '''Thread to save a photo without slowing down main LandingCamera loop.
       From initial testing, saving takes an avg of 51 ms. With a photo capture
       speed of 109 ms average and ArUco processing of 83 ms, this can 
       potentially save 21% processing time or 1 extra photo a second.
       
       Need to be cautious of race conditions between saving photo and erasing 
       prior to taking a new photo.
    '''
    
    def __init__(self, pic, path):
        super(SavePhoto, self).__init__()
        self.pic = pic
        self.path = path
        self.start()
        
    def run(self):
        cv2.imwrite(self.path, self.pic)


'''class AirSensor(threading.Thread):
    """Provide an interface to Christine/Michael's air sensor.

    This class can either connect to an existing air sensor plugged into one of
    the drone's RPi's USB ports or it can generate fake CO2 readings. It
    inherits from python's threading.Thread class and should probably always be
    a daemon.

    """

    def __init__(self, simulated=False):
        """Construct an instance of the AirSensor class.

        simulated -- Whether or not to connect to a real air sensor
        """
        super(AirSensor, self).__init__()
        self.daemon = True
        self._simulated = simulated
        self._delay = 5
        self._serial_speed = 9600
        self._serial_port = '/dev/ttyACM0'
        self._timeout = 1
        self._connection = None
        if not self._simulated:
            self.connect_to_sensor()
            print("connected to air")
        self.start()

    def connect_to_sensor(self):
        """Connect to an air sensor and prepare it for reading data. Should add
        ability to take exception if unable to connect and print error msg"""
        try:
            self._connection = serial.Serial(
                    self._serial_port,
                    self._serial_speed,
                    timeout= self._timeout
            )
            # Ask Michael about why this is necessary
            self._connection.write('{"usb_en":true}')
            # time.sleep(0.01)
            self._connection.write('{"co2_en":true}')
        except serial.serialutil.SerialException as e:
            sys.stderr.write("Could not open serial for RealAirSensor\n")
            sys.stderr.write(e.__repr__())

    def _callback(self, air_data):
        """Publish air sensor data on the appropriate PyPubSub topic."""
        pub.sendMessage("sensor-messages.air-data", arg1=air_data)

    def run(self):
        """Gather and publish data periodically while the thread is alive."""
        if not self._simulated:
            if self._connection is None:
                print("DEBUG: no air conn.")
                return
            while(True):
                data = self.get_reading()
                if data is not None:
                    #print "Got air sensor reading: {}".format(data)
                    self._callback(data)
                else:
                    print("DEBUG: no air data")
        else:
            while(True):
                data = self.generate_fake_reading()
                #print "Got air sensor reading: {}".format(data)
                self._callback(data)
                time.sleep(self._delay / drone_control.Pilot.sim_speedup)

    def get_reading(self):
        """Get a reading from the air sensor and return it."""
        while True:
            latest_raw = self._connection.readline()
            if latest_raw:
                try:
                    readings = json.loads(latest_raw)
                except Exception as e:
                    print "JSON error"
                    return None
		return readings

    def generate_fake_reading(self):
        """Generate a fake CO2 reading.

        This will generate mostly ~410, occasionally higher values.
        """
        raw = random.expovariate(1)
        reading = max(raw, 2) * 200 + random.uniform(5, 15)
        reading_dict = {"CO2":reading}
        return reading_dict
'''

def output_cb(arg1=None):
    print arg1

if __name__ == '__main__':
    print "hey"
    timeout = 3

    pub.subscribe(output_cb, 'sensor-messages.landingcam-data')
    landing_cam = LandingCamera()


    for _ in xrange(timeout):
        time.sleep(1)

    landing_cam.stop()
