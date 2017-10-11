#!/usr/bin/env python
"""
A script to coordinate instances of drone_control.py

This script takes an IP addresses representing the drone. It then launches and 
provides an interface to call known control functions for takeoff, movement, 
and precision landing on a target.

The script should be run on the command line as:

python base_station_landing.py 'primaryip'

Where the IP address is any valid IPv4 address format, such as
'192.168.0.1', or 'localhost'.

Command line arguments:
primary_ip      -- the IP of the primary drone
"""

import math
import threading
import numpy as np
import pprint
from code import interact
from contextlib import contextmanager
import nav_utils
import argparse
import requests
import json
import time
from MissionGenerator import MissionGenerator
from collections import deque
import gdp_access


class DroneCoordinator(threading.Thread):
    """
    A service to coordinate multiple instances of drone_control.py
    
    Instances of DroneCoordinator can also be called to directly enforce action
    on a drone
    """

    def __init__(self, primary_drone_ip, gdp_service=None):
        """
        Initialize an instance of DroneCoordinator

        If you're thinking about using this from another module, this is a good
        place to look for things you can set on runtime if desired. For example
        primary_height and secondary_height, the config file that is read, or
        how the addresses are constructed from the IPs.
        """
        super(DroneCoordinator, self).__init__()
        self.read_config('../database_files/mission_setup.json')
        self.primary_drone_addr = 'http://' + primary_drone_ip + ':5000/'
        self.fly_height = 5
        self.mission_generator = MissionGenerator()
        self.gdp_service = gdp_service
        self._stop_event = threading.Event()
        self._stop_event.clear()
        self.start()
        
    
    def run(self):
        gps_home = [37.863552, -122.249600]
        list_signpost_anomalies = []
        while not(self.stopped()):
            # Process information from gdp service
            if ((self.gdp_service != None) and self.gdp_service.isAlive()):
                b_trigger, list_signposts = self.gdp_service.check_trigger()
                if (b_trigger):
                    list_signpost_anomalies = list(set(list_signpost_anomalies.extend(list_signposts)))
            
            if (len(list_signpost_anomalies) > 0):
                for signpost_target in list_signpost_anomalies:
                    print('Anomaly detected. Go to %s' % (signpost_target['gps']))
                    '''
                    mission_to = self.create_find_target_and_land_mission(gps=signpost_target['gps'],
                                                                          b_download=True)
                    mission_back = self.create_find_target_and_land_mission(gps=gps_home,
                                                                          b_download=False)
                    '''
                list_signpost_anomalies = []
                print('All anomaly data collected. Go to home at %s' % (gps_home))
                
                
            time.sleep(0.1)
    
    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()    
    
    def generate_corner_banana(self):
        """Sometimes you need a corner banana"""
        intervals = range(0, 11, 2)
        south_points = [[0, point] for point in intervals]
        west_points = [[point, 0] for point in intervals] 
        diag_points = [[point, point] for point in intervals]
        #TODO: There has to be a better way to do this. My list comprehension
        # fu is not strong :(
        point_list = []
        for i in range(0, 6, 2):
            point_list.append(south_points[i])
            point_list.append(diag_points[i])
            point_list.append(west_points[i])

            point_list.append(west_points[i+1])
            point_list.append(diag_points[i+1])
            point_list.append(south_points[i+1])
        point_list = point_list[2:]
        point_list = [[lat, lon, 3] for lat, lon in point_list]


    def relative_coords(self, lat1, lon1, lat2, lon2):
        """
        Return the relative vector in meters between two GPS coordinates
        
        This returns the (North, East) distance in meters between two GPS
        coordinates (lat1, lon1), (lat2, lon2). So if (lat2, lon2) is 10 meters
        north and three meters east of (lat1, lon1), this returns (10, 3).
        Distances to the south or west are negative.
        """
        lat_dist = nav_utils.lat_lon_distance(lat1, lon1, lat2, lon1)
        lon_dist = nav_utils.lat_lon_distance(lat1, lon1, lat1, lon2)
        lat_dist = math.copysign(lat_dist, (lat2 - lat1))
        lon_dist = math.copysign(lon_dist, (lon2 - lon1))
        return [lat_dist, lon_dist]


    def relative_triangle(self, drone_addr, drone_name, point, radius=2):
        """Send a drone to fly a triangle around a GPS coordinate and return.

        This is the function intended to let the second drone 'investigate' the
        points found by the first drone in the demo.

        drone_addr -- address (not IP) of the drone you want to send
        drone_name -- name of the drone you want to send
        point -- the point to fly the triangle around, in [lat, lon] format
        radius -- the radius of the triangle, default 2 meters
        """
        self.launch_drone(drone_addr)
        time.sleep(20)
        latest_loc = self.get_latest_loc(drone_name)
        relative = self.relative_coords(
            latest_loc[0],
            latest_loc[1],
            point[0],
            point[1]
        )
        N = relative[0]
        E = relative[1]
        config_dict = {
            'shape':'triangle',
            'radius':radius,
            'loc_start':np.array([N, E]),
            'altitude': 5,
        }
        mission = self.mission_generator.createMission(config_dict)
        print mission
        self.send_mission(mission, drone_addr) 


    def circle_test(self, drone_address, relative):
        """Fly the drone in a circle to test generation of circle missions."""
        mission_generator = MissionGenerator()
        rel_point = relative['relative']
        N = rel_point[0]
        E = rel_point[1]
        offset = np.array([N, E])
        config = mission_generator.create_config_dict(
            'circle', 0, 0, 0, 3, 4, False, np.array([4,4])
        )
        mission = mission_generator.createMission(config)
        self.launch_drone(drone_address)
        self.send_mission(mission, drone_address)


    def read_config(self, filename):
        """Read a config file to find the current mission_name."""
        with open(filename) as fp:
            config = json.load(fp)
        self.mission_name = config['mission_name']


    def run_test_mission(self, filename, drone_address):
        """Launch a drone and send it on a mission from a file."""
        mission = self.load_mission(
                filename
        )
        self.launch_drone(drone_address)
        self.send_mission(mission, drone_address)


    def make_url(self, address, path):
        """Turn an address (not an IP) into a full URL for use with Flask."""
        url = address + path
        return url


    def launch_drone(self, drone_addr=None):
        """Launch the drone at drone_address."""
        if not(drone_addr):
            drone_addr = self.primary_drone_addr
        url = self.make_url(drone_addr, 'launch')
        start_time = json.dumps({'start_time':time.time()})
        r = requests.post(url, start_time)
        return r


    def send_mission(self, mission_json, drone_addr=None):
        """Send a mission (JSON string) to the drone at drone_address."""
        if not(drone_addr):
            drone_addr = self.primary_drone_addr
        url = self.make_url(drone_addr, 'mission')
        mission_string = json.dumps(mission_json)
        r = requests.post(url, mission_string)
        return r


    def send_landing_mission(self, drone_addr=None, target=None):
        """Send a landing target for the find_target_and_land mission to the 
           primary drone."""
        if not(drone_addr):
            drone_addr = self.primary_drone_addr
        url = self.make_url(drone_addr, 'find_target_and_land')
        target_info = json.dumps({'target':target})
        r = requests.post(url, target_info)
        
        return r
    

    def create_launch_land_test(self, target=None):
        mission_dict = { 
            'points': {
                'start_loc_hi': {
                    'N': 0,
                    'E': 0,
                    'D': 25
                    },
                'start_loc_low': {
                    'N': 0,
                    'E': 0,
                    'D': 7.5
                    }
            },
            'plan': [
                {
                    'action': 'launch',
                    'points': [],
                    'repeat': 0,
                },
                {
                    'action': 'go',
                    'points': ['start_loc_hi'],
                    'repeat': 0,
                },
                {
                    'action': 'go',
                    'points': ['start_loc_low'],
                    'repeat': 0,
                },
                {
                    'action': 'find_target_and_land_drone',
                    'points': [],
                    'target': target,
                },
                                {
                    'action': 'launch',
                    'points': [],
                    'repeat': 0,
                },
                {
                    'action': 'go',
                    'points': ['start_loc_hi'],
                    'repeat': 0,
                },
                {
                    'action': 'go',
                    'points': ['start_loc_low'],
                    'repeat': 0,
                },
                {
                    'action': 'find_target_and_land_drone',
                    'points': [],
                    'target': target,
                },
            ],
        }
        
        return mission_dict


    def create_find_target_and_land_mission(self, gps=None, rel=None, target=None, b_download=False):
        """Create a mission (JSON string) for going to a waypoint and then
        initializing the find_target_and_land mission and return it.
        """
        if (gps != None):
            targets = {
                'target_loc_hi': {
                    'lat': gps[0],
                    'lon': gps[1],
                    'alt': 20,
                },
                'target_loc_lo': {
                    'lat': gps[0],
                    'lon': gps[1],
                    'alt': 7,
                },
                'start_loc_hi': {
                    'N': 0,
                    'E': 0,
                    'D': 20
                }
            }
        elif (rel != None):
            targets = {
                'target_loc_hi': {
                    'N': rel[0],
                    'E': rel[1],
                    'D': 20,
                },
                'target_loc_lo': {
                    'N': rel[0],
                    'E': rel[1],
                    'D': 7,
                },
                'start_loc_hi': {
                    'N': 0,
                    'E': 0,
                    'D': 20
                }
            }
        else:
            targets = {
                'target_loc_hi': {
                    'N': 0,
                    'E': 0,
                    'D': 20,
                },
                'target_loc_lo': {
                    'N': 0,
                    'E': 0,
                    'D': 7,
                },
                'start_loc_hi': {
                    'N': 0,
                    'E': 0,
                    'D': 20
                }
            }
                
        mission_dict = { 
            'points': targets,
            'plan': [
                {   
                    'action': 'launch',
                    'points': [],
                    'repeat': 0,
                },
                {
                    'action': 'go',
                    'points': ['start_loc_hi'],
                    'repeat': 0,
                },
                {
                    'action': 'go',
                    'points': ['target_loc_hi'],
                    'repeat': 0,
                },
                {
                    'action': 'go',
                    'points': ['target_loc_lo'],
                    'repeat': 0,
                },
                {
                    'action': 'find_target_and_land_drone',
                    'points': [],
                    'target': target,
                },
                # add action for downloading data
            ],
        }
        
        return mission_dict


    def create_point_mission(self, action, relative_point, name):
        """Create a mission (JSON string) and return it.

        action -- any valid action that can be used by the mission interface,
                  currently probably only works with 'go' and maybe 'land'.
        relative_point -- point in NED format to be used for the mission
        name -- the name of the point (points in missions need names,
                this can be whatever string your heart desires)
        """
        mission_dict = { 
            'points': {
                name: {
                    'N': relative_point[0],
                    'E': relative_point[1],
                    'D': relative_point[2],
                },
            },
            'plan': [
                {
                    'action': action,
                    'points': [name],
                    'repeat': 0,
                },
            ],
        }
        return mission_dict


    def load_mission(self, filename):
        """Load a mission (JSON string) from a JSON file."""
        with open(filename) as fp:
            mission = json.load(fp)
        return mission


    def get_ack(self, drone_address):
        """Request (and return) an acknowledgement from the drone."""
        url = self.make_url(drone_address, 'ack')
        r = requests.get(url)
        return r

def run_test():
    gps_home = [37.863552, -122.249600]
    gps_signpost = [37.863636, -122.249353]
    
    mission_to = dc.create_find_target_and_land_mission(gps=gps_signpost, b_download=False)
    mission_back = dc.create_find_target_and_land_mission(gps=gps_home, b_download=False)
    
    for t in xrange(5,0,-1):
        print(t)
        time.sleep(1)   
    
    dc.send_mission(mission_to)
    dc.send_mission(mission_back)

if __name__ == '__main__':
    #parser = argparse.ArgumentParser()
    #parser.add_argument('primary_ip')
    #args = parser.parse_args()

    DEBUG = False
    
    if not(DEBUG):
        # Initialize Pretty Printer for an eye-pleasing interface
        pp = pprint.PrettyPrinter(indent=4)
        
        # Initialize GDP interface
        list_addr_base = [#'edu.berkeley.eecs.c098e5120003']#,
                          #'edu.berkeley.eecs.c098e5120011',
                          'edu.berkeley.eecs.c098e512000a']
                          #'edu.berkeley.eecs.c098e5120010']                  
        list_addr_sensors = ['signpost_gps.v0-0-1',
                             'signpost_energy.v0-0-1',
                             'signpost_radio.v0-0-1',
                             #'signpost_audio_frequency.v0-0-1',
                             'anomalies.v0-0-1']                       
                             #'signpost_microwave_radar.v0-0-1',
                             #'signpost_ambient.v0-0-1',
                             #'signpost_ucsd_air_quality.v0-0-1']
        list_id_sensors = ['signpost_gps',
                           'signpost_energy',
                           'signpost_radio',
                           'signpost_audio_frequency',
                           'anomalies']
                           #'signpost_microwave_radar',
                           #'signpost_ambient',
                           #'signpost_ucsd_air_quality']
        list_addr_dict = []
        for addr_base in list_addr_base:
            for ind, addr_sensor in enumerate(list_addr_sensors):
                temp = {'id_signpost':addr_base, 
                        'id_sensor':list_id_sensors[ind], 
                        'addr':(addr_base + '.' + addr_sensor)}
                list_addr_dict.append(temp)
        
        gdp_processor = gdp_access.GDPDataProcessor(list_addr_dict)
    else:
        gdp_processor = None
    
    # Initialize Drone Coordinator
    dc = DroneCoordinator('192.168.43.162', gdp_service=gdp_processor)  #args.primary_ip)
    #test_mission = dc.create_launch_land_test()
    #dc.launch_drone(dc.primary_drone_addr)
    
    #interact(local=locals())
    time.sleep(120)
    
    if not(DEBUG):
        gdp_processor.stop()
    