'''
Provide the main classes for the drone-based part of the project.

FlaskServer:
    Flask server that runs on each drone to handle http requests.

LoggerDaemon:
    Daemon thread to receive all logging info and store it in the database.

Pilot:
    Class that interfaces directly with the Dronekit vehicle object.

Navigator:
    Class to handle the high-level navigation and mission execution.
'''

import dronekit
from dronekit import VehicleMode, LocationGlobalRelative
import copy
import dronekit_sitl
from nav_utils import relative_to_global, get_ground_distance
import nav_utils
import threading
import json
import tempfile
import time
import hardware
import logging
import psutil
import os
from pubsub import pub
from flask import Flask, request
from collections import deque
from pymavlink import mavutil

app = Flask(__name__)


class FlaskServer(threading.Thread):
    '''Provide a flask server for managing HTTP requests to/from the drone.

    This class is set up using the Flask documentation. It inherits from the
    threading.Thread class, and therfore needs a run() method and to specify
    self.daemon.

    This class communicates with the other objects on the drone via the
    PyPubSub API.

    '''

    def __init__(self):
        '''Construct an instance of FlaskServer.'''
        super(FlaskServer, self).__init__()
        self.daemon = True
        self.start()


    @app.route('/launch', methods=['POST'])
    def launch_func():
        '''Post the current time and a launch command to the launch topic.'''
        print 'entered flask launch function'
        time = json.loads(request.data)
        pub.sendMessage(
            'flask-messages.launch',
            arg1=time,
        )
        return 'received launch command'


    @app.route('/mission', methods=['POST'])
    def mission_func():
        '''Receive and then post a JSON mission to the mission topic.'''
        print 'entered flask mission function'
        #print request.data
        mission = json.loads(request.data)
        pub.sendMessage(
            'flask-messages.mission',
            arg1=mission,
        )
        return 'received mission'


    @app.route('/RTL_and_land', methods=['GET'])
    def RTL_and_land_func():
        '''Publish True to the RTL topic.'''
        print 'entered flask RTL function'
        pub.sendMessage(
            'flask-messages.RTL',
            arg1=True,
        )
        return 'RTL and landing'


    @app.route('/land', methods=['GET'])
    def land_func():
        '''Publish True to the land topic.'''
        print 'entered flask land function'
        pub.sendMessage(
            'flask-messages.land',
            arg1=True,
        )
        return 'landing'


    @app.route('/ack', methods=['GET'])
    def ack_func():
        '''Send an acknowledgement to whoever sent the request.'''
        print 'entered flask ack function'
        return 'ack'

    @app.route('/find_target_and_land', methods=['POST'])
    def find_target_and_land_func():
        '''Publish the target location and (optional) target symbols to the
           find_target_and_land topic.'''
        print 'entered flask find_target_and_land function'
        target_info = json.loads(request.data)
        pub.sendMessage(
            'flask-messages.find_target_and_land',
            arg1=target_info,
        )
        return 'received target info for find_target_and_land'


    def run(self):
        '''Start the flask server on the local ip and then start the thread.'''
        app.run('0.0.0.0')


    def get_proc_id(self, b_only_pid=False):
        ''' Gets the class name and process id of the thread '''
        if (b_only_pid == True):
            return os.getpid()
        else:
            return {'name':self.__class__.__name__, 'pid':os.getpid()}


class LoggerDaemon(threading.Thread):
    '''Provide a class to receive logging data and store it in the database.

    LoggerDaemon is the central class for all the logging and data storage that
    the drones need. It can read data from and store data in the database,
    keeps track of the current time, and receives data from the navigator and
    sensors via the PyPubSub API.

    This class inherets from threading.Thread and is a daemon.

    '''

    # TODO: put mission_setup in sane place and fix path
    def __init__(self, pilot, drone_name, config_file='../database_files/mission_setup.json'):
        '''Construct an instance of LoggerDaemon.

        This stores the Pilot object that created the LoggerDaemon, sets the
        object as a daemon, sets up the database connection and then finds the
        database records associated with the drone and sensor(s) it's running
        on.

        pilot -- a Pilot object to allow the LoggerDaemon access to the
                 dronekit vehicle object.
        drone_name -- the name of the drone the LoggerDaemon is running on,
                      should be a name in the database's drones table.
        config_file -- configuration file to get the current mission_name and
                       drone info from.
        '''

        super(LoggerDaemon, self).__init__()
        self._pilot = pilot
        self.daemon = True
        self._start_seconds = None
        self.read_config(config_file, drone_name)
        self.setup_logging()
        self.setup_subs()
        self._stop_event = threading.Event()
        self._stop_event.clear() # Unnecessary
        self.start()


    def setup_logging(self):
        filename = time.strftime('Log_' + self.drone_info['name'] + '_' +
                                 self.drone_info['mission'] + '_%Y%m%d_%H%M%S.log',
                                 time.localtime())
        logging.basicConfig(filename='Logs/' + filename, level=logging.DEBUG,
                            format='%(asctime)s, %(message)s',
                            datefmt='%Y%m%d %H%M%S')
        logging.info('Initialized logger')
        logging.info('\'drone_name\':' + self.drone_info['name'] + \
                     ', \'mission_name\':' + self.drone_info['mission'])


    def read_config(self, filename, drone_name):
        '''Read the mission config file and store mission and drone information

        This reads and stores the mission name and sensors associated with the
        current mission and drone, in preparation for finding their records in
        the database.
        '''

        self.drone_info = {
                           'mission':'unknownmission',
                           'name':'unknowndrone'
                           }

        try:
            with open(filename) as fp:
                config = json.load(fp)
            for drone in config['drones']:
                if drone['name'] == drone_name:
                    self.drone_info = drone
            self.drone_info['mission'] = config['mission_name']
        except Exception:
            print 'ERROR Unable to load config file'


    def mission_time(self):
        '''Return the current time in unix era format.

        This function is necessary because the clock on the Raspberry Pi isn't
        real-time and so time.time() returns something bad and wrong, but
        internally consistent.
        '''
        if self._start_seconds is not None:
            miss_seconds = time.time() - self._start_seconds
            miss_time = miss_seconds + self._launch_time
            '''
            print 'Calculated time: {0}\n miss_seconds: {1}\n start_seconds: {2}\n'.format(
                miss_time,
                miss_seconds,
                self._start_seconds,
            )
            '''
            return miss_time
        else:
            return None


    def setup_subs(self):
        '''Set up the subscribers and callbacks for relevant pubsub topics.'''
        pub.subscribe(self.landingcam_data_cb, 'sensor-messages.landingcam-data')
        pub.subscribe(self.mission_data_cb, 'nav-messages.mission-data')
        pub.subscribe(self.launch_cb, 'flask-messages.launch')


    def launch_cb(self, arg1=None):
        '''Set start_seconds and launch_time to appropriate values.'''
        if not self._start_seconds:
            time_dict = arg1
            self._start_seconds = time.time()
            self._launch_time = time_dict['start_time']
            print 'LoggerDaemon got {0}, {1} from launch'.format(arg1, self._launch_time)


    def mission_data_cb(self, arg1=None):
        '''Add incoming mission event to log.'''
        print 'entered mission_data_cb'
        # event_dict = copy.deepcopy(arg1)
        # event_json = event_dict
        # TODO Decide if it should be handled here or in landing_msg_cb in navigator
        #logging.info('\'mission_data\', ' + ', '.join('\'%s\':%r' % (key,val) \
        #             for (key,val) in arg1.iteritems()))


    def wifi_data_cb(self, arg1=None):
        '''Add incoming wifi data to log.'''
        #print 'wifi callback entered: {}'.format(arg1)
        current_time = self.mission_time()
        if current_time is not None:
            print 'entered wifi_data_cb'
            data = copy.deepcopy(arg1)
        # TODO Add logging to file


    def landingcam_data_cb(self, arg1=None):
        '''Add incoming landing camera data to log.'''
        current_time = self.mission_time()
        if current_time is not None:
            print 'entered landingcam_data_cb'
            print arg1
            logging.info('\'landingcam_data\', ' + ', '.join('\'%s\':%r' % (key,val)\
                         for (key,val) in arg1.iteritems()))



    def rel_from_glob(self, global_loc):
        '''Return the relative coordinates of a GPS location in JSON NE format.

        global_loc should be a location object from dronekit that has a lat and
        a lon attribute. The returned value is a JSON string of a dictionary so
        that it can be put directly into the relative attribute of a GPS sensor
        record.
        '''
        home = self._pilot.vehicle.home_location
        north = nav_utils.lat_lon_distance(
            home.lat,
            home.lon,
            global_loc.lat,
            home.lon,
        )
        east = nav_utils.lat_lon_distance(
            home.lat,
            home.lon,
            home.lat,
            global_loc.lon,
        )
        return json.dumps({'relative':[north, east]})


    def sys_recorder(self):
        '''Record the drone's current GPS location and cpu/ram usage every
           second.'''
        p = psutil.Process()

        while not(self.stopped()):
            # GPS Data
            location_global = self._pilot.get_global_location()
            current_time = self.mission_time()
            if (location_global
                    and location_global.lat
                    and location_global.lon
                    and location_global.alt
                    and current_time):
                logging.info('\'gps\', \'lat\':%0.8f, \'lon\':%0.8f, \'alt\':%0.3f, \'time\':%0.2f' \
                             % (location_global.lat, location_global.lon,
                                location_global.alt, current_time))
                gps_info= {'eph': self._pilot.vehicle.gps_0.eph,
                           'epv':self._pilot.vehicle.gps_0.epv,
                           'fix': self._pilot.vehicle.gps_0.fix_type,
                           'num_sats': self._pilot.vehicle.gps_0.satellites_visible}
                logging.info('\'gps_info\', ' + str(gps_info).strip('{}'))

            # System utilization
            # TODO Add in process names and process ids to each thread that
            #      we want to monitor and create and maintain a list of threads
            #      to track
            cpu_usage_total = psutil.cpu_percent(interval=None, percpu=True)
            ram_usage_total = psutil.virtual_memory()
            cpu_usage_proc = p.cpu_percent(interval=None)

            logging.info('\'sys_util\', \'cpu_total\': [' + ', '.join(map(str, cpu_usage_total)) + '], ' + \
                         '\'ram_total\': %0.2f, \'cpu_proc\': %0.2f' % (ram_usage_total.percent, cpu_usage_proc))
            #print 'CPU usage: %0.2f, ram usage: %0.2f' % (sum(cpu_usage_total)/float(len(cpu_usage_total)),
            #                                              ram_usage_total.percent)

            time.sleep(1)


    def get_proc_id(self, b_only_pid=False):
        ''' Gets the class name and process id of the thread '''
        if (b_only_pid == True):
            return os.getpid()
        else:
            return {'name':self.__class__.__name__, 'pid':os.getpid()}


    def run(self):
        '''Start the thread object.'''
        self.sys_recorder()


    def stop(self):
        ''' Call to safely stop the thread'''
        self._stop_event.set()


    def stopped(self):
        ''' Check if thread is stopped or running '''
        return self._stop_event.is_set()


class Pilot(object):
    '''Provide basic piloting functionality and interface to sensors.

    This class is intended to contain all the basic fuctionality of flying the
    drone from place to place, interfacing with sensors, and accessing/updating
    the information in dronekit's vehicle object (which in turn is the API used
    to interact with the instance of ardupilot running on the drone).

    Chris originally wrote most of this class, so if my documentation is
    unclear or lacking he might be able to provide clarification. A lot of the
    code in this class is probably superflous by this point (for example, I'm
    pretty sure self.hold_altitude does nothing) but it works, and refactoring
    hasn't been high on the list of priorities, so if you're going to dig into
    this class be aware it might be confusing.

    '''

    sim_speedup = 1
    instance = -1


    def __init__(self, simulated=False, simulated_landing_camera=False, sim_speedup=None):
        '''Construct an instance of the Pilot class.

        This instantiates the sensors, real or simulated, and the LoggerDaemon.

        simulated -- Are we running this on the simulator?
        simulated_pi_camera -- Is a camera attached or being simulated?
        sim_speedup -- Factor to speed up the simulator, e.g. 2.0 = twice as
                       fast. Somewhat glitchy on higher values.
        '''

        Pilot.instance += 1
        self.instance = Pilot.instance
        print 'I\'m a pilot, instance number {0}'.format(self.instance)
        self.groundspeed = 0.5
        if sim_speedup is not None:
            Pilot.sim_speedup = sim_speedup  # Everyone needs to go the same speed
            simulated = True

        # Altitude relative to starting location
        # All further waypoints will use this altitude
        self.hold_altitude = None

        self.vehicle = None
        self.sitl = None

        LoggerDaemon(self, 'Goose')


    def bringup_drone(self, connection_string=None):
        '''Connect to a dronekit vehicle or instantiate an sitl simulator.

        Call this once everything is set up and you're ready to fly.

        This is some deep magic related to running multiple drones at once. We
        don't technically need it, since the network architecture I've set up
        has the drones all be independent (if you want to simulate multiple
        drones with the current setup you run multiple instances of the SITL
        simulator). Chris got this all working by talking with the dronekit
        devs, so he knows a lot more about it than I do.

        connection_string -- Connect to an existing mavlink (SITL or the actual
                             ArduPilot). Provide None and it'll start its own
                             simulator.
        '''

        if not connection_string:
            # Start SITL if no connection string specified
            print 'Starting SITL'
            self.sitl = dronekit_sitl.SITL()
            self.sitl.download('copter', '3.3', verbose=True)
            sitl_args = ['--model', 'quad',
                         '--home=32.990756,-117.128362,243,0',
                         '--speedup', str(Pilot.sim_speedup),
                         '--instance', str(self.instance)]
            working_dir = tempfile.mkdtemp()
            self.sitl.launch(sitl_args,
                             verbose=True,
                             await_ready=True,
                             restart=True,
                             wd=working_dir)
            time.sleep(6)  # Allow time for the parameter to go back to EEPROM
            connection_string = 'tcp:127.0.0.1:{0}'.format(5760 + 10 * self.instance)
            #connection_string = 'tcp:127.0.0.1:14550')
            new_sysid = self.instance + 1
            vehicle = dronekit.connect(connection_string, wait_ready=True)
            while vehicle.parameters['SYSID_THISMAV'] != new_sysid:
                vehicle.parameters['SYSID_THISMAV'] = new_sysid
                time.sleep(0.1)
            time.sleep(5)  # allow eeprom write
            vehicle.close()
            self.sitl.stop()
            # Do it again, and this time SYSID_THISMAV will have changed
            self.sitl.launch(sitl_args,
                             verbose=True,
                             await_ready=True,
                             restart=True,
                             use_saved_data=True,
                             wd=working_dir)
            self.vehicle = dronekit.connect(connection_string, wait_ready=True)
            print vehicle
        else:
            # Connect to existing vehicle
            print 'Connecting to vehicle on: %s' % connection_string
            print 'Connect to {0}, instance {1}'.format(connection_string, self.instance)
            self.vehicle = dronekit.connect(connection_string, wait_ready=True)
            print 'Success {0}'.format(connection_string)


    def stop(self):
        '''Properly close the vehicle object.'''
        self.shutdown_vehicle()


    def arm_and_takeoff(self, target_alt):
        '''Arm vehicle and fly to target_alt.'''
        if self.vehicle.armed == True:
            return
        self.hold_altitude = target_alt
        print 'Basic pre-arm checks'
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print ' Waiting for vehicle {0} to initialise...'.format(self.instance)
            # print(self.vehicle.gps_0.fix_type)
            time.sleep(1.0 / Pilot.sim_speedup)
            if self.vehicle.gps_0.fix_type < 2:
                print('     Vehicle {0} waiting for GPS fix...'.format(self.instance))

        print 'Getting vehicle commands'
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()

        print 'Home location is ' + str(self.vehicle.home_location)

        print 'Arming motors'
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print ' Waiting for vehicle {0} to arm...'.format(self.instance)
            self.vehicle.mode = VehicleMode('GUIDED')
            self.vehicle.armed = True
            time.sleep(1.0 / Pilot.sim_speedup)

        print('Taking off! Target altitude is %f meters' % target_alt)
        self.vehicle.simple_takeoff(target_alt)  # Take off to target alt

        # Wait until the self.vehicle reaches a safe height before processing
        # the goto (otherwise the command after Vehicle.simple_takeoff will
        # execute immediately).
        while True:
            print 'Vehicle {0} altitude: {1}'.format(self.instance,
                                                     self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if (self.vehicle.location.global_relative_frame.alt >=
                    target_alt * 0.90):
                print 'Reached takeoff altitude of {0} meters'.format(target_alt)
                break
            time.sleep(1.0 / Pilot.sim_speedup)


    def poll(self):
        '''Return string with the vehicle's current location (local frame).'''
        return 'Location: ' + str(self.vehicle.location.local_frame)


    def get_local_location(self):
        '''Return the vehicle's NED location as a LocationGlobalRelative.'''
        if self.vehicle is not None and self.vehicle.location is not None:
            loc = self.vehicle.location.local_frame
            if loc.north is not None and loc.east is not None:
                return self.vehicle.location.local_frame
        return None


    def get_attitude(self):
        '''Return the current attitude.'''
        if self.vehicle is not None:
            return self.vehicle.attitude


    def get_velocity(self):
        '''Return the current velocity.'''
        if self.vehicle is not None:
            vel = self.vehicle.velocity
            if vel.count(None) == 0:
                return self.vehicle.velocity
        return None


    def get_global_location(self):
        '''Return the vehicle's current GPS location as a LocationGlobal.'''
        if self.vehicle is not None and self.vehicle.location is not None:
            loc = self.vehicle.location.global_frame
            if loc.lat is not None and loc.lon is not None:
                return self.vehicle.location.global_frame
        return None


    def goto_relative(self, north, east, altitude_relative):
        '''Go to a NED location.

        This is basically just a wrapper for goto_waypoint to allow it to use
        NED coordinates. nort, ease and altitude_relative should be in meters.
        '''
        location = relative_to_global(self.vehicle.home_location,
                                      north,
                                      east,
                                      altitude_relative)
        self.goto_waypoint(location)


    def goto_waypoint(self, global_relative, ground_tol=0.8, alt_tol=1.0, speed=50):
        '''Go to a waypoint and block until we get there.

        global_relative -- A LocationGlobalRelative, the waypoint
        ground_tol -- the error tolerance for the horizontal distance from the
                      waypoint in meters.
        alt_tol -- the error tolerance for the vertical distance from the
                   waypoint in meters.
        speed -- the maximum speed the drone will try to move at, in cm/s. Note
                 that there are cases where the drone will move faster than
                 this, so DO NOT use this as a safety cutoff.
        '''
        self.vehicle.parameters['WPNAV_SPEED'] = speed
        if self.vehicle.mode != 'GUIDED':
            print 'Vehicle {0} aborted goto_waypoint due to mode switch to {1}'.format(self.instance, self.vehicle.mode.name)
            return False
        #TODO: May want to replace simple_goto with something better
        self.vehicle.simple_goto(global_relative, groundspeed=self.groundspeed)
        good_count = 0  # Count that we're actually at the waypoint for a few times in a row
        while self.vehicle.mode.name == 'GUIDED' and good_count < 3:
            grf = self.vehicle.location.global_relative_frame
            offset = get_ground_distance(grf, global_relative)
            alt_offset = abs(grf.alt - global_relative.alt)
            if offset < ground_tol and alt_offset < alt_tol:
                good_count += 1
            else:
                good_count = 0
            time.sleep(0.2)
        print 'Arrived at global_relative.'
        return True


    def RTL_and_land(self):
        '''Return to home location and land the drone.'''
        print 'Vehicle {0} returning to home location'.format(self.instance)
        self.goto_relative(0, 0, 5)
        print 'Vehicle {0} landing'.format(self.instance)
        self.vehicle.mode = VehicleMode('LAND')


    def land_drone(self):
        '''Land the drone at its current location.'''
        print 'Vehicle {0} landing'.format(self.instance)
        self.vehicle.mode = VehicleMode('LAND')


    def return_to_launch(self):
        '''Return to the home location.'''
        print 'Vehicle {0} returning to home location'.format(self.instance)
        self.goto_relative(0, 0, 15)


    def shutdown_vehicle(self):
        '''Properly close the vehicle object.'''
        print 'Closing vehicle'
        self.vehicle.close()


    def send_land_message(self, x, y, z):
    	message = self.vehicle.message_factory.landing_target_encode(
    		0,	# ms since boot, a time stamp
    		0,	# target ID for case of multiple targets
    		mavutil.mavlink.MAV_FRAME_BODY_NED,	# MAV_FRAME enum specifying frame, try mavutil.mavlink.MAV_FRAME_BODY_NED
    		x,	# X-axis angular offset in radians of target from center of image
    		y,	# Y-axis angular offset
    		z,	# distance to the target from vehicle in meters
    		0,	# size of target in radians along x-axis
    		0)	# size along y-axis
        logging.info('\'send_land_message\', \'x offset\': %0.4f, \'y offset\': %0.4f, \'distance\': %0.4f' % (x, y, z))
    	self.vehicle.send_mavlink(message)
    	self.vehicle.flush()

    # def send_distance_message(self, dist):
    #     message = self.vehicle.message_factory.distance_sensor_encode(
    #         0,          # time since system boot, not used
    #         1,          # min distance cm
    #         10000,      # max distance cm
    #         dist,       # current distance, must be int
    #         0,          # type = laser?
    #         0,          # onboard id, not used
    #         mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
    #         0           # covariance, not used
    #     )
    #     logging.info('\'send_distance_message\', \'distance\': %0.4f' % dist)
    #     self.vehicle.send_mavlink(message)
    #     self.vehicle.flush()


    def get_proc_id(self, b_only_pid=False):
        ''' Gets the class name and process id of the thread '''
        if (b_only_pid == True):
            return os.getpid()
        else:
            return {'name':self.__class__.__name__, 'pid':os.getpid()}


class Navigator(object):
    '''Provide a class to manage high-level navigation and mission execution.

    This class is the compliment to the Pilot class. It manages the high-level
    execution of missions, instantiates the pilot and the Flask server, and
    sends mission logging information to the LoggerDaemon.

    '''

    def __init__(self, simulated=False, simulated_landing_camera=False, takeoff_alt=5):
        '''Construct an instance of the Navigator class.

        simulated -- Are we running this on the simulator?
        simulated_landing_camera -- Use simulated data if True, real landing data else
        takeoff_alt -- the height the drone should launch to in meters
        '''

        print 'I\'m a Navigator!'
        self._waypoint_index = 0
        self.takeoff_alt = takeoff_alt
        self.simulated = simulated
        self.simulated_landing_camera = simulated_landing_camera
        self.bringup_ip = None
        #should this be in the init function or part of the interface?
        #also should there be error handling?
        self.launch_mission = self.load_launch_mission()
        self.instantiate_pilot()
        self.setup_subs()
        FlaskServer()
        self.mission_queue = deque([])
        self.event_loop()


    def load_launch_mission(self):
        '''Load a mission for launching the drone.'''
        with open('launch_mission.json', 'r') as fp:
            mission = json.load(fp)
        return mission


    def event_loop(self):
        '''Maintain a queue for missions and execute them as they come in.'''
        print 'entering run loop'
        while True:
            try:
                time.sleep(0.1)
                if self.mission_queue:
                    next_mission = self.mission_queue.popleft()
                    self.execute_mission(next_mission)
                    if self.pilot.vehicle.mode != 'GUIDED':
                        self.mission_queue = deque([])

            except KeyboardInterrupt:
                self.pilot.RTL_and_land()
                break


    def setup_subs(self):
        '''Set up the PyPubSub subscribers to communicate with FlaskServer.'''
        print 'setting up subs'
        pub.subscribe(self.launch_cb, 'flask-messages.launch')
        pub.subscribe(self.mission_cb, 'flask-messages.mission')
        pub.subscribe(self.land_cb, 'flask-messages.land')
        pub.subscribe(self.RTL_cb, 'flask-messages.RTL')
        pub.subscribe(self.find_target_and_land_cb, 'flask-messages.find_target_and_land')


    def mission_cb(self, arg1=None):
        '''Add an incoming mission to the mission queue.'''
        print 'Navigator entered mission_cb'
        mission_dict = arg1
        self.mission_queue.append(mission_dict)


    def launch_cb(self, arg1=None):
        '''Launch the drone when a message is recieved on the launch topic.'''
        print 'Navigator entered launch callback'
        launch_mission = self.launch_mission
        self.mission_queue.append(launch_mission)
        #self.liftoff(5)


    def land_cb(self, arg1=None):
        '''Tell the pilot to land the drone.'''
        print 'Navigator entered land callback'
        self.pilot.land_drone()


    def find_target_and_land_cb(self, arg1=None):
        '''Tell the pilot to find the target and land the drone'''
        print 'Navigator entered find target and land callback'
        mission = {
            'points': {
                'home':{
                    'N': 0,
                    'E': 0,
                    'D': 0,
                }
            },
            'plan': [
                {
                    'action': 'find_target_and_land_drone',
                    'points': [],
                    'target': arg1,
                },
            ],
        }
        self.mission_queue.append(mission)


    def RTL_cb(self, arg1=None):
        '''Tell the pilot to RTL and land.'''
        print 'Navigator entered RTL callback'
        self.pilot.return_to_launch()
        self.pilot.land_drone()


    def stop(self):
        '''Shut down the pilot/vehicle.'''
        self.pilot.stop()


    def instantiate_pilot(self):
        '''Instantiate a pilot object and store it.'''
        if not self.simulated:
            self.bringup_ip = 'udp:127.0.0.1:14550'
	    #self.bringup_ip = 'tcp:127.0.0.1:57600'
        self.pilot = Pilot(
                simulated=self.simulated,
                simulated_landing_camera=self.simulated_landing_camera,
        )
        print('Bringup ip: ')
        print(self.bringup_ip)
        self.pilot.bringup_drone(connection_string=self.bringup_ip)


    def launch(self, event):
        '''Tell the pilot to arm the drone and take off.'''
        #altitude should be in meters
        altitude = self.takeoff_alt
        if not self.pilot.vehicle.armed:
            self.pilot.arm_and_takeoff(altitude)
            print 'Vehicle {0} ready for guidance'.format(self.pilot.instance)
            return
        print 'Vehicle {0} already armed'.format(self.pilot.instance)


    def parse_mission(self, mission_dict):
        '''Add GPS coordinates to all the points in a mission dictionary.'''
        # TODO Why is this needed??
        if (mission_dict['plan'][0]['action'] == 'launch'):
            return mission_dict

        for name, POI in mission_dict['points'].iteritems():
            if (all(keys in POI for keys in ['N', 'E', 'D'])):
                POI['GPS'] = self.meters_to_waypoint(POI)
            elif (all(keys in POI for keys in ['lat', 'lon', 'alt'])):
                POI['GPS'] = LocationGlobalRelative(POI['lat'], POI['lon'],
                                                    POI['alt'])
            else:
                print ('Error parsing POI of mission file')

        return mission_dict


    def meters_to_waypoint(self, POI):
        '''Construct a GPS location from a NED point.

        POI should be a dictionary, the returned value is a
        LocationGlobalRelative object.
        '''
        global_rel = relative_to_global(
                self.pilot.vehicle.home_location,
                POI['N'],
                POI['E'],
                POI['D']
        )
        return global_rel


    def execute_mission(self, unparsed_mission):
        '''Execute an un-parsed mission and send logging data to the logger.

        unparsed_mission -- a mission in the form of a dictionary, for example
                            from the FlaskServer.
        '''
        try:
            mission = self.parse_mission(unparsed_mission)
            self.current_mission = mission

            for event in mission['plan']:
                if (mission['plan'][0]['action'] != 'launch') and (self.pilot.vehicle.mode != 'GUIDED'):
                    print 'aborting mission due to check'
                    self.mission_queue = deque([])
                    return

                print 'mission executing action {}'.format(event['action'])
                action = getattr(self, event['action'])

                #publish event start
                event_start_dict = {
                    'task':event['action'],
                    'action':'start',
                }
                pub.sendMessage(
                    'nav-messages.mission-data',
                    arg1=event_start_dict
                )

                #do the thing
                action(event)

                #publish event end
                event_end_dict = {
                    'task':event['action'],
                    'action':'end',
                }
                pub.sendMessage(
                    'nav-messages.mission-data',
                    arg1=event_end_dict
                )

        except Exception as e:
            print 'Exception! RTL initiated', e
            # TODO Change for when exception is due to launch
            self.pilot.RTL_and_land()
            self.stop()


    def go(self, event):
        '''Execute a Go action with a mission event.'''
        name = event['points'][0]
        point = self.current_mission['points'][name]
        global_rel = point['GPS']
        print 'Moving to {}'.format(name)
        self.pilot.goto_waypoint(global_rel, speed=70)


    def patrol(self, event):
        '''Execute a Patrol action with a mission event.'''
        count = event['repeat']
        for i in range(count):
            print 'patrolling...'
            for name in event['points']:
		print 'going to {}'.format(name)
                point = self.current_mission['points'][name]
                self.pilot.goto_waypoint(point['GPS'], speed=10)
        print 'Finished patrolling'


    def RTL(self, event):
        '''Execute an RTL action with a mission event.

        Not currently used.
        '''
        self.pilot.return_to_launch()


    def land(self, event):
        '''Execute a Land action with a mission event.

        Not currently used.
        '''
        self.pilot.land_drone()


    def get_proc_id(self, b_only_pid=False):
        ''' Gets the class name and process id of the thread '''
        if (b_only_pid == True):
            return os.getpid()
        else:
            return {'name':self.__class__.__name__, 'pid':os.getpid()}


    def find_target_and_land_drone(self, event):
        ''' Searches for a target and then attempts to land on the target. '''

        if ('target' in event):
            target = event['target']
        else:
            target = None


        self.target_found = False
        self.landing_state = 0  # TODO Change to enumerated value
                                # 0: Take off and head to target
                                # 1: At target GPS location, no sighting
                                # 2: Landing, high altitude
                                # 3: Landing, medium altitude
                                # 4: Landing, low altitude
                                # 5: Landing, on ground
                                # 9: Abort
                                #10: Manual control

        # Search until either target is found or a timeout is reached
        self.landing_state = 1  # 1: At target GPS location, no sighting

        # Initialize landing camera hardware and subscription
        self.hw_landing_cam = hardware.LandingCamera(target=target,
                                                     simulated=self.simulated_landing_camera)
        pub.subscribe(self.landing_adjustment_cb, 'sensor-messages.landingcam-data')
        print 'Subscribed'


        print 'Start search for target'

        # TODO Add movement during initial search
        # Michael: I think this will work. We can test when we need.
        '''self.pilot.vehicle.parameters['CIRCLE_RADIUS'] = 300    # 300 cm radius
        self.pilot.vehicle.parameters['CIRCLE_RATE'] = 15       # +15 deg/s CW
        self.pilot.vehicle.mode = VehicleMode('CIRCLE')

        time_start = time.time()
        timeout = 1     # 1 second to switch modes
        while not(self.pilot.vehicle.mode == VehicleMode('CIRCLE')):
            time_elapsed = time.time() - time_start
            if (time_elapsed >= timeout):
                break
            time.sleep(0.1)

        # If switch to CIRCLE was successful, let the circle run in AUTO mode.
        # Otherwise, revert back to stationary hold in GUIDED mode
        if (self.pilot.vehicle.mode == VehicleMode('CIRCLE')):
            self.pilot.vehicle.mode = VehicleMode('AUTO')
        else:
            self.pilot.vehicle.mode = VehicleMode('GUIDED')
        '''

        self.pilot.vehicle.parameters['PLND_ENABLED'] = 1
        self.pilot.vehicle.parameters['PLND_TYPE'] = 1
        self.pilot.vehicle.flush()

        logging.info('\'find_target_and_land_drone\', PLND_ENABLED is ' + str(self.pilot.vehicle.parameters['PLND_ENABLED']))

        time_start = time.time()
        timeout = 30    # 30 seconds
	    time_switch = 5
        while(1):
            if (self.target_found == True):
                # self.pilot.vehicle.parameters['LAND_SPEED'] = 50 #30 to 200 in increments of 10
                # self.pilot.vehicle.parameters['PLND_TYPE'] = 1
                # self.pilot.vehicle.parameters['PLND_ENABLED'] = 1
                # self.pilot.vehicle.flush()
                #
		        # while ((self.pilot.vehicle.parameters['PLND_ENABLED'] != 1) and  (time.time() - time_start < time_switch)):
			    #     self.pilot.vehicle.parameters['PLND_ENABLED'] = 1
			    #     time.sleep(0.5)

                # if (self.pilot.vehicle.parameters['PLND_ENABLED'] == 1):
                #     logging.info('\'find_target_and_land_drone\', Turned PLND_ENABLED on)
                # else:
                #     logging.info('\'find_target_and_land_drone\', Turning on PLND_ENABLED timed out after 5 seconds)


                # set rangefinder
                # self.pilot.vehicle.parameters['RNGFND_TYPE'] = 10
                # self.pilot.vehicle.parameters['RNGFND_MIN_CM'] = 1
                # self.pilot.vehicle.parameters['RNGFND_MAX_CM'] = 10000
                # self.pilot.vehicle.parameters['RNGFND_GNDCLEAR'] = 5

                print 'Found target, start landing'
                logging.info('\'find_target_and_land_drone\', Found target and start landing')
                break
            time_elapsed = time.time() - time_start
            if (time_elapsed >= timeout):
                self.landing_state = 9  # 9: Abort
                print ('Target not found in %0.3f seconds' % timeout)
                logging.info('\'find_target_and_land_drone\', Target not found in %d seconds' % time_elapsed)
                break
	        if not (self.pilot.vehicle.mode == VehicleMode('GUIDED')):
		        self.landing_state = 10
		        break
            time.sleep(0.5) # TODO Can adjust if different responsiveness is
                            # required

        time_start = time.time()
        timeout = 5     # timeout of 5 seconds
        'Start landing'
        while (self.landing_state in [2,3,4]):
            if (self.target_found == False):
                # TODO Circle around? Adjust height? Loiter?
                time_elapsed = time.time() - time_start
                if (time_elapsed >= timeout) :
                    self.landing_state = 9  # 9: Abort
                    print ('Target lost for %0.3f seconds during landing' % time_elapsed)
                    logging.info('\'find_target_and_land_drone\', Target lost for %d seconds during landing. Mission aborted.' % timeout)
                    break

	        if not (self.pilot.vehicle.mode == VehicleMode('GUIDED')):
		        self.landing_state = 10
		        break

	        time.sleep(0.1)

        # Either landed or aborted, but stop landing camera
        self.hw_landing_cam.stop()
        self.pilot.vehicle.parameters['PLND_ENABLED'] = 0
        pub.unsubscribe(self.landing_adjustment_cb, 'sensor-messages.landingcam-data')

        if (self.landing_state == 9):
            print ('Return to home')
            logging.info('\'find_target_and_land_drone\', Mission aborted. Return to home.')
            self.pilot.vehicle.mode = VehicleMode('RTL')  # TODO Change to a more
                                                    # controlled fly to waypoint
                                                    # and land
        elif (self.landing_state == 5):
            print ('Landed successfully')
            logging.info('\'find_target_and_land_drone\', Landed successfully')
            # TODO Transfer info, add signpost garble here
        else:
            print ('ERROR, landing_state is: ' + str(self.landing_state))


    def landing_adjustment_cb(self, arg1=None):
        landing_dist_low = 1.5
        landing_dist_med = 3.0

        if (self.pilot.vehicle.armed == False):
            self.landing_state = 5
        else:
            if (arg1['found'] == True):
                self.pilot.send_land_message(arg1['xoffset'], arg1['yoffset'],
                                             arg1['distance'])
                #self.pilot.send_distance_message(int(arg1['distance']*100))
                self.target_found = True
                if arg1['distance'] <= landing_dist_low:
                    self.landing_state = 4
                elif arg1['distance'] <= landing_dist_med:
                    self.landing_state = 3
                elif arg1['distance'] > landing_dist_med:
                    self.landing_state = 2
                else:
                    self.landing_state = 9
                print ("Landing message sent and landing state adjusted to: "+\
                       str(self.landing_state))
                logging.info('\'landing_adjustment_cb\', \'msg_land\', ' + \
                             str(arg1).strip('{}') + ', \'landing_state\': %d' % self.landing_state)
            else:
                self.target_found = False
                print ("No landing message sent and landing state is: " +\
                       str(self.landing_state))
                logging.info('\'landing_adjustment_cb\', \'msg_land\', None' + \
                             ', \'landing_state\': %d' % self.landing_state)
