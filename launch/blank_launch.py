import sys
import dronekit
import time
from os import sys, path, pardir

path = path.abspath(pardir) + '/drone_scripts'
sys.path.append(path)
import drone_control

# Set SIMULATED=False if running on Pi. Set sensor values to false if sensors
# are on drone. Set to true if not on drone to use dummy data
SIMULATED=False
SIM_LANDING_CAM=False

drone = None

try:
  drone = drone_control.Navigator(
          simulated=SIMULATED,
          simulated_landing_camera=SIM_LANDING_CAM
  )
  
except KeyboardInterrupt:
  drone.stop()

