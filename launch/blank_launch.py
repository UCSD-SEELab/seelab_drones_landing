import sys
import dronekit
import time
from os import sys, path, pardir

# importing drone_control was broken before...
path = path.abspath(pardir) + '/drone_scripts'
sys.path.append(path)
import drone_control

# Set SIMULATED=False if running on Pi. Set sensor values to false if sensors
# are on drone. Set to true if not on drone to use dummy data
SIMULATED=False
SIM_AIR_SENSOR=True
SIM_RF_SENSOR=False

drone = None

try:
  drone = drone_control.Navigator(
          simulated=SIMULATED,
          simulated_RF_sensor=SIM_RF_SENSOR,
          simulated_air_sensor=SIM_AIR_SENSOR,
  )

  '''
  drone.liftoff(10)
  drone.load_mission('demo_mission.json')
  drone.execute_mission()
  '''

except KeyboardInterrupt:
  drone.stop()

