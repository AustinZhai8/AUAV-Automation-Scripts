import sys
import math
import time
import clr

# Load Mission Planner assembly references
clr.AddReference("MissionPlanner.Utilities")
from MissionPlanner.Utilities import Locationwp

print '--- Starting Autonomous Script ---'

# 1. Wait for a valid GPS fix
while cs.lat == 0 or cs.lng == 0:
    print 'Waiting for GPS Lock...'
    Script.Sleep(2000)
print 'GPS Lock acquired.'

# 2. Switch to GUIDED Mode & Arm
print 'Switching to GUIDED Mode...'
Script.ChangeMode("GUIDED")
Script.Sleep(2000)

print 'Arming Motors...'
MAV.doARM(True)

# Wait until vehicle confirms it is armed
while not cs.armed:
    print 'Waiting for arming confirmation...'
    Script.Sleep(1000)
print 'Motors ARMED!'

# 3. Takeoff
# Set a guided target at the current location but 2 meters up
takeoff_alt = 2.0
print 'Taking off to %sm...' % takeoff_alt

wp_takeoff = Locationwp()
Locationwp.lat.SetValue(wp_takeoff, cs.lat)
Locationwp.lng.SetValue(wp_takeoff, cs.lng)
Locationwp.alt.SetValue(wp_takeoff, takeoff_alt)

MAV.setGuidedModeWP(wp_takeoff)

# Wait until the vehicle reaches close to the target altitude
while cs.alt < (takeoff_alt - 0.3):
    print 'Climbing... Current Altitude: %.2fm' % cs.alt
    Script.Sleep(500)
print 'Target takeoff altitude reached.'
Script.Sleep(2000) # Let it stabilize for 2 seconds

# 4. Calculate 1 Meter Forward
# Earth radius in meters
R = 6378100.0 
distance_m = 1.0

# Use current heading (yaw) to determine the forward direction
heading_rad = math.radians(cs.yaw)

# Calculate coordinate offsets
delta_lat = (distance_m * math.cos(heading_rad)) / R
delta_lng = (distance_m * math.sin(heading_rad)) / (R * math.cos(math.radians(cs.lat)))

target_lat = cs.lat + math.degrees(delta_lat)
target_lng = cs.lng + math.degrees(delta_lng)

print 'Moving 1 meter forward...'
wp_move = Locationwp()
Locationwp.lat.SetValue(wp_move, target_lat)
Locationwp.lng.SetValue(wp_move, target_lng)
Locationwp.alt.SetValue(wp_move, takeoff_alt)

MAV.setGuidedModeWP(wp_move)

# Allow time to travel 1 meter (adjust sleep as needed depending on WP speed)
Script.Sleep(4000) 
print 'Movement action complete.'

# 5. Land
print 'Initiating LAND mode...'
Script.ChangeMode("LAND")

while cs.alt > 0.1:
    print 'Descending... Current Altitude: %.2fm' % cs.alt
    Script.Sleep(1000)

print 'Vehicle has landed successfully!'
print '--- Script Finished ---'