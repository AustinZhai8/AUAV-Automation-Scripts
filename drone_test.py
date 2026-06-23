import sys
import time

print("--- Starting No-GPS Mission Planner Script ---")

# 1. Check if already armed, if not, arm the motors
if not cs.armed:
    print("Arming motors...")
    # Send the MAVLink command to arm
    MAV.doARM(True)
    
    # Wait up to 10 seconds for the vehicle to confirm it is armed
    count = 0
    while not cs.armed and count < 5:
        print("Waiting for arm confirmation...")
        Script.Sleep(2000)
        count += 1

if cs.armed:
    print("Motors ARMED successfully!")
    
    # 2. Change Flight Mode to STABILIZE or ALT_HOLD (safer for non-GPS)
    print("Switching flight mode...")
    Script.ChangeMode("Stabilize")
    
    # 3. Monitor drone status for a brief moment
    for i in range(5):
        print(f"Current Battery Voltage: {cs.battery_voltage}")
        print(f"Current Pitch: {cs.pitch}")
        Script.Sleep(1000)
        
else:
    print("Failed to arm motors. Please check pre-arm flags in Mission Planner.")

print("--- Script Finished ---")