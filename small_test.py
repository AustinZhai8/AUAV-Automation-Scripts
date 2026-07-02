# Minimal real-hardware test: Takeoff -> (optional forward travel) -> Land
# Modified to include a guaranteed timed takeoff pulse and RC override clearing.

import MAVLink
import math

TARGET_ALTITUDE = 0.14        # Increased slightly to clear ground effect noise
DESIRED_DISTANCE_M = 0       # METERS - set to 0 for up/down only test

TRAVEL_PWM = 1480             
PILOT_ANGLE_MAX_DEG = 30.0    
RC_PITCH_RANGE = 500.0
GRAVITY = 9.81
MAX_TRAVEL_DURATION = 3.0     

ARM_POLL_TIMEOUT = 15
ARM_RETRY_LIMIT = 3

# --- AUTOMATED THROTTLE SETTINGS ---
THROTTLE_HOVER_CH = 3
THROTTLE_ZERO = 1000          
THROTTLE_TAKEOFF_PWM = 1599
THROTTLE_HOLD_PWM = 1500      
THROTTLE_LAND_PWM = 1350      

print('--- Minimal Hardware Test: Takeoff / Travel-by-distance / Land ---')

# Step 1: Force AltHold Mode to bypass EKF position safety blocks
Script.ChangeMode('ALTHOLD')
while cs.mode != 'AltHold':
    Script.Sleep(500)
print('Mode: {0}'.format(cs.mode))

# Guard: Ensure throttle is completely down before arming
Script.SendRC(THROTTLE_HOVER_CH, THROTTLE_ZERO, True)

# Step 2: Automated Arming Sequence
armed_ok = False
attempt = 0
while not armed_ok and attempt < ARM_RETRY_LIMIT:
    attempt += 1
    print('Arm attempt {0}/{1}'.format(attempt, ARM_RETRY_LIMIT))
    cs.messages.Clear()
    MAV.doARM(True)
    waited = 0
    while waited < ARM_POLL_TIMEOUT:
        Script.Sleep(1000)
        waited += 1
        if cs.armed:
            armed_ok = True
            break
    if not armed_ok:
        Script.Sleep(1000)

if not armed_ok:
    print('ERROR: failed to arm. Check pre-arm messages in HUD.')
    # Clear overrides even on failure just to be safe
    Script.SendRC(THROTTLE_HOVER_CH, 0, True)
else:
    print('Armed successfully.')
    Script.Sleep(1500)  

    # ==========================================
    # STATE 1: AUTOMATED TAKEOFF (TIMED PULSE)
    # ==========================================
    print('Automated Takeoff: Commencing guaranteed lift-off pulse...')
    Script.SendRC(THROTTLE_HOVER_CH, THROTTLE_TAKEOFF_PWM, True)
    
    # Force a 1.5 second climb pulse to physically get off the ground 
    # before checking the barometer feedback.
    Script.Sleep(1500) 

    # Supplemental climb check if still below target
    climb_timeout = 0
    while cs.alt < (TARGET_ALTITUDE - 0.05) and climb_timeout < 10:
        print('Adjusting altitude to target... Alt: {0:.2f}m'.format(cs.alt))
        Script.Sleep(200)
        climb_timeout += 1

    # ==========================================
    # STATE 2: AUTOMATED HOVER (CENTER THROTTLE)
    # ==========================================
    Script.SendRC(THROTTLE_HOVER_CH, THROTTLE_HOLD_PWM, True)
    print('Takeoff phase complete. Throttle centered to hover: Alt={0:.2f}m'.format(cs.alt))
    Script.Sleep(2000)  

    # ==========================================
    # STATE 3: OPTIONAL HORIZONTAL TRAVEL
    # ==========================================
    if DESIRED_DISTANCE_M > 0:
        pwm_offset = 1500 - TRAVEL_PWM
        angle_deg = (pwm_offset / RC_PITCH_RANGE) * PILOT_ANGLE_MAX_DEG
        accel = GRAVITY * math.tan(math.radians(angle_deg))
        travel_duration = math.sqrt((2.0 * DESIRED_DISTANCE_M) / accel)

        if travel_duration > MAX_TRAVEL_DURATION:
            travel_duration = MAX_TRAVEL_DURATION

        print('Executing automated forward tilt pulse for {0:.2f}s...'.format(travel_duration))
        Script.SendRC(2, TRAVEL_PWM, True)
        Script.Sleep(int(travel_duration * 1000))
        Script.SendRC(2, 1500, True)  
        print('Travel complete. Stabilizing. Alt={0:.2f}m'.format(cs.alt))
        Script.Sleep(1500)
    else:
        print('DESIRED_DISTANCE_M is 0 - Skipping horizontal flight phase.')

    # ==========================================
    # STATE 4: AUTOMATED LANDING
    # ==========================================
    print('Automated Landing: Step throttle down to initiate descent...')
    Script.SendRC(THROTTLE_HOVER_CH, THROTTLE_LAND_PWM, True)

    # Monitor descent until close to the ground
    land_timeout = 0
    while cs.alt > 0.05 and land_timeout < 25:
        print('Descending automatically... Alt: {0:.2f}m'.format(cs.alt))
        Script.Sleep(200)
        land_timeout += 1

    # ==========================================
    # STATE 5: AUTOMATED MOTOR SHUTDOWN / DISARM
    # ==========================================
    print('Ground proximity detected. Cutting throttle completely.')
    Script.SendRC(THROTTLE_HOVER_CH, THROTTLE_ZERO, True)
    
    waited = 0
    while cs.armed and waited < ARM_POLL_TIMEOUT:
        Script.Sleep(1000)
        waited += 1
    
    if cs.armed:
        print('Executing fallback disarm command...')
        MAV.doARM(False)
        Script.Sleep(1000)

    print('Disarmed status: {0}'.format(not cs.armed))

# ==========================================
# CLEANUP: RELEASE OVERRIDES FOR NEXT RUN
# ==========================================
print('Releasing RC overrides to clear autopilot safety flags...')
Script.SendRC(THROTTLE_HOVER_CH, 0, True)
Script.SendRC(2, 0, True) 

print('--- Automated Test Complete ---')