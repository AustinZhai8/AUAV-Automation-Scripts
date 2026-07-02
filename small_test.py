# Minimal real-hardware test: Takeoff -> (optional forward travel) -> Land
# Modified to include a guaranteed timed takeoff pulse and RC override clearing.

import MAVLink
import math

TARGET_ALTITUDE = 0.4       # Increased slightly to clear ground effect noise
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
THROTTLE_TAKEOFF_PWM = 1580   # Reduced from 1630 - gentler climb to limit I-term buildup
THROTTLE_HOLD_PWM = 1500      # True center = hold altitude in ALTHOLD (1510 was still climbing)
THROTTLE_LAND_PWM = 1350

# --- SAFETY LIMITS ---
MAX_SAFE_ALTITUDE = 1.5       # Hard ceiling - immediately cut throttle if exceeded
ALTITUDE_OVERSHOOT_MARGIN = 0.15  # Start backing off throttle this far above target

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
    # STATE 1: AUTOMATED TAKEOFF (RAMPED CLIMB)
    # ==========================================
    print('Automated Takeoff: Ramping throttle for controlled lift-off...')

    # Ramp throttle up gradually over ~1s to avoid I-term windup
    ramp_steps = 5
    ramp_start = THROTTLE_HOLD_PWM
    for i in range(1, ramp_steps + 1):
        ramp_pwm = ramp_start + int((THROTTLE_TAKEOFF_PWM - ramp_start) * i / ramp_steps)
        Script.SendRC(THROTTLE_HOVER_CH, ramp_pwm, True)
        Script.Sleep(200)

    # Hold takeoff throttle but monitor altitude with CEILING CHECK
    safety_triggered = False
    climb_timeout = 0
    while cs.alt < (TARGET_ALTITUDE - 0.05) and climb_timeout < 15:
        # SAFETY: hard ceiling - switch to LAND mode immediately
        if cs.alt > MAX_SAFE_ALTITUDE:
            print('SAFETY: Alt {0:.2f}m exceeds ceiling! Switching to LAND mode.'.format(cs.alt))
            Script.SendRC(THROTTLE_HOVER_CH, 0, True)
            Script.ChangeMode('LAND')
            safety_triggered = True
            break
        # Back off throttle if overshooting target
        if cs.alt > (TARGET_ALTITUDE + ALTITUDE_OVERSHOOT_MARGIN):
            print('Overshoot detected at {0:.2f}m, backing off throttle.'.format(cs.alt))
            Script.SendRC(THROTTLE_HOVER_CH, THROTTLE_HOLD_PWM, True)
            break
        print('Climbing... Alt: {0:.2f}m'.format(cs.alt))
        Script.Sleep(200)
        climb_timeout += 1

    if safety_triggered:
        # Wait for LAND mode to bring drone down and disarm
        print('LAND mode active. Waiting for touchdown...')
        land_wait = 0
        while cs.armed and land_wait < 30:
            print('LAND mode descending... Alt: {0:.2f}m'.format(cs.alt))
            Script.Sleep(1000)
            land_wait += 1
        print('Safety landing complete. Disarmed: {0}'.format(not cs.armed))
    else:
        # ==========================================
        # STATE 2: AUTOMATED HOVER (CENTER THROTTLE)
        # ==========================================
        # Ramp down to hover instead of abrupt step
        current_pwm = THROTTLE_TAKEOFF_PWM
        while current_pwm > THROTTLE_HOLD_PWM:
            current_pwm = max(THROTTLE_HOLD_PWM, current_pwm - 20)
            Script.SendRC(THROTTLE_HOVER_CH, current_pwm, True)
            Script.Sleep(100)

        print('Takeoff phase complete. Throttle centered to hover: Alt={0:.2f}m'.format(cs.alt))

        # Monitor hover with safety ceiling
        hover_time = 0
        while hover_time < 20:  # 2 seconds of hover monitoring
            if cs.alt > MAX_SAFE_ALTITUDE:
                print('SAFETY: Alt {0:.2f}m exceeds ceiling during hover! Switching to LAND.'.format(cs.alt))
                Script.SendRC(THROTTLE_HOVER_CH, 0, True)
                Script.ChangeMode('LAND')
                safety_triggered = True
                break
            Script.Sleep(100)
            hover_time += 1

    if not safety_triggered:
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
        # STATE 4: AUTOMATED LANDING (LAND MODE)
        # ==========================================
        print('Automated Landing: Switching to LAND mode...')
        Script.SendRC(THROTTLE_HOVER_CH, 0, True)
        Script.ChangeMode('LAND')

        land_wait = 0
        while cs.armed and land_wait < 30:
            print('LAND mode descending... Alt: {0:.2f}m'.format(cs.alt))
            Script.Sleep(1000)
            land_wait += 1

        # Fallback disarm if LAND mode didn't auto-disarm
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