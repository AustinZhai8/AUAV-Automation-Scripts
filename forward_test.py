# Forward flight test: Takeoff to 0.5m -> fly 3m forward -> LAND mode
# Uses ramped throttle and altitude ceiling safety from small_test fixes.

import MAVLink
import math

TARGET_ALTITUDE = 0.5
DESIRED_DISTANCE_M = 3.0

TRAVEL_PWM = 1480
PILOT_ANGLE_MAX_DEG = 30.0
RC_PITCH_RANGE = 500.0
GRAVITY = 9.81
MAX_TRAVEL_DURATION = 5.0

ARM_POLL_TIMEOUT = 15
ARM_RETRY_LIMIT = 3

# --- THROTTLE SETTINGS ---
THROTTLE_CH = 3
THROTTLE_ZERO = 1000
THROTTLE_TAKEOFF_PWM = 1580
THROTTLE_HOLD_PWM = 1500
THROTTLE_LAND_PWM = 1350

# --- SAFETY LIMITS ---
MAX_SAFE_ALTITUDE = 1.5
ALTITUDE_OVERSHOOT_MARGIN = 0.15

print('--- Forward Flight Test: 0.5m up, 3m forward, LAND ---')

# Step 1: Set AltHold mode
Script.ChangeMode('ALTHOLD')
while cs.mode != 'AltHold':
    Script.Sleep(500)
print('Mode: {0}'.format(cs.mode))

# Guard: throttle down before arming
Script.SendRC(THROTTLE_CH, THROTTLE_ZERO, True)

# Step 2: Arm
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
    Script.SendRC(THROTTLE_CH, 0, True)
else:
    print('Armed successfully.')
    Script.Sleep(1500)
    safety_triggered = False

    # ==========================================
    # STATE 1: RAMPED TAKEOFF TO 0.5m
    # ==========================================
    print('Takeoff: Ramping throttle...')

    # Ramp throttle up gradually to avoid I-term windup
    ramp_steps = 5
    for i in range(1, ramp_steps + 1):
        ramp_pwm = THROTTLE_HOLD_PWM + int((THROTTLE_TAKEOFF_PWM - THROTTLE_HOLD_PWM) * i / ramp_steps)
        Script.SendRC(THROTTLE_CH, ramp_pwm, True)
        Script.Sleep(200)

    # Climb to target altitude with safety ceiling
    climb_timeout = 0
    while cs.alt < (TARGET_ALTITUDE - 0.05) and climb_timeout < 20:
        if cs.alt > MAX_SAFE_ALTITUDE:
            print('SAFETY: Alt {0:.2f}m exceeds ceiling! Switching to LAND.'.format(cs.alt))
            Script.SendRC(THROTTLE_CH, 0, True)
            Script.ChangeMode('LAND')
            safety_triggered = True
            break
        if cs.alt > (TARGET_ALTITUDE + ALTITUDE_OVERSHOOT_MARGIN):
            print('Overshoot at {0:.2f}m, backing off throttle.'.format(cs.alt))
            Script.SendRC(THROTTLE_CH, THROTTLE_HOLD_PWM, True)
            break
        print('Climbing... Alt: {0:.2f}m'.format(cs.alt))
        Script.Sleep(200)
        climb_timeout += 1

    # ==========================================
    # STATE 2: STABILIZE AT HOVER
    # ==========================================
    if not safety_triggered:
        # Ramp down to hover smoothly
        current_pwm = THROTTLE_TAKEOFF_PWM
        while current_pwm > THROTTLE_HOLD_PWM:
            current_pwm = max(THROTTLE_HOLD_PWM, current_pwm - 20)
            Script.SendRC(THROTTLE_CH, current_pwm, True)
            Script.Sleep(100)

        print('Hover stabilizing at Alt={0:.2f}m'.format(cs.alt))

        # Brief hover with ceiling monitor
        hover_time = 0
        while hover_time < 15:
            if cs.alt > MAX_SAFE_ALTITUDE:
                print('SAFETY: Alt {0:.2f}m exceeds ceiling during hover! Switching to LAND.'.format(cs.alt))
                Script.SendRC(THROTTLE_CH, 0, True)
                Script.ChangeMode('LAND')
                safety_triggered = True
                break
            Script.Sleep(100)
            hover_time += 1

    # ==========================================
    # STATE 3: FLY 3m FORWARD
    # ==========================================
    if not safety_triggered:
        pwm_offset = 1500 - TRAVEL_PWM
        angle_deg = (pwm_offset / RC_PITCH_RANGE) * PILOT_ANGLE_MAX_DEG
        accel = GRAVITY * math.tan(math.radians(angle_deg))
        travel_duration = math.sqrt((2.0 * DESIRED_DISTANCE_M) / accel)

        if travel_duration > MAX_TRAVEL_DURATION:
            travel_duration = MAX_TRAVEL_DURATION

        print('Flying forward for {0:.2f}s to cover ~3m...'.format(travel_duration))
        Script.SendRC(2, TRAVEL_PWM, True)

        # Monitor altitude ceiling during forward flight
        elapsed = 0
        step_ms = 200
        while elapsed < int(travel_duration * 1000):
            if cs.alt > MAX_SAFE_ALTITUDE:
                print('SAFETY: Alt {0:.2f}m exceeds ceiling during travel! Switching to LAND.'.format(cs.alt))
                Script.SendRC(2, 1500, True)
                Script.SendRC(THROTTLE_CH, 0, True)
                Script.ChangeMode('LAND')
                safety_triggered = True
                break
            Script.Sleep(step_ms)
            elapsed += step_ms

        if not safety_triggered:
            Script.SendRC(2, 1500, True)  # Center pitch to stop forward motion
            print('Forward travel complete. Alt={0:.2f}m'.format(cs.alt))
            Script.Sleep(1000)

    # ==========================================
    # STATE 4: LAND
    # ==========================================
    if not safety_triggered:
        print('Switching to LAND mode...')
        Script.SendRC(THROTTLE_CH, 0, True)
        Script.ChangeMode('LAND')

    # Wait for LAND mode to bring drone down
    print('LAND mode active. Waiting for touchdown...')
    land_wait = 0
    while cs.armed and land_wait < 30:
        print('Landing... Alt: {0:.2f}m'.format(cs.alt))
        Script.Sleep(1000)
        land_wait += 1

    # Fallback disarm
    if cs.armed:
        print('Fallback disarm...')
        MAV.doARM(False)
        Script.Sleep(1000)

    print('Disarmed: {0}'.format(not cs.armed))

# ==========================================
# CLEANUP: RELEASE RC OVERRIDES
# ==========================================
print('Releasing RC overrides...')
Script.SendRC(THROTTLE_CH, 0, True)
Script.SendRC(2, 0, True)

print('--- Forward Flight Test Complete ---')
