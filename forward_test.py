# Forward flight test: Takeoff to ~0.5m -> fly ~3m forward -> LAND mode
#
# SAFETY NOTES:
#   - No GPS (GPS_TYPE=0): distance is estimated from tilt + time
#   - No battery monitor (BATT_MONITOR=0): check battery manually
#   - Baro-only altitude (EK3_SRC1_POSZ=1): INACCURATE at low altitude
#     Prop wash ground effect can cause +-0.3m or worse baro errors
#     All flight phases use TIMED DURATIONS, not baro altitude targets
#     Baro is only used as a loose safety ceiling (set high to avoid false trips)
#   - RC overrides expire after 3s (RC_OVERRIDE_TIME=3): must refresh in every loop
#   - FS_THR_ENABLE=0: no throttle failsafe
#
# BEFORE FLIGHT (check in Mission Planner):
#   1. Battery charged - check manually, no voltage telemetry!
#   2. No "Bad Compass" or "Bad Baro" on HUD
#   3. Confirm drone forward matches CH2 below-trim direction
#   4. Clear area: 5m+ forward, 2m+ sides, nothing overhead
#   5. Transmitter on, throttle stick DOWN, ready to flip to STABILIZE if needed

import MAVLink
import math

GRAVITY = 9.81

# ==========================================
# FLIGHT PARAMETERS
# ==========================================
DESIRED_DISTANCE_M = 3.0     # meters (estimated, no GPS)

ARM_POLL_TIMEOUT = 15
ARM_RETRY_LIMIT = 3

# --- TIMED FLIGHT DURATIONS (baro too inaccurate for altitude targets) ---
TAKEOFF_CLIMB_MS = 1200       # How long to hold climb throttle (tuned for ~0.5m)
HOVER_STABILIZE_MS = 1500     # Hover time before forward flight
MAX_TRAVEL_DURATION = 4.0     # Max forward flight seconds
MAX_TOTAL_FLIGHT_TIME = 15.0  # Absolute kill switch: total airborne time

# --- SAFETY LIMITS ---
# Baro ceiling set HIGH because baro is inaccurate - this is a last resort catch
# If baro says 2m, drone is definitely too high regardless of baro error
MAX_SAFE_BARO_ALT = 2.0       # Loose baro ceiling -> LAND mode
MAX_TILT_DEG = 15.0            # Abort if unexpected tilt exceeds this

# --- RC OVERRIDE REFRESH ---
RC_OVERRIDE_REFRESH_MS = 1500  # Refresh within 3s RC_OVERRIDE_TIME

print('=== Forward Flight Test: ~0.5m up, ~3m forward, LAND ===')
print('')

# ==========================================
# DRONE PARAMS (from current parameters.param)
# Update these if you recalibrate RC or change params!
# ==========================================
RC3_MIN = 1000
RC3_MAX = 2000
RC2_MIN = 1008             # Pitch channel - used for forward travel
RC2_MAX = 2000
RC2_TRIM = 1501
RC1_TRIM = 1499            # Roll channel trim (held neutral during flight)
THR_DZ = 100              # ALTHOLD deadzone in PWM
ANGLE_MAX_DEG = 30.0      # ANGLE_MAX=3000
FORWARD_CH = 2            # Pitch channel for forward (below trim = user's forward)

# ALTHOLD: mid-stick = hover (PILOT_THR_BHV=0)
THROTTLE_MID = (RC3_MIN + RC3_MAX) / 2  # = 1500

# Deadzone: THR_DZ=100 means PWM 1400-1600 = no climb/descent
DZ_UPPER = int(THROTTLE_MID + THR_DZ)   # = 1600

# Takeoff: just above deadzone for gentle climb
THROTTLE_TAKEOFF_PWM = DZ_UPPER + 20    # = 1620
THROTTLE_HOLD_PWM = int(THROTTLE_MID)   # = 1500
THROTTLE_ZERO = RC3_MIN                 # = 1000

# Forward tilt: 15 PWM below center on pitch channel (below trim = user's forward)
TRAVEL_PWM = RC2_TRIM - 15              # = 1486

print('--- PARAMETER CHECK ---')
print('Throttle: zero={0} hover={1} dz_top={2} takeoff={3}'.format(
    THROTTLE_ZERO, THROTTLE_HOLD_PWM, DZ_UPPER, THROTTLE_TAKEOFF_PWM))
print('Forward CH{0}: center={1} travel={2}'.format(FORWARD_CH, RC2_TRIM, TRAVEL_PWM))
print('Roll CH1: center={0} (held neutral during flight)'.format(RC1_TRIM))
print('Climb duration: {0}ms  Hover hold: {1}ms'.format(TAKEOFF_CLIMB_MS, HOVER_STABILIZE_MS))
print('Baro safety ceiling: {0}m (loose - baro inaccurate)'.format(MAX_SAFE_BARO_ALT))
print('')

# ==========================================
# PRE-FLIGHT
# ==========================================
abort = False

# Calculate forward travel duration from tilt offset
fwd_half_range = (RC2_MAX - RC2_MIN) / 2.0
pwm_offset = RC2_TRIM - TRAVEL_PWM  # positive = below trim = user's forward
angle_deg = (pwm_offset / fwd_half_range) * ANGLE_MAX_DEG
accel = GRAVITY * math.tan(math.radians(angle_deg))

if accel < 0.01:
    print('ABORT: Tilt angle too small ({0:.2f} deg)'.format(angle_deg))
    abort = True
else:
    travel_duration = math.sqrt((2.0 * DESIRED_DISTANCE_M) / accel)
    if travel_duration > MAX_TRAVEL_DURATION:
        print('NOTE: Clamping travel {0:.1f}s to max {1:.1f}s'.format(travel_duration, MAX_TRAVEL_DURATION))
        travel_duration = MAX_TRAVEL_DURATION
    print('Forward plan: {0:.1f} deg tilt for {1:.1f}s'.format(angle_deg, travel_duration))

if not abort:
    print('')
    print('>>> STARTING IN 5 SECONDS <<<')
    print('>>> Switch to STABILIZE on transmitter to abort <<<')
    for countdown in range(5, 0, -1):
        print('  {0}...'.format(countdown))
        Script.SendRC(3, THROTTLE_ZERO, True)
        Script.Sleep(1000)

    if cs.mode == 'Stabilize' or cs.mode == 'PosHold':
        print('ABORT: Pilot switched mode during countdown.')
        abort = True

# ==========================================
# SET MODE AND ARM
# ==========================================
if not abort:
    Script.ChangeMode('ALTHOLD')
    mode_wait = 0
    while cs.mode != 'AltHold' and mode_wait < 10:
        Script.Sleep(500)
        mode_wait += 1

    if cs.mode != 'AltHold':
        print('ABORT: Could not enter AltHold. Current: {0}'.format(cs.mode))
        abort = True

if not abort:
    print('Mode: {0}'.format(cs.mode))
    Script.SendRC(3, THROTTLE_ZERO, True)

    armed_ok = False
    attempt = 0
    while not armed_ok and attempt < ARM_RETRY_LIMIT:
        attempt += 1
        print('Arm attempt {0}/{1}'.format(attempt, ARM_RETRY_LIMIT))
        cs.messages.Clear()
        MAV.doARM(True)
        waited = 0
        while waited < ARM_POLL_TIMEOUT:
            Script.SendRC(3, THROTTLE_ZERO, True)
            Script.Sleep(1000)
            waited += 1
            if cs.armed:
                armed_ok = True
                break
        if not armed_ok:
            Script.Sleep(1000)

    if not armed_ok:
        print('ABORT: Failed to arm.')
        abort = True

# ==========================================
# MAIN FLIGHT SEQUENCE
# ==========================================
if not abort:
    print('Armed successfully.')
    Script.Sleep(1000)
    safety_triggered = False
    flight_start_ms = 0

    # Helper: check safety in every phase
    # Returns True if something is wrong
    def check_safety():
        if cs.alt > MAX_SAFE_BARO_ALT:
            print('SAFETY: Baro alt {0:.2f}m > ceiling! LAND.'.format(cs.alt))
            return True
        if abs(cs.roll) > MAX_TILT_DEG or abs(cs.pitch) > MAX_TILT_DEG:
            print('SAFETY: Tilt R={0:.1f} P={1:.1f}! LAND.'.format(cs.roll, cs.pitch))
            return True
        if flight_start_ms > (MAX_TOTAL_FLIGHT_TIME * 1000):
            print('SAFETY: Total flight time {0:.1f}s exceeded! LAND.'.format(flight_start_ms / 1000.0))
            return True
        return False

    # ------------------------------------------
    # STATE 1: TIMED TAKEOFF (~0.5m)
    # ------------------------------------------
    print('Takeoff: Ramping throttle...')

    # Ramp from mid through deadzone to takeoff PWM over ~1.2s
    ramp_steps = 8
    for i in range(1, ramp_steps + 1):
        ramp_pwm = THROTTLE_HOLD_PWM + int((THROTTLE_TAKEOFF_PWM - THROTTLE_HOLD_PWM) * i / ramp_steps)
        Script.SendRC(3, ramp_pwm, True)
        Script.Sleep(150)
        flight_start_ms += 150
        if check_safety():
            safety_triggered = True
            break

    # Hold climb throttle for fixed duration (baro too unreliable to target altitude)
    if not safety_triggered:
        climb_elapsed = 0
        step_ms = 150
        print('Climbing for {0}ms...'.format(TAKEOFF_CLIMB_MS))
        while climb_elapsed < TAKEOFF_CLIMB_MS and not safety_triggered:
            Script.SendRC(3, THROTTLE_TAKEOFF_PWM, True)
            Script.Sleep(step_ms)
            climb_elapsed += step_ms
            flight_start_ms += step_ms
            print('  Climbing... baro={0:.2f}m (inaccurate)'.format(cs.alt))
            if check_safety():
                safety_triggered = True

    # ------------------------------------------
    # STATE 2: TIMED HOVER STABILIZE
    # ------------------------------------------
    if not safety_triggered:
        # Ramp to hover smoothly
        current_pwm = THROTTLE_TAKEOFF_PWM
        while current_pwm > THROTTLE_HOLD_PWM:
            current_pwm = max(THROTTLE_HOLD_PWM, current_pwm - 10)
            Script.SendRC(3, current_pwm, True)
            Script.Sleep(100)
            flight_start_ms += 100

        print('Hovering for {0}ms... baro={1:.2f}m'.format(HOVER_STABILIZE_MS, cs.alt))

        hover_elapsed = 0
        step_ms = 150
        while hover_elapsed < HOVER_STABILIZE_MS and not safety_triggered:
            Script.SendRC(3, THROTTLE_HOLD_PWM, True)
            Script.SendRC(1, RC1_TRIM, True)             # Hold roll neutral
            Script.SendRC(FORWARD_CH, RC2_TRIM, True)   # Hold pitch neutral
            Script.Sleep(step_ms)
            hover_elapsed += step_ms
            flight_start_ms += step_ms
            if check_safety():
                safety_triggered = True

    # ------------------------------------------
    # STATE 3: TIMED FORWARD FLIGHT (~3m)
    # ------------------------------------------
    if not safety_triggered:
        print('Flying forward for {0:.1f}s (est. ~{1}m)...'.format(travel_duration, DESIRED_DISTANCE_M))
        Script.SendRC(FORWARD_CH, TRAVEL_PWM, True)
        Script.SendRC(1, RC1_TRIM, True)  # Hold roll neutral
        Script.SendRC(3, THROTTLE_HOLD_PWM, True)

        elapsed = 0
        step_ms = 150
        override_timer = 0

        while elapsed < int(travel_duration * 1000) and not safety_triggered:
            flight_start_ms += step_ms
            override_timer += step_ms

            # Refresh ALL overrides before they expire
            if override_timer >= RC_OVERRIDE_REFRESH_MS:
                Script.SendRC(3, THROTTLE_HOLD_PWM, True)
                Script.SendRC(FORWARD_CH, TRAVEL_PWM, True)
                Script.SendRC(1, RC1_TRIM, True)
                override_timer = 0

            # Safety checks (pitch limit relaxed for expected forward tilt on CH2)
            if cs.alt > MAX_SAFE_BARO_ALT:
                print('SAFETY: Baro alt {0:.2f}m ceiling during travel! LAND.'.format(cs.alt))
                safety_triggered = True
                break
            if abs(cs.roll) > MAX_TILT_DEG or abs(cs.pitch) > (angle_deg + 10):
                print('SAFETY: Tilt R={0:.1f} P={1:.1f}! LAND.'.format(cs.roll, cs.pitch))
                safety_triggered = True
                break
            if flight_start_ms > (MAX_TOTAL_FLIGHT_TIME * 1000):
                print('SAFETY: Total flight time exceeded! LAND.')
                safety_triggered = True
                break

            Script.Sleep(step_ms)
            elapsed += step_ms

        # ALWAYS center pitch first to stop forward momentum
        Script.SendRC(FORWARD_CH, RC2_TRIM, True)
        Script.SendRC(1, RC1_TRIM, True)

        if not safety_triggered:
            print('Forward travel complete. baro={0:.2f}m'.format(cs.alt))
            Script.SendRC(3, THROTTLE_HOLD_PWM, True)
            Script.Sleep(500)
            flight_start_ms += 500

    # ------------------------------------------
    # STATE 4: LAND MODE
    # ------------------------------------------
    if safety_triggered:
        Script.SendRC(FORWARD_CH, RC2_TRIM, True)
        Script.SendRC(1, RC1_TRIM, True)
        Script.SendRC(3, 0, True)
        Script.ChangeMode('LAND')
    else:
        print('Switching to LAND mode...')
        Script.SendRC(FORWARD_CH, RC2_TRIM, True)
        Script.SendRC(1, RC1_TRIM, True)
        Script.SendRC(3, 0, True)
        Script.ChangeMode('LAND')

    print('LAND mode active. Waiting for touchdown...')
    land_wait = 0
    while cs.armed and land_wait < 20:
        Script.SendRC(1, 0, True)
        Script.SendRC(FORWARD_CH, 0, True)
        Script.SendRC(3, 0, True)
        print('Landing... baro={0:.2f}m'.format(cs.alt))
        Script.Sleep(1000)
        land_wait += 1

    if cs.armed:
        print('Fallback disarm...')
        MAV.doARM(False)
        Script.Sleep(2000)

    if cs.armed:
        print('WARNING: Still armed after fallback disarm!')
    else:
        print('Disarmed OK.')

# ==========================================
# CLEANUP: RELEASE ALL RC OVERRIDES
# ==========================================
print('Releasing all RC overrides...')
Script.SendRC(1, 0, True)
Script.SendRC(2, 0, True)
Script.SendRC(3, 0, True)
Script.SendRC(4, 0, True)

if abort:
    print('=== TEST ABORTED - see messages above ===')
else:
    print('=== Forward Flight Test Complete ===')
