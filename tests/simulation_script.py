import MAVLink
import math

# ============================================
# THE ONE VARIABLE YOU CHANGE FOR DISTANCE:
DESIRED_DISTANCE_M = 25.0
# ============================================

TARGET_ALTITUDE = 2.0
ARM_POLL_TIMEOUT = 15
ARM_RETRY_LIMIT = 5

# Fixed pitch angle settings (don't need to change these for different distances)
PITCH_PWM = 1410             # ~90 PWM offset from neutral -> ~8.1 deg lean, fixed
PILOT_ANGLE_MAX_DEG = 45.0   # default ANGLE_MAX
RC_PITCH_RANGE = 500.0
GRAVITY = 9.81
MAX_PITCH_DURATION = 12.0    # safety cap - script won't compute a longer pulse than this

print('--- Mission: Takeoff / Travel / Land ---')

# 1. Mode + Arm
Script.ChangeMode('GUIDED')
while cs.mode != 'Guided':
    Script.Sleep(500)
print('Mode: {0}'.format(cs.mode))

armed_ok = False
attempt = 0
while not armed_ok and attempt < ARM_RETRY_LIMIT:
    attempt += 1
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
    print('ERROR: failed to arm.')
else:
    print('Armed.')

    # 2. Takeoff via real nav command (GUIDED)
    print('Sending TAKEOFF command to {0}m...'.format(TARGET_ALTITUDE))
    MAV.doCommand(MAVLink.MAV_CMD.TAKEOFF, 0, 0, 0, 0, 0, 0, TARGET_ALTITUDE)

    while cs.alt < (TARGET_ALTITUDE - 0.2):
        print('climbing: alt={0:.2f}'.format(cs.alt))
        Script.Sleep(500)
    print('Reached altitude: {0:.2f}m'.format(cs.alt))
    Script.Sleep(2000)

    # 3. Travel - switch to ALTHOLD
    print('Switching to ALTHOLD for travel pulse...')
    Script.ChangeMode('ALTHOLD')
    while cs.mode != 'AltHold':
        Script.Sleep(500)
    print('Mode now: {0}'.format(cs.mode))

    Script.SendRC(3, 1500, True)
    Script.Sleep(500)

    print('Direction note: pitch-forward moves the vehicle toward whatever')
    print('heading its nose is currently facing (current yaw={0:.1f} deg).'.format(cs.yaw))

    # --- Solve for pulse duration needed to hit DESIRED_DISTANCE_M ---
    pwm_offset = 1500 - PITCH_PWM
    commanded_angle_deg = (pwm_offset / RC_PITCH_RANGE) * PILOT_ANGLE_MAX_DEG
    commanded_angle_rad = math.radians(commanded_angle_deg)
    accel = GRAVITY * math.tan(commanded_angle_rad)

    # d = 0.5 * a * t^2  ->  t = sqrt(2d / a)
    pitch_duration = math.sqrt((2.0 * DESIRED_DISTANCE_M) / accel)

    capped = False
    if pitch_duration > MAX_PITCH_DURATION:
        capped = True
        pitch_duration = MAX_PITCH_DURATION

    theoretical_end_speed = accel * pitch_duration
    actual_theoretical_distance = 0.5 * accel * (pitch_duration ** 2)

    print('--- THEORETICAL distance/time calculation (idealized physics, not measured) ---')
    print('  Desired distance: {0:.1f} m'.format(DESIRED_DISTANCE_M))
    print('  Fixed lean angle: {0:.1f} deg (PWM {1})'.format(commanded_angle_deg, PITCH_PWM))
    print('  Estimated horizontal accel: {0:.2f} m/s^2'.format(accel))
    print('  Calculated pulse duration needed: {0:.2f} s'.format(pitch_duration))
    if capped:
        print('  WARNING: capped at MAX_PITCH_DURATION={0}s for safety.'.format(MAX_PITCH_DURATION))
        print('  Distance at capped duration: {0:.2f} m (less than requested).'.format(actual_theoretical_distance))
    print('  Theoretical end speed: {0:.2f} m/s'.format(theoretical_end_speed))
    print('  NOTE: real distance will likely be lower due to drag/ramp-up not modeled here.')

    print('Pitching forward for travel...')
    Script.SendRC(2, PITCH_PWM, True)

    elapsed = 0.0
    aborted = False
    while elapsed < pitch_duration:
        Script.Sleep(500)
        elapsed += 0.5
        print('travel: t={0:.1f}s, alt={1:.2f}, roll={2:.1f}, pitch={3:.1f}'.format(
            elapsed, cs.alt, cs.roll, cs.pitch))
        if abs(cs.pitch) > 20 or abs(cs.roll) > 20:
            print('ABORT: attitude exceeded 20 deg, leveling out early.')
            aborted = True
            break

    Script.SendRC(2, 1500, True)
    if aborted:
        print('Travel pulse aborted early for safety.')
    print('Travel pulse complete. alt={0:.2f}, roll={1:.1f}, pitch={2:.1f}, yaw={3:.1f}'.format(
        cs.alt, cs.roll, cs.pitch, cs.yaw))
    Script.Sleep(3000)

    # 4. Switch back to GUIDED before landing
    print('Switching back to GUIDED for landing...')
    Script.ChangeMode('GUIDED')
    while cs.mode != 'Guided':
        Script.Sleep(500)
    print('Mode now: {0}'.format(cs.mode))

    print('Sending LAND command...')
    MAV.doCommand(MAVLink.MAV_CMD.LAND, 0, 0, 0, 0, 0, 0, 0)

    while cs.alt > 0.15:
        print('landing: alt={0:.2f}'.format(cs.alt))
        Script.Sleep(500)

    waited = 0
    while cs.armed and waited < ARM_POLL_TIMEOUT:
        Script.Sleep(1000)
        waited += 1
    print('Disarmed: {0}'.format(not cs.armed))

print('--- Mission Complete ---')