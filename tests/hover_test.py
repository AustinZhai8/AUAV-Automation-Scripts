# Hover test: Take off to ~0.5m, hover 3 seconds, land
# PILOT OVERRIDE: Switch to STABILIZE on TX at any time to abort
import MAVLink

# RC params (from new params.param)
THROTTLE_ZERO = 1000       # RC3_MIN
THROTTLE_MID = 1500        # hover = mid stick
THROTTLE_CLIMB = 1620      # mid + THR_DZ(100) + 20, just above deadzone
RC1_TRIM = 1499            # roll center
RC2_TRIM = 1501            # pitch center

abort = False

def hold(throttle):
    """Send throttle + hold roll/pitch at center to prevent drift."""
    Script.SendRC(1, RC1_TRIM, True)
    Script.SendRC(2, RC2_TRIM, True)
    Script.SendRC(3, throttle, True)

def check_pilot():
    """Returns True if pilot switched mode — abort and hand back control."""
    if cs.mode != 'AltHold':
        print('PILOT OVERRIDE: Mode changed to ' + cs.mode)
        Script.SendRC(1, 0, True)
        Script.SendRC(2, 0, True)
        Script.SendRC(3, 0, True)
        return True
    return False

print('=== Hover Test: ~0.5m up, 3s hover, land ===')
print('>>> Switch to STABILIZE on TX to abort at any time <<<')

# Set mode to AltHold
Script.ChangeMode('ALTHOLD')
while cs.mode != 'AltHold':
    Script.Sleep(500)
print('Mode: ' + cs.mode)

# Arm
hold(THROTTLE_ZERO)
MAV.doARM(True)
waited = 0
while not cs.armed and waited < 15:
    hold(THROTTLE_ZERO)
    Script.Sleep(1000)
    waited += 1

if not cs.armed:
    print('ERROR: Failed to arm')
else:
    print('Armed. Starting in 3s...')
    for i in range(3, 0, -1):
        print('  ' + str(i) + '...')
        hold(THROTTLE_ZERO)
        Script.Sleep(1000)
    if check_pilot():
        abort = True

    # Ramp throttle up to climb PWM
    if not abort:
        for i in range(1, 6):
            pwm = THROTTLE_MID + int((THROTTLE_CLIMB - THROTTLE_MID) * i / 5)
            hold(pwm)
            Script.Sleep(200)
            if check_pilot():
                abort = True
                break

    # Hold climb throttle for ~1.2s (gets to ~0.5m)
    if not abort:
        for i in range(8):
            hold(THROTTLE_CLIMB)
            Script.Sleep(150)
            print('Climbing... alt={0:.2f}m'.format(cs.alt))
            if check_pilot():
                abort = True
                break

    # Hover at mid-throttle for 3 seconds
    if not abort:
        print('Hovering for 3s...')
        for i in range(20):
            hold(THROTTLE_MID)
            Script.Sleep(150)
            print('Hovering... alt={0:.2f}m'.format(cs.alt))
            if check_pilot():
                abort = True
                break

    # Land (only if script wasn't aborted — pilot has control otherwise)
    if not abort:
        print('Landing...')
        Script.SendRC(1, 0, True)
        Script.SendRC(2, 0, True)
        Script.SendRC(3, 0, True)
        Script.ChangeMode('LAND')

        land_wait = 0
        while cs.armed and land_wait < 20:
            print('Landing... alt={0:.2f}m'.format(cs.alt))
            Script.Sleep(1000)
            land_wait += 1

        if cs.armed:
            MAV.doARM(False)
            Script.Sleep(2000)

        print('Disarmed: ' + str(not cs.armed))

# Release overrides
Script.SendRC(1, 0, True)
Script.SendRC(2, 0, True)
Script.SendRC(3, 0, True)
if abort:
    print('=== ABORTED — Pilot has control ===')
else:
    print('=== Hover Test Complete ===')
