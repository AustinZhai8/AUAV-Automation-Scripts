# Hover test: Take off to ~0.5m, hover 3 seconds, land
import MAVLink

# RC params (from new params.param)
THROTTLE_ZERO = 1000       # RC3_MIN
THROTTLE_MID = 1500        # hover = mid stick
THROTTLE_CLIMB = 1620      # mid + THR_DZ(100) + 20, just above deadzone

print('=== Hover Test: ~0.5m up, 3s hover, land ===')

# Set mode to AltHold
Script.ChangeMode('ALTHOLD')
while cs.mode != 'AltHold':
    Script.Sleep(500)
print('Mode: ' + cs.mode)

# Arm
Script.SendRC(3, THROTTLE_ZERO, True)
MAV.doARM(True)
waited = 0
while not cs.armed and waited < 15:
    Script.SendRC(3, THROTTLE_ZERO, True)
    Script.Sleep(1000)
    waited += 1

if not cs.armed:
    print('ERROR: Failed to arm')
else:
    print('Armed. Starting in 3s...')
    for i in range(3, 0, -1):
        print('  ' + str(i) + '...')
        Script.SendRC(3, THROTTLE_ZERO, True)
        Script.Sleep(1000)

    # Ramp throttle up to climb PWM
    for i in range(1, 6):
        pwm = THROTTLE_MID + int((THROTTLE_CLIMB - THROTTLE_MID) * i / 5)
        Script.SendRC(3, pwm, True)
        Script.Sleep(200)

    # Hold climb throttle for ~1.2s (gets to ~0.5m)
    for i in range(8):
        Script.SendRC(3, THROTTLE_CLIMB, True)
        Script.Sleep(150)
        print('Climbing... alt={0:.2f}m'.format(cs.alt))

    # Hover at mid-throttle for 3 seconds
    print('Hovering for 3s...')
    for i in range(20):
        Script.SendRC(3, THROTTLE_MID, True)
        Script.Sleep(150)
        print('Hovering... alt={0:.2f}m'.format(cs.alt))

    # Land
    print('Landing...')
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
Script.SendRC(3, 0, True)
print('=== Hover Test Complete ===')
