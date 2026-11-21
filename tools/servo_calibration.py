"""
servo_calibration.py
====================
Manual jog tool used to find each servo's safe minimum/maximum angle BEFORE
setting PAN_ANGLE_LIMITS / TILT_ANGLE_LIMITS in src/config.py.

Hold a key to move the servo; release to stop. The current angle is printed
when you let go. Stop before the gimbal binds or a cable is pulled taut, and
write that angle down as your limit.

    w / s  -> tilt up / down   (channel 1)
    a / d  -> pan  left / right (channel 0)

Both servos jump to 90 degrees on start-up, so make sure the mount has room.
Needs:  pip install sshkeyboard adafruit-circuitpython-servokit
Run on the Raspberry Pi:  python tools/servo_calibration.py   (Ctrl+C to quit)
"""
from sshkeyboard import listen_keyboard
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

pan_angle = 90
tilt_angle = 90

release_a = False
release_d = False
release_w = False
release_s = False

kit.servo[0].angle = pan_angle
kit.servo[1].angle = tilt_angle

def press(key):
    global pan_angle, tilt_angle, release_a, release_d, release_w, release_s

    if key == "w":
        release_w = False
        while(tilt_angle > 0 and not release_w):
            tilt_angle -= 1 #Swap to += and tilt_angle < 180 if servo is moving in the opposite direction than expected
            kit.servo[1].angle = tilt_angle
            time.sleep(0.01)
        print(f"Tilt angle: {tilt_angle}")

    elif key == "s":
        release_s = False
        while(tilt_angle < 180 and not release_s):
            tilt_angle += 1 #Swap to -= and tilt_angle > 0 if servo is moving in the opposite direction than expected
            kit.servo[1].angle = tilt_angle
            time.sleep(0.01)
        print(f"Tilt angle: {tilt_angle}")

    elif key == "a":
        release_a = False
        while(pan_angle < 180 and not release_a):
            pan_angle += 1
            kit.servo[0].angle = pan_angle
            time.sleep(0.01)
        print(f"Pan angle: {pan_angle}")

    elif key == "d":
        release_d = False
        while(pan_angle > 0 and not release_d):
            pan_angle -= 1
            kit.servo[0].angle = pan_angle
            time.sleep(0.01)
        print(f"Pan angle: {pan_angle}")

def release(key):
    global release_a, release_d, release_w, release_s

    if key == "w":
        release_w = True

    elif key == "s":
        release_s = True

    elif key == "a":
        release_a = True

    elif key == "d":
        release_d = True

listen_keyboard(
    on_press=press,
    on_release=release,
    delay_second_char = 0.001
)