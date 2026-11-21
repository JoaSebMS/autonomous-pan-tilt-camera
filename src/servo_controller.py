"""
servo_controller.py
====================
Hardware driver for the pan-tilt camera mount. Wraps an Adafruit PCA9685
ServoKit with:
    - A low-pass filter + PID controller, for smooth tracking of a target's
      pixel position (used by both modes while a target/motion is locked).
    - An automatic waypoint-based area scan (used by Idle Mode while nothing
      is locked).

All tunable numbers (angle limits, PID gains, scan waypoints...) live in
config.py under the "Servo / Pan-Tilt Controller" section -- edit that file
to tune your hardware, not this one.

If 'adafruit_servokit' isn't installed or the PCA9685 isn't reachable (e.g.
you're developing on a laptop instead of the Pi), this still runs -- it just
skips the actual hardware writes and logs a warning, so you can test the
vision/tracking logic without the servo rig attached.
"""

import config


def _debug_log(*args):
    if config.DEBUG:
        print("[DEBUG]", *args)


class ServoController:
    """
    Handles PCA9685 I2C servo control, PID angle updates, low-pass filtering
    of target coordinates, and the automatic area-scanning sequence used by
    Idle Mode while no target is locked.
    """

    def __init__(self):
        self.pan_ch = config.SERVO_PAN_CHANNEL
        self.tilt_ch = config.SERVO_TILT_CHANNEL

        self.pan_min, self.pan_max = config.PAN_ANGLE_LIMITS
        self.tilt_min, self.tilt_max = config.TILT_ANGLE_LIMITS

        self.pan_angle, self.tilt_angle = config.HOME_ANGLES

        self.kp_pan, self.ki_pan, self.kd_pan = config.PAN_PID_GAINS
        self.kp_tilt, self.ki_tilt, self.kd_tilt = config.TILT_PID_GAINS
        self.integral_clamp = config.PID_INTEGRAL_CLAMP

        # Internal PID memory
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

        # Low-pass filter memory
        self.lpf_alpha = config.LPF_ALPHA
        self.filtered_error_x = 0.0
        self.filtered_error_y = 0.0

        # Area-scan state
        self.scan_waypoints = config.SCAN_WAYPOINTS
        self.current_waypoint_idx = 0
        self.scan_speed = config.SCAN_SPEED

        # Hardware handle (None if unavailable -- see note in module docstring)
        self.kit = None
        try:
            from adafruit_servokit import ServoKit
            self.kit = ServoKit(channels=config.SERVO_KIT_CHANNELS)
            self.set_angles(*config.HOME_ANGLES)
            print("[INFO] PCA9685 servo controller initialized successfully.")
        except ImportError:
            print("[WARNING] 'adafruit_servokit' not installed -- running WITHOUT servo hardware.")
            print("          Install on the Pi with: pip install adafruit-circuitpython-servokit")
        except Exception as e:
            print(f"[ERROR] Failed to connect to PCA9685 hardware: {e}")
            print("        Check wiring / that I2C is enabled (sudo raspi-config).")

    # -------------------------------------------------------------------
    # Low-level angle control
    # -------------------------------------------------------------------

    def set_angles(self, pan, tilt):
        """Clamp and write target angles to the PCA9685."""
        self.pan_angle = max(self.pan_min, min(self.pan_max, pan))
        self.tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt))

        if self.kit:
            try:
                self.kit.servo[self.pan_ch].angle = self.pan_angle
                self.kit.servo[self.tilt_ch].angle = self.tilt_angle
            except Exception as err:
                _debug_log(f"Servo write error: {err}")

    def move_home(self):
        """Move pan and tilt to the configured home position."""
        self.reset_pid()
        self.set_angles(*config.HOME_ANGLES)

    def reset_pid(self):
        """Reset PID integral/derivative and filter memory when switching targets/states."""
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.filtered_error_x = 0.0
        self.filtered_error_y = 0.0

    # -------------------------------------------------------------------
    # Tracking (low-pass filtered PID)
    # -------------------------------------------------------------------

    def update_tracking(self, target_cx, target_cy, frame_cx, frame_cy, dt):
        """Nudge pan/tilt to keep (target_cx, target_cy) centered in the frame."""
        if dt <= 0:
            return

        raw_error_x = frame_cx - target_cx
        raw_error_y = frame_cy - target_cy

        # 1. Low-pass filter the raw pixel error to reduce jitter
        self.filtered_error_x = (self.lpf_alpha * raw_error_x) + ((1 - self.lpf_alpha) * self.filtered_error_x)
        self.filtered_error_y = (self.lpf_alpha * raw_error_y) + ((1 - self.lpf_alpha) * self.filtered_error_y)

        # 2. PID -- Pan (X axis)
        self.integral_x += self.filtered_error_x * dt
        self.integral_x = max(-self.integral_clamp, min(self.integral_clamp, self.integral_x))
        derivative_x = (self.filtered_error_x - self.prev_error_x) / dt
        self.prev_error_x = self.filtered_error_x
        pan_adj = (self.kp_pan * self.filtered_error_x) + (self.ki_pan * self.integral_x) + (self.kd_pan * derivative_x)

        # 3. PID -- Tilt (Y axis)
        self.integral_y += self.filtered_error_y * dt
        self.integral_y = max(-self.integral_clamp, min(self.integral_clamp, self.integral_y))
        derivative_y = (self.filtered_error_y - self.prev_error_y) / dt
        self.prev_error_y = self.filtered_error_y
        tilt_adj = (self.kp_tilt * self.filtered_error_y) + (self.ki_tilt * self.integral_y) + (self.kd_tilt * derivative_y)

        # 4. Apply (tilt is inverted: pixel Y increases downward, and on this mount a LOWER tilt angle points the camera up -- see tools/servo_calibration.py)
        new_pan = self.pan_angle + pan_adj
        new_tilt = self.tilt_angle - tilt_adj
        self.set_angles(new_pan, new_tilt)

    # -------------------------------------------------------------------
    # Automatic area scan (used while nothing is locked in Idle Mode)
    # -------------------------------------------------------------------

    def update_scan_area(self):
        """Step one increment further through the scan waypoint sequence."""
        target_pan, target_tilt = self.scan_waypoints[self.current_waypoint_idx]

        diff_pan = target_pan - self.pan_angle
        diff_tilt = target_tilt - self.tilt_angle

        # Waypoint reached -> advance to the next one
        tolerance = max(self.scan_speed, 1.0)
        if abs(diff_pan) < tolerance and abs(diff_tilt) < tolerance:
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.scan_waypoints)
            return

        step_p = self.scan_speed if diff_pan > 0 else -self.scan_speed
        step_t = self.scan_speed if diff_tilt > 0 else -self.scan_speed

        next_pan = self.pan_angle + step_p if abs(diff_pan) >= 1.0 else target_pan
        next_tilt = self.tilt_angle + step_t if abs(diff_tilt) >= 1.0 else target_tilt

        self.set_angles(next_pan, next_tilt)
