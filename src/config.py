# =============================================================================
# General / shared settings
# =============================================================================

CAMERA_INDEX = 0             # Used only by the OpenCV backend (USB webcams / legacy stack)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Which camera backend to use:
#   "auto"       -> try Picamera2 first (for the CSI Camera Module 2), fall back to OpenCV
#   "picamera2"  -> force Picamera2 (raise an error if it's unavailable)
#   "opencv"     -> force cv2.VideoCapture (USB webcams, or Pi OS with the legacy camera stack)
CAMERA_BACKEND = "auto"

WINDOW_NAME = "Camera App"   # Title used for the OpenCV display window
HEADLESS = False              # True -> skip cv2.imshow (for running over SSH with no display)

DEBUG = False                 # True -> print extra diagnostic info to the console
SHOW_FPS = True               # True -> overlay a live FPS counter on the video

# Hotkeys (single lowercase characters) -- only relevant when HEADLESS is False
QUIT_KEY = "q"    # Quits the entire application
MENU_KEY = "m"    # Leaves the current mode and returns to the mode-selection menu


# =============================================================================
# Surveillance Mode settings (motion detection / frame differencing)
# =============================================================================

MIN_AREA = 500                 # Minimum contour area (px^2) required to count as motion
BLUR_SIZE = 10                  # Kernel size for blurring the difference image
THRESHOLD_SENSITIVITY = 25     # Pixel intensity difference threshold (0-255)
CIRCLE_SIZE = 10                  # Radius (px) of the marker drawn on detected motion
LINE_THICKNESS = 2                # Thickness (px) of the marker's outline
MOTION_MARKER_COLOR = (0, 255, 0)    # BGR color of the motion marker (green)

# How long (seconds) motion can be absent before the camera gives up tracking
# and returns to the home position.
SURVEILLANCE_RETURN_HOME_TIMEOUT = 2.0


# =============================================================================
# Idle Mode settings (YOLO object detection + simple target-locking tracker)
# =============================================================================

YOLO_MODEL_PATH = "yolov8n.pt"            # Ultralytics model to load (auto-downloads if missing)
TRACKER_CONFIG = "bytetrack.yaml"           # Ultralytics tracker config to use

TARGET_OBJECTS = ["person", "cell phone"]  # Class names to detect/track (must exist in model.names)

CONFIDENCE_THRESHOLD = 0.40    # Minimum detection confidence to accept a new target into the queue
MAX_MISSING_FRAMES = 10           # Frames a locked target can go undetected before being dropped

LOCK_BOX_COLOR = (255, 255, 255)   # BGR color of the "locked target" inner box (white)
LOCK_BOX_INSET = 10                    # Pixels the inner lock box is inset from the detection box
LOCK_BOX_THICKNESS = 2

STATUS_TEXT_COLOR = (255, 255, 255)
STATUS_TEXT_POS = (10, 30)
STATUS_FONT_SCALE = 0.6

# How long (seconds) a locked target can be missing before the camera gives up
# and resumes the automatic area scan.
IDLE_RESUME_SCAN_TIMEOUT = 2.0


# =============================================================================
# Servo / Pan-Tilt Controller settings (PCA9685 via adafruit_servokit)
# =============================================================================

SERVO_KIT_CHANNELS = 16    # Total channels on the PCA9685 board
SERVO_PAN_CHANNEL = 0
SERVO_TILT_CHANNEL = 1

# Physical angle bounds in degrees. Found with tools/servo_calibration.py -- adjust to match your mount's real limits.
PAN_ANGLE_LIMITS = (0, 110)
TILT_ANGLE_LIMITS = (0, 170)   # Calibrated for my mount -- re-calibrate for YOUR mount with tools/servo_calibration.py

HOME_ANGLES = (90.0, 25.0)      # (pan, tilt) the mount returns to when idle/no target

# PID gains: (kp, ki, kd) -- tune these on your actual hardware, not in simulation
PAN_PID_GAINS = (0.04, 0.00005, 0.0000000)#(0,0,0)#
TILT_PID_GAINS = (0.04, 0.000055, 0.0000009)

PID_INTEGRAL_CLAMP = 50   # Anti-windup clamp applied to both axes' integral term

# Low-pass filter alpha applied to the raw pixel error before PID (0 < alpha <= 1).
# Lower = smoother but laggier motion; higher = snappier but more jittery.
LPF_ALPHA = 0.35

# Idle Mode's automatic scan sequence, built from named pan/tilt positions
# instead of a hand-written list of raw (pan, tilt) tuples -- edit the angles
# below to change scan coverage, or edit SCAN_WAYPOINTS itself to change the
# visiting order/pattern.
SCAN_PAN_LEFT = 100
SCAN_PAN_CENTER = 55
SCAN_PAN_RIGHT = 10

SCAN_TILT_UP = 0
SCAN_TILT_DOWN = 160

SCAN_WAYPOINTS = [
    (SCAN_PAN_LEFT, SCAN_TILT_UP),      # Left
    (SCAN_PAN_CENTER, SCAN_TILT_UP),    # Center
    (SCAN_PAN_RIGHT, SCAN_TILT_UP),     # Right
    (SCAN_PAN_RIGHT, SCAN_TILT_DOWN),   # Tilt down (right)
    (SCAN_PAN_CENTER, SCAN_TILT_DOWN),  # Center (down)
    (SCAN_PAN_LEFT, SCAN_TILT_DOWN),    # Left (down) -- loops back to Left/Up next
]
SCAN_SPEED = 9   # Degrees moved per frame while scanning

