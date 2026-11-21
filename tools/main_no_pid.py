"""
main_no_pid.py
==============
Autonomous Pan-Tilt Camera -- NO-PID TEST VERSION

This is a copy of main.py with the PID tracking controller swapped out for a
simple "dead-zone" (bang-bang) controller, so you can verify the camera,
detection, and servo wiring/direction all work correctly BEFORE tuning any
PID gains.

How the dead-zone controller works:
    A small square (NO_PID_DEAD_ZONE_SIZE pixels per side, default 40) is
    imagined at the center of the frame. Each frame:
        - If the target's center falls inside that square -> do nothing,
          the servos hold still.
        - If it's outside the square -> step pan/tilt by a fixed amount
          (NO_PID_STEP_DEGREES degrees, default 1) toward the target, one
          axis at a time as needed. No proportional/integral/derivative math,
          no low-pass filtering -- just "am I close enough? If not, nudge."

This makes it easy to sanity-check: does increasing pan angle actually turn
the camera toward a target on the left (or does it need to be flipped in
config.py)? Does the mount reach and hold a target without PID overshoot
hiding a wiring problem? Once this looks right, move on to servo_controller.py's
PID tracking and use the tuning guide to dial it in.

Architecture:
    CameraSource -> Detection (YOLO / Motion) -> Tracking (ByteTrack / Frame Diff)
    -> Center Calculation -> Dead-Zone Step Controller (this file) -> PCA9685 Servos

Modes (each a function below):
    1) Surveillance Mode  -> Sits at Home (config.HOME_ANGLES). On motion, steps toward it
                              until it's inside the center dead-zone square.
                              Returns Home after config.SURVEILLANCE_RETURN_HOME_TIMEOUT
                              seconds of no motion.
    2) Idle Mode          -> Continuously scans config.SCAN_WAYPOINTS.
                              Locks onto a YOLO+ByteTrack target, steps toward
                              it until it's inside the dead-zone square. Resumes
                              scanning after config.IDLE_RESUME_SCAN_TIMEOUT
                              seconds of no target.

Usage
-----
    python tools/main_no_pid.py                       interactive menu, asks which mode to run
    python tools/main_no_pid.py --mode surveillance   skip the menu, start directly in Surveillance Mode
    python tools/main_no_pid.py --mode idle           skip the menu, start directly in Idle Mode
    python tools/main_no_pid.py --camera 1            use camera index 1 (OpenCV backend only)
    python tools/main_no_pid.py --headless            no display window (for running over SSH)

While a mode is running (non-headless):
    q  -> quit the whole application
    m  -> stop the current mode and go back to the mode-selection menu

All tunable values (thresholds, colors, camera settings, scan waypoints,
hotkeys...) live in config.py -- edit that file rather than this one.
The dead-zone size and step amount for THIS test controller are the two
constants right below the imports.
Servo/scan hardware logic (set_angles, move_home, area scan) lives in
servo_controller.py and is reused as-is; only the PID tracking call is
bypassed.
Camera backend selection (Picamera2 vs OpenCV) lives in camera_source.py.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2

# Make the modules in ../src importable when running this script from the repo root
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import config
from camera_source import CameraSource
from servo_controller import ServoController

# ---------------------------------------------------------------------------
# No-PID test controller settings
# ---------------------------------------------------------------------------
NO_PID_DEAD_ZONE_SIZE = 40   # pixels per side of the "close enough" square at frame center
NO_PID_STEP_DEGREES = 1      # degrees to nudge pan/tilt per frame when outside the square


# =============================================================================
# Shared helpers (used by both modes)
# =============================================================================

def debug_log(*args):
    """Print a message only when config.DEBUG is True."""
    if config.DEBUG:
        print("[DEBUG]", *args)


class FPSCounter:
    """Tracks a smoothed FPS estimate and the raw delta-time (dt) between frames."""

    def __init__(self, smoothing=0.9):
        self.smoothing = smoothing
        self.fps = 0.0
        self._prev_time = time.time()
        self.dt = 0.033  # sane default for the very first frame

    def tick(self):
        now = time.time()
        self.dt = now - self._prev_time
        self._prev_time = now
        if self.dt > 0:
            instantaneous = 1.0 / self.dt
            self.fps = (self.fps * self.smoothing) + (instantaneous * (1 - self.smoothing))
        return self.fps, self.dt


def draw_fps(frame, fps):
    """Overlay an FPS readout in the top-right corner, if enabled in config."""
    if not config.SHOW_FPS:
        return
    text = f"FPS: {fps:.1f}"
    (text_w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    x = max(10, frame.shape[1] - text_w - 10)
    cv2.putText(frame, text, (x, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


def draw_hint_bar(frame, state_text=""):
    """Draw a small reminder of the hotkeys + current FSM state."""
    hint = f"[{config.QUIT_KEY}] quit  [{config.MENU_KEY}] menu | State: {state_text}"
    y = frame.shape[0] - 10
    cv2.putText(frame, hint, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


def show_frame(frame, state_text=""):
    """Display the frame (with overlays) unless running headless."""
    if config.HEADLESS:
        return None
    draw_hint_bar(frame, state_text)
    cv2.imshow(config.WINDOW_NAME, frame)
    return handle_common_keys(cv2.waitKey(1) & 0xFF)


def handle_common_keys(key):
    """Check a cv2.waitKey() result against the shared hotkeys. Returns 'quit', 'menu', or None."""
    if key == ord(config.QUIT_KEY):
        return "quit"
    if key == ord(config.MENU_KEY):
        return "menu"
    return None


# =============================================================================
# No-PID dead-zone step controller (replaces ServoController.update_tracking)
# =============================================================================

def simple_center_tracking(servo, target_cx, target_cy, frame_cx, frame_cy):
    """
    No-PID tracking test: if (target_cx, target_cy) already falls inside the
    NO_PID_DEAD_ZONE_SIZE square at the center of the frame, do nothing -- the
    servos hold their current position. Otherwise, nudge pan and/or tilt by
    NO_PID_STEP_DEGREES toward the target (only the axis/axes still outside
    the square move), using the same sign convention as
    ServoController.update_tracking so left/right and up/down behave the same
    way they will once PID is switched back on.
    """
    half = NO_PID_DEAD_ZONE_SIZE // 2

    error_x = frame_cx - target_cx  # positive -> target is left of center
    error_y = frame_cy - target_cy  # positive -> target is above center

    if abs(error_x) <= half and abs(error_y) <= half:
        return  # target is inside the square -> stop, don't move at all

    new_pan = servo.pan_angle
    new_tilt = servo.tilt_angle

    if abs(error_x) > half:
        new_pan += NO_PID_STEP_DEGREES if error_x > 0 else -NO_PID_STEP_DEGREES
    if abs(error_y) > half:
        new_tilt -= NO_PID_STEP_DEGREES if error_y > 0 else -NO_PID_STEP_DEGREES

    servo.set_angles(new_pan, new_tilt)


def draw_dead_zone(frame, frame_cx, frame_cy):
    """Draw the center dead-zone square on screen so you can see the target
    needs to be inside this box for the servos to stop."""
    half = NO_PID_DEAD_ZONE_SIZE // 2
    cv2.rectangle(
        frame,
        (frame_cx - half, frame_cy - half),
        (frame_cx + half, frame_cy + half),
        (255, 255, 0), 2
    )


# =============================================================================
# Surveillance Mode: frame differencing + motion tracking FSM
# =============================================================================

def motion_detect(gray_img_1, gray_img_2):
    """Compare two grayscale frames and return the center (x, y) of the largest
    area of motion above config.MIN_AREA, or () if no motion is found."""
    motion_found = False
    biggest_area = config.MIN_AREA
    mx = my = mw = mh = 0

    differenceimage = cv2.absdiff(gray_img_1, gray_img_2)
    differenceimage = cv2.blur(differenceimage, (config.BLUR_SIZE, config.BLUR_SIZE))

    _, thresholdimage = cv2.threshold(
        differenceimage, config.THRESHOLD_SENSITIVITY, 255, cv2.THRESH_BINARY
    )

    # Handles both old/new OpenCV findContours return signatures
    try:
        thresholdimage, contours, hierarchy = cv2.findContours(
            thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
    except ValueError:
        contours, hierarchy = cv2.findContours(
            thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

    for c in contours:
        found_area = cv2.contourArea(c)
        if found_area > biggest_area:
            biggest_area = found_area
            (mx, my, mw, mh) = cv2.boundingRect(c)
            motion_found = True

    if motion_found:
        return (int(mx + mw / 2), int(my + mh / 2))
    return ()


def run_surveillance_mode(cap, servo):
    """
    Surveillance Mode FSM:
    Home position -> wait for motion -> track it -> after
    config.SURVEILLANCE_RETURN_HOME_TIMEOUT seconds with no motion -> Home.
    Returns 'quit' or 'menu'.
    """
    print(f"\n--- Surveillance Mode --- (press '{config.QUIT_KEY}' to quit, "
          f"'{config.MENU_KEY}' for menu)")

    ret, frame1 = cap.read()
    if not ret:
        print("[ERROR] Could not read from camera.")
        return "menu"
    grayimage1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

    fps_counter = FPSCounter()
    state = "WAIT_FOR_MOTION"
    last_motion_time = time.time()
    servo.move_home()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Lost connection to camera.")
            break

        h, w = frame.shape[:2]
        frame_cx, frame_cy = w // 2, h // 2
        fps, dt = fps_counter.tick()

        grayimage2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        motion_center = motion_detect(grayimage1, grayimage2)
        grayimage1 = grayimage2

        if motion_center != ():
            cx, cy = motion_center
            if state != "TRACKING_MOVEMENT":
                debug_log("Motion acquired -> tracking")
            state = "TRACKING_MOVEMENT"
            last_motion_time = time.time()

            simple_center_tracking(servo, cx, cy, frame_cx, frame_cy)
            cv2.circle(frame, (cx, cy), config.CIRCLE_SIZE, config.MOTION_MARKER_COLOR, config.LINE_THICKNESS)
        elif state == "TRACKING_MOVEMENT" and \
                time.time() - last_motion_time > config.SURVEILLANCE_RETURN_HOME_TIMEOUT:
            debug_log("Motion lost -> returning home")
            state = "WAIT_FOR_MOTION"
            servo.move_home()

        draw_dead_zone(frame, frame_cx, frame_cy)
        draw_fps(frame, fps)
        action = show_frame(frame, state_text=state)
        if action:
            return action

    return "menu"


# =============================================================================
# Idle Mode: YOLO + ByteTrack + area-scan FSM
# =============================================================================

_yolo_model = None


def get_yolo_model():
    """Lazily load (and cache) the YOLO model so switching modes doesn't
    reload it every time, and so Surveillance Mode works without ultralytics
    installed at all."""
    global _yolo_model
    if _yolo_model is None:
        try:
            from ultralytics import YOLO
        except ImportError:
            print("[ERROR] The 'ultralytics' package is required for Idle Mode.")
            print("        Install dependencies with: pip install -r requirements.txt")
            return None
        print(f"Loading YOLO model '{config.YOLO_MODEL_PATH}' (first run may take a moment)...")
        _yolo_model = YOLO(config.YOLO_MODEL_PATH)
    return _yolo_model


def run_idle_mode(cap, servo):
    """
    Idle Mode FSM:
    Scan config.SCAN_WAYPOINTS -> lock a YOLO+ByteTrack target -> track it -> after
    config.IDLE_RESUME_SCAN_TIMEOUT seconds with no target -> resume scanning.
    Returns 'quit' or 'menu'.
    """
    model = get_yolo_model()
    if model is None:
        return "menu"

    name_to_id = {name: cls_id for cls_id, name in model.names.items()}
    target_class_ids = [name_to_id[obj] for obj in config.TARGET_OBJECTS if obj in name_to_id]

    missing_objs = [obj for obj in config.TARGET_OBJECTS if obj not in name_to_id]
    if missing_objs:
        print(f"[WARNING] These TARGET_OBJECTS aren't in the model's class list "
              f"and will be ignored: {missing_objs}")

    print(f"\n--- Idle Mode --- (press '{config.QUIT_KEY}' to quit, "
          f"'{config.MENU_KEY}' for menu)")
    print(f"Tracking: {config.TARGET_OBJECTS}")

    # Queue tracking structures (reset every time the mode is (re)entered)
    active_targets_queue = []
    missing_frame_counts = {}  # {track_id: consecutive_missing_frames}

    fps_counter = FPSCounter()
    state = "SCANNING"
    last_target_seen_time = time.time()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Lost connection to camera.")
            break

        h, w = frame.shape[:2]
        frame_cx, frame_cy = w // 2, h // 2
        fps, dt = fps_counter.tick()

        results = model.track(
            frame, persist=True, tracker=config.TRACKER_CONFIG,
            classes=target_class_ids, verbose=False
        )
        annotated_frame = results[0].plot()

        current_frame_ids = []
        detected_boxes_map = {}

        if results[0].boxes is not None and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            track_ids = results[0].boxes.id.int().cpu().numpy()
            confs = results[0].boxes.conf.cpu().numpy()

            for box, track_id, conf in zip(boxes, track_ids, confs):
                current_frame_ids.append(track_id)
                detected_boxes_map[track_id] = box
                missing_frame_counts[track_id] = 0

                if conf > config.CONFIDENCE_THRESHOLD and track_id not in active_targets_queue:
                    active_targets_queue.append(track_id)
                    debug_log(f"New target queued: id={track_id} conf={conf:.2f}")

        # Purge targets missing too long
        purged_ids = []
        for tid in active_targets_queue:
            if tid not in current_frame_ids:
                missing_frame_counts[tid] = missing_frame_counts.get(tid, 0) + 1
                if missing_frame_counts[tid] > config.MAX_MISSING_FRAMES:
                    purged_ids.append(tid)
        for tid in purged_ids:
            active_targets_queue.remove(tid)
            missing_frame_counts.pop(tid, None)
            debug_log(f"Target purged (lost too long): id={tid}")

        locked_id = active_targets_queue[0] if active_targets_queue else None

        if locked_id is not None and locked_id in detected_boxes_map:
            if state != "TRACKING_TARGET":
                debug_log("Target acquired -> tracking")
            state = "TRACKING_TARGET"
            last_target_seen_time = time.time()

            x1, y1, x2, y2 = detected_boxes_map[locked_id]
            target_cx = int((x1 + x2) / 2)
            target_cy = int((y1 + y2) / 2)
            simple_center_tracking(servo, target_cx, target_cy, frame_cx, frame_cy)

            inset = config.LOCK_BOX_INSET
            cv2.rectangle(
                annotated_frame,
                (int(x1) + inset, int(y1) + inset),
                (int(x2) - inset, int(y2) - inset),
                config.LOCK_BOX_COLOR, config.LOCK_BOX_THICKNESS
            )
        elif time.time() - last_target_seen_time > config.IDLE_RESUME_SCAN_TIMEOUT:
            if state != "SCANNING":
                debug_log("Target lost -> resuming scan")
                servo.reset_pid()
            state = "SCANNING"
            servo.update_scan_area()

        status_text = f"Queue: {active_targets_queue} | Locked: {locked_id}"
        cv2.putText(
            annotated_frame, status_text, config.STATUS_TEXT_POS,
            cv2.FONT_HERSHEY_SIMPLEX, config.STATUS_FONT_SCALE,
            config.STATUS_TEXT_COLOR, 2
        )

        draw_dead_zone(annotated_frame, frame_cx, frame_cy)
        draw_fps(annotated_frame, fps)
        action = show_frame(annotated_frame, state_text=state)
        if action:
            return action

    return "menu"


# =============================================================================
# Menu / entry point
# =============================================================================

MODE_RUNNERS = {
    "surveillance": run_surveillance_mode,
    "idle": run_idle_mode,
}


def choose_mode():
    """Ask the user which mode to run. Returns 'surveillance', 'idle', or None to quit."""
    print("\n=== Select Camera Mode ===")
    print("  1) Surveillance Mode  (Stationary + Motion Tracking)")
    print("  2) Idle Mode          (Area Scan + YOLO Object Tracking)")
    print("  q) Quit")

    while True:
        choice = input("Enter choice [1/2/q]: ").strip().lower()
        if choice in ("1", "surveillance", "s"):
            return "surveillance"
        if choice in ("2", "idle", "i"):
            return "idle"
        if choice in ("q", "quit", "exit"):
            return None
        print("Please enter 1, 2, or q.")


def parse_args():
    parser = argparse.ArgumentParser(description="Autonomous Pan-Tilt Camera (no-PID dead-zone test)")
    parser.add_argument(
        "--mode", choices=["surveillance", "idle"], default=None,
        help="Start directly in this mode instead of showing the menu."
    )
    parser.add_argument(
        "--camera", type=int, default=None,
        help="Camera index to use (OpenCV backend only; overrides config.CAMERA_INDEX)."
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without a display window (overrides config.HEADLESS). Useful over SSH."
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="Enable debug logging (overrides config.DEBUG)."
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.debug:
        config.DEBUG = True
    if args.headless:
        config.HEADLESS = True
    if args.camera is not None:
        config.CAMERA_INDEX = args.camera

    cap = CameraSource()
    if not cap.isOpened():
        print("[ERROR] Could not open any camera. Check connections/config.CAMERA_BACKEND.")
        sys.exit(1)

    servo = ServoController()

    # If --mode was passed, run it once then fall back to the interactive menu
    # afterward (unless the user quits, or we're headless with no menu to show).
    next_mode = args.mode
    if config.HEADLESS and next_mode is None:
        print("[ERROR] --headless requires --mode (there's no display to show a menu on).")
        cap.release()
        sys.exit(1)

    try:
        while True:
            if next_mode is None:
                next_mode = choose_mode()
                if next_mode is None:
                    print("Exiting application...")
                    break

            runner = MODE_RUNNERS[next_mode]
            result = runner(cap, servo)

            if result == "quit" or config.HEADLESS:
                print("Exiting application...")
                break
            next_mode = None
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        servo.move_home()
        cap.release()
        if not config.HEADLESS:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
