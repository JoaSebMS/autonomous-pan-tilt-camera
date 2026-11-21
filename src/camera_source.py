"""
camera_source.py
=================
Unifies two ways of grabbing frames behind one small interface
(`.isOpened()`, `.read()`, `.release()` -- matching cv2.VideoCapture) so the
rest of the app doesn't care which one is in use:

    - Picamera2: for the CSI ribbon-cable Camera Module 2 on modern Raspberry
      Pi OS (Bullseye/Bookworm), which uses the libcamera stack. Plain
      cv2.VideoCapture(0) generally does NOT see this camera unless you're on
      the legacy camera stack.
    - cv2.VideoCapture: for USB webcams, or a Pi Camera on the legacy stack.

Which one gets used is controlled by config.CAMERA_BACKEND ("auto" tries
Picamera2 first and falls back to OpenCV).

Note on color format: Picamera2's "RGB888" format is confusingly named -- it
actually returns pixels in (B, G, R) byte order, which is exactly what
OpenCV expects. So no cv2.cvtColor(...) conversion is needed here; adding one
would flip the colors.
"""

import cv2

import config


def _debug_log(*args):
    if config.DEBUG:
        print("[DEBUG]", *args)


class CameraSource:
    """Drop-in camera wrapper that transparently uses Picamera2 or OpenCV."""

    def __init__(self, index=None, width=None, height=None, backend=None):
        self.index = config.CAMERA_INDEX if index is None else index
        self.width = config.FRAME_WIDTH if width is None else width
        self.height = config.FRAME_HEIGHT if height is None else height
        self.backend_requested = config.CAMERA_BACKEND if backend is None else backend

        self._picam2 = None
        self._cap = None
        self.backend = None

        if self.backend_requested in ("auto", "picamera2"):
            self._try_open_picamera2()

        if self._picam2 is None:
            if self.backend_requested == "picamera2":
                # User explicitly required Picamera2 and it failed -- don't silently
                # fall back, since that could mask a real setup problem.
                return
            self._try_open_opencv()

    def _try_open_picamera2(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            _debug_log("Picamera2 not installed; skipping this backend.")
            return

        try:
            picam2 = Picamera2()
            video_config = picam2.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            picam2.configure(video_config)
            picam2.start()
            self._picam2 = picam2
            self.backend = "picamera2"
            print(f"[INFO] Camera opened via Picamera2 (CSI camera) at {self.width}x{self.height}.")
        except Exception as e:
            print(f"[WARNING] Picamera2 backend failed to start: {e}")
            self._picam2 = None

    def _try_open_opencv(self):
        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            print(f"[ERROR] Could not open camera at index {self.index} via OpenCV either.")
            self._cap = None
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap = cap
        self.backend = "opencv"
        print(f"[INFO] Camera opened via OpenCV (index {self.index}) at {self.width}x{self.height}.")

    def isOpened(self):
        if self._picam2 is not None:
            return True
        return self._cap is not None and self._cap.isOpened()

    def read(self):
        if self._picam2 is not None:
            try:
                frame = self._picam2.capture_array()
                return True, frame
            except Exception as e:
                _debug_log(f"Picamera2 read error: {e}")
                return False, None
        if self._cap is not None:
            return self._cap.read()
        return False, None

    def release(self):
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            except Exception:
                pass
        if self._cap is not None:
            self._cap.release()
