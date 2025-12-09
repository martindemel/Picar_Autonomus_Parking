#!/usr/bin/env python3
"""
auto_park_between_squares.py

Vision-based parking behavior for PiCar-X using Picamera2:
The car uses its camera to find two black squares on a light floor and
parks centered between them.

Requirements:
- PiCar-X Python libs installed under ~/picar-x/lib
- OpenCV (cv2) and NumPy installed
- Picamera2 (python3-picamera2) installed
"""

import sys
import time
import os

import cv2
import numpy as np
from picamera2 import Picamera2

# --- PiCar-X imports ---------------------------------------------------------
PX_LIB_PATH = os.path.expanduser('~/picar-x/lib')
if os.path.isdir(PX_LIB_PATH) and PX_LIB_PATH not in sys.path:
    sys.path.append(PX_LIB_PATH)

try:
    from picarx import Picarx
except ImportError as e:
    print("[ERROR] Could not import Picarx. "
          "Check that the PiCar-X Python libraries are installed under ~/picar-x/lib.")
    raise

# ---------------------------------------------------------------------------
# Parameters you will likely want to tune
# ---------------------------------------------------------------------------

# Camera configuration
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# Thresholding for black squares on light background
# more permissive so we can see squares when far/near
GRAY_THRESHOLD = 80         # a bit higher; treat more dark as "marker"
MIN_MARKER_AREA = 200       # allow smaller blobs when far away
MAX_MARKER_AREA = 150000    # allow larger blobs when close

# Parking control parameters
KP_STEER = 60.0             # proportional gain for steering (deg per normalized error)
MAX_STEER_ANGLE = 30.0      # physical limit of steering servo in degrees (±30 is typical)
CENTER_TOLERANCE = 0.06     # normalized [-1,1] allowed center error (e.g. 6%)

# Target distance based on marker size (height in pixels)
TARGET_MARKER_HEIGHT = 160  # bigger => park closer to the squares
HEIGHT_TOLERANCE = 15       # how close to target height to count as "parked"

# Speeds (0–100 is typical range; PiCar-X docs use ~20–40 for gentle motion)
SPEED_FAR = 30
SPEED_NEAR = 18

# Ultrasonic safety
# (currently disabled for tuning; vision decides when to stop)
USE_ULTRASONIC = False
SAFE_DISTANCE_CM = 3.0      # ignored when USE_ULTRASONIC = False

# Debug visualization (requires a desktop / VNC session)
SHOW_DEBUG = False


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def clamp(val, lo, hi):
    return max(lo, min(val, hi))


def detect_black_squares(frame_bgr):
    """Detect roughly square dark blobs (your printed black squares) on a light
    background. Returns a list of bounding boxes (x, y, w, h)."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Invert threshold: dark -> white, light -> black
    _, thresh = cv2.threshold(blur, GRAY_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

    # Morphological opening to remove small noise
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    markers = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_MARKER_AREA or area > MAX_MARKER_AREA:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if h == 0:
            continue

        aspect = w / float(h)

        # Keep things that are roughly square-ish
        if 0.6 <= aspect <= 1.4:
            markers.append((x, y, w, h))

    if len(markers) < 1:
        return []

    # Choose the two largest blobs (by area)
    markers = sorted(markers, key=lambda b: b[2] * b[3], reverse=True)[:2]
    # Sort them left to right by center x
    markers.sort(key=lambda b: b[0] + b[2] / 2.0)

    return markers


def compute_steering_and_speed(markers, frame_width, px):
    """Compute steering angle and forward speed from marker positions.
    Returns (steer_angle_deg, forward_speed, parked_flag)."""

    (x1, y1, w1, h1), (x2, y2, w2, h2) = markers
    cx1 = x1 + w1 / 2.0
    cx2 = x2 + w2 / 2.0

    mid_x = (cx1 + cx2) / 2.0
    img_center_x = frame_width / 2.0

    # Normalized lateral error in [-1, 1]
    error_x = (mid_x - img_center_x) / img_center_x

    # Steering: positive error_x means markers to the right, so steer right (or left).
    # If this is inverted on your car, flip the sign of error_x or KP_STEER.
    steer_angle = clamp(KP_STEER * error_x, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

    # Distance estimation using average marker height
    avg_h = (h1 + h2) / 2.0
    height_error = TARGET_MARKER_HEIGHT - avg_h

    # Ultrasonic safety stop (currently disabled via USE_ULTRASONIC)
    distance = None
    if USE_ULTRASONIC:
        try:
            distance = px.ultrasonic.read()  # may return -1 if invalid
        except Exception:
            distance = None

    if USE_ULTRASONIC and distance is not None and distance != -1 and distance < SAFE_DISTANCE_CM:
        # Too close to something – stop and treat as parked for safety
        print(f"[INFO] Ultrasonic: {distance:.1f} cm < SAFE_DISTANCE_CM, stopping.")
        return 0.0, 0, True

    # Decide if we are "parked" based on lateral error and marker size
    centered = abs(error_x) < CENTER_TOLERANCE
    good_height = abs(height_error) < HEIGHT_TOLERANCE

    if centered and good_height:
        # Already in a good spot
        return 0.0, 0, True

    # Otherwise, keep moving forward with speed based on distance/height
    if height_error > 30:
        speed = SPEED_FAR   # far away, move a bit faster
    else:
        speed = SPEED_NEAR  # close, crawl in slowly

    return steer_angle, speed, False


def draw_debug(frame_bgr, markers):
    """Draw markers and midline for visualization."""
    if len(markers) < 1:
        return frame_bgr

    (x1, y1, w1, h1) = markers[0]
    cv2.rectangle(frame_bgr, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)
    cx1, cy1 = int(x1 + w1 / 2), int(y1 + h1 / 2)
    cv2.circle(frame_bgr, (cx1, cy1), 4, (0, 0, 255), -1)

    if len(markers) > 1:
        (x2, y2, w2, h2) = markers[1]
        cv2.rectangle(frame_bgr, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 2)
        cx2, cy2 = int(x2 + w2 / 2), int(y2 + h2 / 2)
        cv2.circle(frame_bgr, (cx2, cy2), 4, (0, 0, 255), -1)

        mid_x = int((cx1 + cx2) / 2.0)
        cv2.line(frame_bgr, (mid_x, 0), (mid_x, frame_bgr.shape[0]), (255, 0, 0), 2)

    # Draw image center line
    img_center_x = frame_bgr.shape[1] // 2
    cv2.line(frame_bgr, (img_center_x, 0), (img_center_x, frame_bgr.shape[0]), (255, 255, 0), 1)

    return frame_bgr


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main():
    # Init PiCar-X
    px = Picarx()
    px.set_dir_servo_angle(0)
    px.forward(0)

    # Init Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)  # let camera warm up

    print("[INFO] Starting auto-park between squares.")
    print("      Place the car so the two black squares are in front of it,")
    print("      then this script will try to align and drive into the spot.")
    print("      Press Ctrl+C to abort.")

    parked_frames = 0
    REQUIRED_PARKED_FRAMES = 8  # require stability over N frames

    try:
        while True:
            # Capture frame as RGB and convert to BGR for OpenCV
            frame_rgb = picam2.capture_array()
            if frame_rgb is None:
                print("[WARN] Failed to read frame from camera.")
                px.forward(0)
                time.sleep(0.1)
                continue

            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            markers = detect_black_squares(frame_bgr)

            if len(markers) == 0:
                # Nothing visible – safest is to stop
                print("[INFO] No markers visible; stopping.")
                px.forward(0)
                px.set_dir_servo_angle(0)
                parked_frames = 0
            else:
                if len(markers) == 1:
                    # Only one marker visible – use it twice so we can still
                    # compute a "center" and keep moving forward.
                    markers = [markers[0], markers[0]]

                steer_angle, speed, parked_flag = compute_steering_and_speed(
                    markers, frame_bgr.shape[1], px
                )
                print(
                    f"[CTRL] steer={steer_angle:5.1f} deg  speed={speed:3d} "
                    f"parked={parked_flag}"
                )

                px.set_dir_servo_angle(steer_angle)
                px.forward(speed)

                if parked_flag:
                    parked_frames += 1
                else:
                    parked_frames = 0

                # Require several consecutive frames "parked" before we really stop
                if parked_frames >= REQUIRED_PARKED_FRAMES:
                    print("[INFO] Parking complete. Holding position.")
                    px.forward(0)
                    px.set_dir_servo_angle(0)
                    break

            if SHOW_DEBUG:
                debug_frame = frame_bgr.copy()
                debug_frame = draw_debug(debug_frame, markers)
                cv2.imshow("Parking debug", debug_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user (Ctrl+C). Stopping car.")
    finally:
        px.forward(0)
        px.set_dir_servo_angle(0)
        picam2.stop()
        if SHOW_DEBUG:
            cv2.destroyAllWindows()
        print("[INFO] Clean exit.")


if __name__ == "__main__":
    main()

