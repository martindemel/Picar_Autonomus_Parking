# PiCar-X Autonomous Parking

This project implements a vision-based autonomous parking behavior for the SunFounder PiCar-X using a Raspberry Pi 5, Picamera2, and OpenCV. The car detects two printed black squares on a light floor and parks itself centered between them.

![Autonomous Parking Example](example.png)

## Features

- Uses the PiCar-X camera (Picamera2) to detect black square markers.
- Computes the midpoint between two markers and steers to stay centered.
- Moves forward while adjusting steering angle based on image-space error.
- Stops when the car is centered and close enough to the markers (based on their apparent size in the image).
- Ultrasonic safety is currently disabled for tuning, but logic is in place and can be re-enabled.

## Requirements

- Raspberry Pi 5 (or compatible Pi) running Raspberry Pi OS (Bookworm).
- SunFounder PiCar-X with Python libraries installed under `~/picar-x/lib`.
- Python packages:
  - `python3-opencv`
  - `python3-numpy`
  - `python3-picamera2`
  - `libcamera-apps` (system package)

Install the main dependencies:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-picamera2 libcamera-apps
```

## Setup

1. Clone or copy this repository to your Pi.
2. Ensure the PiCar-X Python libraries are installed and available under `~/picar-x/lib`.
3. Place the file `auto_park_between_squares.py` in your `~/picar-x` folder (or adjust the path in the script).

## Preparing the Parking Markers

* Print two solid black squares on white paper (e.g., US Letter or A4).
* Place the papers on a light-colored floor with a gap between them like a parking slot.
* Put the PiCar-X a short distance away, facing the two squares.
* Make sure the area is reasonably well-lit.

## Running the Script

From the Pi:

```bash
cd ~/picar-x
python3 auto_park_between_squares.py
```

You should see console output like:

```text
[INFO] Starting auto-park between squares.
      Place the car so the two black squares are in front of it,
      then this script will try to align and drive into the spot.
      Press Ctrl+C to abort.
[CTRL] steer=  3.4 deg  speed= 30 parked=False
...
```

Press `Ctrl + C` any time to stop the script.

If your system requires `sudo` for Picamera2 (like some SunFounder examples), run:

```bash
sudo python3 auto_park_between_squares.py
```

## Tuning Parameters

Key parameters are defined near the top of the script:

* `GRAY_THRESHOLD` – threshold for detecting dark markers. Increase if the markers appear too light; decrease if too much noise is detected.
* `MIN_MARKER_AREA`, `MAX_MARKER_AREA` – filter small/large blobs. Reduce `MIN_MARKER_AREA` if squares are far away and appear small.
* `TARGET_MARKER_HEIGHT` – controls how close the car gets before considering itself parked. Increase to stop closer; decrease to stop further away.
* `KP_STEER` and `MAX_STEER_ANGLE` – adjust how aggressively the car steers toward the center.
* `USE_ULTRASONIC`, `SAFE_DISTANCE_CM` – enable and configure ultrasonic safety (currently disabled for tuning).

## Notes

* When the car is far away, both squares should be visible; as it comes closer, it may only see one square. The script falls back to using a single detected marker and continues to move.
* The current configuration is tuned for a reasonably bright room and black-on-white markers. You may need to adjust thresholds for your environment.
* Always test in an open, safe area and be ready to stop the car with `Ctrl + C` or by physically blocking it.

## License

You can adapt and extend this project freely for educational and personal use.

