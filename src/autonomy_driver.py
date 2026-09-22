#!/usr/bin/env python3
"""
autonomy_driver.py

Closed-loop driving code for the WRO Future Engineers robot. Runs on the
Raspberry Pi 5, fuses camera + lidar data, decides steering/speed, and
sends commands to the Arduino Nano over serial using the same
M<speed>,<steer> protocol the Nano sketch (nano_motor_control.ino) expects.

Camera and lidar handling here match the standalone viewer you validated
(lidar_USBcam2.py): Picamera2 with explicit RGB->BGR conversion, and the
LDROBOT STL-19P (LD19) lidar read directly over raw serial with the same
packet parser -- no third-party lidar library needed.

While driving, this also shows the same camera+lidar side-by-side preview
window the viewer did, so you can watch what the robot is actually seeing
in real time. Closing that window (X button or 'q') only hides the
preview -- it does NOT stop the robot. Ctrl+z in the terminal is the only
way to stop the robot itself.

STARTUP: on launch, the script connects everything (Nano, lidar, camera),
shows the preview, and explicitly holds the robot at zero speed while it
waits for the start switch (a simple on/off switch wired to a GPIO pin,
read via gpiozero) to change state -- matching the WRO rule that a
separate action must start the timed run after power-on. Nothing drives
until that switch flips.

This is a *baseline* -- it will drive the robot, but the constants (gains,
HSV thresholds, wall-following target distance, pillar avoidance offsets)
all need tuning on your actual mat before it's competition-ready. Treat it
as a skeleton to iterate on, not a finished solution. In particular:

  - The pillar distance estimate (area -> distance) is a crude placeholder.
    Replace it with a real camera+lidar fusion (match the detected pillar's
    angle in frame to the nearest lidar point at that bearing) once you
    have real numbers to calibrate against -- see the reasoning discussed
    for the reference repo this project was benchmarked against.
  - The "wall coming up, turn now" reflex is not real corner detection.
    A stronger approach analyzes lidar wall segments to recognize you're
    approaching a corner section before you're right on top of it.
  - The front ToF sensor is used as an independent emergency-stop check
    (see TOF_EMERGENCY_MM below) layered on top of the lidar/camera logic,
    not as a primary navigation sensor -- lidar covers that job better.
    See the conversation notes on why 4 ToF sensors (one per side) stopped
    making sense once lidar was working: lidar's continuous 360 degree
    coverage does what 4 discrete side sensors were meant to do, better.

SAFETY: the first time you run this, get the drive wheels off the ground
(blocks, a stand, whatever) or clear a large empty area. A wrong steering
sign or an untuned gain will send the robot into a wall at full speed
before you can react.

Two driving modes, matching the two WRO FE challenges:
  open      -- lidar-based wall following only, 3 laps, stops on lap complete
  obstacle  -- wall following + camera/lidar pillar avoidance, 3 laps, then
               seeks the purple parking markers and creeps in using camera
               alignment + front ToF distance to know when to stop

Parking (obstacle mode only): after lap 3, the state machine switches to
SEEKING_PARKING -- steers to center between the two purple boundary
markers (same HSV-masking technique as the pillars, kept in a separate
color range so parking markers can never be mistaken for a pillar to
avoid), creeping forward at PARK_APPROACH_SPEED until the front ToF reads
under PARK_STOP_MM. A PARK_SEEK_TIMEOUT_S safety stop is included in case
the markers are never found -- better to stop and fail the parking bonus
than drive indefinitely on a bad detection.

NOTE: this does not yet solve the Open Challenge's "stop inside the start
section" scoring requirement -- open mode still just halts the instant lap
3 completes, wherever that happens to be. That's a related but separate
problem (no colored markers to align against there) and isn't built yet.

Install dependencies:
    sudo apt install -y python3-opencv
    pip install pyserial
    (Picamera2 ships with Raspberry Pi OS; no lidar library needed --
    the STL-19P/LD19 protocol is parsed directly from raw serial bytes.)

Run:
    python3 autonomy_driver.py --mode open
    python3 autonomy_driver.py --mode obstacle
"""

import argparse
import re
import struct
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto

import cv2
import numpy as np
import serial
from gpiozero import Button
from picamera2 import Picamera2

# ---------------------------------------------------------------------
# Configuration -- tune all of this against your actual robot and mat.
# Check `ls /dev/serial/by-id/` to find which USB device is the Nano and
# which is the lidar; don't assume ttyUSB0/1 stay consistent across reboots.
# ---------------------------------------------------------------------
NANO_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"           # CH340 -- the Nano
NANO_BAUD = 115200
DEBUG_SERIAL = True   # print every raw command sent / line received on the
                       # Nano link -- noisy at 50Hz, but exactly what you
                       # need while diagnosing a dead link. Set False once
                       # things are confirmed working.
LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"  # CP2102 -- the lidar

# Connection resilience -- USB devices sometimes aren't enumerated yet the
# instant this script starts (right after boot, or right after a replug),
# and a link can drop momentarily mid-run (e.g. a USB brownout). These
# control how hard NanoLink/LidarSource/Picamera2 retry instead of just
# giving up once.
NANO_CONNECT_RETRIES = 10
NANO_CONNECT_RETRY_DELAY_S = 1.0
NANO_RECONNECT_BACKOFF_S = 1.0
LIDAR_CONNECT_RETRIES = 10
LIDAR_CONNECT_RETRY_DELAY_S = 1.0
LIDAR_RECONNECT_BACKOFF_S = 1.0
CAMERA_INIT_RETRIES = 10
CAMERA_INIT_RETRY_DELAY_S = 1.0

# Mode switch -- a 3-way (ON-OFF-ON, i.e. CENTER-OFF) rocker switch. Wire
# the CENTER/common terminal to any GND pin. Wire one side terminal to
# OPEN_MODE_PIN, the other side terminal to OBSTACLE_MODE_PIN. Uses the
# Pi's internal pull-ups, so each pin reads HIGH (inactive) in the neutral
# center position, and LOW (active) when thrown to that side. One flip of
# this switch both selects the mode AND starts the run.
#
# IMPORTANT: this must be a genuine center-off switch (three stable
# positions), not a two-position switch that merely has three terminals --
# without a real neutral, the robot would try to start the instant it's
# powered on, with no window to check the preview first.
OPEN_MODE_PIN = 17       # BCM numbering -- verify against your wiring
OBSTACLE_MODE_PIN = 27   # BCM numbering -- verify against your wiring
LIDAR_BAUDRATE = 230400        # LDROBOT STL-19P / LD19 standard rate
LIDAR_MIN_CONFIDENCE = 100     # 0-255, matches the validated viewer script
LIDAR_VIEW_SIZE = 500          # pixels, square preview canvas
LIDAR_MAX_RANGE_M = 2.0        # meters shown at the preview canvas edge

CAMERA_SIZE = (640, 480)

# Wall following (lidar)
TARGET_WALL_DIST_M = 0.30      # how far to stay from the nearer side wall
WALL_FOLLOW_KP = 400.0         # proportional gain: distance error (m) -> steering (deg)
FRONT_STOP_DIST_M = 0.35       # treat a wall this close ahead as "corner incoming"
STEER_LIMIT_DEG = 28

BASE_SPEED = 130               # nominal forward PWM (0-255)
TURN_SPEED = 90                # PWM while executing a sharp turn

# Pillar avoidance (camera + lidar fusion)
HSV_RANGES = {
    "red":   [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((35, 80, 80), (85, 255, 255))],
}
PILLAR_AREA_THRESHOLD = 400
PILLAR_TRIGGER_DIST_M = 0.45   # start avoiding once estimated distance is under this
PILLAR_STEER_DEG = 22          # how hard to steer away while passing a pillar

# Parking markers -- deliberately NOT part of HSV_RANGES above. detect_pillars()
# iterates HSV_RANGES for red/green avoidance decisions; keeping purple out of
# that dict means a parking marker can never accidentally get treated as a
# pillar to dodge mid-race. Magenta (255,0,255) -> roughly hue 150 in OpenCV's
# 0-179 scale.
PARKING_HSV_RANGES = [((140, 80, 80), (165, 255, 255))]
PARKING_AREA_THRESHOLD = 150   # markers are narrower than pillars (20mm vs 50mm)
PARK_APPROACH_SPEED = 80       # slow creep while aligning/entering the box
PARK_ALIGN_STEER_LIMIT = 20    # cap steering while creeping into the box, degrees
PARK_STOP_MM = 80              # front ToF distance (mm) that triggers the final stop
PARK_SEEK_TIMEOUT_S = 15.0     # safety: give up and stop if markers are never found

# Front ToF -- independent emergency-stop backstop, not primary navigation
# (see module docstring for why this changed from "one per side" to just this)
TOF_EMERGENCY_MM = 100

# Lap counting via heading integration (gyro Z from the Nano's IMU)
DEG_PER_LAP = 360.0
LAP_COUNT_TARGET = 3
# ---------------------------------------------------------------------


class Mode(Enum):
    OPEN = auto()
    OBSTACLE = auto()


class DriveState(Enum):
    LANE_FOLLOW = auto()
    AVOID_LEFT = auto()
    AVOID_RIGHT = auto()
    SEEKING_PARKING = auto()
    FINISHED = auto()


@dataclass
class Telemetry:
    tof_front_mm: int = -1
    gyro_z: float = 0.0
    timestamp: float = 0.0


class NanoLink:
    """Background serial link to the Arduino Nano: sends drive commands,
    parses telemetry lines as they arrive. Reconnects automatically if the
    link drops mid-run (e.g. a momentary USB brownout) instead of dying
    permanently and spamming errors forever."""

    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = self._connect(retries=NANO_CONNECT_RETRIES)
        if self.ser is None:
            raise RuntimeError(
                f"Could not open Nano link on {port} after {NANO_CONNECT_RETRIES} attempts"
            )
        self.telemetry = Telemetry()
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _connect(self, retries=1):
        for attempt in range(retries):
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.05)
                print(f"Nano link opened on {self.port} @ {self.baud} baud")
                time.sleep(2.0)  # let the Nano finish its reset after the port opens
                return ser
            except Exception as e:
                print(f"Nano port not ready (attempt {attempt + 1}/{retries}): {e}")
                if attempt < retries - 1:
                    time.sleep(NANO_CONNECT_RETRY_DELAY_S)
        return None

    def _read_loop(self):
        pattern = re.compile(
            r"^T,(-?\d+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),"
            r"(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)$"
        )
        while self.running:
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
            except Exception as e:
                print(f"Nano link error: {e} -- reconnecting in {NANO_RECONNECT_BACKOFF_S:.0f}s...")
                try:
                    self.ser.close()
                except Exception:
                    pass
                time.sleep(NANO_RECONNECT_BACKOFF_S)
                new_ser = self._connect(retries=1)
                if new_ser is not None:
                    self.ser = new_ser
                    print("Nano link restored.")
                continue
            if not line:
                continue
            if DEBUG_SERIAL:
                print(f"<- {line}")
            m = pattern.match(line)
            if not m:
                if DEBUG_SERIAL:
                    print("   (did not match expected telemetry format)")
                continue
            tof_front = int(m.group(1))
            gz = float(m.group(7))
            with self.lock:
                self.telemetry = Telemetry(tof_front, gz, time.time())

    def get_telemetry(self):
        with self.lock:
            return self.telemetry

    def send_command(self, speed, steer):
        speed = int(max(-255, min(255, speed)))
        steer = int(max(-STEER_LIMIT_DEG, min(STEER_LIMIT_DEG, steer)))
        cmd = f"M{speed},{steer}\n"
        if DEBUG_SERIAL:
            print(f"-> {cmd.strip()}")
        try:
            self.ser.write(cmd.encode("ascii"))
            self.ser.flush()  # force the write out now, don't rely on OS buffering timing
        except Exception as e:
            # The read loop already handles reconnection -- just drop this
            # one command rather than crashing the whole script over it.
            print(f"Nano write failed (link likely reconnecting): {e}")

    def stop(self):
        self.running = False
        try:
            self.send_command(0, 0)
        except Exception:
            pass
        time.sleep(0.05)
        try:
            self.ser.close()
        except Exception:
            pass


def parse_stl19p_packet(data):
    """Parses a single 47-byte packet from the LDROBOT STL-19P / LD19 frame.
    Lifted directly from the validated viewer script -- don't change this
    without retesting against the real sensor."""
    if len(data) != 47 or data[0] != 0x54 or data[1] != 0x2C:
        return []

    start_angle = struct.unpack("<H", data[4:6])[0] / 100.0
    end_angle = struct.unpack("<H", data[42:44])[0] / 100.0
    angle_diff = (end_angle - start_angle) % 360.0
    angle_step = angle_diff / 11.0 if angle_diff != 0 else 0.0

    points = []
    offset = 6
    for i in range(12):
        dist_mm, confidence = struct.unpack("<HB", data[offset:offset + 3])
        offset += 3
        if confidence >= LIDAR_MIN_CONFIDENCE and dist_mm > 0:
            angle = (start_angle + i * angle_step) % 360.0
            points.append((angle, dist_mm / 1000.0))
    return points


class LidarSource:
    """Background LDROBOT STL-19P (LD19) reader over raw serial. Exposes the
    latest scan as a list of (angle_deg, dist_m) tuples -- same interface as
    before. Reconnects automatically if the link drops mid-run instead of
    dying permanently."""

    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.points_lock = threading.Lock()
        self.points = []
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _connect(self, retries=1):
        for attempt in range(retries):
            try:
                ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
                print(f"Connected to lidar on {self.port} @ {self.baudrate} baud.")
                return ser
            except Exception as e:
                print(f"Lidar port not ready (attempt {attempt + 1}/{retries}): {e}")
                if attempt < retries - 1:
                    time.sleep(LIDAR_CONNECT_RETRY_DELAY_S)
        return None

    def _read_loop(self):
        ser = self._connect(retries=LIDAR_CONNECT_RETRIES)
        if ser is None:
            print("Failed to open lidar serial port after retries -- giving up.")
            return

        buffer = bytearray()
        accumulated = []
        while self.running:
            try:
                raw = ser.read(ser.in_waiting or 1)
                if raw:
                    buffer.extend(raw)
                while len(buffer) >= 47:
                    if buffer[0] == 0x54 and buffer[1] == 0x2C:
                        packet = buffer[:47]
                        del buffer[:47]
                        accumulated.extend(parse_stl19p_packet(packet))
                        if len(accumulated) >= 300:
                            with self.points_lock:
                                self.points = list(accumulated)
                            accumulated.clear()
                    else:
                        buffer.pop(0)
            except Exception as e:
                print(f"Lidar link error: {e} -- reconnecting in {LIDAR_RECONNECT_BACKOFF_S:.0f}s...")
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(LIDAR_RECONNECT_BACKOFF_S)
                new_ser = self._connect(retries=1)
                if new_ser is not None:
                    ser = new_ser
                    buffer.clear()
                    accumulated.clear()
                    print("Lidar link restored.")

        try:
            ser.close()
        except Exception:
            pass

    def get_points(self):
        with self.points_lock:
            return list(self.points)

    def stop(self):
        self.running = False


def sector_min_distance(points, center_deg, width_deg):
    """Return the minimum distance (m) within an angular sector, or None."""
    lo = (center_deg - width_deg / 2) % 360
    hi = (center_deg + width_deg / 2) % 360
    vals = []
    for angle, dist in points:
        if dist <= 0:
            continue
        in_range = (lo <= hi and lo <= angle <= hi) or (lo > hi and (angle >= lo or angle <= hi))
        if in_range:
            vals.append(dist)
    return min(vals) if vals else None


def wall_follow_steer(points):
    """Proportional wall-follow: steer to balance distance from the left
    and right walls, aiming for TARGET_WALL_DIST_M if only one is visible."""
    left = sector_min_distance(points, 270, 40)    # ~90 deg left of forward
    right = sector_min_distance(points, 90, 40)     # ~90 deg right of forward
    front = sector_min_distance(points, 0, 30)

    if left is None and right is None:
        return 0, front  # no wall data this frame; drive straight and hope

    if left is not None and right is not None:
        error = right - left  # positive => more room on the right => steer right
    elif right is not None:
        error = TARGET_WALL_DIST_M - right  # hugging the right wall too closely
    else:
        error = left - TARGET_WALL_DIST_M  # hugging the left wall too closely

    steer = WALL_FOLLOW_KP * error / 100.0  # empirically scaled; tune on the mat
    return steer, front


def detect_pillars(frame_bgr):
    """Return list of (color, center_x_norm, area) for pillars found in frame.
    center_x_norm is -1..1, left to right, 0 = image center."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    hsv[: h // 2, :] = 0  # ignore ceiling/background clutter

    results = []
    for color_name, ranges in HSV_RANGES.items():
        mask = np.zeros((h, w), dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < PILLAR_AREA_THRESHOLD:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            center_x = x + bw / 2
            center_x_norm = (center_x - w / 2) / (w / 2)
            results.append((color_name, center_x_norm, area))
    return results


def largest_pillar(pillars):
    return max(pillars, key=lambda p: p[2]) if pillars else None


def detect_parking_markers(frame_bgr):
    """Return list of (center_x_norm, area) for purple parking-box boundary
    markers. Uses PARKING_HSV_RANGES, not HSV_RANGES -- kept separate on
    purpose so these can never be mistaken for a red/green pillar to avoid."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    hsv[: h // 2, :] = 0  # ignore ceiling/background clutter, same as pillars

    mask = np.zeros((h, w), dtype=np.uint8)
    for lower, upper in PARKING_HSV_RANGES:
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < PARKING_AREA_THRESHOLD:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        center_x = x + bw / 2
        center_x_norm = (center_x - w / 2) / (w / 2)
        results.append((center_x_norm, area))
    return results


def parking_align_steer(markers):
    """Steer to center between the two parking markers. With only one
    visible, steer toward it as a first approximation until the second
    comes into frame. Returns None if nothing is detected at all, so the
    caller can fall back to normal lane-following until markers appear."""
    if not markers:
        return None
    if len(markers) >= 2:
        # Two largest, most likely the real boundary pair rather than noise.
        top2 = sorted(markers, key=lambda m: m[1], reverse=True)[:2]
        mid = (top2[0][0] + top2[1][0]) / 2.0
    else:
        mid = markers[0][0]
    steer = mid * PARK_ALIGN_STEER_LIMIT
    return max(-PARK_ALIGN_STEER_LIMIT, min(PARK_ALIGN_STEER_LIMIT, steer))


def draw_lidar_view(points):
    """Render the latest lidar scan as a top-down radar plot, same as the
    standalone viewer -- used for the live preview while driving."""
    canvas = np.zeros((LIDAR_VIEW_SIZE, LIDAR_VIEW_SIZE, 3), dtype=np.uint8)
    center = LIDAR_VIEW_SIZE // 2
    scale = center / LIDAR_MAX_RANGE_M

    for r in np.arange(0.5, LIDAR_MAX_RANGE_M + 0.01, 0.5):
        cv2.circle(canvas, (center, center), int(r * scale), (40, 40, 40), 1)

    cv2.drawMarker(
        canvas, (center, center), (0, 255, 255),
        markerType=cv2.MARKER_TRIANGLE_UP, markerSize=10, thickness=2,
    )

    for angle_deg, dist_m in points:
        if dist_m <= 0 or dist_m > LIDAR_MAX_RANGE_M:
            continue
        rad = np.deg2rad(angle_deg)
        x = center + int(dist_m * scale * np.sin(rad))
        y = center - int(dist_m * scale * np.cos(rad))
        if 0 <= x < LIDAR_VIEW_SIZE and 0 <= y < LIDAR_VIEW_SIZE:
            cv2.circle(canvas, (x, y), 2, (0, 200, 255), -1)

    cv2.putText(
        canvas, f"{len(points)} pts", (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
    )
    return canvas


def annotate_pillars(frame_bgr):
    """Draw color-labeled boxes on a copy of the frame, for the live preview
    only -- kept separate from detect_pillars() so the display path can't
    accidentally affect driving decisions."""
    annotated = frame_bgr.copy()
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h = hsv.shape[0]
    hsv[: h // 2, :] = 0

    for color_name, ranges in HSV_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        draw_color = (0, 0, 255) if color_name == "red" else (0, 255, 0)
        for c in contours:
            if cv2.contourArea(c) < PILLAR_AREA_THRESHOLD:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), draw_color, 2)
            cv2.putText(annotated, color_name, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 1)

    # Parking markers -- separate pass/threshold/color, same reasoning as
    # detect_parking_markers(): kept out of the HSV_RANGES loop above.
    park_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in PARKING_HSV_RANGES:
        park_mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
    park_contours, _ = cv2.findContours(park_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in park_contours:
        if cv2.contourArea(c) < PARKING_AREA_THRESHOLD:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (255, 0, 255), 2)
        cv2.putText(annotated, "park", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    return annotated


def run(mode):
    """mode: a Mode enum value to force it (bench testing), or None to wait
    for the hardware mode switch to decide at runtime (competition use)."""
    nano = NanoLink(NANO_PORT, NANO_BAUD)
    lidar = LidarSource(LIDAR_PORT, LIDAR_BAUDRATE)

    picam2 = None
    for attempt in range(CAMERA_INIT_RETRIES):
        try:
            picam2 = Picamera2()
            break
        except Exception as e:
            print(f"Camera not ready (attempt {attempt + 1}/{CAMERA_INIT_RETRIES}): {e}")
            if attempt < CAMERA_INIT_RETRIES - 1:
                time.sleep(CAMERA_INIT_RETRY_DELAY_S)
    if picam2 is None:
        raise RuntimeError(
            f"Camera never became available after {CAMERA_INIT_RETRIES} attempts -- "
            "check the USB connection."
        )
    picam2.configure(picam2.create_preview_configuration(
        main={"format": "RGB888", "size": CAMERA_SIZE}
    ))
    picam2.start()

    preview_active = True
    preview_window_name = "Camera (left) + Lidar (right) -- close window or press 'q' to hide preview"

    try:
        if mode is None:
            open_switch = Button(OPEN_MODE_PIN, pull_up=True, bounce_time=0.05)
            obstacle_switch = Button(OBSTACLE_MODE_PIN, pull_up=True, bounce_time=0.05)

            print("Waiting for mode switch (one side = open, other side = obstacle)...")
            while not open_switch.is_active and not obstacle_switch.is_active:
                # Hold the robot explicitly still and keep the preview live
                # so you can confirm camera/lidar/telemetry are all healthy
                # before committing to a run.
                nano.send_command(0, 0)
                raw_frame = picam2.capture_array()
                frame = cv2.cvtColor(raw_frame, cv2.COLOR_RGB2BGR)
                points = lidar.get_points()

                annotated = annotate_pillars(frame)
                lidar_view = draw_lidar_view(points)
                h_cam = annotated.shape[0]
                lidar_resized = cv2.resize(lidar_view, (h_cam, h_cam))
                combined = np.hstack([annotated, lidar_resized])
                cv2.putText(combined, "WAITING FOR MODE SWITCH", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow(preview_window_name, combined)
                cv2.waitKey(1)
                time.sleep(0.05)

            mode = Mode.OPEN if open_switch.is_active else Mode.OBSTACLE
            print(f"Mode switch thrown -- starting {mode.name} mode. Go!")
        else:
            # --mode was passed explicitly (bench testing) -- start right
            # away rather than waiting on hardware that may not be wired up.
            print(f"Mode forced to {mode.name} via --mode; skipping switch wait.")

        state = DriveState.LANE_FOLLOW
        heading_deg = 0.0
        lap_count = 0
        last_t = time.time()
        park_seek_deadline = None
        last_heartbeat_check = time.time()  # not 0 -- avoids a false "no telemetry" warning in the first instant

        print(f"Starting in {mode.name} mode. Ctrl+C to stop.")
        while state != DriveState.FINISHED:
            now = time.time()
            dt = now - last_t
            last_t = now

            points = lidar.get_points()
            steer, front_dist = wall_follow_steer(points)
            speed = BASE_SPEED

            # Lap counting: integrate yaw rate (deg/s) from the Nano's gyro.
            telem = nano.get_telemetry()
            heading_deg += telem.gyro_z * dt

            # Heartbeat: make a dead Nano link loud and obvious instead of
            # silently doing nothing forever. Throttled to once a second.
            if now - last_heartbeat_check > 1.0:
                last_heartbeat_check = now
                if telem.timestamp == 0.0:
                    print("WARNING: no telemetry ever received from the Nano -- "
                          "check NANO_PORT and the physical connection.")
                elif now - telem.timestamp > 1.0:
                    print(f"WARNING: no telemetry from the Nano in "
                          f"{now - telem.timestamp:.1f}s -- link may have dropped.")

            if abs(heading_deg) >= DEG_PER_LAP:
                heading_deg -= DEG_PER_LAP if heading_deg > 0 else -DEG_PER_LAP
                lap_count += 1
                print(f"Lap {lap_count} complete")
                if lap_count >= LAP_COUNT_TARGET:
                    if mode == Mode.OBSTACLE:
                        state = DriveState.SEEKING_PARKING
                        park_seek_deadline = time.time() + PARK_SEEK_TIMEOUT_S
                        print("Laps complete -- seeking parking box")
                    else:
                        state = DriveState.FINISHED
                        break

            raw_frame = picam2.capture_array()
            frame = cv2.cvtColor(raw_frame, cv2.COLOR_RGB2BGR)

            if state == DriveState.SEEKING_PARKING:
                markers = detect_parking_markers(frame)
                park_steer = parking_align_steer(markers)
                if park_steer is not None:
                    steer = park_steer
                # Slow to creep speed the moment laps are done, even before
                # markers are visible yet -- better to approach the final
                # stretch cautiously than arrive at full BASE_SPEED.
                speed = PARK_APPROACH_SPEED

                if 0 < telem.tof_front_mm < PARK_STOP_MM:
                    speed = 0
                    state = DriveState.FINISHED
                    print("Parked.")
                elif park_seek_deadline is not None and time.time() > park_seek_deadline:
                    speed = 0
                    state = DriveState.FINISHED
                    print("WARNING: parking timeout -- stopped without confirming parked position.")

            elif mode == Mode.OBSTACLE:
                pillars = detect_pillars(frame)
                target = largest_pillar(pillars)

                if target is not None:
                    color, center_x_norm, area = target
                    # Placeholder area -> distance heuristic. Replace with
                    # real camera+lidar fusion once calibrated.
                    approx_dist = max(0.15, 1.2 - area / 20000.0)
                    if approx_dist < PILLAR_TRIGGER_DIST_M:
                        new_state = DriveState.AVOID_RIGHT if color == "red" else DriveState.AVOID_LEFT
                        if new_state != state:
                            print(f"Pillar detected: {color} at ~{approx_dist:.2f}m -> {new_state.name}")
                        state = new_state
                    else:
                        state = DriveState.LANE_FOLLOW
                else:
                    state = DriveState.LANE_FOLLOW

            if state == DriveState.AVOID_RIGHT:
                steer = PILLAR_STEER_DEG
                speed = TURN_SPEED
            elif state == DriveState.AVOID_LEFT:
                steer = -PILLAR_STEER_DEG
                speed = TURN_SPEED
            elif state == DriveState.SEEKING_PARKING:
                pass  # steer/speed already set above -- don't let the reflex below touch it
            elif front_dist is not None and front_dist < FRONT_STOP_DIST_M:
                # Crude reflex, not real corner detection -- see module docstring.
                speed = TURN_SPEED
                steer = STEER_LIMIT_DEG if steer >= 0 else -STEER_LIMIT_DEG

            # Independent safety backstop: this fires regardless of what the
            # lidar/camera logic above decided, on purpose -- it's a last
            # line of defense if lidar processing hiccups for a frame.
            if 0 < telem.tof_front_mm < TOF_EMERGENCY_MM:
                speed = 0
                print(f"ToF emergency stop: {telem.tof_front_mm}mm ahead")

            nano.send_command(speed, steer)

            # Live preview -- purely for your eyes, has no effect on driving.
            # Closing the window (X button or 'q') only hides the preview;
            # it does NOT stop the robot. Ctrl+C in the terminal is now the
            # only way to stop the robot itself.
            if preview_active:
                annotated = annotate_pillars(frame)
                lidar_view = draw_lidar_view(points)
                h_cam = annotated.shape[0]
                lidar_resized = cv2.resize(lidar_view, (h_cam, h_cam))
                combined = np.hstack([annotated, lidar_resized])
                cv2.imshow(preview_window_name, combined)
                key = cv2.waitKey(1) & 0xFF
                window_closed = cv2.getWindowProperty(preview_window_name, cv2.WND_PROP_VISIBLE) < 1
                if key == ord("q") or window_closed:
                    print("Preview closed -- robot keeps driving. Ctrl+C in the terminal to stop it.")
                    cv2.destroyWindow(preview_window_name)
                    preview_active = False

            time.sleep(0.02)  # ~50 Hz control loop

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        nano.stop()
        lidar.stop()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode", choices=["open", "obstacle"], default=None,
        help="Force a mode for bench testing, skipping the hardware mode "
             "switch wait and starting immediately. Omit this for "
             "competition use -- the physical mode switch decides at runtime."
    )
    args = parser.parse_args()
    forced_mode = None
    if args.mode == "open":
        forced_mode = Mode.OPEN
    elif args.mode == "obstacle":
        forced_mode = Mode.OBSTACLE
    run(forced_mode)
