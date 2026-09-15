# WRO 2026 — Future Engineers — Self-Driving Car

<!-- TODO: Replace with your team photo -->
<!-- <img src="./t-photos/team_photo.jpg" alt="Team Photo" width="600"> -->

**Team Name:** Cyper X

**Country:** Saudi Arabia

## Team Members

- _ Fahad Saad
- _ Khalid Aref
- **Coach:** _ Mr.Khairey Nouhe

This repository documents our full engineering process for the WRO 2026 Future Engineers challenge: design decisions, components, wiring, power management, obstacle strategy, and the source code of our autonomous vehicle.

<a name="top"></a>

## Table of Contents

- [1. Overview](#1-overview)
  - [1.1 About the Project](#11-about-the-project)
  - [1.2 Robot Images](#12-robot-images)
  - [1.3 Performance Video](#13-performance-video)
- [2. Mobility Management](#2-mobility-management)
  - [2.1 Drive System](#21-drive-system)
  - [2.2 Steering](#22-steering)
  - [2.3 Chassis Design](#23-chassis-design)
- [3. Power and Sense Management](#3-power-and-sense-management)
  - [3.1 Power Source](#31-power-source)
  - [3.2 Sensors](#32-sensors)
  - [3.3 Processing Unit](#33-processing-unit)
  - [3.4 Wiring Diagram](#34-wiring-diagram)
- [4. Obstacle Management](#4-obstacle-management)
  - [4.1 Open Challenge](#41-open-challenge)
  - [4.2 Obstacle Challenge](#42-obstacle-challenge)
  - [4.3 Parallel Parking](#43-parallel-parking)
- [5. Source Code](#5-source-code)
- [6. List of Components](#6-list-of-components)
- [7. Repository Structure](#7-repository-structure)

## 1. Overview

### 1.1 About the Project

This project focuses on designing, building, and programming an autonomous vehicle capable of completing the WRO Future Engineers challenges: the Open Challenge (laps around the field), the Obstacle Challenge (navigating around red and green traffic signs), and parallel parking.

Our vehicle is built around a **Raspberry Pi 5 (4 GB)** as the main computer. It combines a **D500 360° LiDAR** for wall detection and localization, an **Innomaker USB camera** for traffic-sign color detection, **VL53L0X time-of-flight sensors** for close-range distance measurement, and an **MPU-9250 IMU** for heading control. A **TT DC gear motor** driven by an **L298N motor driver** provides propulsion, while an **MG90S servo** steers the front wheels.

_TODO: Add a short paragraph about your team's goals, design philosophy, and development process (brainstorming → prototyping → testing → iterating)._

### 1.2 Robot Images

_TODO: Add 6 photos of the vehicle (front, back, left, right, top, bottom) to the `v-photos` folder and link them here._

| Front | Back | Left |
| --- | --- | --- |
| _photo_ | _photo_ | _photo_ |

| Right | Top | Bottom |
| --- | --- | --- |
| _photo_ | _photo_ | _photo_ |

### 1.3 Performance Video

_TODO: Add a link to your YouTube performance video (at least 30 seconds of driving). Also record it in [`video/video.md`](./video/video.md)._

<p align="right"><a href="#top">Back To Top</a></p>

## 2. Mobility Management

### 2.1 Drive System

**Motor: TT DC Gear Motor**

Specifications (typical):

- Operating voltage: 3–6 V
- Gear ratio: 1:48
- No-load speed: ~200 RPM at 6 V
- Shaft: double-sided, compatible with standard TT wheels

**Motor Driver: L298N**

- Dual H-bridge, controls direction and speed via PWM from the Raspberry Pi
- Operating voltage: 5–35 V, up to 2 A per channel
- Built-in 5 V regulator

**Reason for selection:**

- _TODO: Explain why you chose the TT motor (e.g., lightweight, cheap, easy to mount, sufficient torque for the flat WRO field)._
- _TODO: Explain the drive configuration (rear-wheel drive? differential gear? direct drive on one axle?)._

**Mounting:**

_TODO: Describe how the motor is mounted to the chassis and how the wheels are attached._

**Potential improvements:**

_TODO: e.g., upgrade to a motor with an encoder for closed-loop speed control, or replace the L298N (which has a high voltage drop) with a more efficient TB6612FNG driver._

### 2.2 Steering

**Servo: MG90S**

Specifications:

- Torque: 2.2 kg·cm (4.8 V)
- Speed: 0.1 s / 60°
- Metal gears
- Operating voltage: 4.8–6 V

**Reason for selection:**

- Compact and lightweight with metal gears for durability
- Enough torque to steer the front wheels responsively
- Simple PWM control from the Raspberry Pi

**Steering geometry:**

_TODO: Describe your steering mechanism (simple pivot? Ackermann geometry?) and how you calibrated the center position and maximum steering angles._

**Mounting:**

_TODO: Describe how the servo connects to the steering linkage._

### 2.3 Chassis Design

_TODO: Describe your chassis — material (3D printed? acrylic? kit?), dimensions (width × length × height), component layout, and why you arranged it that way. Add photos or CAD renders. If you have 3D-printed parts, put the model files in the `models` folder._

<p align="right"><a href="#top">Back To Top</a></p>

## 3. Power and Sense Management

### 3.1 Power Source

**Battery: 18650 Lithium-ion cells**

- Nominal voltage: 3.7 V per cell
- Configuration: _TODO: e.g., 2S (7.4 V) or 3S (11.1 V)_
- Capacity: _TODO: mAh_

**Voltage Regulation: LM2596 DC-DC Step-Down Module**

The LM2596 buck converter steps the battery voltage down to a stable **5 V** for the Raspberry Pi 5 and the sensors, while the L298N receives battery voltage directly for the drive motor.

_TODO: Describe your exact power distribution — which rail feeds the Pi, the servo, the LiDAR, and the motor driver — and any switches or protection you added. Note: the Raspberry Pi 5 prefers 5 V / 5 A; verify your LM2596 can supply enough current under load._

### 3.2 Sensors

| Sensor | Purpose |
| --- | --- |
| **D500 LiDAR (360°)** | Wall detection, distance to inner/outer walls, corner detection, localization on the field |
| **Innomaker USB Camera** | Detecting red/green traffic signs and their position for the Obstacle Challenge |
| **GY-530 VL53L0X (ToF)** | Close-range distance measurement (e.g., parking, wall proximity) |
| **TCA9548A I2C Multiplexer** | Connecting multiple VL53L0X sensors (which share a fixed I2C address) to one I2C bus |
| **MPU-9250 IMU** | Gyro/accelerometer/magnetometer for heading control and precise turns |

_TODO: For each sensor, describe where it is mounted and how it is used in your algorithm._

### 3.3 Processing Unit

**Raspberry Pi 5 (4 GB)**

The Raspberry Pi 5 runs the full software stack: reading the LiDAR and camera, sensor fusion with the IMU and ToF sensors, decision-making, and driving the motor driver and servo.

_TODO: Mention your OS, programming language(s), and key libraries (e.g., OpenCV for vision)._

### 3.4 Wiring Diagram

_TODO: Add a wiring/schematic diagram (e.g., drawn in Fritzing) to the `schemes` folder and link it here._

<p align="right"><a href="#top">Back To Top</a></p>

## 4. Obstacle Management

### 4.1 Open Challenge

_TODO: Describe your strategy for driving 3 laps: how you use the LiDAR to keep distance from the walls, how you detect corners, how the IMU keeps the heading straight, and how you count laps and stop in the starting section._

### 4.2 Obstacle Challenge

_TODO: Describe how the camera detects red and green traffic signs (color thresholding? HSV masks?), how the robot decides to pass on the right (red) or left (green), and how vision is combined with LiDAR/ToF data to plan the path._

### 4.3 Parallel Parking

_TODO: Describe how the robot finds the parking area and the maneuver sequence it performs to park, and which sensors (ToF, LiDAR, IMU) control each phase._

<p align="right"><a href="#top">Back To Top</a></p>

## 5. Source Code

All source code lives in the [`src`](./src) folder.

_TODO: Describe your code structure (modules for LiDAR, camera, motor control, main loop), how to install dependencies, and how to run the program on the Raspberry Pi (including how it starts automatically at competition)._

## 6. List of Components

| # | Component | Qty | Purpose |
| --- | --- | --- | --- |
| 1 | Raspberry Pi 5 (4 GB) | 1 | Main processing unit |
| 2 | D500 LiDAR | 1 | 360° distance sensing / navigation |
| 3 | Innomaker USB Camera | 1 | Traffic sign detection |
| 4 | TT DC Gear Motor | _TODO_ | Drive |
| 5 | L298N Motor Driver | 1 | Motor control |
| 6 | GY-530 VL53L0X ToF Sensor | _TODO_ | Close-range distance |
| 7 | TCA9548A I2C Multiplexer | 1 | Multiple I2C sensors on one bus |
| 8 | MG90S Servo | 1 | Steering |
| 9 | MPU-9250 IMU | 1 | Heading / orientation |
| 10 | 18650 Lithium Battery | _TODO_ | Power source |
| 11 | LM2596 Step-Down Module | 1 | 5 V regulation |
| 12 | Screws (M2 / M3 / M4) | — | Assembly |
| 13 | Wires | — | Connections |

## 7. Repository Structure

| Folder | Contents |
| --- | --- |
| [`t-photos`](./t-photos) | Team photos (one official, one funny) |
| [`v-photos`](./v-photos) | Vehicle photos from all 6 sides |
| [`video`](./video) | `video.md` with the link to the driving demonstration video |
| [`schemes`](./schemes) | Wiring diagrams and electromechanical schematics |
| [`src`](./src) | Source code of the vehicle software |
| [`models`](./models) | 3D print / CAD files for custom parts |
| [`other`](./other) | Any other documentation (build instructions, datasets, etc.) |

<p align="right"><a href="#top">Back To Top</a></p>
