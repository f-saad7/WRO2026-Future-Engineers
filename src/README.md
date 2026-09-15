# Source Code

All vehicle control code goes here.

Suggested structure:

```
src/
  main.py            # entry point / main control loop
  lidar.py           # D500 LiDAR reading and processing
  camera.py          # traffic sign detection (OpenCV)
  motors.py          # L298N motor + MG90S servo control
  sensors.py         # VL53L0X (via TCA9548A) + MPU-9250
  open_challenge.py  # Open Challenge strategy
  obstacle.py        # Obstacle Challenge strategy
  parking.py         # parallel parking routine
```

_TODO: Document how to install dependencies and run the program on the Raspberry Pi._
