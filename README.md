# Autonomous Mobile Robot — Full ROS2 Stack

> *From Arduino encoder interrupts to Nav2 smooth waypoint navigation — one complete system.*

A production-grade AMR control stack built from scratch. Arduino Mega handles real hardware — motors, encoders, IMU — while ROS2 bridges to Nav2 for autonomous navigation. No shortcuts, no simulation-only code. Validated on a real robot.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        FULL AMR STACK                           │
│                                                                 │
│  RViz Click  ──►  Waypoint Navigator  ──►  Nav2 BasicNavigator │
│                          │                        │             │
│                    Velocity Profile          goToPose()         │
│                   (accel/cruise/decel)            │             │
│                                                   ▼             │
│                                          Base Controller Node   │
│                                          (Python ROS2)          │
│                                               │    │            │
│                                        /odom  │    │ /cmd_vel   │
│                                        /imu   │    │            │
│                                               ▼    ▼            │
│                                     ┌──────────────────┐        │
│                                     │  Serial Bridge   │        │
│                                     │  115200 baud     │        │
│                                     └────────┬─────────┘        │
│                                              │                  │
│                                              ▼                  │
│                                   ┌──────────────────┐         │
│                                   │  Arduino Mega    │         │
│                                   │  2560            │         │
│                                   │                  │         │
│                                   │  Encoders (×2)   │         │
│                                   │  MPU6050 IMU     │         │
│                                   │  Motor PWM/DIR   │         │
│                                   │  EKF Odometry    │         │
│                                   └──────────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

---

## Hardware Stack

| Component | Spec | Role |
|---|---|---|
| **Arduino Mega 2560** | ATmega2560, 16MHz | Real-time motor + encoder control |
| **MPU6050 IMU** | 6-axis, I2C | Gyro-fused odometry correction |
| **Dual DC Motors** | PWM + DIR control | Differential drive |
| **Quadrature Encoders** | Left: 2320 PPR / Right: 2350 PPR | Wheel odometry |
| **Wheel Diameter** | 127mm | Distance calibration |
| **Wheel Base** | 460mm | Rotation calibration |

---

## Layer 1 — Arduino Base Controller

The firmware handles everything time-critical at 20Hz. ROS2 cannot guarantee real-time timing — so all motor control, encoder counting, and odometry computation lives here.

### Encoder ISRs
```cpp
void leftEncoderISR() {
  if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}
```
Interrupt-driven quadrature decoding on pins 2, 3, 18, 19 (hardware interrupt capable). Direction determined by phase relationship between A and B channels.

### Gyro-Fused Odometry
Standard wheel odometry drifts on uneven surfaces. The firmware fuses encoder-derived rotation with MPU6050 gyro at 50/50 weight:
```cpp
if (imu_ok) {
    dTheta = 0.5 * dTheta + 0.5 * gyro_z_rad * dt;
}
```
This is the same principle as EKF sensor fusion — two independent rotation estimates blended by confidence weight.

### Dead-Band Compensation
Motors have a physical dead-band where low PWM produces no motion. The firmware enforces a minimum PWM floor:
```cpp
if (pwm_left > 0 && pwm_left < MIN_PWM) pwm_left = MIN_PWM;
```
Without this, velocity commands near zero produce no motion but the odometry still integrates — causing position drift.

### Serial Protocol
```
CMD_VEL:linear,angular    → Set velocity
STOP                      → Emergency stop
RESET_ODOM                → Zero position
GET_ENCODERS              → Raw tick counts
GET_CALIBRATION           → Print correction factors

ODOM:x,y,theta,v_lin,v_ang,gyro_z,enc_l,enc_r  ← Odometry output
```
Float values serialized with `dtostrf()` — Arduino's `print(float)` has known precision bugs. Fixed explicitly.

### Safety Timeout
If no `CMD_VEL` received for 2 seconds — motors stop automatically. Prevents runaway if ROS2 node crashes.

---

## Layer 2 — ROS2 Base Controller Node

Python node bridges serial to ROS2 topics. Runs two independent timers:

| Timer | Rate | Job |
|---|---|---|
| `cmd_loop` | 10 Hz | Forward `/cmd_vel` to Arduino serial |
| `tf_loop` | 50 Hz | Broadcast `odom → base_link` TF transform |

**Why separate timers?** TF must publish at high rate for Nav2 to function correctly. If cmd and TF share one loop, a serial write stall blocks TF — Nav2 loses the transform and aborts navigation. Decoupled timers prevent this.

### Thread Safety
Serial read runs in a daemon thread. All shared state (`x`, `y`, `theta`, `target_v`, `target_w`) protected by `threading.Lock()`:
```python
with self.lock:
    self.x = float(parts[0])
    self.y = float(parts[1])
    self.theta = float(parts[2])
```

### Published Topics
```
/odom          nav_msgs/Odometry      — Full pose + velocity + covariance
/imu/data      sensor_msgs/Imu        — Angular velocity + linear acceleration
/tf            odom → base_link       — 50Hz transform broadcast
```

### Odometry Covariance
Covariance values set to 0.01 — tuned for Nav2's cost function. Too high: Nav2 ignores odometry. Too low: Nav2 trusts odometry over sensor corrections. 0.01 is the working value for this hardware.

---

## Layer 3 — Waypoint Navigation Node

RViz-driven multi-waypoint navigation with smooth velocity profiles. Click points in RViz → robot follows them with human-like acceleration and deceleration.

### Velocity Profile
Each path segment gets a trapezoidal velocity profile:

```
Velocity
   │      ┌───────────────┐
   │     /│               │\
   │    / │    cruise     │ \
   │   /  │               │  \
   │  /   │               │   \
   │ /    │               │    \
   │/     │               │     \
   └──────┴───────────────┴──────── Distance
   accel                      decel
```

Smooth ease-in (quadratic acceleration) and ease-out (quadratic deceleration). Robot never jerks to full speed — starts at `min_vel=0.05 m/s`, ramps to `max_vel=0.5 m/s`, decelerates to stop.

### Segment-Based Navigation
Rather than sending all waypoints at once (Nav2 default — produces sharp corners), the navigator sends the path in segments with early completion:
```python
if end_idx < total_poses and distance < tolerance:
    self.navigator.cancelTask()
    break  # Smooth handoff to next segment
```
This eliminates the stop-and-turn behaviour at each waypoint — the robot follows a continuous smooth path.

### 2D Pose Stop
At each waypoint, the robot:
1. Completes the approach
2. Sends 10× stop commands (guarantees zero velocity)
3. Re-navigates to exact waypoint pose for precision alignment
4. Sends 5× final stop commands

This gives Nav2 time to correct any accumulated drift before continuing.

### Line Deviation Monitoring
Real-time shortest-distance calculation from robot position to each path segment:
```python
t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
t = max(0, min(1, t))  # Clamp to segment
closest_x = x1 + t * dx
closest_y = y1 + t * dy
```
Standard perpendicular distance formula — robot knows exactly how far it has drifted from the intended line.

---

## Key Engineering Decisions

**Why Arduino for motor control, not Raspberry Pi?**
Real-time encoder counting requires microsecond-level interrupt response. Linux (even with RT patches) cannot guarantee this. Arduino's bare-metal ISRs never miss an encoder pulse — critical for accurate odometry at high speeds.

**Why 50Hz TF, 10Hz CMD?**
Nav2's transform lookup timeout is ~100ms. TF at 50Hz gives 20ms between broadcasts — 5× margin. CMD at 10Hz is sufficient for smooth motion since the Arduino applies its own 20Hz control loop.

**Why gyro fusion at 50/50 weight?**
Wheel odometry is accurate on flat surfaces but drifts on turns due to wheel slip. Gyro is accurate for rotation but drifts over time due to bias. Equal weight balances both failure modes without a full EKF implementation.

**Why dtostrf() instead of Serial.print(float)?**
Arduino's built-in `Serial.print(float)` silently rounds to 2 decimal places in some firmware versions. `dtostrf()` gives explicit control over precision — critical when ROS2 is parsing position values.

---

## Usage

### Hardware Setup
```bash
# Flash Arduino firmware
# Connect via USB → /dev/ttyACM0

# Install Python dependencies
pip install pyserial

# Install ROS2 dependencies
sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations
```

### Launch
```bash
# Terminal 1: Base controller
ros2 run amr_robot base_controller --ros-args -p serial_port:=/dev/ttyACM0

# Terminal 2: Nav2 stack
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3: Waypoint navigator
ros2 run amr_robot waypoint_nav
```

### RViz Workflow
1. Open RViz → set fixed frame to `map`
2. Use **Publish Point** tool to click waypoints on the map
3. Press **ENTER** in Terminal 3 terminal
4. Robot follows waypoints with smooth velocity

---

## Repository Structure
```
amr_robot/
├── firmware/
│   └── base_controller.ino    # Arduino Mega firmware
├── amr_robot/
│   ├── base_controller.py     # ROS2 serial bridge node
│   └── waypoint_nav.py        # ROS2 Nav2 waypoint navigator
├── README.md
└── package.xml
```

---

## Author

**Praveen S** — Mechatronics Engineering, Bannari Amman Institute of Technology  
Part of a broader autonomous systems stack including computer vision tracking and drone control.  
[LinkedIn](https://linkedin.com/in/praveen-sb12b13293) · [GitHub](https://github.com/PRAVEENS-2005)

---

*Built to run on real hardware. Every design decision was made because the robot told us it was wrong.*
