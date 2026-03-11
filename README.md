
# High-Speed Autonomous Line Following Robot

## Overview

This project implements a high-speed autonomous line-following robot capable of navigating complex tracks with sharp turns, intersections, and discontinuities. The system utilizes an infrared reflectance sensor array for real-time line detection and employs a closed-loop PID controller for dynamic trajectory correction.

The robot was developed through multiple design iterations to improve navigation stability, sensor interpretation, and decision-making at intersections.

The platform is built on an embedded control architecture using an Arduino-based microcontroller and a TB6612FNG dual H-bridge motor driver for differential drive actuation.

---

# System Architecture

The robot consists of the following major subsystems:

• **Sensor System** – Infrared reflectance sensor array for line detection
• **Control System** – PID-based closed-loop control algorithm
• **Actuation System** – Differential drive using N20 DC gear motors
• **Motor Driver** – TB6612FNG dual H-bridge driver
• **Navigation Logic** – Intersection detection, turn handling, and recovery algorithms

---

# Hardware Components

| Component       | Description                                      |
| --------------- | ------------------------------------------------ |
| Microcontroller | Arduino Nano                                     |
| Sensor Array    | QTR Infrared Reflectance Sensors                 |
| Motor Driver    | TB6612FNG Dual H-Bridge                          |
| Motors          | N20 Gear Motors                                  |
| Power           | Lithium-polymer battery pack                     |
| Communication   | HC-05 Bluetooth module (Version 1)               |
| Configuration   | DIP switches for navigation priority (Version 3) |

---

# Control Algorithm

The robot operates using a closed-loop feedback control system.

1. The infrared sensor array measures reflectance values across the track.
2. The line position is computed using weighted sensor readings.
3. A positional error relative to the robot center is calculated.
4. A PID controller generates corrective motor speed commands.
5. Differential PWM signals are applied to the motor driver.

This enables continuous trajectory correction and stable line tracking at high speeds.

---

---

# Key Control Features

• Closed-loop PID trajectory control
• Integral windup prevention
• Differential drive motor control
• Ramp-based acceleration control
• Intersection detection and handling
• Dashed-line navigation recovery
• Hardware-configurable navigation priority

---

# Results

The robot successfully navigates tracks containing:

• Sharp 90° turns
• T-junction intersections
• Dashed or broken lines
• Dead-end segments

The final iteration demonstrates stable trajectory tracking and reliable recovery from line-loss conditions.

---





Dhanu Pragateesh
Embedded Systems • Robotics • Autonomous Systems
