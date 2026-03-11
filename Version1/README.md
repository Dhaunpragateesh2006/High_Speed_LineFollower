# Version 1 – PID Controlled Line Follower with Bluetooth Tuning

## Overview

This version implements the initial closed-loop control architecture for the line following robot. The robot uses an infrared reflectance sensor array to detect the track and applies a PID-based control algorithm to maintain alignment with the line.

A Bluetooth interface is integrated using the HC-05 module, allowing real-time tuning of control parameters such as proportional, integral, and derivative gains.

This version focuses on establishing a stable feedback control system and providing runtime parameter adjustment for rapid tuning.

---

## Key Features

• PID-based trajectory correction
• Real-time PID tuning through Bluetooth communication
• Differential drive motor control using TB6612FNG driver
• Acceleration ramping to prevent sudden motor jerks
• Automatic sensor calibration routine
• Sharp turn detection using outer sensor thresholds

---

## Control Strategy

The robot computes the line position using the QTR sensor array and calculates the positional error relative to the center of the robot.

The PID controller generates a corrective control signal:

control = Kp × P + Ki × I + Kd × D

The control output adjusts the differential motor speeds, enabling the robot to steer toward the line.

---

## Bluetooth Control Commands

| Command | Function                        |
| ------- | ------------------------------- |
| i / d   | Increase / decrease Kp          |
| j / e   | Increase / decrease Ki          |
| k / f   | Increase / decrease Kd          |
| l / g   | Increase / decrease robot speed |
| S       | Start robot                     |
| s       | Stop robot                      |

These commands allow live tuning of control parameters without reprogramming the microcontroller.

---

## Limitations

• Limited handling of broken lines
• No intersection navigation logic
• Oscillations may occur at higher speeds

These limitations motivated the improvements implemented in Version 2.
