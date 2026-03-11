# Version 3 – Priority-Based Intersection Navigation

## Overview

Version 3 introduces advanced navigation capabilities, enabling the robot to handle intersections, T-junctions, and dead-end paths.

A configurable priority system is implemented using hardware DIP switches, allowing the robot to dynamically select navigation behavior during intersections.

This version represents the most complete iteration of the robot’s navigation system.

---

## Key Features

• Priority-based intersection navigation
• DIP-switch configurable turning behavior
• Dead-end detection and automatic recovery
• Integral windup protection in PID controller
• Hard-turn execution for sharp corners
• Improved stability at moderate speeds

---

## Intersection Detection

An intersection is detected when both extreme sensors simultaneously detect the line.

Depending on the DIP switch configuration, the robot performs one of the following actions:

| DIP1 | DIP2 | Behavior      |
| ---- | ---- | ------------- |
| ON   | OFF  | Turn Left     |
| OFF  | ON   | Turn Right    |
| OFF  | OFF  | Move Straight |

This allows hardware-level configuration without modifying the firmware.

---

## Dead-End Recovery

If all sensors detect white and no turn was previously detected, the robot assumes a dead-end condition.

The recovery strategy:

1. Perform a 180° rotation
2. Search for the line using center sensors
3. Resume PID-based navigation

---

## PID Enhancements

To improve stability, this version introduces integral windup protection by constraining the accumulated error.

This prevents excessive correction when the robot deviates significantly from the track.

---

## Final Capabilities

The robot can now navigate tracks containing:

• 90° turns
• T-junction intersections
• Dead ends
• Broken or dashed lines

This version represents the most robust implementation of the system.
