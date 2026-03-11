# Version 2 – Robust Line Following with Dashed Line Recovery

## Overview

Version 2 improves the navigation robustness of the line-following robot by introducing algorithms to handle broken tracks and sharp turns.

In many competition tracks, the robot may encounter dashed lines or temporary loss of the track. This version introduces recovery mechanisms that allow the robot to continue navigating even when all sensors temporarily detect white space.

---

## Improvements over Version 1

• Dashed-line recovery mechanism
• Turn memory using last detected direction
• Improved extreme-turn detection
• Enhanced calibration routine
• Increased turn response speed

---

## Navigation Logic

The robot continuously monitors the reflectance sensor array to determine track position.

If the robot temporarily loses the line:

1. The system checks whether all sensors detect white.
2. The last detected turn direction is retrieved.
3. The robot attempts recovery by steering toward the last known track direction.

This approach enables navigation across dashed segments and incomplete tracks.

---

## Turn Handling

Extreme sensors are used to detect sharp turns.

When a turn is detected:

• The robot enters a turning state
• One motor slows while the other accelerates
• The robot exits the turn once the center sensors detect the track again

---

## Known Behavior

This version allows higher speeds but introduces:

• Increased oscillations due to aggressive correction
• Higher sensitivity to PID tuning

These issues are addressed in the next iteration.
