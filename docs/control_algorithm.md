# Control Algorithm

## PID Control Strategy

The robot uses a proportional–integral–derivative controller to maintain alignment with the track.

The positional error is defined as:

error = desired_position − measured_position

Where the desired position corresponds to the center of the sensor array.

The control signal is calculated as:

control = Kp * P + Ki * I + Kd * D

Where:

P = current error
I = accumulated error
D = rate of change of error

The control output adjusts differential motor speeds to steer the robot back toward the line.

---

## Motor Control

Motor speeds are generated as:

left_motor = base_speed + control_signal
right_motor = base_speed − control_signal

This creates differential steering.

---

## Intersection Detection

Intersections are detected when both extreme sensors detect the line simultaneously.

Decision logic is determined by hardware DIP switches:

Switch 1 → Left priority
Switch 2 → Right priority

If both switches are inactive, the robot continues forward.

---

## Recovery Mechanisms

Several recovery strategies are implemented:

• Last-seen turn memory
• Dashed line recovery
• Dead-end detection
• Hard-turn execution
