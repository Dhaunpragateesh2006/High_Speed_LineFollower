# PID Control for Line Following Robot

## Introduction

The robot uses a Proportional–Integral–Derivative (PID) controller to maintain alignment with the track.

The objective of the controller is to minimize the deviation between the robot's current position and the center of the line.

---

## Control Loop Architecture

![PID Control Loop](../images/pid_block_diagram.png)

The control system operates as a feedback loop:

1. The sensor array measures the line position.
2. The position is compared with the desired center position.
3. The error is computed.
4. The PID controller generates a correction signal.
5. The correction adjusts the motor speeds.

This feedback loop continuously corrects the robot's trajectory.

---

## Error Calculation

The QTR sensor array returns a weighted position value ranging from:

0 → far left
7000 → far right

The desired position is the center of the sensor array.

```
error = desired_position - measured_position
```

---

## PID Controller Equation

The control output is computed as:

```
control = Kp × P + Ki × I + Kd × D
```

Where:

P → proportional error
I → accumulated error over time
D → rate of change of error

---

## Role of Each Component

### Proportional Term (P)

Provides immediate correction based on the current error.

Higher Kp results in faster response but may cause oscillations.

---

### Integral Term (I)

Accumulates past errors to eliminate steady-state offset.

However, excessive integral accumulation can lead to instability.

To prevent this, the integral term is constrained in Version 3.

---

### Derivative Term (D)

Predicts future error by measuring the rate of change.

This term helps damp oscillations and stabilizes the robot during high-speed motion.

---

## Differential Motor Control

The control signal adjusts motor speeds as:

```
Left Motor  = Base Speed + Control
Right Motor = Base Speed - Control
```

This produces differential steering which directs the robot back toward the line.

---

## Stability Considerations

Several improvements were introduced across project versions:

• Integral windup protection
• Speed ramping to avoid sudden acceleration
• Turn detection logic to override PID during sharp turns

These improvements significantly increased navigation stability.

---

## Conclusion

The PID controller forms the core of the robot's trajectory tracking system. By continuously correcting positional error, the robot can follow complex tracks at relatively high speeds while maintaining stability.
