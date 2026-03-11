# Hardware Connections

## Microcontroller

Arduino Nano / ATmega328 based board.

---

## Motor Driver (TB6612FNG)

| Pin  | Arduino Connection |
| ---- | ------------------ |
| PWMA | Pin 5              |
| PWMB | Pin 6              |
| AIN1 | Pin 3              |
| AIN2 | Pin 4              |
| BIN1 | Pin 7              |
| BIN2 | Pin 8              |
| STBY | Pin 9              |

---

## Sensor Array

The QTR reflectance sensor array is connected to analog pins.

| Sensor | Arduino Pin |
| ------ | ----------- |
| S0     | A0          |
| S1     | A1          |
| S2     | A2          |
| S3     | A3          |
| S4     | A4          |
| S5     | A5          |
| S6     | A6          |
| S7     | A7          |

Emitter control pin: Digital Pin 2

---

## Additional Components

Bluetooth Module (HC-05)
Used for runtime PID tuning in Version 1.

DIP Switches
Used in Version 3 for intersection navigation priority configuration.

---

## Power System

The robot is powered using a rechargeable lithium battery pack providing sufficient current for both motors and control electronics.
