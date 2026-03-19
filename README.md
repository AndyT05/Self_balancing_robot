# Self-Balancing Robot

A two-wheeled self-balancing robot built on the **TM4C123GH6PM / EK-TM4C123GXL** using an **ICM-20948 IMU** and an **L298N motor driver**. The firmware is written in **C** and uses real-time sensor acquisition, a complementary filter, and a PID controller to keep the robot balanced.

## Overview

This project implements a closed-loop balance control system for a two-wheeled robot. The robot reads accelerometer and gyroscope data from the ICM-20948 over I2C, estimates pitch angle using a complementary filter, and adjusts motor speed and direction through PWM to correct tilt in real time.

The system also includes:
- IMU calibration using the onboard push button
- UART debug output for tuning and monitoring
- Safety stop when the tilt angle exceeds a defined threshold
- Dead-zone compensation for motor response

## Features

- Embedded C firmware on **TM4C123GH6PM**
- **ICM-20948** accelerometer and gyroscope interfacing over **I2C**
- **Complementary filter** for pitch angle estimation
- **PID control** for real-time balance correction
- PWM-based motor speed control
- Bidirectional motor drive control
- Push-button calibration routine
- UART serial output for debugging
- Safety shutoff for excessive tilt

## Hardware Used

- **TM4C123GH6PM / EK-TM4C123GXL**
- **ICM-20948 IMU**
- **L298N motor driver**
- 2 DC motors
- Two-wheeled robot chassis
- Power supply / battery pack

## How It Works

1. The microcontroller initializes GPIO, PWM, I2C, UART, and timers.
2. The IMU is configured and calibrated to reduce sensor bias.
3. Accelerometer and gyroscope data are read continuously.
4. A complementary filter combines both sensor readings to estimate pitch angle.
5. The PID controller calculates the correction needed to keep the robot upright.
6. PWM duty cycle and motor direction are updated based on the control output.
7. If the robot tilts beyond the safety limit, the motors are disabled.

## Control System

### Complementary Filter
The robot estimates pitch angle by combining:
- Accelerometer-based angle estimation for long-term stability
- Gyroscope angular velocity for short-term responsiveness

This improves noise rejection while maintaining fast response.

### PID Controller
The PID controller computes motor correction using:
- **Proportional** term for immediate error response
- **Integral** term for accumulated error correction
- **Derivative** term for damping and stability

The output is converted into PWM duty cycle commands for the motors.

## Software Details

- Language: **C**
- Sensor communication: **I2C**
- Debug interface: **UART**
- Motor control: **PWM**
- Platform: **TM4C123GH6PM**

## Safety Features

- Stops motors if tilt angle exceeds the allowed range
- Includes motor dead-zone compensation
- Supports manual calibration before operation

## Project Goals

This project was built to strengthen hands-on experience in:
- Embedded systems
- Sensor interfacing
- Real-time control
- PID tuning
- Sensor fusion
- Robotics firmware development

## Possible Improvements

- Add wheel encoder feedback for better motion estimation
- Improve PID auto-tuning
- Replace complementary filter with a Kalman filter
- Upgrade motor driver for smoother control
- Add wireless telemetry or Bluetooth monitoring

## Demo

https://youtube.com/shorts/mrXsTtO0UMk

## Author

**Andy Tran**

## Tags

`embedded-c` `tm4c123gh6pm` `tm4c123gxl` `self-balancing-robot` `pid-control` `imu` `icm20948` `robotics` `embedded-systems`
