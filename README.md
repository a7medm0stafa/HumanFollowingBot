# Human-Following Robot

This repository contains the code for a human-following robot developed using STM32 microcontroller and ultrasonic sensors.

## Description

The robot is designed to follow a human user at a fixed distance using three ultrasonic sensors placed at the front, left, and right sides of the robot. It measures the distance to the user and adjusts its movement accordingly to maintain the desired distance.

## Hardware Components

- STM32 Microcontroller
- Ultrasonic Sensors (3x)
- Motor Drivers
- DC Motors
- Power Supply

## How It Works

The robot uses three ultrasonic sensors to measure the distance to the user:
- **Center Sensor**: Mounted on the front of the robot, it measures the distance directly in front of the robot.
- **Left Sensor**: Mounted on the left side of the robot, it measures the distance to the left of the robot.
- **Right Sensor**: Mounted on the right side of the robot, it measures the distance to the right of the robot.

Based on the distance measured by these sensors, the robot adjusts its movement to maintain a desired distance from the user.

## Installation

To run the code on your STM32 microcontroller, follow these steps:

1. Clone this repository to your local machine.
2. Open the project in your preferred Integrated Development Environment (IDE) that supports STM32 development, such as STM32CubeIDE.
3. Connect your STM32 microcontroller to your computer.
4. Build the project and flash the firmware to your STM32 microcontroller.

## Usage

1. Power on the robot and ensure that the ultrasonic sensors are properly calibrated and facing the correct direction.
2. Place the robot and the user in an open area with enough space for movement.
3. The robot will start following the user at the specified distance.

## License

This software is licensed under terms that can be found in the LICENSE file in the root directory of this software component. If no LICENSE file comes with this software, it is provided AS-IS.
