# MicroProcessor-MidtermProject

## Overview

Welcome to the STM32F3Discovery Elevator Management System project! In this project, we'll design and implement a simple yet functional elevator system using an STM32F3Discovery board and various hardware components. This system will demonstrate elevator functionality, floor control, alarm, and admin mode.

## Hardware Components

To create the Elevator Management System, you'll need the following hardware components:

- **STM32F3Discovery Board**: The core of the project, responsible for controlling the elevator system.

- **UART**: Used for communication and administration of the elevator settings.

- **7-Segment Display**: To display the current floor and elevator direction.

- **LEDs**: Used to indicate alarm status and perform visual feedback.

- **Buzzer**: Produces alarm sounds.

- **External Buttons**: Three external buttons for selecting the desired floor and activating the alarm.

## Project Description

The Elevator Management System simulates an elevator with the following features:

- Display of Current Floor: The 7-segment display shows the current floor and direction of movement. The right digit represents the floor, and the left digit is used for input.

- Setting Desired Floor: Use the two buttons to increase and decrease the desired floor value (0 to last available floor). The left digit should not exceed the last floor.

- Initiating Movement: Press the third external button to command the elevator to move to the desired floor.

- Alarm System: An external button activates the alarm, producing a sound (4 Hz) and blinking LEDs.

- Floor Queue: Elevator manages a queue of floors to visit. It stops at each floor in the order they are selected, ensuring safe and efficient travel.

- Admin Mode: Access admin mode using the command `ADMIN#{Pass}` (with an optional password of at least 4 characters). Admin mode allows you to configure elevator settings.

- Admin Commands:
  1. `SET MAX LEVEL [N]`: Set the number of elevator floors (0 to 9). The current floor becomes zero after this command.
  2. `SET LEVEL [N]`: Change the current floor to [N] (0 to the last possible floor).
  3. `SET WAIT [N]`: Set the elevator stop time on each floor in milliseconds (between 500 and 5000).
  4. `SET LED {ON/OFF}`: Control the LED behavior during an alarm.

- Admin Error Handling: Admin commands should validate input and provide appropriate error messages via UART.

- Elevator Test: Use the `TEST#{sample}` command to fill the elevator queue with allowed floor numbers. View the queue status and execute the elevator's test movement.

## Usage

To use the Elevator Management System:

1. Connect the specified hardware components to the STM32F3Discovery board.

2. Load the system software onto the board.

3. Follow the on-screen instructions to use the elevator, select floors, activate alarms, and access admin mode.

4. Use admin commands for configuration, and test the elevator's movement.

## Note
Unfortunately, I don't have a demo video of my project to show you it visually.

## Author

This project is authored by Mehrnaz Sadeghieh.

---

Experience the functionality of a real elevator system on your STM32F3Discovery board. Have fun and explore the world of embedded systems!
