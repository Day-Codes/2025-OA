# FIRST Robotics Competition Robot - Autonomous Path

This README provides an overview of the code for the robot designed for the FIRST Robotics Competition (FRC). The robot follows an autonomous path starting from the starting line, driving to the center, scoring a level 1 (L1) object, and then returning to the starting line.

## Overview

This code is implemented using the WPILib library and written in Java. The robot uses the following key components:

- **Drive Train:** Powered by four VictorSPX motor controllers, controlling two sets of motors for the left and right sides of the robot.
- **Roller Mechanism:** A PWMVictorSPX motor that controls a roller for scoring the Level 1 (L1) object.
- **Controller Input:** An Xbox controller to drive the robot in teleoperated mode.
- **Autonomous Routine:** A simple autonomous routine that moves the robot forward, activates the roller to score the object, and returns to the starting line.

## Features

- **Autonomous Mode:**
  - The robot drives forward to the center for 2 seconds.
  - After reaching the center, it activates the roller mechanism for 1 second to score the L1 object.
  - It then drives backward to return to the starting line.

- **Teleoperated Mode:**
  - The robot can be driven using an Xbox controller, and the roller can be controlled using the A button.

## Hardware Requirements

This robot uses the following hardware components:

- **Drive Motors:**
  - 2x **VictorSPX** (5, 6) for left and right motors.
  - 2x **VictorSPX** (2, 3) for left and right motor followers.
  
- **Roller Mechanism:**
  - 1x **PWMVictorSPX** (4) motor controller for the roller.

- **Controller:**
  - 1x **Xbox Controller** for manual driving and controlling the roller.

## Setup Instructions

1. **Connect the Motors:**
   - Connect the left and right motors to the corresponding VictorSPX motor controllers (5, 6, 2, 3).
   - Connect the roller motor to the PWMVictorSPX (4).

2. **Controller Setup:**
   - Ensure the Xbox controller is connected and recognized by the robot.

3. **SmartDashboard Setup:**
   - The SmartDashboard will display the robot's position on the field (`Field2d`), autonomous time, and roller speed.

4. **Autonomous Path:**
   - The robot will drive forward for 2 seconds, activate the roller, and then move backward to return to the starting line.

## Code Explanation

### `robotInit()`
- Initializes the robot and configures the drive motors and roller.
- Sets up motor followers and inversions for proper movement.
- Configures the deadband for the drive and initializes the SmartDashboard.

### `robotPeriodic()`
- Runs periodically to update the robot’s state on the SmartDashboard, including:
  - Time since the robot started.
  - Roller motor speed.
  - Simulated robot pose.

### `autonomousInit()` and `autonomousPeriodic()`
- The robot starts the autonomous period by resetting and starting the timer.
- In the first 2 seconds, it moves forward.
- In the next second (2 to 3 seconds), it activates the roller.
- In the last 2 seconds (3 to 5 seconds), it moves backward to return to the starting line.

### `teleopPeriodic()`
- In teleoperated mode, the Xbox controller is used to control the robot.
  - Left joystick controls forward/backward motion.
  - Right joystick controls turning.
  - Pressing the A button activates the roller.

## Autonomous Path Breakdown

1. **Start at the Starting Line:** The robot begins at the starting line.
   
2. **Drive Forward:** The robot moves forward at full speed (`drive_speed = 1.0`) for 2 seconds.
   - This simulates driving from the starting line toward the center of the field.

3. **Activate Roller to Score:** After 2 seconds of moving forward, the robot stops and activates the roller (`roller_speed = 0.5`) for 1 second.
   - This simulates scoring a Level 1 (L1) object.

4. **Drive Back to Starting Line:** After scoring, the robot drives backward at full speed (`drive_speed = 1.0`) for 2 seconds to return to the starting line.

5. **End of Autonomous Mode:** The robot stops after returning to the starting line.

## SmartDashboard

- **Field:** Displays the robot’s simulated position on the field.
- **Time On:** Displays the time elapsed since the robot started.
- **Roller Speed RPM:** Shows the roller motor's speed.
- **Auto Time (Secs):** Displays the time elapsed during the autonomous period.

## Teleoperated Mode

- **Drive Controls:** Use the Xbox controller’s left and right joysticks to control the robot’s forward/backward movement and rotation.
- **Roller Control:** Press the A button to activate the roller for scoring.

## Conclusion

This code demonstrates the basic autonomous and teleoperated functionality of a FIRST Robotics Competition robot. The autonomous mode includes a simple path that drives the robot forward, activates the roller to score an object, and then returns to the starting line. This setup can be modified for more complex behaviors based on the team's strategy.
