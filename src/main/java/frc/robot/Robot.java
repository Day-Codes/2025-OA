// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot {

  private final Timer m_timer = new Timer();
  private final Field2d m_field = new Field2d();
  private boolean isdriving = false;

  // Values for the Robot
  private final double drive_deadband = 0.2; // Deadband for the drive
  private final double roller_speed = 0.5; // Roller speed setting
  private final double drive_speed = 1; // Drive speed setting
  
  // Simulated Position
  private double posX = 0.0, posY = 0.0, heading = 0.0;

  // Motor Controllers for the Drive Train
  WPI_VictorSPX m_leftMotor_victor = new WPI_VictorSPX(5);
  WPI_VictorSPX m_rightMotor_victor = new WPI_VictorSPX(6);
  WPI_VictorSPX m_leftMotor_victor_two = new WPI_VictorSPX(2);
  WPI_VictorSPX m_rightMotor_victor_two = new WPI_VictorSPX(3);
  
  // Roller Motor Controller
  PWMVictorSPX m_roller_victor = new PWMVictorSPX(4);
  
  // DifferentialDrive for the Drive Train
  DifferentialDrive m_robotDrive_two = new DifferentialDrive(m_leftMotor_victor, m_rightMotor_victor); // VictorSPX Drive Train

  // Xbox Controller for user input
  XboxController m_driverController = new XboxController(0);

  @Override
  public void robotInit() {
    m_timer.start(); // Start the timer for autonomous timing

    // Set motor followers for drive train
    m_leftMotor_victor_two.follow(m_leftMotor_victor);
    m_rightMotor_victor_two.follow(m_rightMotor_victor);

    // Set motor inversion for proper movement direction
    m_rightMotor_victor.setInverted(true);
    m_rightMotor_victor_two.setInverted(true);

    // Set deadband for drive train and create DifferentialDrive object
    m_robotDrive_two = new DifferentialDrive(m_leftMotor_victor, m_rightMotor_victor);
    m_robotDrive_two.setDeadband(drive_deadband);

    // Add Field visualization to SmartDashboard for simulation or debugging
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Driving", m_robotDrive_two); // Drive train
    SmartDashboard.putData("Roller", m_roller_victor); // Roller motor
    SmartDashboard.putBoolean("Auto", isdriving); // Display auto mode status
    SmartDashboard.putNumber("Auto Time(Secs)", m_timer.get()); // Display time in autonomous mode
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); // Run scheduled commands

    // Display robot runtime information on SmartDashboard
    SmartDashboard.putNumber("Time On (Secs)", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Roller Speed RPM", m_roller_victor.getVoltage());
    
    // Update simulated odometry on SmartDashboard
    m_field.setRobotPose(new Pose2d(posX, posY, Rotation2d.fromDegrees(heading)));
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll(); // Cancel all running commands when disabled
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_timer.reset(); // Reset the autonomous timer
    m_timer.start(); // Start the timer
  }

  @Override
  public void autonomousPeriodic() {
    double time = m_timer.get(); // Get the current time in autonomous

    // Autonomous routine: Move forward for 2 seconds, then perform actions
    if (time < 2.0) {
      m_robotDrive_two.arcadeDrive(drive_speed, 0.0); // Move forward
      posX += drive_speed * 0.04; // Simulate forward movement
      isdriving = true;
    } else if (time < 4.0) {
      m_robotDrive_two.arcadeDrive(0.0, 0.0); // Stop driving
      m_roller_victor.set(roller_speed); // Activate roller
    } else if(time < 7.0) {
      m_roller_victor.set(0.0); // Deactivate roller
      m_robotDrive_two.arcadeDrive(-drive_speed, 0.0); // Move backward
      posX -= drive_speed * 0.04; // Simulate backward movement
     } 
      else if (time < 9.0) {
        m_robotDrive_two.arcadeDrive(0.0, 0.0); // Stop driving
        isdriving = false;
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Score L1 with Roller when A button is pressed
    if (m_driverController.getAButton()) {
      m_roller_victor.set(roller_speed);
    } else {
      m_roller_victor.set(0.0); // Stop roller if not pressed
    }

    // Drive the robot based on the controller input
    m_robotDrive_two.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll(); // Cancel all commands at the start of test mode
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
