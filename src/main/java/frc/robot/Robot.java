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

  // Vaules for the Robot
  private final double drive_deadband = 0.2;
  private final double roller_speed = 0.5;
  private final double drive_speed = 1;
  // Simulated Position
    private double posX = 0.0, posY = 0.0, heading = 0.0;

  

 // VictorSPX 
WPI_VictorSPX m_leftMotor_victor = new WPI_VictorSPX(5);
WPI_VictorSPX m_rightMotor_victor = new WPI_VictorSPX(6);
WPI_VictorSPX m_leftMotor_victor_two = new WPI_VictorSPX(2);
WPI_VictorSPX m_rightMotor_victor_two = new WPI_VictorSPX(3);
 // Roller 
PWMVictorSPX m_roller_victor = new PWMVictorSPX(4);
DifferentialDrive m_robotDrive_two = new DifferentialDrive(m_leftMotor_victor, m_rightMotor_victor); // VictorSPX Drive Train

// Xbox Controller
XboxController m_driverController = new XboxController(0);



  @Override
  public void robotInit() {
    m_timer.start();
  

    // DriveTrain Followers 
    m_leftMotor_victor_two.follow(m_leftMotor_victor);
    m_rightMotor_victor_two.follow(m_rightMotor_victor);

    // DriveTrain Inversion
    m_rightMotor_victor.setInverted(true);
    m_rightMotor_victor_two.setInverted(true);

    // DriveTrain Deadband & DifferentialDrive
    m_robotDrive_two = new DifferentialDrive(m_leftMotor_victor, m_rightMotor_victor);
    m_robotDrive_two.setDeadband(drive_deadband);

    // Add Feild to SmartDashboard
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Driving", m_robotDrive_two); // ??
    SmartDashboard.putData("Roller", m_roller_victor); // ??
    SmartDashboard.putBoolean("Auto", isdriving);
    SmartDashboard.putNumber("Auto Time(Secs)", m_timer.get());

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // Count the Time On
    SmartDashboard.putNumber("Time On (Secs)", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Roller Speed RPM", m_roller_victor.getVoltage());
     // Simulated Odometry Update
    m_field.setRobotPose(new Pose2d(posX, posY, Rotation2d.fromDegrees(heading)));
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double time = m_timer.get();

    if (time < 2.0) {
      m_robotDrive_two.arcadeDrive(drive_speed, 0.0);
      posX += drive_speed * 0.02; // Simulate forward movement
      isdriving = true;
    } else if (time < 3.0) {
      m_robotDrive_two.arcadeDrive(0.0, 0.0);
      m_roller_victor.set(roller_speed);
    } else if(time < 5.0) {
      m_roller_victor.set(0.0);
      m_robotDrive_two.arcadeDrive(-drive_speed, 0.0);
        posX -= drive_speed * 0.02; // Simulate backward movement
    }
  }


//   @Override
//   public void autonomousInit() {
//     m_timer.start();
//   m_rightMotor_victor.set(drive_speed);
//   m_leftMotor_victor.set(drive_speed);
//     isdriving = true;
//     m_timer.delay(2.0);
//     m_rightMotor_victor.set(0);
//     m_leftMotor_victor.set(0);
//     m_roller_victor.set(roller_speed);
//     m_timer.delay(1);
//     m_roller_victor.set(0.0);
//     m_timer.stop();
//     }
  
  

//   @Override
//   public void autonomousPeriodic() {
//   }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
 // Score L1 with Roller
    if (m_driverController.getAButton()) {
      m_roller_victor.set(roller_speed);
    } else {
      m_roller_victor.set(0.0);
    }

// Drive Train
    m_robotDrive_two.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
