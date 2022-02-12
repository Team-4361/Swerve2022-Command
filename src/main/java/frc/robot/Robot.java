// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeShooter;
import frc.robot.robot_utils.Camera;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.Camera.CAMERA_NAME;
import static frc.robot.Constants.Camera.CAMERA_HEIGHT;
import static frc.robot.Constants.Camera.CAMERA_PITCH;
import static frc.robot.Constants.Camera.NETWORK_TABLE_HOSTNAME;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static SwerveDriveSubsystem swerveDrive;
  public static StorageSubsystem storage;
  public static ShooterSubsystem shooter;
  public static IntakeSubsystem intake;
  public static ClimberSubsystem climber;

  public static Camera camera;
  public static boolean leftHandedMode = false;

  private final StorageSubsystem.AcceptColor INIT_TARGET_COLOR = StorageSubsystem.AcceptColor.BLUE;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  
    swerveDrive = new SwerveDriveSubsystem();
    swerveDrive.resetGyro();

    storage = new StorageSubsystem(INIT_TARGET_COLOR);
    shooter = new ShooterSubsystem(storage);

    intake = new IntakeSubsystem();
    climber = new ClimberSubsystem();

    //camera = new ShooterVisionCamera(CAMERA_NAME, CAMERA_HEIGHT, CAMERA_PITCH);

    //Should be the last thing in this function
    m_robotContainer = new RobotContainer();
  }

 
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}


  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
