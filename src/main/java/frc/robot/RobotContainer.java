// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.*;

public class RobotContainer {

  private final Joystick xyStick = new Joystick(Constants.XY_STICK_ID);
  private final Joystick zStick = new Joystick(Constants.Z_STICK_ID);
  private final XboxController controller =
          new XboxController(Constants.CONTROLLER_ID);
  private boolean leftHandedMode = false;
  
  
  // The robot's subsystems and commands are defined here...
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(INTAKE_PORT, ACCEPTOR_PORT, PHOTOELECTRIC_DIO, COLOR_SENSOR_PORT);

  private final ArcadeCommand arcadeCommand = new ArcadeCommand(swerveDriveSubsystem, () -> {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
      -adjustJoystickValues(xyStick.getX(), Constants.DEADZONE),
      adjustJoystickValues(xyStick.getY(), Constants.DEADZONE),
      adjustJoystickValues(zStick.getTwist(), Constants.DEADZONE),
      Rotation2d.fromDegrees(0)
    );
  });

  public RobotContainer() {
    swerveDriveSubsystem.setDefaultCommand(arcadeCommand);
    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  //Adds a deadzone
  public double adjustJoystickValues(double value, double deadzone){
    if(Math.abs(value) > deadzone){
        return value;
    }
    else{
        return 0;
    }
  }
}
