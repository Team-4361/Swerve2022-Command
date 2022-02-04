// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ACCEPTOR_PORT;
import static frc.robot.Constants.COLOR_SENSOR_PORT;
import static frc.robot.Constants.INTAKE_PORT;
import static frc.robot.Constants.PHOTOELECTRIC_DIO;
import static frc.robot.Constants.PHOTOELECTRIC_DIO_2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.CenterShooterToHubCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.StorageSubsystem.AcceptColor;

public class RobotContainer {

  private final Joystick xyStick = new Joystick(Constants.XY_STICK_ID);
  private final Joystick zStick = new Joystick(Constants.Z_STICK_ID);
  private final XboxController controller =
          new XboxController(Constants.CONTROLLER_ID);
  private boolean leftHandedMode = false;
  
  // The robot's subsystems and commands are defined here...
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
  // private final StorageSubsystem intakeSubsystem = new StorageSubsystem(
  //   INTAKE_PORT, 
  //   ACCEPTOR_PORT, 
  //   PHOTOELECTRIC_DIO, 
  //   PHOTOELECTRIC_DIO_2, 
  //   COLOR_SENSOR_PORT,
  //   AcceptColor.BLUE
  // );

  private final ArcadeCommand arcadeCommand = new ArcadeCommand(swerveDriveSubsystem, () -> {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
      -adjustJoystickValues(xyStick.getX(), Constants.DEADZONE),
      adjustJoystickValues(xyStick.getY(), Constants.DEADZONE),
      adjustJoystickValues(zStick.getTwist(), Constants.DEADZONE),
      swerveDriveSubsystem.getGyro()
    );
  });

  // private final CenterShooterToHubCommand lineUpRobotCommand = new CenterShooterToHubCommand();
  // private final AutoShootCommand autoShootCommand = new AutoShootCommand();
  // private final ShootCommand shoot = new ShootCommand();

  public RobotContainer() {
    swerveDriveSubsystem.setDefaultCommand(arcadeCommand);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // TODO: Add button bindings

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
