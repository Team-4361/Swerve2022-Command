// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.RevAutoShootCommand;
import frc.robot.commands.CenterShooterToHubCommand;
import frc.robot.commands.ChangeHandMode;
import frc.robot.commands.RevShooterCommand;


public class RobotContainer {

  private final Joystick xyStick = new Joystick(Constants.XY_STICK_ID);
  private final Joystick zStick = new Joystick(Constants.Z_STICK_ID);
  private final XboxController controller =
          new XboxController(Constants.CONTROLLER_ID);

  //Xbox Extracted Controller Buttons
  private final JoystickButton xBTN = new JoystickButton(controller, 3);


  private final JoystickButton xyBTNFIVE = new JoystickButton(xyStick, 5);


  public RobotContainer() {
    
    Robot.swerveDrive.setDefaultCommand(new ArcadeCommand(() -> {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
        -adjustJoystickValues(xyStick.getX(), Constants.DEADZONE),
        adjustJoystickValues(xyStick.getY(), Constants.DEADZONE),
        adjustJoystickValues(zStick.getTwist(), Constants.DEADZONE),
        Robot.swerveDrive.getGyro()
      );
    }));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // TODO: Add button bindings
    //xBTN.debounce(0.2).whenActive(new CenterShooterToHubCommand());
    xyBTNFIVE.whenActive(new ChangeHandMode());
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
