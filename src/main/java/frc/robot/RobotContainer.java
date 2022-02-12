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

import static frc.robot.Constants.*;

import frc.robot.commands.intake_commands.SpinIntakeInward;
import frc.robot.commands.intake_commands.SpinIntakeOutward;
import frc.robot.commands.intake_commands.UserTransIntakeIn;
import frc.robot.commands.intake_commands.UserTransIntakeOut;
import frc.robot.commands.shooter_commands.RevAutoShootCommand;
import frc.robot.commands.shooter_commands.RevShooterCommand;
import frc.robot.commands.chassis_commands.ArcadeCommand;
import frc.robot.commands.chassis_commands.CenterShooterToHubCommand;
import frc.robot.commands.chassis_commands.ToggleLeftHandMode;
import frc.robot.commands.climber_commands.MoveClimberDown;
import frc.robot.commands.climber_commands.MoveClimberUp;


public class RobotContainer {

  private final Joystick xyStick = new Joystick(Control.XY_STICK_ID);
  private final Joystick zStick = new Joystick(Control.Z_STICK_ID);
  private final XboxController controller = new XboxController(Control.CONTROLLER_ID);

  //Xbox Extracted Controller Buttons
  private final JoystickButton xButton = new JoystickButton(controller, 3);
  private final JoystickButton yButton = new JoystickButton(controller, 4);
  private final JoystickButton xyButtonFive = new JoystickButton(xyStick, 5);
  private final JoystickButton aButton = new JoystickButton(controller, 1);
  private final JoystickButton bButton = new JoystickButton(controller, 2);

  public RobotContainer() {
    Robot.swerveDrive.setDefaultCommand(new ArcadeCommand(() -> {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
        -adjustJoystickValues(xyStick.getX(), Chassis.DEADZONE),
        adjustJoystickValues(xyStick.getY(), Chassis.DEADZONE),
        adjustJoystickValues(zStick.getTwist(), Chassis.DEADZONE),
        Robot.swerveDrive.getGyro()
      );
    }));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // TODO: Add button bindings
    //xButton.debounce(0.2).whenActive(new CenterShooterToHubCommand());
    xyButtonFive.whenActive(new ToggleLeftHandMode());
    aButton.whenHeld(new UserTransIntakeOut());
    bButton.whenHeld(new UserTransIntakeIn());

    xButton.whenHeld(new SpinIntakeOutward());
    yButton.whenHeld(new SpinIntakeInward());
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
