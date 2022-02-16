// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
=======
>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.*;

import frc.robot.commands.intake_commands.SpinIntakeInward;
import frc.robot.commands.intake_commands.SpinIntakeOutward;
import frc.robot.commands.intake_commands.UserTransIntakeIn;
import frc.robot.commands.intake_commands.UserTransIntakeOut;
import frc.robot.commands.pathfinder.RectangleTestCommand;
import me.wobblyyyy.pathfinder2.Pathfinder;
import frc.robot.commands.chassis_commands.ArcadeCommand;
<<<<<<< HEAD
import frc.robot.commands.chassis_commands.CenterShooterToHubCommand;
import frc.robot.commands.chassis_commands.MoveBCKCMD;
import frc.robot.commands.chassis_commands.MoveFWDCMD;
import frc.robot.commands.chassis_commands.MoveLeftCMD;
import frc.robot.commands.chassis_commands.MoveRightCMD;
=======
>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951
import frc.robot.commands.chassis_commands.ToggleLeftHandMode;


public class RobotContainer {

    private final Joystick xyStick = new Joystick(Control.XY_STICK_ID);
    private final Joystick zStick = new Joystick(Control.Z_STICK_ID);
    private final XboxController controller = new XboxController(Control.CONTROLLER_ID);

<<<<<<< HEAD
  //Xbox Extracted Controller Buttons
  private final JoystickButton xButton = new JoystickButton(controller, 3);
  private final JoystickButton yButton = new JoystickButton(controller, 4);
  private final JoystickButton xyButtonFive = new JoystickButton(xyStick, 5);
  private final JoystickButton aButton = new JoystickButton(controller, 1);
  private final JoystickButton bButton = new JoystickButton(controller, 2);
  private final JoystickButton lBumper = new JoystickButton(controller, 5);
  private final JoystickButton rBumper = new JoystickButton(controller, 6);

  private final SequentialCommandGroup testSwerveDrive = new SequentialCommandGroup(new MoveRightCMD(), new MoveLeftCMD(), new MoveFWDCMD(), new MoveBCKCMD());
=======
    //Xbox Extracted Controller Buttons
    private final JoystickButton xButton = new JoystickButton(controller, 3);
    private final JoystickButton yButton = new JoystickButton(controller, 4);
    private final JoystickButton xyButtonFive = new JoystickButton(xyStick, 5);
    private final JoystickButton aButton = new JoystickButton(controller, 1);
    private final JoystickButton bButton = new JoystickButton(controller, 2);
>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951

    public RobotContainer() {
        Robot.swerveDrive.setDefaultCommand(new ArcadeCommand(() -> {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    -adjustJoystickValues(xyStick.getX(), Chassis.DEAD_ZONE),
                    adjustJoystickValues(xyStick.getY(), Chassis.DEAD_ZONE),
                    adjustJoystickValues(zStick.getTwist(), Chassis.DEAD_ZONE),
                    Robot.swerveDrive.getGyro()
            );
        }));

<<<<<<< HEAD
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

    xButton.and(aButton).and(yButton).and(bButton).whenActive(testSwerveDrive);

    lBumper.whenActive(new MoveClimberDown());
    rBumper.whenActive(new MoveClimberUp());
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  //Adds a deadzone
  public double adjustJoystickValues(double value, double deadzone){
    if(Math.abs(value) > deadzone){
        return value;
=======
        configureButtonBindings();
>>>>>>> 3587b6a18f88668141704d9ae3980ae035997951
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

    public Command getAutonomousCommand(Pathfinder pathfinder) {
        return new RectangleTestCommand(pathfinder);
    }

    //Adds a deadzone
    public double adjustJoystickValues(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }
}
