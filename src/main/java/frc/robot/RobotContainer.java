// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis_commands.*;
import frc.robot.commands.climber_commands.MoveClimberDown;
import frc.robot.commands.climber_commands.MoveClimberUp;
import frc.robot.commands.intake_commands.SpinIntakeInward;
import frc.robot.commands.intake_commands.SpinIntakeOutward;
import frc.robot.commands.intake_commands.UserTransIntakeIn;
import frc.robot.commands.intake_commands.UserTransIntakeOut;
import frc.robot.commands.pathfinder.RectangleTestCommand;
import frc.robot.commands.shooter_commands.RevAutoShootCommand;
import me.wobblyyyy.pathfinder2.Pathfinder;

import static frc.robot.Constants.*;

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
    private final JoystickButton lBumper = new JoystickButton(controller, 5);
    private final JoystickButton rBumper = new JoystickButton(controller, 6);

    private final SequentialCommandGroup testSwerveDrive = new SequentialCommandGroup(new MoveRightCMD(), new MoveLeftCMD(), new MoveFWDCMD(), new MoveBCKCMD());

    public RobotContainer() {
        Robot.swerveDrive.setDefaultCommand(new ArcadeCommand(() -> ChassisSpeeds.fromFieldRelativeSpeeds(
                -deadzone(xyStick.getX(), Chassis.DEAD_ZONE),
                deadzone(xyStick.getY(), Chassis.DEAD_ZONE),
                deadzone(zStick.getTwist(), Chassis.DEAD_ZONE),
                Robot.swerveDrive.getGyro()
        )));

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

    public Command getAutonomousCommand(Pathfinder pathfinder) {
        return new RectangleTestCommand(pathfinder);
    }

    public Command getCenterShooterToHubCommand(){
        return new CenterShooterToHubCommand();
    }

    public Command revAutoShootCommand(){
        return new RevAutoShootCommand();
    }
    
    

    //Adds a deadzone
    public double deadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }
}
