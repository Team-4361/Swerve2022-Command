// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Control.CONTROLLER_ID;
import static frc.robot.Constants.Control.XBOX_A;
import static frc.robot.Constants.Control.XBOX_B;
import static frc.robot.Constants.Control.XBOX_LEFT_TRIGGER;
import static frc.robot.Constants.Control.XBOX_RIGHT_TRIGGER;
import static frc.robot.Constants.Control.XBOX_X;
import static frc.robot.Constants.Control.XBOX_Y;
import static frc.robot.Constants.Control.XY_STICK_ID;
import static frc.robot.Constants.Control.Z_STICK_ID;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Chassis;
import frc.robot.commands.autonomous_commands.TestAutonomous;
import frc.robot.commands.chassis_commands.ArcadeCommand;
import frc.robot.commands.chassis_commands.CenterShooterToHubCommand;
import frc.robot.commands.chassis_commands.MoveBCKCMD;
import frc.robot.commands.chassis_commands.MoveFWDCMD;
import frc.robot.commands.chassis_commands.MoveLeftCMD;
import frc.robot.commands.chassis_commands.MoveRightCMD;
import frc.robot.commands.chassis_commands.ToggleLeftHandMode;
import frc.robot.commands.climber_commands.MoveClimberDown;
import frc.robot.commands.climber_commands.MoveClimberUp;
//import frc.robot.commands.climber_commands.ClimberCommandDown;
//import frc.robot.commands.climber_commands.ClimberCommandUp;
import frc.robot.commands.intake_commands.ExtendIntake;
import frc.robot.commands.intake_commands.SpinIntakeReject;
import frc.robot.commands.shooter_commands.RevAutoShootCommand;
import frc.robot.commands.shooter_commands.ShootCMD;
import frc.robot.commands.storage_commands.ProcessBallCommand;
import me.wobblyyyy.pathfinder2.wpilib.PathfinderSubsystem;

public class RobotContainer {

    private final Joystick xyStick = new Joystick(XY_STICK_ID);
    private final Joystick zStick = new Joystick(Z_STICK_ID);
    private final XboxController controller = new XboxController(CONTROLLER_ID);

    // Xbox Extracted Controller Buttons
    private final JoystickButton xButton = new JoystickButton(controller, XBOX_X);
    private final JoystickButton yButton = new JoystickButton(controller, XBOX_Y);
    private final JoystickButton aButton = new JoystickButton(controller, XBOX_A);
    private final JoystickButton bButton = new JoystickButton(controller, XBOX_B);
    private final JoystickButton lBumper = new JoystickButton(controller, XBOX_LEFT_TRIGGER);
    private final JoystickButton rBumper = new JoystickButton(controller, XBOX_RIGHT_TRIGGER);

    private final JoystickButton xyButtonFive = new JoystickButton(xyStick, 5);

    private final SequentialCommandGroup testSwerveDrive = new SequentialCommandGroup(
            new MoveRightCMD(),
            new MoveLeftCMD(),
            new MoveFWDCMD(),
            new MoveBCKCMD()
    );

    private final SequentialCommandGroup AutoShoot = new SequentialCommandGroup(
            new CenterShooterToHubCommand(),
            new RevAutoShootCommand(),
            new ShootCMD()
    );

    private final ParallelCommandGroup storageEnableGroup = new ParallelCommandGroup()
            .alongWith(new ProcessBallCommand())
            .alongWith(new SpinIntakeReject())
            .alongWith(new ExtendIntake());

    public RobotContainer() {
        Robot.swerveDrive.setDefaultCommand(new ArcadeCommand(() ->
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        deadzone(xyStick.getX(), Chassis.CONTROLLER_DEADZONE),
                        -deadzone(xyStick.getY(), Chassis.CONTROLLER_DEADZONE),
                        -deadzone(zStick.getTwist(), Chassis.CONTROLLER_DEADZONE),
                        Robot.swerveDrive.getGyro()
                )
        ));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        xyButtonFive.whenActive(new ToggleLeftHandMode());
        xyButtonFive.debounce(0.2).whenActive(new CenterShooterToHubCommand());

        aButton.whenHeld(AutoShoot);
        //bButton.whenHeld(new RevShooterCommand(false));

        //yButton.whenActive(new ProcessBallCommand());

        yButton.whenActive(new ExtendIntake());

        xButton.and(aButton).whenActive(storageEnableGroup);

        // xButton.and(aButton).and(yButton).and(bButton).whenActive(testSwerveDrive);

        lBumper.whenActive(new MoveClimberUp());
        rBumper.whenActive(new MoveClimberDown());
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

    public Command getAutonomousCommand(PathfinderSubsystem pathfinderSubsystem) {
        return new TestAutonomous(pathfinderSubsystem);
    }

    public SequentialCommandGroup getAutoShoot() {
        return new SequentialCommandGroup(new CenterShooterToHubCommand(), new RevAutoShootCommand());
    }

    public double deadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }
}
