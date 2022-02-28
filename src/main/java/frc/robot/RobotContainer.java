// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Chassis;
import frc.robot.commands.autonomous_commands.TestAutonomous;
import frc.robot.commands.chassis_commands.*;
import frc.robot.commands.climber_commands.MoveClimberDown;
import frc.robot.commands.climber_commands.MoveClimberUp;
import frc.robot.commands.shooter_commands.AutoShootCommand;
import frc.robot.commands.shooter_commands.TimedShooterCommand;
import frc.robot.commands.storage_commands.SequentialStorageCMDs.IntakeProcessAccept;
import frc.robot.commands.storage_commands.SequentialStorageCMDs.StorageDecision;
import frc.robot.commands.storage_commands.SequentialStorageCMDs.StorageExtendIntake;
import frc.robot.commands.storage_commands.SimpleProcessBallCMD;
import me.wobblyyyy.pathfinder2.wpilib.PathfinderSubsystem;

import static frc.robot.Constants.Control.*;

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

    // TODO: may need to add/remove commands from this group.
    private final SequentialCommandGroup autoShootGroup = new SequentialCommandGroup(
            new CenterShooterToHubCommand(),
            new AutoShootCommand()
    );

    private final SequentialCommandGroup processBallCMD = new SequentialCommandGroup(new StorageExtendIntake(),
            new IntakeProcessAccept(),
            new StorageDecision());


    public RobotContainer() {
        Robot.swerveDrive.setDefaultCommand(new ArcadeCommand(() ->
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        deadzone(xyStick.getX(), Chassis.CONTROLLER_DEADZONE),
                        -deadzone(xyStick.getY(), Chassis.CONTROLLER_DEADZONE),
                        -deadzone(zStick.getTwist(), Chassis.CONTROLLER_DEADZONE),
                        Rotation2d.fromDegrees(0)
                )
        ));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        xyButtonFive.whenActive(new ToggleLeftHandMode());
        xyButtonFive.debounce(0.2).whenActive(new CenterShooterToHubCommand());

        aButton.whenActive(new TimedShooterCommand());

        yButton.whenActive(processBallCMD);

        xButton.whileHeld(new SimpleProcessBallCMD());


        lBumper.whenActive(new MoveClimberUp());
        rBumper.whenActive(new MoveClimberDown());
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

    public Command getAutonomousCommand(PathfinderSubsystem pathfinderSubsystem) {
        return (Command) new TestAutonomous(pathfinderSubsystem);
    }

    public SequentialCommandGroup getAutoShootGroup() {
        return autoShootGroup;
    }

    public double deadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }
}
