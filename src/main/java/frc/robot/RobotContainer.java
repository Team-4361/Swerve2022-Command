// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Chassis;
import frc.robot.commands.autonomous_commands.TestAutonomous;
import frc.robot.commands.chassis_commands.*;
import frc.robot.commands.climber_commands.*;
import frc.robot.commands.intake_commands.adjustor.CalibrateRetractIntake;
import frc.robot.commands.intake_commands.adjustor.ExtendIntakeMagnet;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;
import frc.robot.commands.shooter_commands.IncrementShooterAngle;
import frc.robot.commands.shooter_commands.RevShooterCMD;
import frc.robot.commands.shooter_commands.SetShooterAngleCommand;
import frc.robot.commands.shooter_commands.ShootCMD;
import frc.robot.commands.shooter_commands.TimedShootCMD;
import frc.robot.commands.shooter_commands.UserShootCMD;
import frc.robot.commands.storage_commands.SequentialStorageCMDs.IntakeProcessAccept;
import frc.robot.commands.storage_commands.SequentialStorageCMDs.StorageDecision;
import frc.robot.robot_utils.ShooterCamera;
import frc.robot.robot_utils.trigger.DPADUPButton;
import frc.robot.robot_utils.trigger.MultiTrigger;
import frc.robot.robot_utils.trigger.TriggerButtonLeft;
import frc.robot.robot_utils.trigger.TriggerButtonRight;
import me.wobblyyyy.pathfinder2.wpilib.PathfinderSubsystem;
import frc.robot.commands.storage_commands.RunStorageAcceptor;
import frc.robot.commands.storage_commands.RunStorageCMD;

import static frc.robot.Constants.Control.*;

import java.util.Map;

public class RobotContainer {

    private final Joystick xyStick = new Joystick(XY_STICK_ID);
    private final Joystick zStick = new Joystick(Z_STICK_ID);
    public static final XboxController controller = new XboxController(CONTROLLER_ID);

    private double currentShooterAngle = 0;

    // Xbox Extracted Controller Buttons
    private final JoystickButton xButton = new JoystickButton(controller, XBOX_X);
    private final JoystickButton yButton = new JoystickButton(controller, XBOX_Y);
    private final JoystickButton aButton = new JoystickButton(controller, XBOX_A);
    private final JoystickButton bButton = new JoystickButton(controller, XBOX_B);
    private final JoystickButton lBumper = new JoystickButton(controller, XBOX_LEFT_TRIGGER);
    private final JoystickButton rBumper = new JoystickButton(controller, XBOX_RIGHT_TRIGGER);
    private final JoystickButton lStick = new JoystickButton(controller, XBOX_LEFT_STICK);
    private final JoystickButton rStick = new JoystickButton(controller, XBOX_RIGHT_STICK);
    private final JoystickButton startButton = new JoystickButton(controller, XBOX_START);
    private final JoystickButton leftTriggerBTN = new TriggerButtonLeft(controller, 100);
    private final JoystickButton rightTriggerBTN = new TriggerButtonRight(controller, 101);
    private final JoystickButton dpadUPBTN = new DPADUPButton(controller, 102);

    private final JoystickButton endButton = new JoystickButton(controller, XboxController.Button.kBack.value);

    private final JoystickButton xyButtonFive = new JoystickButton(xyStick, 5);

    private final SequentialCommandGroup testSwerveDrive = new SequentialCommandGroup(
            new MoveRightCMD(),
            new MoveLeftCMD(),
            new MoveFWDCMD(),
            new MoveBCKCMD()
    );

    private final MultiTrigger lowerLeftTrigger = new MultiTrigger(endButton, lBumper);
    private final MultiTrigger lowerRightTrigger = new MultiTrigger(endButton, rBumper);

    private final SequentialCommandGroup simpleAutoonomousCMD = new SequentialCommandGroup(new ParallelCommandGroup(new TimedShootCMD(6, 4500), new SetShooterAngleCommand(10)), new MoveFWDCMD());

    private boolean isRobotCalibrated = false;

    private final ParallelCommandGroup raiseClimberGroup = new ParallelCommandGroup(
            new MoveLeftClimberUp(), new MoveRightClimberUp()
    );

    private final ParallelCommandGroup lowerClimberGroup = new ParallelCommandGroup(
            new MoveLeftClimberDown(), new MoveRightClimberDown()
    );

    // // TODO: may need to add/remove commands from this group.
    // private final SequentialCommandGroup autoShootGroup = new SequentialCommandGroup(
    //         new ParallelCommandGroup(new SetShooterAngleCommand(20) ,new CenterShooterToHubCommand()),
    //         new AutoShootCommand()
    // );

    private final SequentialCommandGroup processBallCMD = new SequentialCommandGroup(
            new ExtendIntakeMagnet(),
            new IntakeProcessAccept());

    private final ParallelCommandGroup calibrateGroup = new ParallelCommandGroup(
            new CalibrateRetractIntake()
    );

    public RobotContainer() {
        Robot.swerveDrive.setDefaultCommand(new ArcadeCommand(() ->
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        deadzone(xyStick.getX(), Chassis.CONTROLLER_DEADZONE),
                        -deadzone(xyStick.getY(), Chassis.CONTROLLER_DEADZONE),
                        -deadzone(zStick.getTwist(), Chassis.CONTROLLER_DEADZONE),
                        Rotation2d.fromDegrees(0)
                )
        ));

        //Robot.shooter.setDefaultCommand(new RevShooterCMD(4500));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        lStick.whenHeld(new CenterShooterToHubCommand());
        rStick.whenHeld(new RunStorageAcceptor());

        aButton.whenHeld(new UserShootCMD(4500));

        yButton.whenActive(processBallCMD);

        xButton.whenActive(new RetractIntakeMagnet());

        bButton.whenActive(new SetShooterAngleCommand(10));

        startButton.whenActive(new CalibrateRetractIntake()); 

        leftTriggerBTN.whenHeld(new ManualMoveLeftClimber(true));
        rightTriggerBTN.whenHeld(new ManualMoveRightClimber(true));

        lBumper.whenHeld(new ManualMoveLeftClimber(false));
        rBumper.whenHeld(new ManualMoveRightClimber(false));

        dpadUPBTN.whenHeld(new RunStorageCMD());
    }

    // public Command getAutonomousCommand() {
    //     // An ExampleCommand will run in autonomous
    //     return null;
    // }

    public Command getAutonomousCommand(PathfinderSubsystem pathfinder) {
        return new TestAutonomous(pathfinder);
    }

    // public Command getAutonomousCommand(PathfinderSubsystem pathfinderSubsystem) {
    //     return (Command) new TestAutonomous(pathfinderSubsystem);
    // }

    // public SequentialCommandGroup getAutoShootGroup() {
    //     return autoShootGroup;
    // }
    
    public SequentialCommandGroup getSimpleAutoCommand(){
        return simpleAutoonomousCMD;
    }

    public double deadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }

    public void calibrateRobot(){
        calibrateGroup.schedule();
        isRobotCalibrated = true;
    }

    public boolean isRobotCalibrated(){
        return isRobotCalibrated;
    }

    public XboxController getXbox(){
        return controller;
    }
}
