

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Chassis;
import frc.robot.commands.autonomous_commands.CameraAuto;
import frc.robot.commands.autonomous_commands.TestAutonomous;
import frc.robot.commands.chassis_commands.*;
import frc.robot.commands.climber_commands.*;
import frc.robot.commands.intake_commands.adjustor.CalibrateRetractIntake;
import frc.robot.commands.intake_commands.adjustor.ExtendIntakeMagnet;
import frc.robot.commands.intake_commands.adjustor.RetractIntakeMagnet;
import frc.robot.commands.shooter_commands.*;
import frc.robot.commands.storage_commands.SequentialStorageCMDs.IntakeProcessAccept;
import frc.robot.robot_utils.trigger.*;
import me.wobblyyyy.pathfinder2.wpilib.PathfinderSubsystem;
import frc.robot.commands.storage_commands.RunStorageAcceptor;
import frc.robot.commands.storage_commands.RunStorageCMD;
import frc.robot.commands.storage_commands.SimpleProcessBallCMD;

import static frc.robot.Constants.Control.*;

public class RobotContainer {
    private final Joystick xyStick = new Joystick(XY_STICK_ID);
    private final Joystick zStick = new Joystick(Z_STICK_ID);
    public static final XboxController controller = new XboxController(CONTROLLER_ID);

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

    private final ConditionalButton leftTriggerButton = new ConditionalButton(controller, 100);
    private final ConditionalButton rightTriggerButton = new ConditionalButton(controller, 101);
    private final ConditionalButton dpadDownButton = new ConditionalButton(controller, 102);


    private final SequentialCommandGroup simpleAutonomousCMD = new SequentialCommandGroup(
            new TimedShootCMD(3.5, 4500),
            new TimedMoveFWDCMD()
    );

    //public final ParallelCommandGroup autoShootCMD = new ParallelCommandGroup(new AutoAdjustShooterAngle(), new ShootCMD(4500));

    public static final IncrementShooterAngle incrementAngleCMD = new IncrementShooterAngle();

    private final SequentialCommandGroup autoShootGroup = new SequentialCommandGroup(
            new ParallelCommandGroup(new AutoAdjustShooterAngle(), new CenterShooterToHubCommand()),
            new TimedShootCMD(3, 4500)
    );

    // FIXME: Sometimes the intake decides to not extend for some reason, gets partially
    // FIXME: stuck right at the beginning, and completely freezes up even when pressing
    // FIXME: the calibrateGroup button (Start) that should be retracting the intake.
    private final SequentialCommandGroup processBallCMD = new SequentialCommandGroup(
            new ExtendIntakeMagnet(),
            new IntakeProcessAccept());

    private final ParallelCommandGroup calibrateGroup = new ParallelCommandGroup(
            new CalibrateRetractIntake()
            //new CalibrateAdjustorCMD()
    );

    private boolean isRobotCalibrated = false;

    public RobotContainer() {
        Robot.swerveDrive.setDefaultCommand(new ArcadeCommand(() ->
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        deadzone(xyStick.getX(), Chassis.CONTROLLER_DEADZONE),
                        -deadzone(xyStick.getY(), Chassis.CONTROLLER_DEADZONE),
                        -deadzone(zStick.getTwist(), Chassis.CONTROLLER_DEADZONE),
                        Rotation2d.fromDegrees(0)
                )
        ));

        //Robot.adjustor.setDefaultCommand(new AutoAdjustShooterAngle());

        this.leftTriggerButton.setSupplier(() -> (controller.getLeftTriggerAxis() > 0.8));
        this.rightTriggerButton.setSupplier(() -> (controller.getRightTriggerAxis() > 0.8));
        this.dpadDownButton.setSupplier(() -> (controller.getPOV() >= 315 || controller.getPOV() >= 90));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        lStick.whenHeld(new CenterShooterToHubCommand());
        rStick.whenHeld(new RunStorageAcceptor());

 
        aButton.whenHeld(new ShootCMD(4500));

        yButton.whenHeld(new SimpleProcessBallCMD());

        xButton.whenActive(new RetractIntakeMagnet());

        bButton.whenActive(new InstantCommand(()->{
            if(CommandScheduler.getInstance().requiring(Robot.intake) != null){
                CommandScheduler.getInstance().requiring(Robot.intake).cancel();
            }
            
        }));

        startButton.whenActive(new CalibrateRetractIntake());

        leftTriggerButton.whenHeld(new ManualMoveLeftClimber(true));
        rightTriggerButton.whenHeld(new ManualMoveRightClimber(true));

        lBumper.whenHeld(new ManualMoveLeftClimber(false));
        rBumper.whenHeld(new ManualMoveRightClimber(false));

        dpadDownButton.whenHeld(new RunStorageCMD());
    }


    public Command getAutonomousCommand(PathfinderSubsystem pathfinder) {
        return new TestAutonomous(pathfinder);
    }

    // public Command getAutonomousCommand(PathfinderSubsystem pathfinderSubsystem) {
    //     return (Command) new TestAutonomous(pathfinderSubsystem);
    // }

    public SequentialCommandGroup getAutoShootGroup() {
        return autoShootGroup;
    }

    public SequentialCommandGroup getSimpleAutoCommand() {
        return simpleAutonomousCMD;
    }

    public double deadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0;
    }

    public void calibrateRobot() {
        calibrateGroup.schedule();
        isRobotCalibrated = true;
    }

    public boolean isRobotCalibrated() {
        return isRobotCalibrated;
    }

    public XboxController getXbox() {
        return controller;
    }

    public void resetIncrementAngle(){
        incrementAngleCMD.resetAngle();
    }

}
